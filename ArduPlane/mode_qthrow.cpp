#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

namespace {

constexpr float THROW_ATTITUDE_GOOD_COS = 0.866f;
constexpr float THROW_UPRIGHT_THROTTLE = 0.5f;

}

bool ModeQThrow::_enter()
{
    if (!quadplane.tailsitter.enabled()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "QThrow: tailsitter only");
        return false;
    }
    if (!wing_deploy_servo_available()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "QThrow: wing servo missing");
        return false;
    }

    stage = Stage::Disarmed;
    throw_detect_state = ThrowDetectState::Idle;
    throw_accel_start_ms = 0;
    throw_release_start_ms = 0;
    deploy_start_ms = 0;
    upright_start_ms = 0;
    last_log_ms = 0;
    armed_height_m = 0.0f;
    deploy_height_m = 0.0f;
    target_height_m = 0.0f;
    throw_min_alt_reached = false;
    next_mode_attempted = false;
    prev_stage = stage;
    prev_throw_detect_state = throw_detect_state;
    AP_Notify::flags.waiting_for_throw = false;
    relax_wing();

    return true;
}

void ModeQThrow::_exit()
{
    AP_Notify::flags.waiting_for_throw = false;
}

bool ModeQThrow::_pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!quadplane.tailsitter.enabled()) {
        hal.util->snprintf(buffer, buflen, "tailsitter only");
        return false;
    }
    if (!wing_deploy_servo_available()) {
        hal.util->snprintf(buffer, buflen, "wing servo missing");
        return false;
    }
    const int8_t deploy_channel = quadplane.qthrow_deploy_channel.get();
    if ((deploy_channel > 0) && (rc().channel(deploy_channel - 1) == nullptr)) {
        hal.util->snprintf(buffer, buflen, "invalid THROW_CHAN");
        return false;
    }

    return Mode::_pre_arm_checks(buflen, buffer);
}

void ModeQThrow::update()
{
    // Q_THROW is tailsitter-only, so use tailsitter stick mapping directly
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    if (quadplane.tailsitter.max_roll_angle > 0) {
        plane.nav_roll_cd = quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * quadplane.attitude_control->lean_angle_max_cd();
    }

    plane.nav_pitch_cd = pitch_input * quadplane.attitude_control->lean_angle_max_cd();
    quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

void ModeQThrow::run()
{
    const uint32_t now = AP_HAL::millis();

    if (!plane.arming.is_armed_and_safety_off()) {
        stage = Stage::Disarmed;
        throw_detect_state = ThrowDetectState::Idle;
        throw_accel_start_ms = 0;
        throw_release_start_ms = 0;
        deploy_start_ms = 0;
        upright_start_ms = 0;
        armed_height_m = 0.0f;
        deploy_height_m = 0.0f;
        target_height_m = 0.0f;
        throw_min_alt_reached = false;
        next_mode_attempted = false;
        AP_Notify::flags.waiting_for_throw = false;
        if (manual_wing_deploy_requested()) {
            deploy_wing();
        } else {
            relax_wing();
        }
    } else if (stage == Stage::Disarmed) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: waiting for throw");
        armed_height_m = pos_control->get_pos_estimate_U_m();
        throw_min_alt_reached = is_zero(quadplane.qthrow_min_alt.get());
        relax_wing();
        stage = Stage::WaitingForThrow;
    } else {
        const float rel_alt_m = pos_control->get_pos_estimate_U_m() - armed_height_m;
        if (!throw_min_alt_reached && (rel_alt_m >= quadplane.qthrow_min_alt.get())) {
            throw_min_alt_reached = true;
            gcs().send_text(MAV_SEVERITY_INFO, "QThrow: MIN_HEIGHT reached.");
        }

        if ((stage == Stage::WaitingForThrow) && throw_detected()) {
            gcs().send_text(MAV_SEVERITY_INFO, "QThrow: throw detected");
            deploy_height_m = pos_control->get_pos_estimate_U_m();
            target_height_m = deploy_height_m + quadplane.qthrow_altitude_ascend.get();
            deploy_wing();
            deploy_start_ms = now;
            upright_start_ms = 0;
            AP_Notify::flags.waiting_for_throw = false;
            stage = Stage::DeployingWing;
        } else if ((stage == Stage::DeployingWing) &&
                   ((now - deploy_start_ms) >= (uint32_t)MAX<int16_t>(0, quadplane.qthrow_deploy_delay_ms.get()))) {
            gcs().send_text(MAV_SEVERITY_INFO, "QThrow: motors enabled");
            relax_wing();
            stage = Stage::Uprighting;
        } else if (stage == Stage::Uprighting) {
            if (throw_attitude_good()) {
                gcs().send_text(MAV_SEVERITY_INFO, "QThrow: uprighted - controlling height");
                pos_control->D_init_controller_no_descent();
                pos_control->set_pos_desired_U_m(target_height_m);
                stage = Stage::HeightStabilize;
            }
        } else if (stage == Stage::HeightStabilize) {
            if (throw_height_good() && !next_mode_attempted) {
                gcs().send_text(MAV_SEVERITY_INFO, "QThrow: height achieved - switching next mode");
                next_mode_attempted = true;
                IGNORE_RETURN(switch_to_next_mode());
            }
        }
    }

    switch (stage) {
    case Stage::Disarmed:
    case Stage::WaitingForThrow:
    case Stage::DeployingWing:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        quadplane.relax_attitude_control();
        AP_Notify::flags.waiting_for_throw = (stage == Stage::WaitingForThrow);
        break;

    case Stage::Uprighting:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        attitude_control->set_throttle_out(THROW_UPRIGHT_THROTTLE, false, 0.0f);
        output_rudder_and_steering(0.0f);
        AP_Notify::flags.waiting_for_throw = false;
        break;

    case Stage::HeightStabilize:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f);
        pos_control->D_update_controller();
        output_rudder_and_steering(0.0f);
        AP_Notify::flags.waiting_for_throw = false;
        break;
    }

#if HAL_LOGGING_ENABLED
    // log at 10Hz or if stage/detector state changes
    if ((stage != prev_stage) ||
        (throw_detect_state != prev_throw_detect_state) ||
        ((now - last_log_ms) > 100U)) {
        prev_stage = stage;
        prev_throw_detect_state = throw_detect_state;
        last_log_ms = now;

        const float accel_mss = plane.ins.get_accel().length();
        const float rel_alt_m = pos_control->get_pos_estimate_U_m() - armed_height_m;
        const uint32_t spike_age_ms = (throw_accel_start_ms == 0) ? 0 : (now - throw_accel_start_ms);
        const uint32_t release_age_ms = (throw_release_start_ms == 0) ? 0 : (now - throw_release_start_ms);
        const bool throw_detect = (stage > Stage::WaitingForThrow);
        const bool attitude_ok = (stage > Stage::Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Stage::HeightStabilize) || throw_height_good();

// @LoggerMessage: QTHR
// @Description: Q_THROW mode messages
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of Q_THROW mode
// @Field: DState: Current throw detector state
// @Field: Acc: Total acceleration magnitude
// @Field: SpikeMS: Milliseconds since spike detection started
// @Field: RelMS: Milliseconds since release-detect window started
// @Field: RelAlt: Altitude gain since arming
// @Field: MinAlt: True if the minimum altitude gate has been reached
// @Field: Throw: True if throw has been detected and launch has progressed beyond detection
// @Field: AttOk: True if the aircraft is upright enough
// @Field: HgtOk: True if the target height has been reached

        AP::logger().WriteStreaming(
            "QTHR",
            "TimeUS,Stage,DState,Acc,SpikeMS,RelMS,RelAlt,MinAlt,Throw,AttOk,HgtOk",
            "s-n--m-----",
            "F-00-0-----",
            "QBBfIIfbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (uint8_t)throw_detect_state,
            (double)accel_mss,
            spike_age_ms,
            release_age_ms,
            (double)rel_alt_m,
            throw_min_alt_reached,
            throw_detect,
            attitude_ok,
            height_ok);
    }
#endif
}

bool ModeQThrow::throw_detected()
{
    const uint32_t now = AP_HAL::millis();
    const float accel_threshold_mss = MAX(1.1f, quadplane.qthrow_accel_trigger.get()) * GRAVITY_MSS;
    const float release_threshold_mss = quadplane.qthrow_accel_release_g.get() * GRAVITY_MSS;
    const uint32_t hold_ms = MAX<int16_t>(0, quadplane.qthrow_accel_hold_ms.get());
    const uint32_t release_timeout_ms = MAX<int16_t>(20, quadplane.qthrow_accel_release_timeout_ms.get());
    const float accel_mss = plane.ins.get_accel().length();
    const bool spike_present = accel_mss >= accel_threshold_mss;

    switch (throw_detect_state) {
    case ThrowDetectState::Idle:
        if (spike_present) {
            throw_detect_state = ThrowDetectState::SpikeSeen;
            throw_accel_start_ms = now;
        }
        return false;

    case ThrowDetectState::SpikeSeen:
        if (!spike_present) {
            throw_detect_state = ThrowDetectState::Idle;
            throw_accel_start_ms = 0;
            return false;
        }
        if ((now - throw_accel_start_ms) >= hold_ms) {
            throw_detect_state = ThrowDetectState::HoldSatisfied;
            throw_release_start_ms = now;
        }
        return false;

    case ThrowDetectState::HoldSatisfied:
        if ((accel_mss <= release_threshold_mss) && throw_min_alt_reached) {
            throw_detect_state = ThrowDetectState::Idle;
            throw_accel_start_ms = 0;
            throw_release_start_ms = 0;
            return true;
        }
        if ((now - throw_release_start_ms) >= release_timeout_ms) {
            throw_detect_state = ThrowDetectState::Idle;
            throw_accel_start_ms = 0;
            throw_release_start_ms = 0;
        }
        return false;
    }

    throw_detect_state = ThrowDetectState::Idle;
    throw_accel_start_ms = 0;
    throw_release_start_ms = 0;
    return false;
}

bool ModeQThrow::throw_attitude_good() const
{
    const Matrix3f &rot_mat = ahrs.get_rotation_body_to_ned();
    return rot_mat.c.z > THROW_ATTITUDE_GOOD_COS;
}

bool ModeQThrow::throw_height_good() const
{
    return fabsf(pos_control->get_pos_error_D_m()) < 0.5f;
}

bool ModeQThrow::wing_deploy_servo_available() const
{
    return SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control);
}

bool ModeQThrow::manual_wing_deploy_requested() const
{
    const int8_t deploy_channel = quadplane.qthrow_deploy_channel.get();
    if (deploy_channel <= 0) {
        return false;
    }

    RC_Channel *chan = rc().channel(deploy_channel - 1);
    if (chan == nullptr) {
        return false;
    }

    return chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
}

void ModeQThrow::deploy_wing()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_landing_gear_control, SRV_Channel::Limit::MAX);
}

void ModeQThrow::relax_wing()
{
    SRV_Channels::set_output_limit(SRV_Channel::k_landing_gear_control, SRV_Channel::Limit::TRIM);
}

bool ModeQThrow::switch_to_next_mode()
{
    if (!plane.set_mode_by_number(Mode::Number::QHOVER, ModeReason::THROW_COMPLETE)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: QHOVER transition failed");
        return false;
    }

    const int8_t next_mode_num = quadplane.throw_next_mode.get();
    if (next_mode_num < 0) {
        return true;
    }

    const Mode::Number next_mode = static_cast<Mode::Number>(next_mode_num);
    if (next_mode == Mode::Number::QHOVER) {
        return true;
    }

    Mode *next_mode_ptr = plane.mode_from_mode_num(next_mode);
    if ((next_mode_ptr == nullptr) ||
        (next_mode == Mode::Number::INITIALISING) ||
        (next_mode == Mode::Number::QTHROW)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: invalid THROW_NEXT_MODE");
        return false;
    }

    if (!plane.set_mode_by_number(next_mode, ModeReason::THROW_COMPLETE)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: next mode transition failed");
        return false;
    }

    return true;
}

#endif
