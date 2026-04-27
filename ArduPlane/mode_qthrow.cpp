#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

namespace {

constexpr float THROW_HIGH_SPEED_MS = 5.0f;
constexpr float THROW_ATTITUDE_GOOD_COS = 0.866f;
constexpr float THROW_STABILIZE_THROTTLE = 0.5f;
constexpr float THROW_RELEASE_ACCEL_G = 1.15f;
constexpr uint32_t THROW_ACCEL_HOLD_MS = 20;
constexpr uint32_t THROW_WING_DEPLOY_DELAY_MS = 200;
constexpr uint32_t THROW_ATTITUDE_HOLD_MS = 500;

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
    throw_accel_start_ms = 0;
    deploy_start_ms = 0;
    upright_start_ms = 0;
    next_mode_attempted = false;
    relax_wing();

    return true;
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
        throw_accel_start_ms = 0;
        deploy_start_ms = 0;
        upright_start_ms = 0;
        next_mode_attempted = false;
        if (manual_wing_deploy_requested()) {
            deploy_wing();
        } else {
            relax_wing();
        }
    } else if (stage == Stage::Disarmed) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: waiting for throw");
        relax_wing();
        stage = Stage::WaitingForThrow;
    } else if ((stage == Stage::WaitingForThrow) && throw_detected()) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: throw detected");
        deploy_wing();
        deploy_start_ms = now;
        upright_start_ms = 0;
        stage = Stage::DeployingWing;
    } else if ((stage == Stage::DeployingWing) &&
               ((now - deploy_start_ms) >= THROW_WING_DEPLOY_DELAY_MS)) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: motors enabled");
        relax_wing();
        upright_start_ms = 0;
        stage = Stage::VerticalRecover;
    } else if (stage == Stage::VerticalRecover) {
        if (throw_attitude_good()) {
            if (upright_start_ms == 0) {
                upright_start_ms = now;
            } else if (((now - upright_start_ms) >= THROW_ATTITUDE_HOLD_MS) && !next_mode_attempted) {
                gcs().send_text(MAV_SEVERITY_INFO, "QThrow: launch stabilized");
                next_mode_attempted = true;
                IGNORE_RETURN(switch_to_next_mode());
            }
        } else {
            upright_start_ms = 0;
        }
    }

    switch (stage) {
    case Stage::Disarmed:
    case Stage::WaitingForThrow:
    case Stage::DeployingWing:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        quadplane.relax_attitude_control();
        break;

    case Stage::VerticalRecover:
        quadplane.hold_stabilize(THROW_STABILIZE_THROTTLE);
        plane.stabilize_roll();
        plane.stabilize_pitch();
        output_rudder_and_steering(0.0f);
        break;
    }
}

bool ModeQThrow::throw_detected()
{
    const uint32_t now = AP_HAL::millis();
    const float accel_threshold_mss = MAX(1.1f, quadplane.qthrow_accel_trigger.get()) * GRAVITY_MSS;
    const float release_threshold_mss = THROW_RELEASE_ACCEL_G * GRAVITY_MSS;
    const float accel_mss = plane.ins.get_accel().length();

    if (accel_mss >= accel_threshold_mss) {
        if (throw_accel_start_ms == 0) {
            throw_accel_start_ms = now;
        }
        return false;
    }

    if (throw_accel_start_ms == 0) {
        return false;
    }

    const bool accel_held_long_enough = (now - throw_accel_start_ms) >= THROW_ACCEL_HOLD_MS;
    throw_accel_start_ms = 0;

    return accel_held_long_enough && (accel_mss <= release_threshold_mss);
}

bool ModeQThrow::throw_attitude_good() const
{
    const Matrix3f &rot_mat = ahrs.get_rotation_body_to_ned();
    return rot_mat.c.z > THROW_ATTITUDE_GOOD_COS;
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
