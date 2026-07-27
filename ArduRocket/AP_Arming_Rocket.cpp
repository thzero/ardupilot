#include "AP_Arming_Rocket.h"
#include "ArduRocket.h"

// "On the rail" gates. Kept as constants rather than parameters: they are safety
// limits, not tuning knobs.
//
// 20 degrees is the maximum launch-rail tilt permitted by both NAR and Tripoli
// safety codes, so an airframe sitting further off vertical than this is either
// mis-mounted or the attitude solution is wrong. Either way it must not arm.
#define RKT_ARM_TILT_MAX_DEG   20.0f
#define RKT_ARM_GYRO_MAX_DPS   15.0f

bool AP_Arming_Rocket::pre_arm_checks(bool display_failure)
{
    if (armed) {
        return true;
    }

    // The rocket has no GPS requirement, no RC and no mission, but it absolutely
    // depends on the IMU and barometer, so let the base class check what it can.
    if (!AP_Arming::pre_arm_checks(display_failure)) {
        return false;
    }

    // Arming means "on the rail". Refuse to arm unless the airframe is standing up
    // near-vertical and stationary: arming on the bench, or lying on its side, is a
    // configuration error we can catch here rather than on the pad.
    const AP_AHRS &ahrs = AP::ahrs();

    // Tilt from vertical, as the real geometric angle. This is the SAME call that
    // feeds the TILT telemetry, so what the pad crew reads is what is gated on.
    //
    // NOTE this was previously |roll| + |pitch|, which overestimates whenever both
    // axes are non-zero and so tightened this 20 deg limit to as little as 14.1 deg
    // on a diagonally-leaning rail -- refusing to arm on setups that NAR and Tripoli
    // both permit.
    if (rocket.ahrs_view != nullptr) {
        const float tilt_deg = rocket.tilt_from_vertical_deg();
        if (tilt_deg > RKT_ARM_TILT_MAX_DEG) {
            check_failed(display_failure, "not vertical (%.0f deg off)", (double)tilt_deg);
            return false;
        }
    }

    // Stillness: not being carried or shaken.
    if (ahrs.get_gyro().length() > radians(RKT_ARM_GYRO_MAX_DPS)) {
        check_failed(display_failure, "not still");
        return false;
    }

    /*
      Every fin must be assigned to a real output channel. This catches the
      "forgot to set SERVO3_FUNCTION" class of misconfiguration, where the vehicle
      would arm and fly with one fin permanently dead.

      NOTE what this does NOT check: whether the fins move in the correct
      DIRECTION. That is not verifiable in software on the pad. Confirming
      direction means confirming that a deflection produces the right motion, and
      a clamped airframe with no airflow produces no motion to observe. Comparing
      the mixer's output against measured attitude is circular -- the command is
      derived from that attitude, so the signs agree by construction even with a
      servo horn fitted backwards. Fin direction must be confirmed by hand before
      the rocket goes on the rail; see ARDUROCKET_PLAN.md.
     */
    const SRV_Channel::Function fin_fn[] = {
        SRV_Channel::k_rocketFin1, SRV_Channel::k_rocketFin2,
        SRV_Channel::k_rocketFin3, SRV_Channel::k_rocketFin4,
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(fin_fn); i++) {
        if (!SRV_Channels::function_assigned(fin_fn[i])) {
            check_failed(display_failure, "fin %u has no output (set SERVOn_FUNCTION=%u)",
                         (unsigned)(i + 1), (unsigned)fin_fn[i]);
            return false;
        }
    }

    /*
      The fin check must have been run and confirmed on the rail. This is the gate
      that makes the human fin-direction check unskippable: ARM stays blocked, with
      this reason shown in the ground station's standard pre-arm readout, until the
      operator has triggered the fin check on the rail and watched it. See
      ArduRocket.h for the operator flow. The arm press itself is the attestation.
     */
    if (!rocket.fin_check_ok()) {
        check_failed(display_failure, "Fin Check required (Run it on the rail)");
        return false;
    }

    /*
      NOTE: there is deliberately no "is GPS in the EKF?" check here.

      An earlier version warned when the EKF had a horizontal position solution,
      but that fires whenever the EKF has an ORIGIN -- which happens regardless of
      whether GPS is fused for control -- so it was a false positive, and being in
      pre_arm_checks it repeated on every periodic check and flooded the link.

      Keeping GPS out of the control solution is enforced by configuration
      (EK3_SRC1_POSXY/VELXY = 0, documented in rocket.parm), not by a runtime check.
     */

    return true;
}

bool AP_Arming_Rocket::arm(AP_Arming::Method method, bool do_arming_checks)
{
    if (rocket.motors != nullptr && rocket.motors->armed()) {
        return true;    // already armed
    }

    /*
      Single-press arm, gated on the fin check via pre_arm_checks() above.

      Fin direction cannot be verified in software, so it is a human check -- made
      unskippable by requiring fin_check_ok() in pre-arm. The operator triggers the
      fin check on the rail (a ground-station "Fin Check" button), watches each
      announced fin move, and then arms. Until that check is latched, this arm call
      fails pre-arm with a clear "fin check required" reason in the GCS, so ARM
      never spuriously errors -- it simply stays blocked with the reason shown, then
      succeeds once the check is done. The arm press is the operator's attestation
      that the fins moved correctly.
     */
    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        return false;
    }

    /*
      Every arming cycle starts on the rail, pre-launch. Re-latching here rather
      than at boot means a disarm/re-arm after a scrubbed countdown does not leave
      the vehicle believing it has already launched.
     */
    rocket.g2.rocket.reset();

    if (rocket.attitude_control != nullptr) {
        rocket.attitude_control->reset_rate_controller_I_terms();
        rocket.attitude_control->reset_yaw_target_and_rate();
    }

    /*
      Capture the rail attitude. From here the controller holds THIS, not true
      vertical, so the rocket flies straight off a tilted rail instead of fighting
      the tilt at the moment fin authority is lowest. The pre-arm check above has
      already bounded it to RKT_ARM_TILT_MAX_DEG.
     */
    if (rocket.ahrs_view != nullptr) {
        rocket.rail_roll_rad  = rocket.ahrs_view->roll;
        rocket.rail_pitch_rad = rocket.ahrs_view->pitch;
        gcs().send_text(MAV_SEVERITY_INFO, "Rocket: rail attitude %.1f/%.1f deg",
                        (double)degrees(rocket.rail_roll_rad),
                        (double)degrees(rocket.rail_pitch_rad));
    }

    /*
      This vehicle has no GPS, so the EKF never gets a home from one. Zero the
      height datum at arming instead: every altitude and climb rate we use -- which
      is what apogee detection rides on -- is then referenced to the launch rail.
     */
    AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.home_is_set()) {
        ahrs.resetHeightDatum();
        LOGGER_WRITE_EVENT(LogEvent::EKF_ALT_RESET);
    }

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(true);
#endif
    AP_Notify::flags.armed = true;

    /*
      Propagate the armed state outward. Both of these are easy to forget and each
      one silently breaks something: without set_soft_armed() the HAL/simulator
      never sees an armed vehicle, and without motors->armed() the mixer stays shut
      down and the flight stage machine never leaves PREP.
     */
    hal.util->set_soft_armed(true);
    if (rocket.motors != nullptr) {
        rocket.motors->armed(true);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: armed, on the rail");

    return true;
}

bool AP_Arming_Rocket::disarm(AP_Arming::Method method, bool do_disarm_checks)
{
    if (rocket.motors != nullptr && !rocket.motors->armed()) {
        return true;    // already disarmed
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

    if (rocket.motors != nullptr) {
        rocket.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        rocket.motors->armed(false);
    }
    hal.util->set_soft_armed(false);

    // A fresh arming cycle must re-run the fin check: clear the latch on disarm so a
    // scrubbed countdown cannot be re-armed on a stale confirmation.
    rocket.fin_check_valid = false;

#if HAL_LOGGING_ENABLED
    AP::logger().set_vehicle_armed(false);
#endif
    AP_Notify::flags.armed = false;

    return true;
}
