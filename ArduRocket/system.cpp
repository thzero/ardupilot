#include "ArduRocket.h"

#include <AP_BoardConfig/AP_BoardConfig.h>

/*
  ArduRocket startup.
 */

static void failsafe_check_static()
{
    rocket.failsafe_check();
}

void ArduRocket::init_ardupilot()
{
    // initialise notify system
    notify.init();

    battery.init();

    barometer.init();

    // Bring up the AHRS and IMU. This is mandatory and easy to miss: without
    // ins.init() the SITL/hardware IMU sample rate is never set, no samples are
    // ever scheduled, and the main loop blocks forever in wait_for_sample().
    // The rocket flies the multicopter attitude stack, so use the COPTER class.
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);
    ins.init(scheduler.get_loop_rate_hz());

    // setup telem slots with serial ports
    gcs().setup_uarts();

    // No receiver will ever be attached, but the RC_Channels object has to be
    // initialised because shared code assumes the rc() singleton is live.
    rc().init();

    // allocate the control objects before anything tries to use them
    allocate_motors();

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);
    AP_Param::invalidate_count();

    // setup the 'main loop is dead' check
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();

    AP::compass().init();

    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise the fin outputs
    AP::srv().init();
    AP::srv().enable_aux_servos();

    // Only FRAME_CLASS=FINS can reach here (allocate_motors() refuses anything else),
    // so the fin frame class is correct. A future TVC mixer must pass its own frame
    // class here; if it is forgotten, set_initialised_ok() rejects the mismatch and
    // the config_error below fires rather than flying the wrong mixer.
    motors->init(AP_Motors::MOTOR_FRAME_ROCKET, AP_Motors::MOTOR_FRAME_TYPE_PLUS);
    if (!motors->initialised_ok()) {
        AP_BoardConfig::config_error("MOTOR init failed");
    }
    motors->update_throttle_range();
    motors->set_dt_s(scheduler.get_loop_period_s());

    attitude_control->set_dt_s(scheduler.get_loop_period_s());
    attitude_control->parameter_sanity_check();

    /*
      There is no throttle to interlock: the solid motor is not on an output at
      all. The interlock exists to keep propellers stopped, so for this vehicle it
      is always satisfied and the fins go live purely on arming state.
     */
    motors->set_interlock(true);

    // reset AHRS including gyro bias now that the IMU is calibrated
    ahrs.reset();

    stage = FlightStage::PREP;

    initialised = true;
}

/*
  allocate the motors and attitude control objects
 */
void ArduRocket::allocate_motors(void)
{
    /*
      FRAME_CLASS picks the mixer, and is read exactly once here -- the mixer cannot
      be swapped at runtime, which is why the parameter is reboot-required.

      An unimplemented selection must refuse to boot. Falling back to fins would be
      worse: the operator would have configured a gimbal, seen the vehicle come up
      apparently fine, and discovered on the pad that the wrong actuator was driven.
     */
    switch (ParametersG2::FrameClass(g2.frame_class.get())) {
    case ParametersG2::FrameClass::FINS:
        motors = NEW_NOTHROW AP_FinMixerRocket(scheduler.get_loop_rate_hz());
        if (motors == nullptr) {
            AP_BoardConfig::allocation_error("AP_FinMixerRocket");
        }
        motors_var_info = AP_FinMixerRocket::var_info;
        break;

    case ParametersG2::FrameClass::TVC:
        // Scoped in ARDUROCKET_PLAN.md Appendix A, not built. Fail loudly.
        AP_BoardConfig::config_error("FRAME_CLASS=1 (TVC) not implemented; use 0 (Fins)");
        break;

    default:
        AP_BoardConfig::config_error("FRAME_CLASS=%d invalid", (int)g2.frame_class);
        break;
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    /*
      This single line is what makes the vehicle a rocket rather than a
      multicopter lying on its back.

      ROTATION_PITCH_90 rotates the AHRS solution so the nose-up airframe is
      presented to the attitude controller as though it were level. "Hold
      vertical" therefore becomes "hold level", and AC_AttitudeControl_Multi
      applies unmodified. The cost is that the controller's axes no longer mean
      what their names say on this airframe: view roll and view pitch are the two
      tilt axes, and view yaw is spin about the long axis. See AP_FinMixerRocket.h.
     */
    ahrs_view = ahrs.create_view(ROTATION_PITCH_90);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

    /*
      AC_AttitudeControl_Multi, not AC_AttitudeControl_TS. The tailsitter variant
      adds exactly two things -- relaxing roll/yaw while holding pitch, and
      blending body-frame roll into yaw as pitch passes 90 degrees -- and both
      exist only to survive the hover-to-forward-flight transition. This vehicle
      never transitions.
     */
    attitude_control = NEW_NOTHROW AC_AttitudeControl_Multi(*ahrs_view, *motors);
    if (attitude_control == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControl");
    }
    attitude_control_var_info = AC_AttitudeControl_Multi::var_info;
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control_var_info);
}

MAV_TYPE ArduRocket::get_frame_mav_type()
{
    // there is no MAV_TYPE_ROCKET upstream
    return MAV_TYPE_GENERIC;
}

const char* ArduRocket::get_frame_string()
{
    return "ROCKET";
}

void ArduRocket::failsafe_check()
{
    // Deliberately empty. This vehicle has no failsafe reactions: there is
    // nowhere to divert to, no throttle to cut, and a mode change mid-boost is a
    // hazard rather than a mitigation. Burnout shutdown is handled by the flight
    // stage machine in rocket_control.cpp, from measured acceleration.
}
