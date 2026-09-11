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

    // Now that motors (MOT_*) and attitude_control (ATC_*) exist, bake their firmware defaults.
    // Must be after allocate_motors() (the params don't exist before it) and is placed after the
    // reload below so nothing re-clobbers them. See apply_late_defaults() in Parameters.cpp.
    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);
    AP_Param::invalidate_count();
    apply_late_defaults();

    // If the operator entered the measured airframe (RKT_MASS>0), compute the ascent gains from
    // the physics now, overriding RKT_TILT_*/SPIN_DAMP. No-op otherwise (direct gains used). §9e.
    compute_airframe_gains();

    // setup the 'main loop is dead' check
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();
    // Force GPS OFF as the vehicle default. This MUST be after gps.init(): AP_GPS::init()
    // re-defaults the type to HAL_GPS1_TYPE_DEFAULT (1 = auto), which would otherwise clobber
    // it. GPS out of the flight solution is ArduRocket architecture, not tuning -- a stray GPS
    // correction glitches the DCM tilt off-axis (AHRS_GPS_USE=0 does NOT gate it; GPS1_TYPE 0
    // does). Set as a DEFAULT, so an operator can still explicitly enable a GPS for
    // tracking/recovery by setting GPS1_TYPE in storage.
    AP_Param::set_default_by_name("GPS1_TYPE", 0);

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
  PLAN §9e -- the real-rocket gain path. If the operator has entered the measured airframe
  (RKT_MASS > 0), compute the ascent gains from the physics and OVERRIDE RKT_TILT_P/D/I and
  RKT_SPIN_DAMP. No OpenRocket, no sim: every input is measured with a scale, calipers, tape and a
  balance point. If RKT_MASS <= 0 (default) this is a no-op and the direct RKT_TILT_* gains stand.

    J_tilt   = m*L^2/12        (slender-rod estimate -- no swing test)
    J_spin   = 0.5*m*(D/2)^2   (cylinder estimate)
    force_gain = fin planform + tab   (same formula as SIM_Rocket / ork_to_rocket.py derive_fin)
    fin_arm  = NOSE_FIN - NOSE_CG     (front-of-root convention)
    gain     = C * J / (force_gain * fin_arm)

  The C constants are calibrated so the reference airframe (run through THESE formulas) reproduces
  its validated 2.5/0.5/2.0/0.006. C_P/C_D/C_I match ork_to_rocket.py (the reference length makes
  m*L^2/12 = the reference J_tilt); C_S is re-anchored because the 0.5*m*r^2 spin estimate differs
  from the model's spin inertia. See PLAN §9e for the full rationale and honest limits.
 */
void ArduRocket::compute_airframe_gains(void)
{
    const float mass = g2.af_mass;
    if (!is_positive(mass)) {
        return;   // disabled -- use the direct RKT_TILT_*/SPIN_DAMP gains as entered
    }

    const float L       = g2.af_length;
    const float rb      = g2.af_body_d * 0.5f;
    const float Cr      = g2.af_fin_root;
    const float Ct      = g2.af_fin_tip;
    const float sspan   = g2.af_fin_span;
    const float fin_arm = g2.af_nose_fin - g2.af_nose_cg;

    // Require a complete, sane set; a half-filled airframe must not silently produce junk gains.
    // Length is only needed for the rod tilt-inertia estimate -- not if RKT_JTILT was given.
    const bool need_L = !is_positive(g2.af_jtilt);
    if (!is_positive(rb) || !is_positive(Cr) || !is_positive(sspan) || !is_positive(fin_arm) ||
        (need_L && !is_positive(L))) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Rocket: RKT_MASS set but airframe incomplete; using direct gains");
        return;
    }

    // Gain-derivation constants (PLAN §9e), anchored to the reference airframe's REAL inertia --
    // same values as ork_to_rocket.py. Real inertia in => exact; the rod estimate carries its own
    // (~10-25%) error, which is why RKT_JTILT/JSPIN are worth setting when you know them.
    const float C_P = 1.882456e-03f;
    const float C_D = 3.764911e-04f;
    const float C_I = 1.505964e-03f;
    const float C_S = 1.077778e-03f;

    // Real inertia (RKT_JTILT/JSPIN) used directly if given; else a slender-body estimate.
    const float J_tilt = is_positive(g2.af_jtilt) ? g2.af_jtilt : (mass * sq(L) / 12.0f);
    const float J_spin = is_positive(g2.af_jspin) ? g2.af_jspin : (0.5f * mass * sq(rb));

    // fin aerodynamic force per unit q at full deflection -- mirrors derive_fin() /
    // SIM_Rocket::recompute_fin_geometry(); keep identical to those so the anchor stays valid.
    const float S_fin = 0.5f * (Cr + Ct) * sspan;
    const float AR    = 2.0f * sq(sspan) / S_fin;
    const float CLa   = 2.0f * M_PI * AR / (2.0f + sqrtf(sq(AR) + 4.0f));
    const float Kfb   = 1.0f + rb / (sspan + rb);
    const float mean_chord     = 0.5f * (Cr + Ct);
    const float tab_chord_frac = constrain_float(g2.af_tab_chord / mean_chord, 0.0f, 1.0f);
    const float tab_span_frac  = constrain_float(g2.af_tab_span / sspan, 0.0f, 1.0f);
    const float theta = acosf(constrain_float(2.0f * tab_chord_frac - 1.0f, -1.0f, 1.0f));
    const float tau   = (1.0f - (theta - sinf(theta)) / M_PI) * 0.85f * tab_span_frac;
    const float force_gain = S_fin * CLa * Kfb * tau * radians(g2.af_tab_max);

    const float denom = force_gain * fin_arm;
    if (!is_positive(denom)) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Rocket: airframe gain denom <= 0; using direct gains");
        return;
    }

    // Flight-speed correction. Fixed-gain closed-loop frequency is omega_n^2 = q*C (the airframe
    // terms cancel), so a rocket that flies at LOW dynamic pressure (small/slow) is chronically
    // under-gained -- confirmed on test1 (peaks ~100 m/s vs the reference ~390, ~15x less q, needed
    // ~10x hotter gains). To hold omega_n at the airframe's operating q, scale by (V_REF/vmax)^2
    // (q ~ v^2). RKT_VMAX from the OpenRocket flight sim; 0 = no correction (reference-like flight).
    const float V_REF = 390.0f;   // reference airframe max airspeed (m/s); C is anchored there
    float speed_factor = 1.0f;
    if (is_positive(g2.af_vmax)) {
        speed_factor = constrain_float(sq(V_REF / g2.af_vmax), 0.25f, 25.0f);
    }

    // Compute and clamp to each param's declared range, then override in RAM (not saved).
    const float tilt_p = constrain_float(speed_factor * C_P * J_tilt / denom, 0.0f, 20.0f);
    const float tilt_d = constrain_float(speed_factor * C_D * J_tilt / denom, 0.0f, 2.0f);
    const float tilt_i = constrain_float(speed_factor * C_I * J_tilt / denom, 0.0f, 5.0f);
    const float sdamp  = constrain_float(speed_factor * C_S * J_spin / denom, 0.0f, 0.5f);
    g2.tilt_p.set(tilt_p);
    g2.tilt_d.set(tilt_d);
    g2.tilt_i.set(tilt_i);
    g2.spin_damp.set(sdamp);

    // Log the result so it can be read on the pad BEFORE committing to a flight.
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: airframe gains P%.2f D%.2f I%.2f S%.4f",
                    (double)tilt_p, (double)tilt_d, (double)tilt_i, (double)sdamp);
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: Jt%.2f Js%.4f fg%.5f arm%.3f spd_x%.1f",
                    (double)J_tilt, (double)J_spin, (double)force_gain, (double)fin_arm,
                    (double)speed_factor);
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
