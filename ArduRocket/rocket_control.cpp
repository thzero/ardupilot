#include "ArduRocket.h"

/*
  Local (console) debug narration -- SITL ONLY. In SITL the firmware's hal.console is
  the MAVLink TCP port, not the terminal, so plain ::printf (stdout) is what lands in
  the same place the simulator prints ignition / burnout / apogee. Compiles to nothing
  on real hardware, where the ground-station STATUSTEXT messages are the feedback.
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #include <cstdio>
 #define RKT_LOCAL_DEBUG(fmt, ...) ::printf("Rocket: " fmt "\n", ##__VA_ARGS__)
#else
 #define RKT_LOCAL_DEBUG(fmt, ...) do {} while (0)
#endif

/*
  The ArduRocket flight controller.

  This one file replaces the mode*.cpp family of every other vehicle, because
  there is only one thing to do: hold vertical. What varies is the flight stage,
  and the stage is inferred from measured acceleration rather than commanded by
  anyone. See defines.h for the stage definitions.
 */

const char *ArduRocket::stage_string() const
{
    switch (stage) {
    case FlightStage::PREP:     return "PREP";
    case FlightStage::FINCHECK: return "FINCHECK";
    case FlightStage::ARMED:    return "ARMED";
    case FlightStage::BOOST:    return "BOOST";
    case FlightStage::COAST:    return "COAST";
    case FlightStage::DESCENT:  return "DESCENT";
    case FlightStage::LANDED:   return "LANDED";
    }
    return "?";
}

/*
  Pre-arm fin check sequence.

  Each fin in turn: full one way, full the other, then center -- announced as it
  starts, so the pad crew can confirm that the fin which moves is the fin that was
  named (catching a swapped output channel) and that it moves the expected way
  (catching a reversed servo or a backwards linkage). Neither of those is
  detectable in software; this exists to put the fins in front of a human.

  The vehicle is NOT armed while this runs.
 */
// Each fin: full one way, full the other, then center -- every position held long
// enough for the operator to watch the fin actually reach it. 1.5 s each: the throw
// announcement takes a few tenths of a second to reach and render in the ground
// station, so a shorter hold means the fin has already moved on by the time the
// operator reads it. This leaves ~1 s of visible hold after the message lands. Center
// gets the same hold so it is clearly seen before moving to the next fin.
#define FIN_CHECK_PHASE_MS   1500u  // hold each throw this long
#define FIN_CHECK_CENTER_MS  1500u  // hold at center this long, before the next fin
#define FIN_CHECK_PER_FIN_MS (2 * FIN_CHECK_PHASE_MS + FIN_CHECK_CENTER_MS)
#define FIN_CHECK_TOTAL_MS   (AP_FIN_MIXER_ROCKET_NUM_FINS * FIN_CHECK_PER_FIN_MS)

// Touchdown detection (DESCENT -> LANDED). Keyed on vertical speed settling to ~zero
// and STAYING there, NOT on altitude: the rocket drifts under its chute and can land on
// a hill or in a ditch, so its resting baro altitude need not match the pad. Vertical
// speed at rest is zero wherever it comes down. The long hold is well past the brief
// zero-crossing at apogee, so only a real stop qualifies. Announce only; the vehicle
// stays armed until a manual disarm.
#define RKT_LAND_RATE_MS  0.5f      // vertical speed treated as "at rest", m/s
#define RKT_LAND_MS       3000u     // must hold this long

// Tilt give-up backstop debounce and gyro corroboration.
//
// A real departure is a large tilt that is ALSO a real rotation of the airframe, held for
// a while. Without GPS, the EKF attitude estimate can briefly glitch to a huge tilt during
// the high-thrust boost or the low-speed apogee transition -- a spike in the ESTIMATE with
// no matching motion on the raw gyro. So the backstop requires BOTH a sustained over-tilt
// (RKT_GIVEUP_MS) AND that the raw gyro shows the airframe is genuinely rotating
// (RKT_GIVEUP_RATE_DPS). A frozen estimate glitch fails the gyro test; a brief one fails the
// long debounce. Neither can centre the fins mid-ascent on a phantom tilt.
#define RKT_GIVEUP_MS       1000u    // over-tilt must hold this long (was 200)
#define RKT_GIVEUP_RATE_DPS 25.0f    // ...AND the airframe must actually be rotating this fast

// Direct spin-rate damper gain: yaw fin command = -RKT_SPIN_DAMP * spin_rate(rad/s), clamped
// to +/-1. 0.10 commands full spin fin at ~5.7 rad/s (~570 deg/s), so it opposes the spin
// hard well before it can run away. Applied only while steering (BOOST/COAST).
#define RKT_SPIN_DAMP       0.10f

// Direct tilt controller gains (fin = -RKT_TILT_P*angle - RKT_TILT_D*rate, angle/rate in
// radians). RKT_TILT_P 4.0 -> full fin already at ~14 deg of lean, so the fins slam hard on
// the whole 20 deg lean instead of creeping; RKT_TILT_D 0.3 damps the swing to stop overshoot.
#define RKT_TILT_P          4.0f
#define RKT_TILT_D          0.3f

void ArduRocket::start_fin_check(bool on_rail)
{
    fin_check_start_ms = AP_HAL::millis();
    fin_check_on_rail = on_rail;
    set_stage(FlightStage::FINCHECK);
    gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: FIN CHECK%s - watch the fins",
                    on_rail ? "" : " (bench, will NOT arm)");
    RKT_LOCAL_DEBUG("fin test: starting%s", on_rail ? " on the rail" : " (bench)");
}

/*
  Trigger the fin check from a ground station. Bound to MAV_CMD_DO_AUX_FUNCTION so a
  GCS button labelled "Fin Check" can fire it (see GCS_MAVLink_Rocket.cpp) -- the same
  wiggle whether it will count toward arming or is just a bench exercise.

  The SAME command serves both: whether it counts is decided by where the airframe
  is. On the rail (vertical and still) the completed run latches fin_check_valid, so
  the operator can then arm with a single press. Held in the hand on the bench, it
  still drives the fins so you can verify them, but does NOT latch the gate -- a
  workshop wiggle must never let someone arm on the rail without re-checking.
 */
bool ArduRocket::trigger_fin_check()
{
    if (motors == nullptr) {
        return false;
    }
    if (motors->armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: fin check refused - vehicle is armed");
        return false;
    }
    if (fin_check_start_ms != 0) {
        return false;   // one already running
    }

    // On the rail = near-vertical and still. Same thresholds the arming gate uses.
    const bool on_rail = (tilt_from_vertical_deg() <= 20.0f) &&
                         (ahrs.get_gyro().length() <= radians(15.0f));
    start_fin_check(on_rail);
    return true;
}

/*
  Invalidate a latched fin check if the airframe is disturbed or too much time has
  passed, so a stale confirmation cannot be used to arm. Called every control loop
  while disarmed.
 */
void ArduRocket::update_fin_check_validity()
{
    if (!fin_check_valid) {
        return;
    }
    // ~5 minutes, and a gyro spike (being carried / bumped on the rail)
    const bool expired = (AP_HAL::millis() - fin_check_valid_ms) > 300000u;
    const bool disturbed = ahrs.get_gyro().length() > radians(30.0f);
    if (expired || disturbed) {
        fin_check_valid = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: fin check cleared (%s) - re-run",
                        expired ? "timeout" : "moved");
    }
}

bool ArduRocket::fin_check_ok() const
{
    return fin_check_valid;
}

void ArduRocket::run_fin_check()
{
    if (motors == nullptr || fin_check_start_ms == 0) {
        return;
    }

    const uint32_t elapsed = AP_HAL::millis() - fin_check_start_ms;

    if (elapsed >= FIN_CHECK_TOTAL_MS) {
        // sequence complete: center the fins and return to PREP
        motors->set_fin_test(-1, 0.0f);
        fin_check_start_ms = 0;
        set_stage(FlightStage::PREP);
        RKT_LOCAL_DEBUG("fin test: complete (%s)",
                        fin_check_on_rail ? "on rail - gate latched" : "bench");

        if (fin_check_on_rail) {
            // On the rail: latch the gate so ARM becomes a single clean press.
            //
            // Deliberately NOT announced as "OK": the software cannot see the fins,
            // only that the motion command finished. The verdict is the operator's.
            // ARM is how they record it -- so this message hands the decision back
            // rather than claiming a pass the code has no way to reach.
            fin_check_valid = true;
            fin_check_valid_ms = AP_HAL::millis();
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Rocket: fin check done - ARM for flight");
        } else {
            // Bench run: verified the fins but does NOT satisfy the arming gate.
            gcs().send_text(MAV_SEVERITY_INFO,
                            "Rocket: fin bench test complete (does not arm)");
        }
        return;
    }

    const uint8_t fin = elapsed / FIN_CHECK_PER_FIN_MS;
    const uint32_t in_fin = elapsed % FIN_CHECK_PER_FIN_MS;

    // Three phases per fin: full one way, full the other, then center. Phase tracked as
    // an int (the build forbids == on floats).
    uint8_t phase;
    float deflection;
    if (in_fin < FIN_CHECK_PHASE_MS) {
        phase = 0; deflection = 1.0f;
    } else if (in_fin < 2 * FIN_CHECK_PHASE_MS) {
        phase = 1; deflection = -1.0f;
    } else {
        phase = 2; deflection = 0.0f;
    }

    // GROUND STATION: announce ONCE PER FIN. The GCS throttles its on-screen
    // notifications (a queued toaster with a minimum display time), so announcing every
    // throw -- 12 messages -- makes the display fall seconds behind the actual movement.
    // The firmware sends them in real time; it is the GCS display that lags. One message
    // per fin keeps the GCS in step with the fin the operator is watching, which visibly
    // does all three throws. WARNING severity so it surfaces (INFO gets buried).
    static uint8_t announced_fin = 0xFF;
    if (fin != announced_fin) {
        announced_fin = fin;
        gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: testing fin %u",
                        (unsigned)(fin + 1));
    }

    // LOCAL CONSOLE: the throw-by-throw detail, in real time and unthrottled, for SITL
    // where there is no physical fin to watch. The command is +/-full throw; the physical
    // angle is whatever the servo endpoints (SERVOn_MIN/MAX) are -- watch SERVO_OUTPUT_RAW
    // for the actual PWM.
    static uint8_t dbg_phase = 0xFF;
    static uint8_t dbg_fin = 0xFF;
    if (phase != dbg_phase || fin != dbg_fin) {
        dbg_phase = phase;
        dbg_fin = fin;
        RKT_LOCAL_DEBUG("fin test: fin %u -> %s", (unsigned)(fin + 1),
                        phase == 0 ? "+100%" : phase == 1 ? "-100%" : "center");
    }

    motors->set_fin_test(fin, deflection);
}

float ArduRocket::tilt_from_vertical_deg() const
{
    if (ahrs_view == nullptr) {
        return 0.0f;
    }
    // The view is rotated so that a vertical airframe reads as level, which puts
    // tilt-from-vertical at the view's tilt-from-level and keeps it far away from
    // the Euler singularity that makes the raw body angles useless here.
    const float c = cosf(ahrs_view->roll) * cosf(ahrs_view->pitch);
    return degrees(acosf(constrain_float(c, -1.0f, 1.0f)));
}

/*
  Low-rate telemetry that exists purely so a human at the pad can see what matters.

  A nose-up airframe sits exactly on the Euler yaw singularity, so the heading and
  roll a ground station displays swing through ~166 degrees while the rocket stands
  perfectly still -- measured at 0.086 deg of actual tilt. That is not fixable and
  not a fault: at near-zero tilt the direction the nose leans is genuinely undefined.

  The number that IS well conditioned, and the one that decides whether the vehicle
  will arm, is tilt from vertical. NAMED_VALUE_FLOAT carries it under the name TILT
  so any ground station can display or chart it without vehicle-specific support.
 */
void ArduRocket::send_rocket_telemetry()
{
    gcs().send_named_float("TILT", tilt_from_vertical_deg());
}

void ArduRocket::set_stage(FlightStage new_stage)
{
    if (new_stage == stage) {
        return;
    }
    stage = new_stage;
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: %s", stage_string());

    /*
      Plain-language callouts for the key flight events, so the ground station narrates
      the flight the same way the SITL console does -- liftoff / burnout / apogee /
      landed -- instead of only the terse stage name (e.g. "COAST" for burnout, which is
      easy to miss). Firmware-side, so these reach QGC on real hardware, not only SITL.
     */
    switch (new_stage) {
    case FlightStage::BOOST:
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rocket: liftoff");
        break;
    case FlightStage::COAST:
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rocket: burnout");
        break;
    case FlightStage::DESCENT:
        // this edge IS apogee (climb rate went negative). Baro altitude is launch-
        // referenced (height datum zeroed at arm), so it is height above the pad.
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rocket: apogee at %.0f m",
                        (double)barometer.get_altitude());
        break;
    case FlightStage::LANDED:
        // Stays armed by design -- remind the operator that disarm is now their job.
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rocket: landed - disarm when recovered");
        break;
    case FlightStage::PREP:
    case FlightStage::FINCHECK:
    case FlightStage::ARMED:
        break;   // the stage name above is enough for these
    }
}

/*
  Estimate dynamic pressure q = 1/2 rho v^2, used to schedule the fin gains.

  Fin force scales with q, so the mixer needs it to keep the authority seen by the
  rate controllers roughly constant across the burn. The vehicle is (by
  construction) going almost straight up, so vertical speed is speed; that lets
  the estimate come from the EKF's baro + IMU solution with no GPS and no pitot.
  If the EKF has no velocity solution we report zero, which the mixer reads as
  "no airflow" and clamps to its maximum gain boost.
 */
void ArduRocket::update_dynamic_pressure()
{
    // No GPS: the speed estimate is the vertical speed from the EKF, fed by baro +
    // IMU. For a near-vertical rocket that is essentially the airspeed, which is all
    // the fin gain scheduling needs.
    //
    // The `true` is the high_vibes flag. It does NOT mean "avoid GPS" -- it selects
    // get_vert_pos_rate_D(), the vertical rate that is kinematically CONSISTENT with
    // the EKF's vertical position, instead of the EKF's velocity state, which can
    // diverge from position while the filter corrects errors. A rocket is guaranteed
    // high-vibration, which is exactly the case that flag exists for.
    float velD;
    if (!ahrs.get_velocity_D(velD, true)) {
        dynamic_pressure_pa = 0.0f;
        return;
    }

    const float speed = fabsf(velD);

    // Air density falls with altitude, which matters because fin force tracks it
    // directly. It is refreshed at 10 Hz in update_altitude() and cached, so this
    // 400 Hz path does not repeat the powf inside get_air_density_for_alt_amsl.
    dynamic_pressure_pa = 0.5f * air_density_kgm3 * sq(speed);
}

void ArduRocket::run_rocket_control()
{
    if (!initialised || motors == nullptr || attitude_control == nullptr) {
        return;
    }

    AP_Rocket &rkt = g2.rocket;

    // Vertical velocity, positive up. This is an EKF3 state fed by baro + IMU, NOT a
    // numerical derivative of the barometer -- that distinction is what makes apogee
    // detection viable, since differentiating a barometer would amplify its noise.
    // get_velocity_D returns velocity DOWN, hence the negation; see
    // update_dynamic_pressure() for what the high_vibes flag actually selects.
    //
    // If there is no estimate at all, report a climbing value so a missing reading
    // can never fake an apogee.
    float climb_rate_ms = 1.0f;
    float velD;
    if (ahrs.get_velocity_D(velD, true)) {
        climb_rate_ms = -velD;
    }

    // Feed the stage detector the body-frame specific force and the climb rate.
    // This is the only input that advances the stage machine: no timer, no command,
    // no operator.
    rkt.update(ins.get_accel(), climb_rate_ms);

    update_dynamic_pressure();
    motors->set_dynamic_pressure(dynamic_pressure_pa);

    // Map the detector's stage onto the vehicle's, which additionally knows about
    // being disarmed (PREP covers both the table and the rail).
    if (!motors->armed()) {
        if (fin_check_start_ms != 0) {
            // fin check wiggle in progress: drive the sequence, leave the
            // controller alone. Still disarmed.
            set_stage(FlightStage::FINCHECK);
            run_fin_check();
            return;
        }
        // Expire/invalidate a latched fin check if the airframe is disturbed, so a
        // stale confirmation cannot be used to arm.
        update_fin_check_validity();
        set_stage(FlightStage::PREP);
    } else if (stage != FlightStage::LANDED && stage != FlightStage::DESCENT) {
        // DESCENT and LANDED are terminal (until touchdown / a manual disarm). The ascent
        // detector only knows stages up to DESCENT and would otherwise map us back to
        // BOOST/COAST -- which would undo a give-up-triggered DESCENT while the vehicle is
        // still nominally ascending (see the tilt backstop below).
        switch (rkt.stage()) {
        case AP_Rocket::Stage::PRE_LAUNCH:
            set_stage(FlightStage::ARMED);
            break;
        case AP_Rocket::Stage::BOOST:
            set_stage(FlightStage::BOOST);
            break;
        case AP_Rocket::Stage::COAST:
            set_stage(FlightStage::COAST);
            break;
        case AP_Rocket::Stage::DESCENT:
            set_stage(FlightStage::DESCENT);
            break;
        }
    }

    /*
      Tilt give-up backstop. While steering (BOOST/COAST), if the airframe has departed
      more than RKT_GIVEUP_DEG from vertical, held for a short debounce, stop steering and
      centre the fins by moving to DESCENT -- past that angle a fin-steered rocket has lost
      authority and the fins would only flail. This complements the climb-rate apogee,
      which can lag if the vehicle tumbles while still ascending. 0 disables. The mapping
      guard above keeps this DESCENT from being reverted while climb rate is still positive.
     */
    if ((stage == FlightStage::BOOST || stage == FlightStage::COAST) &&
        is_positive(g2.giveup_deg)) {
        // Corroborate the tilt ESTIMATE with the raw gyro: a genuine departure past the
        // give-up angle is a real rotation; an EKF attitude glitch is not. Requiring both
        // (plus the long debounce) stops a single estimate spike from centering the fins.
        const bool departed = tilt_from_vertical_deg() > g2.giveup_deg;
        const bool rotating = degrees(ahrs.get_gyro().length()) > RKT_GIVEUP_RATE_DPS;
        if (departed && rotating) {
            if (giveup_start_ms == 0) {
                giveup_start_ms = AP_HAL::millis();
            } else if (AP_HAL::millis() - giveup_start_ms > RKT_GIVEUP_MS) {
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "Rocket: tilt %.0f deg - giving up, fins centered",
                                (double)tilt_from_vertical_deg());
                set_stage(FlightStage::DESCENT);
            }
        } else {
            giveup_start_ms = 0;
        }
    }

    /*
      Coast the attitude estimate on the gyro once launched. Under motor thrust (tens of g
      along the nose) and under fin steering, the accelerometer measures those forces, not
      gravity, so DCM must not use it as a "down" reference -- it would read the thrust/steer
      acceleration as a tilt and corrupt the estimate (measured: a 20 deg lean read as ~6 deg
      while the fins steered). On the pad (PREP/FINCHECK/ARMED) the accelerometer is left ON so
      DCM aligns to true vertical from gravity; from BOOST through DESCENT it is gated OFF and
      the gyro carries the attitude, which is accurate over the short flight. This is the
      technique the flown MatrixPilot fin-steered rocket uses (rmat.c: accel correction only
      while `launched == 0`).
     */
    const bool in_powered_flight = (stage == FlightStage::BOOST ||
                                    stage == FlightStage::COAST ||
                                    stage == FlightStage::DESCENT);
    ahrs.set_attitude_gyro_only(in_powered_flight);

    switch (stage) {

    case FlightStage::PREP:
    case FlightStage::FINCHECK:
        // FINCHECK is handled above and returns early; it is listed here only so
        // the compiler can prove the switch is exhaustive.
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        break;

    case FlightStage::ARMED:
    case FlightStage::BOOST:
    case FlightStage::COAST: {
        /*
          Steer from arming toward apogee. Burnout (the BOOST->COAST edge)
          deliberately changes nothing: aerodynamic fins still have upward airflow
          while coasting, so they keep holding vertical.

          But stop early if there is not enough dynamic pressure left to produce a
          useful moment. Approaching apogee q collapses, and the gain scheduling
          (Q_REF/q, capped at GAIN_MAX) would otherwise drive the fins to near-full
          deflection against a plant that cannot respond -- achieving nothing, and
          winding up the integrators in the last moments before shutdown. On the
          rail q is also ~0, but ARMED is excluded because the fins must already be
          tracking attitude when the rail releases.
         */
        const bool no_authority = (stage != FlightStage::ARMED) &&
                                  is_positive(g2.min_q) &&
                                  (dynamic_pressure_pa < g2.min_q);
        if (no_authority) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            attitude_control->reset_rate_controller_I_terms();
            break;
        }

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        /*
          THE TARGET IS TRUE VERTICAL. The rail attitude measured at arming is a
          calibration, not the goal: it says how far off vertical the airframe
          started, so the vehicle can get to vertical without applying that whole
          angle as a step input at the worst possible moment.

          A launch rail is routinely tilted a few degrees (into wind, or away from
          the crowd; NAR and Tripoli cap it at 20). Commanding vertical the instant
          the rocket lights would demand the full correction while it is still on
          the rail and again at rail exit, where dynamic pressure -- and therefore
          fin authority -- is at its lowest. So the target starts at the measured
          rail attitude and blends to vertical as q rises, i.e. as the fins actually
          gain the authority to fly it there. By RKT_LEVEL_Q the command is fully
          vertical and stays there for the rest of the flight.

          Spin (view yaw = roll about the long axis) MUST be actively controlled. Left
          alone it runs away to ~1200 deg/s -- the spin inertia is ~240x smaller than the
          tilt inertia, so a tiny roll torque spins it up fast -- and a fast spin combined
          with a tilt is CONING, which systematically corrupts the DCM tilt estimate and
          makes it impossible to fly vertical. An earlier version reset the yaw target to the
          current heading every loop "so the fins would not fight a yaw error"; that snapped
          the target onto the spinning measurement, forced the yaw error to zero, and so the
          controller never opposed the spin at all. That was the bug. Instead we HOLD a yaw
          heading (captured once on the rail, in ARMED) and let the controller fight any spin
          back to it -- the "fighting" is the spin damping we want.
         */
        float to_vertical = 1.0f;   // 0 = hold rail attitude, 1 = true vertical
        if (is_positive(g2.level_q)) {
            to_vertical = constrain_float(dynamic_pressure_pa / g2.level_q, 0.0f, 1.0f);
        }
        const float tgt_roll_rad  = rail_roll_rad  * (1.0f - to_vertical);
        const float tgt_pitch_rad = rail_pitch_rad * (1.0f - to_vertical);

        // Only track the heading on the rail (ARMED), where there is no spin to fight; in
        // BOOST/COAST hold it, so the controller damps the spin instead of following it.
        if (stage == FlightStage::ARMED) {
            attitude_control->reset_yaw_target_and_rate(false);
        }
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(
            degrees(tgt_roll_rad) * 100.0f,
            degrees(tgt_pitch_rad) * 100.0f,
            0.0f);

        /*
          On the rail (ARMED) the fins have no airflow and cannot move the vehicle,
          so any integrator would wind up against a constraint it cannot beat and
          then dump that windup into the fins at the worst possible moment: the
          instant the rocket comes off the rail. P and D stay live throughout.
         */
        if (rkt.hold_integrators()) {
            attitude_control->reset_rate_controller_I_terms();
        }
        break;
    }

    case FlightStage::DESCENT: {
        /*
          Apogee is behind us: the rocket is falling, there is no upward airflow to
          steer with, and nothing should flail on the way down. Shut the controller
          down and center the fins. Recovery (pyro, chute) is deliberately not this
          controller's job -- it runs on a dedicated altimeter.

          The vehicle stays ARMED. Disarm is a manual operator action after recovery,
          not automatic, so the flight computer keeps logging and reporting the whole
          way down. We watch for touchdown only to ANNOUNCE it (the LANDED stage);
          nothing here disarms.
         */
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->reset_rate_controller_I_terms();

        // Touchdown: no longer moving vertically, held for a debounce so a momentary
        // reading cannot fake it. Deliberately NOT gated on altitude -- the rocket
        // drifts under its chute and may land on a hill or in a ditch, where the
        // resting baro altitude differs from the pad; the vertical SPEED is zero at
        // rest regardless of terrain. climb_rate_ms is the same EKF estimate apogee
        // rides on. No GPS needed.
        const bool at_rest = fabsf(climb_rate_ms) < RKT_LAND_RATE_MS;
        if (at_rest) {
            if (land_start_ms == 0) {
                land_start_ms = AP_HAL::millis();
            } else if (AP_HAL::millis() - land_start_ms > RKT_LAND_MS) {
                set_stage(FlightStage::LANDED);
            }
        } else {
            land_start_ms = 0;
        }
        break;
    }

    case FlightStage::LANDED:
        /*
          On the ground after the descent. Controller down, fins centered. Terminal:
          the vehicle waits here, still ARMED, until the operator disarms after
          recovery. Nothing auto-disarms.
         */
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->reset_rate_controller_I_terms();
        break;
    }

    // There is no commanded thrust. The attitude controller still needs a throttle
    // value to exist for its own bookkeeping; the mixer ignores it.
    attitude_control->set_throttle_out(0.0f, false, 0.0f);

    // run the rate controllers and produce the fin demands
    attitude_control->rate_controller_run();

    /*
      Direct spin-rate damper, in the style of the flown MatrixPilot rocket. While steering
      (BOOST/COAST) the airframe otherwise spins up to ~1200 deg/s -- the spin inertia is
      ~240x smaller than the tilt inertia, so a tiny roll torque runs it away -- and that fast
      spin plus a tilt is CONING, which systematically corrupts the tilt estimate and makes
      vertical flight impossible. The cascade attitude controller does NOT drive the fins
      against the spin (verified in SITL: spin unchanged by yaw-gain and yaw-mode changes), so
      we command the spin fins DIRECTLY from the gyro instead: fin ~ -k * spin_rate. This
      OVERRIDES the controller's yaw output (set after rate_controller_run). Spin about the
      long axis shows up as the view's yaw rate, which the fin mixer's yaw channel commands.
     */
    if (stage == FlightStage::BOOST || stage == FlightStage::COAST) {
        // Damp the spin about the long axis (body X = the nose). Sign matters and is subtle:
        // a POSITIVE yaw fin command produces a positive roll torque (positive body-X spin),
        // so to oppose a positive spin the command must be NEGATIVE -- hence -k * gyro.x on the
        // raw body rate. (Using the view yaw rate here inverts the sign, because the pitch-90
        // view makes view-yaw = -body-X-spin, which is the positive-feedback trap the cascade
        // controller fell into and why the spin ran away.)
        const float spin_rate = ahrs.get_gyro().x;              // rad/s, nose spin
        motors->set_yaw(constrain_float(-RKT_SPIN_DAMP * spin_rate, -1.0f, 1.0f));

        /*
          Direct proportional+damping TILT control, overriding the ATC cascade. In the
          ROTATION_PITCH_90 view a vertical rocket reads level, so view roll and view pitch ARE
          the two tilt-from-vertical axes -- drive them to zero and the nose is vertical. The
          cascade controller throttles this (its input shaping / integrator wind up so slowly
          the fins barely move on a standing lean), so we command it straight, exactly like the
          spin damper above and like the flown MatrixPilot rocket (P on the gravity vector).
          fin = -Kp*angle - Kd*rate. The q gain schedule in the mixer still scales the output.
         */
        const Vector3f vg = ahrs_view->get_gyro();              // view-frame rates, rad/s
        motors->set_roll (constrain_float(-RKT_TILT_P * ahrs_view->roll  - RKT_TILT_D * vg.x, -1.0f, 1.0f));
        motors->set_pitch(constrain_float(-RKT_TILT_P * ahrs_view->pitch - RKT_TILT_D * vg.y, -1.0f, 1.0f));
    }
}
