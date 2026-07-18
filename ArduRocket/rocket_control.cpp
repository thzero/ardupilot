#include "ArduRocket.h"

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
    }
    return "?";
}

/*
  Pre-arm fin check sequence.

  Each fin in turn: full one way, full the other, then centre -- announced as it
  starts, so the pad crew can confirm that the fin which moves is the fin that was
  named (catching a swapped output channel) and that it moves the expected way
  (catching a reversed servo or a backwards linkage). Neither of those is
  detectable in software; this exists to put the fins in front of a human.

  The vehicle is NOT armed while this runs.
 */
#define FIN_CHECK_PHASE_MS   500u   // per deflection
#define FIN_CHECK_CENTRE_MS  300u   // settle between fins
#define FIN_CHECK_PER_FIN_MS (2 * FIN_CHECK_PHASE_MS + FIN_CHECK_CENTRE_MS)
#define FIN_CHECK_TOTAL_MS   (AP_FIN_MIXER_ROCKET_NUM_FINS * FIN_CHECK_PER_FIN_MS)
#define FIN_CHECK_CONFIRM_TIMEOUT_MS 60000u   // operator has this long to confirm

void ArduRocket::start_fin_check(bool gating)
{
    fin_check_start_ms = AP_HAL::millis();
    fin_check_awaiting = false;
    fin_check_bench = !gating;
    set_stage(FlightStage::FINCHECK);
    gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: FIN %s - watch the fins",
                    gating ? "CHECK" : "BENCH TEST");
}

/*
  On-demand fin test, triggered by MAV_CMD_DO_MOTOR_TEST from a ground station so the
  fins can be exercised on the bench without going through an arming attempt.

  Runs the identical sequence to the arming check -- same code path, so what you
  verify on the bench is what runs on the rail -- but deliberately does NOT satisfy
  the arming gate. A bench run in the workshop must never let someone skip the
  on-the-rail check.
 */
bool ArduRocket::start_bench_fin_test()
{
    if (motors == nullptr) {
        return false;
    }
    if (motors->armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: fin test refused - vehicle is armed");
        return false;
    }
    start_fin_check(false);
    return true;
}

void ArduRocket::run_fin_check()
{
    if (motors == nullptr || fin_check_start_ms == 0) {
        return;
    }

    const uint32_t elapsed = AP_HAL::millis() - fin_check_start_ms;

    if (elapsed >= FIN_CHECK_TOTAL_MS) {
        // sequence complete: centre the fins
        motors->set_fin_test(-1, 0.0f);

        if (fin_check_bench) {
            // Bench test: just finish. Deliberately does NOT set fin_check_awaiting,
            // so this run cannot be used as the arming confirmation.
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket: fin bench test complete");
            fin_check_start_ms = 0;
            fin_check_bench = false;
            set_stage(FlightStage::PREP);
            return;
        }

        if (!fin_check_awaiting) {
            fin_check_awaiting = true;
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Rocket: fin check done - ARM again to confirm");
        }
        // Do not wait forever half-committed; fall back to PREP.
        if (elapsed >= FIN_CHECK_TOTAL_MS + FIN_CHECK_CONFIRM_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: fin check timed out");
            fin_check_start_ms = 0;
            fin_check_awaiting = false;
            set_stage(FlightStage::PREP);
        }
        return;
    }

    const uint8_t fin = elapsed / FIN_CHECK_PER_FIN_MS;
    const uint32_t in_fin = elapsed % FIN_CHECK_PER_FIN_MS;

    // announce each fin as it starts moving
    static uint8_t announced = 0xFF;
    if (announced != fin) {
        announced = fin;
        gcs().send_text(MAV_SEVERITY_INFO, "Rocket: fin %u", (unsigned)(fin + 1));
    }

    float deflection;
    if (in_fin < FIN_CHECK_PHASE_MS) {
        deflection = 1.0f;
    } else if (in_fin < 2 * FIN_CHECK_PHASE_MS) {
        deflection = -1.0f;
    } else {
        deflection = 0.0f;
    }

    motors->set_fin_test(fin, deflection);
}

void ArduRocket::set_stage(FlightStage new_stage)
{
    if (new_stage == stage) {
        return;
    }
    stage = new_stage;
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: %s", stage_string());
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
    // No GPS: the speed estimate is the vertical speed from baro + IMU. For a
    // near-vertical rocket that is essentially the airspeed, which is all the fin
    // gain scheduling needs. get_velocity_D(..., true) uses the high-vibration
    // path, i.e. the baro/IMU vertical rate rather than a GPS-backed velocity.
    float velD;
    if (!ahrs.get_velocity_D(velD, true)) {
        dynamic_pressure_pa = 0.0f;
        return;
    }

    const float speed = fabsf(velD);

    // Air density falls with altitude, which matters because fin force tracks it
    // directly. Over the few hundred metres a hobby rocket covers the correction
    // is small, but it is free.
    const float density = AP_Baro::get_air_density_for_alt_amsl(barometer.get_altitude_AMSL());

    dynamic_pressure_pa = 0.5f * density * sq(speed);
}

void ArduRocket::run_rocket_control()
{
    if (!initialised || motors == nullptr || attitude_control == nullptr) {
        return;
    }

    AP_Rocket &rkt = g2.rocket;

    // Vertical velocity, positive up. Comes from baro + IMU (no GPS): get_velocity_D
    // returns velocity DOWN, and the high-vibration flag makes it use the baro/IMU
    // vertical rate rather than a GPS-backed value. If there is no estimate at all,
    // report a climbing value so a missing reading can never fake an apogee.
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
            // fin check in progress or awaiting confirmation: drive the sequence
            // and leave the controller alone. Still disarmed.
            set_stage(FlightStage::FINCHECK);
            run_fin_check();
            return;
        }
        set_stage(FlightStage::PREP);
    } else {
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
        // Steer from arming all the way to apogee. Burnout (the BOOST->COAST edge)
        // deliberately changes nothing here: aerodynamic fins still have upward
        // airflow while coasting, so they keep holding vertical. Only DESCENT stops.
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

          Yaw is commanded as a RATE of zero rather than an angle. Spin about the
          long axis is not something the vehicle cares about, and holding a yaw
          angle would accumulate an error the fins would then fight. Resetting the
          yaw target each loop keeps it tracking the measurement.
         */
        float to_vertical = 1.0f;   // 0 = hold rail attitude, 1 = true vertical
        if (is_positive(g2.level_q)) {
            to_vertical = constrain_float(dynamic_pressure_pa / g2.level_q, 0.0f, 1.0f);
        }
        const float tgt_roll_rad  = rail_roll_rad  * (1.0f - to_vertical);
        const float tgt_pitch_rad = rail_pitch_rad * (1.0f - to_vertical);

        attitude_control->reset_yaw_target_and_rate(false);
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

    case FlightStage::DESCENT:
        /*
          Apogee is behind us: the rocket is falling, there is no upward airflow to
          steer with, and nothing should flail on the way down. Shut down, centre the
          fins, and disarm. Recovery (pyro, altimeter) is deliberately not this
          controller's job.
         */
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->reset_rate_controller_I_terms();
        if (motors->armed()) {
            arming.disarm(AP_Arming::Method::LANDED);
        }
        break;
    }

    // There is no commanded thrust. The attitude controller still needs a throttle
    // value to exist for its own bookkeeping; the mixer ignores it.
    attitude_control->set_throttle_out(0.0f, false, 0.0f);

    // run the rate controllers and produce the fin demands
    attitude_control->rate_controller_run();
}
