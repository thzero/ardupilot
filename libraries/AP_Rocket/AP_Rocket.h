#pragma once

#include "AP_Rocket_config.h"

#if AP_ROCKET_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/*
  AP_Rocket - vehicle-agnostic support for autonomous vertical-hold rocket
  flight. Holds the flight-stage detector (launch, burnout, apogee) and the
  integrator-gating policy, plus the RKT_ parameters. Driven by ArduRocket.

  Deliberately vehicle-free: the public API speaks only in plain inputs (a
  body-frame acceleration vector and a climb rate) and plain decisions (which
  flight stage we are in, whether to hold the integrators, whether to keep
  steering). All vehicle coupling lives in ArduRocket/rocket_control.cpp so this
  library stays portable.
*/
class AP_Rocket {
public:
    AP_Rocket();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Rocket);

    static const struct AP_Param::GroupInfo var_info[];

    /*
      Flight stage, inferred from measured acceleration and climb rate alone.

      PRE_LAUNCH -> sitting on the rail. Hold the rate integrators so they do not
                    wind up while the fins have no airflow and cannot move the
                    vehicle off the rail.
      BOOST      -> motor burning. Full authority.
      COAST      -> burnt out, still ascending. Burnout is RECORDED ONLY: it drives
                    no control change. Aerodynamic fins still have airflow while the
                    rocket is moving up, so the caller keeps steering through COAST.
                    (A jet-vane / CV airframe is the exception -- a vane in the
                    exhaust loses authority at burnout -- but that policy belongs in
                    the vehicle, not here.)
      DESCENT    -> apogee reached (climb rate negative). The caller MUST stop
                    driving the fins: there is no longer any upward airflow to
                    steer with, and nothing should flail on the way down.
     */
    enum class Stage : uint8_t {
        PRE_LAUNCH = 0,
        BOOST      = 1,
        COAST      = 2,
        DESCENT    = 3,
    };

    // true when the feature is enabled by parameter. When disabled the stage
    // machine parks in BOOST: no gating, no burnout, no apogee, fins always live.
    // That is the bench-test configuration.
    bool enabled() const { return _enable != 0; }

    // Feed the latest body-frame specific-force acceleration (m/s/s), as read from
    // the IMU (gravity reaction included), and the current climb rate (m/s, positive
    // up). Advances the stage machine. Safe to call every loop; transitions are
    // one-way until reset().
    void update(const Vector3f &accel_body, float climb_rate_ms);

    // current flight stage
    Stage stage() const { return _stage; }

    // true once launch acceleration has been detected (BOOST, COAST or DESCENT)
    bool launched() const { return _stage != Stage::PRE_LAUNCH; }

    // true while the caller should keep steering: launched and still going up.
    // Burnout does NOT end this -- only apogee does.
    bool steering_active() const { return _stage == Stage::BOOST || _stage == Stage::COAST; }

    // true while the caller should hold (zero) the rate-controller integrators:
    // enabled, pre-launch, sitting on the rail. P/D terms are unaffected.
    bool hold_integrators() const { return enabled() && _stage == Stage::PRE_LAUNCH; }

    // true once the motor has burnt out (COAST or DESCENT). Informational: it drives
    // no control decision for a finned rocket, it is here so the vehicle can log it.
    bool burnt_out() const { return _stage == Stage::COAST || _stage == Stage::DESCENT; }

    // true once apogee has been passed and steering must cease
    bool descending() const { return _stage == Stage::DESCENT; }

    // return to PRE_LAUNCH; call on arm
    void reset();

    // most recent body-up acceleration, in g (for logging and bench tuning)
    float up_accel_g() const { return _up_accel_g; }

private:
    AP_Int8  _enable;
    AP_Float _launch_g;     // launch-detect threshold, in g
    AP_Int8  _launch_axis;  // body up-axis: 0=X, 1=Y, 2=Z
    AP_Int16 _launch_ms;    // launch condition must hold this long
    AP_Float _burnout_g;    // burnout-detect threshold, in g
    AP_Int16 _burnout_ms;   // burnout condition must hold this long
    AP_Int16 _apogee_ms;    // apogee (descent) condition must hold this long

    Stage _stage;

    // Two independent debounce timers. Burnout (accel) and apogee (climb rate) can
    // both be pending at once during BOOST, so they cannot share one timer.
    // millis() at which the candidate transition first became true; 0 when not met.
    uint32_t _accel_start_ms;   // launch while PRE_LAUNCH, burnout while BOOST
    uint32_t _apogee_start_ms;  // apogee while BOOST or COAST

    float _up_accel_g;

    // returns true once `condition` has held continuously for `hold_ms`, using the
    // caller's own timer variable
    static bool debounce(uint32_t &start_ms, bool condition, uint16_t hold_ms);
};

#endif  // AP_ROCKET_ENABLED
