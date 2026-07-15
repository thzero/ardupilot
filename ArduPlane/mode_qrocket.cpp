#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED && AP_ROCKET_ENABLED

/*
  QROCKET - thin ArduPlane shim for the AP_Rocket vertical-hold behavior.

  This is the ONLY place that couples the rocket logic to Plane/QuadPlane
  internals. It reuses the QSTABILIZE tailsitter controller wiring and adds the
  three rocket behaviors:
    _enter(): force air-mode ON (full authority at zero throttle on the pad),
              and re-latch launch detection.
    update(): force the attitude target to vertical (zero tilt).
    run():    feed the launch detector and hold rate integrators until liftoff,
              then run the inherited QSTABILIZE controllers.
*/

bool ModeQRocket::_enter()
{
    // reuse QSTABILIZE entry (clears throttle_wait, etc.)
    if (!ModeQStabilize::_enter()) {
        return false;
    }

    // Force air-mode ON so hold_stabilize() never relaxes attitude control at
    // zero throttle while sitting on the launch rail. disarm() clears air_mode
    // when there is no AIRMODE RC aux channel, so we re-assert on every entry.
    quadplane.air_mode = AirMode::ON;

    // start each arming/mode cycle pre-launch
    plane.g2.rocket.reset();

    return true;
}

void ModeQRocket::update()
{
    // The rocket target is always vertical: zero tilt. Ignore any (absent) RC
    // stick input rather than reading it as QSTABILIZE does.
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

void ModeQRocket::run()
{
    AP_Rocket &rocket = plane.g2.rocket;

    // Feed the launch detector the body-frame specific-force acceleration.
    rocket.update(AP::ins().get_accel());

    // Before liftoff, hold (zero) the rate-controller integrators to prevent
    // wind-up against the launch rail. P/D terms stay live so the fins/gimbal
    // still track vertical on the pad.
    if (rocket.hold_integrators()) {
        quadplane.attitude_control->reset_rate_controller_I_terms();
    }

    // Reuse the exact QSTABILIZE tailsitter controller wiring. With air-mode ON
    // this keeps the control surfaces / TVC live at zero throttle.
    ModeQStabilize::run();
}

#endif  // HAL_QUADPLANE_ENABLED && AP_ROCKET_ENABLED
