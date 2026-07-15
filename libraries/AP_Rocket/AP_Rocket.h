#pragma once

#include "AP_Rocket_config.h"

#if AP_ROCKET_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/*
  AP_Rocket - vehicle-agnostic support for autonomous vertical-hold rocket
  flight. Holds launch detection and integrator-gating policy plus the RKT_
  parameters. Driven by the thin QROCKET flight mode in ArduPlane.

  Deliberately Plane-free: the public API speaks only in plain inputs (a
  body-frame acceleration vector) and plain decisions (launched / hold
  integrators). All ArduPlane/QuadPlane coupling lives in
  ArduPlane/mode_qrocket.cpp so this library stays portable.
*/
class AP_Rocket {
public:
    AP_Rocket();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Rocket);

    static const struct AP_Param::GroupInfo var_info[];

    // true when the feature is enabled by parameter
    bool enabled() const { return _enable != 0; }

    // Feed the latest body-frame specific-force acceleration (m/s/s), as read
    // from the IMU (gravity reaction included). Latches the launched state once
    // the configured body-up axis exceeds the launch threshold. Safe to call
    // every loop; a no-op once launched.
    void update(const Vector3f &accel_body);

    // true once launch acceleration has been detected (or when disabled)
    bool launched() const { return _launched; }

    // true while the caller should hold (zero) the rate-controller integrators:
    // enabled, pre-launch, sitting on the pad. P/D terms are unaffected.
    bool hold_integrators() const { return enabled() && !_launched; }

    // clear the latched launch state; call on mode entry
    void reset() { _launched = false; }

private:
    AP_Int8  _enable;
    AP_Float _launch_g;     // launch-detect threshold, in g
    AP_Int8  _launch_axis;  // body up-axis: 0=X, 1=Y, 2=Z

    bool _launched;   // latched true once launch acceleration detected

    // Note: _launched is (re)initialised to false by reset(), which the QROCKET
    // mode calls on entry before update()/run() ever run.
};

#endif  // AP_ROCKET_ENABLED
