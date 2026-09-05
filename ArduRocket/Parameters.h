#pragma once

#define AP_PARAM_VEHICLE_NAME rocket

#include <AP_Common/AP_Common.h>
#include <AP_Rocket/AP_Rocket.h>
#include <SRV_Channel/SRV_Channel.h>
#include "RC_Channel_Rocket.h"

// Global parameter class.
//
class Parameters
{
public:
    // The version of the layout as described by the parameter enum.
    //
    // When changing the parameter enum in an incompatible fashion, this
    // value should be incremented by one.
    static const uint16_t k_format_version = 1;

    // Parameter identities.
    //
    // WARNING: Care should be taken when editing this enumeration as the
    //          AP_Param load/save code depends on the values here to identify
    //          variables saved in EEPROM.
    enum {
        // Layout version number, always key zero.
        k_param_format_version = 0,
        k_param_ins,
        k_param_g2,
        k_param_NavEKF3,
        k_param_can_mgr,

        // simulation
        k_param_sitl = 10,

        // barometer object (needed for SITL)
        k_param_barometer,

        // scheduler object (for debugging)
        k_param_scheduler,

        // BoardConfig object
        k_param_BoardConfig,

        // GPS object
        k_param_gps,

        // Misc
        k_param_battery,
        k_param_notify,
        k_param_log_bitmask,
        k_param_serial_manager,
        k_param_arming,

        // Attitude estimation. These MUST be registered in Parameters.cpp or their
        // parameters silently do not exist -- AP_Vehicle does not register them for
        // you, and a setting in a .parm file that matches no registered parameter is
        // dropped without any error. AHRS_ORIENTATION and the EK3_SRC1_* settings
        // that keep GPS out of the flight solution were both dead for exactly this
        // reason.
        k_param_compass,
        k_param_ahrs,

        // Control
        k_param_motors = 30,
        k_param_attitude_control,

        // the AP_Vehicle base class parameter group
        k_param_vehicle = 40,

        k_param_last, // sentinel
    };

    AP_Int16 format_version;
    AP_Int32 log_bitmask;

    Parameters() {}
};

/*
  2nd block of parameters, to avoid going past 256 top level keys
 */
class ParametersG2
{
public:
    ParametersG2(void);

    static const struct AP_Param::GroupInfo var_info[];

    /*
      Which actuator the airframe steers with. Read once, at startup, by
      ArduRocket::allocate_motors() -- the mixer cannot be swapped at runtime, so
      this is reboot-required.
     */
    enum class FrameClass : uint8_t {
        FINS = 0,   // four aerodynamic fins (implemented, and what flies today)
        TVC  = 1,   // gimballed motor, two servos (SCOPED ONLY -- not implemented)
    };
    AP_Enum<FrameClass> frame_class;

    /*
      Dynamic pressure at which the vehicle commands FULL vertical.

      The goal is always true vertical. But commanding it the instant the rocket
      leaves the rail would be a step input of the whole rail tilt at the moment
      dynamic pressure -- and therefore fin authority -- is lowest. So the target
      blends from the measured rail attitude toward vertical as q builds, i.e. as
      the fins actually gain the authority to do something about it.
     */
    AP_Float level_q;

    /*
      Dynamic pressure below which the fins stop being driven.

      Fin force scales with q, so below some threshold the fins simply cannot
      produce a useful moment no matter how far they deflect. Without this the gain
      scheduling (Q_REF/q, capped at GAIN_MAX) drives them hard against a plant that
      cannot respond -- near apogee that means near-full deflection accomplishing
      nothing, wasting travel and risking integrator windup right before shutdown.

      Set 0 to disable and keep steering all the way to apogee.
     */
    AP_Float min_q;

    /*
      Tilt give-up backstop, degrees from vertical. If the airframe departs past this
      angle while steering (BOOST/COAST), stop steering and centre the fins -- treat it
      like apogee. Complements the climb-rate apogee, which can lag if the vehicle tumbles
      while still ascending. 0 disables. See rocket_control.cpp.
     */
    AP_Float giveup_deg;

    /*
      Direct tilt-controller gains (BOOST/COAST). The ascent law overrides the ATC cascade with
      fin = -TILT_P*view_angle - TILT_D*view_rate - TILT_I*integral(view_angle), q-scaled by the
      mixer. P sizes the immediate correction (4.0 -> full fin near 14 deg lean); D damps the
      swing; I nulls the steady weathercock lean that P alone balances at a nonzero tilt. IMAX
      caps the integral's fin share (anti-windup). These were compile-time #defines -- now params
      so the ascent tune is one visible, consistent, rebuild-free surface (the ATC_* gains do NOT
      drive the ascent). Defaults are the flown tune.
     */
    AP_Float tilt_p;
    AP_Float tilt_d;
    AP_Float tilt_i;
    AP_Float tilt_imax;

    /*
      Direct spin-rate damper gain (BOOST/COAST): yaw fin = -SPIN_DAMP*gyro.x, q-scaled. The
      effective damping MOMENT is ~ SPIN_DAMP*MOT_Q_REF, so these two MUST track each other --
      if MOT_Q_REF changes, rescale this by the inverse ratio or the spin chatters/runs. Default
      0.042 is matched to MOT_Q_REF 12000.
     */
    AP_Float spin_damp;

    /*
      Fin gain schedule enable. 1 (default) = the mixer scales fin deflection by MOT_Q_REF/q
      (baro-derived q) to hold the control MOMENT roughly constant across the huge boost speed
      range. 0 = MatrixPilot-style FIXED gain: the mixer runs at a constant scale (MOT_GAIN_MAX)
      and the aerodynamic moment scales with real q on its own. The real q still gates apogee and
      RKT_MIN_Q. An EXPERIMENT toggle to test whether the schedule earns its airframe-physics
      dependency.
     */
    AP_Int8 qsched;

    // Flight stage detection: launch gating and burnout shutdown.
    AP_Rocket rocket;

    // the fin servo outputs
    SRV_Channels servo_channels;

    // Never read. Exists because shared code assumes the rc() singleton is there.
    // See RC_Channel_Rocket.h.
    RC_Channels_Rocket rc_channels;
};

extern const AP_Param::Info var_info[];
