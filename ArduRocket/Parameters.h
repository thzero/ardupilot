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

    // Flight stage detection: launch gating and burnout shutdown.
    AP_Rocket rocket;

    // the fin servo outputs
    SRV_Channels servo_channels;

    // Never read. Exists because shared code assumes the rc() singleton is there.
    // See RC_Channel_Rocket.h.
    RC_Channels_Rocket rc_channels;
};

extern const AP_Param::Info var_info[];
