#include "ArduRocket.h"

/*
 *  ArduRocket parameter definitions
 *
 *  The parameter surface is deliberately small. There is no RC, no mission, no
 *  navigation and no throttle to configure. What is left is the attitude
 *  controller (ATC_), the fin mixer (MOT_), the servo outputs (SERVO_) and the
 *  flight stage detection (RKT_).
 */

const AP_Param::Info ArduRocket::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "FORMAT_VERSION", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what log types to enable in on-board logger. The rocket burn is short and gives one attempt per motor, so the attitude and IMU logs are the whole tuning surface.
    // @Bitmask: 0:Fast Attitude,1:Medium Attitude,2:GPS,3:Performance,4:Control Tuning,5:RC Input,6:IMU,7:Battery,8:RC Output,9:Fast IMU,10:Raw IMU,11:PID
    // @User: Standard
    GSCALAR(log_bitmask, "LOG_BITMASK", DEFAULT_LOG_BITMASK),

    // @Group: INS
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins, "INS", AP_InertialSensor),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery, "BATT", AP_BattMonitor),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_", AP_Notify),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp,AP_Arming_Rocket.cpp
    GOBJECT(arming, "ARMING_", AP_Arming_Rocket),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig, "BRD_", AP_BoardConfig),

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    GOBJECTVARPTR(attitude_control, "ATC_", &rocket.attitude_control_var_info),

    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp,../libraries/AP_Motors/AP_FinMixerRocket.cpp
    GOBJECTVARPTR(motors, "MOT_", &rocket.motors_var_info),

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "", ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Param: FRAME_CLASS
    // @DisplayName: Rocket frame class
    // @Description: Which actuator this airframe steers with. 0 (Fins) is the only implemented option and is the default. Selecting TVC on firmware that does not implement it will refuse to boot rather than fly with no control. The mixer is allocated once at startup, so a reboot is required for a change to take effect.
    // @Values: 0:Fins,1:ThrustVectoring
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 2, ParametersG2, frame_class, uint8_t(ParametersG2::FrameClass::FINS)),

    // @Param: RKT_LEVEL_Q
    // @DisplayName: Dynamic pressure for full vertical command
    // @Description: The rocket always aims for true vertical, but commanding it the instant the airframe leaves the rail would be a step input of the entire rail tilt at the moment fin authority is lowest. The attitude target therefore blends from the rail attitude measured at arming toward vertical as dynamic pressure rises, reaching full vertical at this value. Set lower to level out sooner and more aggressively, higher to level out more gently. Set to 0 to command vertical immediately with no blend.
    // @Units: Pa
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("RKT_LEVEL_Q", 3, ParametersG2, level_q, 600.0f),

    // @Param: RKT_MIN_Q
    // @DisplayName: Dynamic pressure below which fins stop being driven
    // @Description: Fin force scales with dynamic pressure, so below some value the fins cannot produce a useful moment however far they deflect. Below this threshold the controller stops driving them and centres them, rather than saturating against a plant that cannot respond. This matters approaching apogee, where the gain scheduling would otherwise command near-full deflection to no effect. Set 0 to keep steering all the way to apogee.
    // @Units: Pa
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("RKT_MIN_Q", 4, ParametersG2, min_q, 50.0f),

    // @Group: RKT_
    // @Path: ../libraries/AP_Rocket/AP_Rocket.cpp
    AP_SUBGROUPINFO(rocket, "RKT_", 1, ParametersG2, AP_Rocket),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Rocket),

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void ArduRocket::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROCKET);
}
