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

    /*
      Attitude estimation. AP_Vehicle does NOT register these for you -- Copter and
      Blimp each register them in their own Parameters.cpp, and ArduRocket originally
      did not. The failure is silent: a name in a .parm file that matches no
      registered parameter is discarded with no error, so the settings simply had no
      effect while appearing correct in the file.

      Two settings this vehicle actually depends on were dead because of it:
        AHRS_ORIENTATION - how the flight controller is mounted. Harmless in SITL,
                           where the simulated airframe already starts nose-up, but
                           on real hardware it is the only thing telling the filter
                           which way the board is bolted in.
        EK3_SRC1_*       - keeps GPS out of the flight solution, which is a stated
                           safety property of this vehicle (see 3c). It was never on.
     */

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass, "COMPASS_", Compass),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs, "AHRS_", AP_AHRS),

    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.ekf3.EKF3, NavEKF3, "EK3_", NavEKF3),

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
    AP_GROUPINFO("RKT_LEVEL_Q", 3, ParametersG2, level_q, 0.0f),

    // @Param: RKT_MIN_Q
    // @DisplayName: Dynamic pressure below which fins stop being driven
    // @Description: Fin force scales with dynamic pressure, so below some value the fins cannot produce a useful moment however far they deflect. Below this threshold the controller stops driving them and centers them, rather than saturating against a plant that cannot respond. This matters approaching apogee, where the gain scheduling would otherwise command near-full deflection to no effect. Set 0 to keep steering all the way to apogee.
    // @Units: Pa
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("RKT_MIN_Q", 4, ParametersG2, min_q, 50.0f),

    // @Param: RKT_GIVEUP_DEG
    // @DisplayName: Tilt give-up angle
    // @Description: If the airframe tilts more than this many degrees from vertical while steering (BOOST/COAST), the controller stops steering and centres the fins, the same as at apogee. Past this angle a fin-steered rocket has lost control authority, so continuing to drive the fins does nothing but flail. This is a backstop that complements the climb-rate apogee detection, which can lag if the vehicle tumbles while still ascending. Set 0 to disable.
    // @Units: deg
    // @Range: 30 90
    // @User: Standard
    AP_GROUPINFO("RKT_GIVEUP_DEG", 5, ParametersG2, giveup_deg, 60.0f),

    // @Param: RKT_TILT_P
    // @DisplayName: Tilt controller proportional gain
    // @Description: Direct tilt-law P gain (BOOST/COAST): fin = -TILT_P*view_angle - TILT_D*view_rate - TILT_I*integral, scaled by the mixer. Default 2.5 is the validated fixed-gain (RKT_QSCHED=0) tune; full fin near 23 deg lean. This is the ascent tune; the ATC_* gains do not drive the ascent. On an imported airframe ork_to_rocket.py derives this from inertia + fin geometry.
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("RKT_TILT_P", 6, ParametersG2, tilt_p, 2.5f),

    // @Param: RKT_TILT_D
    // @DisplayName: Tilt controller rate gain
    // @Description: Direct tilt-law D gain (BOOST/COAST), on the view-frame body rate. Damps the correction swing to stop overshoot.
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("RKT_TILT_D", 7, ParametersG2, tilt_d, 0.5f),

    // @Param: RKT_TILT_I
    // @DisplayName: Tilt controller integral gain
    // @Description: Direct tilt-law I gain (BOOST/COAST). PD alone balances the steady weathercock/gravity-turn moment at a nonzero tilt; the integral accumulates that residual lean and drives the STEADY tilt to zero. 0 disables (PD only).
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("RKT_TILT_I", 8, ParametersG2, tilt_i, 2.0f),

    // @Param: RKT_TILT_IMAX
    // @DisplayName: Tilt integral limit
    // @Description: Anti-windup cap on the tilt integral's fin contribution (0..1 fin units). Bounds how much of the fin the integral alone can command.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RKT_TILT_IMAX", 9, ParametersG2, tilt_imax, 0.6f),

    // @Param: RKT_SPIN_DAMP
    // @DisplayName: Spin-rate damper gain
    // @Description: Direct spin damper (BOOST/COAST): yaw fin = -SPIN_DAMP*gyro.x, scaled by the mixer. Default 0.006 is the validated fixed-gain (RKT_QSCHED=0) value. NOTE the effective damping moment scales with the mixer gain, so under the RKT_QSCHED=1 schedule this must be ~0.042 (see scheduled.parm). On an imported airframe ork_to_rocket.py derives it from spin inertia + fin geometry.
    // @Range: 0 0.5
    // @User: Standard
    AP_GROUPINFO("RKT_SPIN_DAMP", 10, ParametersG2, spin_damp, 0.006f),

    // @Param: RKT_QSCHED
    // @DisplayName: Fin gain schedule enable
    // @Description: 0 (default) = MatrixPilot-style fixed gain (mixer runs at constant MOT_GAIN_MAX scale; aero moment scales with real q on its own) -- validated across the flight envelope, and the mode the derived gains target. 1 = scale fin deflection by MOT_Q_REF/q (dynamic-pressure schedule); a fallback kept for one hardware flight's confidence -- needs the scheduled gain set (see Tools/ArduRocket/sitl_tests/scheduled.parm). Real q gates apogee/RKT_MIN_Q either way.
    // @Values: 0:FixedGain,1:QSchedule
    // @User: Standard
    AP_GROUPINFO("RKT_QSCHED", 11, ParametersG2, qsched, 0),

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

/*
  The COMPLETE ArduRocket config baked as firmware defaults, for UTTER CONSISTENCY: a wipe (-w),
  a fresh flash, or an UNREGISTERED SITL model (one not in vehicleinfo.json, e.g. rocket-tilt0 or
  a new -az/-roll variant -- which otherwise falls back to generic firmware defaults: EKF3+GPS,
  no fins, wrong q-schedule) all boot the IDENTICAL, flyable rocket. rocket.parm mirrors this and
  stays the human-editable source; these two MUST stay in sync (rocket.parm still layers on top
  for registered models via ROMFS, and for sim_vehicle). Ascent TUNING is not duplicated here --
  it lives in real params with their own defaults: RKT_TILT_P/D/I, RKT_TILT_IMAX, RKT_SPIN_DAMP
  (ParametersG2), RKT_LEVEL_Q/MIN_Q, and MOT_Q_REF below. NOTE: GPS1_TYPE, not GPS_TYPE.
 */
static const struct AP_Param::defaults_table_struct rocket_defaults[] = {
    // estimator: no-GPS DCM (architecture -- silent + dangerous if wrong)
    { "AHRS_EKF_TYPE",  0 },   // DCM, not EKF3
    { "EK3_SRC1_POSXY", 0 },   // no horizontal position/velocity source (no GPS)
    { "EK3_SRC1_VELXY", 0 },
    { "EK3_SRC1_POSZ",  1 },   // baro for vertical, which is all the rocket uses
    { "COMPASS_USE",    0 },   // magnetometer out of the solution
    { "COMPASS_USE2",   0 },
    { "COMPASS_USE3",   0 },
    // fin outputs, no RC, self-managed arming
    { "SERVO1_FUNCTION", 190 },   // the four fins are motor outputs 190..193
    { "SERVO2_FUNCTION", 191 },
    { "SERVO3_FUNCTION", 192 },
    { "SERVO4_FUNCTION", 193 },
    { "RC_PROTOCOLS",    0 },      // no receiver is ever attached
    { "ARMING_SKIPCHK", -1 },     // the rocket gates arming itself (fin check + rail)
    { "RKT_ENABLE",   1 },        // the flight-stage detector
    // MOT_* and ATC_* are NOT here: their objects (motors / attitude_control) are allocated in
    // allocate_motors() AFTER load_parameters(), so their params don't exist yet and baking them
    // here hard-errors (Config Error: param deflt fail). They go in rocket_late_defaults[],
    // applied by apply_late_defaults() once allocation is done. GPS1_TYPE likewise (AP_GPS::init
    // re-defaults it) is forced after gps.init() in init_ardupilot().
};

/*
  Defaults for params whose objects are allocated AFTER load_parameters() -- the fin mixer
  (MOT_*) and the attitude controller (ATC_*). apply_late_defaults() sets these once
  allocate_motors() has created them.
 */
static const struct AP_Param::defaults_table_struct rocket_late_defaults[] = {
    // Fixed gain (RKT_QSCHED=0, the default) runs the mixer at this constant scale -- 1.0 = the
    // controller's fin command maps 1:1 to deflection. Under the RKT_QSCHED=1 fallback schedule
    // this is instead the low-q boost cap and wants ~4 (see scheduled.parm).
    { "MOT_GAIN_MAX", 1.0 },
    // Only used by the RKT_QSCHED=1 fallback schedule (fin deflection = command * MOT_Q_REF/q).
    // Ignored under fixed gain. Kept so the schedule remains reachable for one hardware flight.
    { "MOT_Q_REF", 12000 },
    // ATC cascade gains: used ONLY on the rail (ARMED); in BOOST/COAST the direct tilt/spin law
    // (RKT_TILT_*, RKT_SPIN_DAMP) overrides the cascade, so these do NOT drive the ascent. Baked
    // only so the full param set is identical on every model (registered or not).
    { "ATC_RAT_RLL_P", 0.8 }, { "ATC_RAT_RLL_I", 0.6 }, { "ATC_RAT_RLL_D", 0.04 },
    { "ATC_RAT_PIT_P", 0.8 }, { "ATC_RAT_PIT_I", 0.6 }, { "ATC_RAT_PIT_D", 0.04 },
    { "ATC_RAT_YAW_P", 0.1 }, { "ATC_RAT_YAW_I", 0.01 },
    { "ATC_ANG_RLL_P", 18 },  { "ATC_ANG_PIT_P", 18 },  { "ATC_ANG_YAW_P", 2 },
};

void ArduRocket::apply_late_defaults(void)
{
    AP_Param::set_defaults_from_table(rocket_late_defaults, ARRAY_SIZE(rocket_late_defaults));
}

void ArduRocket::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);

    // Force the no-GPS DCM architecture regardless of what is (or isn't) in storage. Must run
    // before init_ardupilot()'s ahrs.init()/gps.init()/compass().init(), which read these.
    AP_Param::set_defaults_from_table(rocket_defaults, ARRAY_SIZE(rocket_defaults));

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROCKET);
}
