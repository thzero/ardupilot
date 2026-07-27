#include "GCS_MAVLink_Rocket.h"

#include "ArduRocket.h"

#include <AP_RPM/AP_RPM_config.h>
#include <AP_EFI/AP_EFI_config.h>

bool GCS_MAVLINK_Rocket::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never try to send parameters if we are in config error
        return true;
    }
    return rocket.initialised;
}

void GCS_MAVLINK_Rocket::send_banner()
{
    GCS_MAVLINK::send_banner();
    send_text(MAV_SEVERITY_INFO, "Frame: %s", rocket.get_frame_string());
}

/*
  The rocket has no modes. Report that it is always stabilising and never
  guided: nothing can command it anywhere, and the only autonomy it has is
  holding vertical.
 */
uint8_t GCS_MAVLINK_Rocket::base_mode() const
{
    uint8_t base = MAV_MODE_FLAG_STABILIZE_ENABLED;

    // Report the armed state, or every ground station shows the vehicle as
    // disarmed while it is live on the rail.
    if (rocket.motors != nullptr && rocket.motors->armed()) {
        base |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    return base;
}

MAV_STATE GCS_MAVLINK_Rocket::vehicle_system_status() const
{
    if (!rocket.initialised) {
        return MAV_STATE_BOOT;
    }
    if (AP_BoardConfig::in_config_error()) {
        return MAV_STATE_CRITICAL;
    }
    /*
      Report ACTIVE only while actually airborne, and MUST agree with landed_state()
      below. The vehicle stays ARMED from the rail through touchdown, so keying this on
      the armed flag would report ACTIVE on the ground -- which contradicts
      landed_state()=ON_GROUND and makes the ground station flicker between "armed" and
      "flying" after landing. Airborne = BOOST/COAST/DESCENT; everything else (on the
      rail, or landed) is STANDBY.
     */
    switch (rocket.stage) {
    case FlightStage::BOOST:
    case FlightStage::COAST:
    case FlightStage::DESCENT:
        return MAV_STATE_ACTIVE;
    case FlightStage::PREP:
    case FlightStage::FINCHECK:
    case FlightStage::ARMED:
    case FlightStage::LANDED:
        break;
    }
    return MAV_STATE_STANDBY;
}

/*
  Ground-vs-air state for EXTENDED_SYS_STATE.

  The ground station's "Flying" indicator keys off this. The vehicle stays ARMED from
  the rail all the way through touchdown (disarm is manual), so reporting flight from
  the armed flag alone would leave the GCS saying "Flying" on the ground both before
  launch and after landing. Drive it from the flight stage instead: only BOOST, COAST
  and DESCENT are actually airborne.
 */
MAV_LANDED_STATE GCS_MAVLINK_Rocket::landed_state() const
{
    switch (rocket.stage) {
    case FlightStage::PREP:
    case FlightStage::FINCHECK:
    case FlightStage::ARMED:      // armed on the rail, but not yet launched
    case FlightStage::LANDED:
        return MAV_LANDED_STATE_ON_GROUND;
    case FlightStage::BOOST:
    case FlightStage::COAST:
    case FlightStage::DESCENT:
        return MAV_LANDED_STATE_IN_AIR;
    }
    return MAV_LANDED_STATE_UNDEFINED;
}

void GCS_MAVLINK_Rocket::send_nav_controller_output() const
{
    // There is no navigation controller. The vehicle holds vertical and has
    // nowhere to go. This override exists because the base class requires it.
}

uint64_t GCS_MAVLINK_Rocket::capabilities() const
{
    return GCS_MAVLINK::capabilities();
}

float GCS_MAVLINK_Rocket::vfr_hud_airspeed() const
{
    // No airspeed sensor, so report the EKF's speed over ground. For a vertical
    // rocket that is essentially airspeed.
    //
    // NOTE this is get_velocity_NED(), which is NOT the call the fin gain scheduling
    // uses -- that is get_velocity_D(velD, true), the position-consistent vertical
    // rate. The two track each other closely here only because the motion is very
    // nearly vertical (measured in flight: 408.6 vs 407.9 m/s peak). Do not treat
    // this field as a readout of the gain-scheduling input; it is a separate estimate.
    Vector3f vel_ned;
    if (!rocket.ahrs.get_velocity_NED(vel_ned)) {
        return 0;
    }
    return vel_ned.length();
}

int16_t GCS_MAVLINK_Rocket::vfr_hud_throttle() const
{
    // a solid motor has no throttle
    return 0;
}

float GCS_MAVLINK_Rocket::vfr_hud_alt() const
{
    return rocket.barometer.get_altitude();
}

void GCS_MAVLINK_Rocket::send_pid_tuning()
{
    // no PID tuning stream: there is nothing to tune in flight on a 2 second burn
}

/*
  Advertise the flight stages as AVAILABLE_MODES so a ground station shows the
  stage by NAME ("BOOST") rather than as a bare custom_mode number.

  Every stage is flagged NOT_USER_SELECTABLE. The stages are not modes a person can
  command -- set_mode() refuses everything, and the machine advances only on sensed
  flight events (launch accel, apogee). Advertising them as selectable would put a
  dropdown in the GCS that silently does nothing; the flag makes them read-only
  labels instead, which is what they are.

  Called once per index, 1-based, and must return the TOTAL count each time (the
  base class walks the list by asking for each index in turn). The custom_mode
  numbers here MUST equal the FlightStage enum values, since get_mode() returns the
  stage as the heartbeat's custom_mode and the GCS matches the two up.
 */
uint8_t GCS_MAVLINK_Rocket::send_available_mode(uint8_t index) const
{
    struct {
        FlightStage stage;
        const char *name;
    } static const modes[] {
        { FlightStage::PREP,     "Prep" },
        { FlightStage::FINCHECK, "Fin check" },
        { FlightStage::ARMED,    "Armed" },
        { FlightStage::BOOST,    "Boost" },
        { FlightStage::COAST,    "Coast" },
        { FlightStage::DESCENT,  "Descent" },
        { FlightStage::LANDED,   "Landed" },
    };
    const uint8_t mode_count = ARRAY_SIZE(modes);

    // out of range: report the count but send nothing
    if (index == 0 || index > mode_count) {
        return mode_count;
    }
    const auto &m = modes[index - 1];

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        (uint8_t)m.stage,
        MAV_MODE_PROPERTY_NOT_USER_SELECTABLE,   // stages are sensed, never commanded
        m.name);

    return mode_count;
}

/*
  ATTITUDE, reported in the rotated view instead of the raw body frame.

  THIS DELIBERATELY CHANGES WHAT ATTITUDE MEANS FOR THIS VEHICLE. Read this before
  "fixing" it back.

  The raw body frame of a nose-up rocket sits at pitch = 90 degrees, which is exactly
  the Euler singularity: roll and yaw describe the same rotation and neither is
  individually defined. A ground station therefore shows heading and roll swinging
  through ~166 degrees while the airframe stands perfectly still -- measured at 0.086
  degrees of actual tilt. That is not noise and no filter can fix it, because the
  information is not in the signal.

  The view (ROTATION_PITCH_90) is the frame the controller already flies in, where a
  vertical rocket presents as LEVEL. Reporting it means:

    roll, pitch  ->  tilt away from vertical, near zero on the rail

  Both are far from the singularity and well conditioned, and they are what an
  operator actually wants: the artificial horizon becomes "am I vertical".

  Yaw is NOT taken from the view. Even in the view the EKF's yaw state is the same
  badly-conditioned quantity, so it comes from the magnetometer instead -- see
  rocket_heading_rad().

  The cost is that ATTITUDE no longer means what the MAVLink spec says it means, so a
  generic consumer will be misled. That is why ATTITUDE_QUATERNION is deliberately
  left alone: it still carries the TRUE body attitude, so an honest ground-truth
  channel always exists. If you need the raw body Euler angles, derive them from the
  quaternion -- do not change this back without also solving the singularity.
 */
void GCS_MAVLINK_Rocket::send_attitude() const
{
    const AP_AHRS_View *view = rocket.ahrs_view;
    if (view == nullptr) {
        // no view yet (very early boot): the raw frame is all there is
        GCS_MAVLINK::send_attitude();
        return;
    }

    const Vector3f omega = view->get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        view->roll,
        view->pitch,
        // Yaw comes from the MAGNETOMETER, not the EKF -- see rocket_heading_rad().
        // The EKF's yaw is meaningless nose-up (it wandered 95.7 degrees while the
        // vehicle sat still), so streaming it just spins the ground station's compass
        // rose. Nothing on this vehicle consumes heading: the controller commands yaw
        // RATE zero off the gyro, never a yaw angle. ATTITUDE_QUATERNION is left
        // untouched and still carries the true body attitude as ground truth.
        rocket_heading_rad(),
        omega.x,
        omega.y,
        omega.z);
}

/*
  Heading for the ground station, taken STRAIGHT OFF THE MAGNETOMETER.

  This is deliberately independent of the EKF, and the compass is deliberately kept
  out of the flight solution (COMPASS_USE=0 in rocket.parm). The split is the point:

    the flight code    - never uses heading at all. Attitude hold works on tilt, and
                         spin is held by commanding yaw RATE zero off the gyro.
    the ground station - gets a real compass heading, because a spinning or frozen
                         compass rose on the pad is useless to the crew.

  Why not just report the EKF's yaw? Because it is meaningless on this airframe. The
  EKF expresses yaw as "which compass direction is the nose pointing", and a rocket's
  nose points at the sky, so that has no answer -- measured, it wandered 95.7 degrees
  while the vehicle sat still. Raising it to 3-axis mag fusion (EK3_MAG_CAL=4) cut
  that to 9.4 degrees but was rejected: 3-axis fusion assumes a magnetically stable
  environment, and a steel motor casing plus igniter current is the opposite of that.

  Computing it here sidesteps the whole problem: the field vector is rotated into the
  VIEW frame, in which a vertical rocket reads as level, and then tilt-compensated
  with the view's own matrix. The tilt correction is therefore small and well known,
  and the result is the airframe's clocking about its long axis -- a quantity that is
  genuinely observable from the field vector at any attitude.

  Returns 0 if there is no healthy compass, which reads as north rather than as a
  gap. Acceptable for a display-only value that nothing acts on.
 */
float GCS_MAVLINK_Rocket::rocket_heading_rad() const
{
    const Compass &compass = AP::compass();
    if (!compass.available() || !compass.healthy() || rocket.ahrs_view == nullptr) {
        return 0.0f;
    }

    /*
      This repeats Compass::calculate_heading()'s tilt compensation rather than
      calling it, because that helper pairs the field with the RAW body DCM and we
      need the VIEW. Both must be in the same frame -- passing the view's matrix to
      the helper mixes frames and produced a heading that swung 347 degrees.

      So: rotate the field into the view frame first, then tilt-compensate with the
      view's matrix. In the view a vertical rocket reads level, so the correction is
      small and the result is the airframe's clocking about its long axis.
     */
    Vector3f field = compass.get_field();
    field.rotate(ROTATION_PITCH_90);        // same rotation used to build ahrs_view

    const Matrix3f &dcm = rocket.ahrs_view->get_rotation_body_to_ned();

    // dcm.c is the gravity direction in the view frame, i.e. roll/pitch only
    const float cos_pitch_sq = 1.0f - (dcm.c.x * dcm.c.x);
    const float headY = field.y * dcm.c.z - field.z * dcm.c.y;
    const float headX = field.x * cos_pitch_sq -
                        dcm.c.x * (field.y * dcm.c.y + field.z * dcm.c.z);

    const float heading = constrain_float(atan2f(-headY, headX), -M_PI, M_PI);
    return wrap_PI(heading + compass.get_declination());
}

/*
  GLOBAL_POSITION_INT with the heading field taken from the magnetometer.

  Mirrors GCS_MAVLINK::send_global_position_int() exactly except for the final
  argument, which upstream fills with ahrs.yaw_sensor. See send_attitude() for why
  heading does not come from the EKF on this airframe.
 */
void GCS_MAVLINK_Rocket::send_global_position_int()
{
    AP_AHRS &ahrs = AP::ahrs();

    // Upstream caches into its private global_position_current_loc, which a subclass
    // cannot touch, so use a local copy. Location default-constructs to zeros, which
    // matches upstream's behaviour of sending stale/empty data rather than nothing.
    // There is no horizontal fix on this vehicle anyway (GPS is out of the EKF).
    Location loc;
    UNUSED_RESULT(ahrs.get_location(loc));

    Vector3f vel;
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }

    mavlink_msg_global_position_int_send(
        chan,
        AP_HAL::millis(),
        loc.lat,
        loc.lng,
        loc.alt * 10UL,     // same expression as GCS_MAVLINK::global_position_int_alt()
        global_position_int_relative_alt(),
        vel.x * 100,
        vel.y * 100,
        vel.z * 100,
        // heading in centidegrees, from the magnetometer rather than the EKF
        (uint16_t)(wrap_360(degrees(rocket_heading_rad())) * 100));
}

/*
  VFR_HUD with the heading field taken from the magnetometer.

  GCS_MAVLINK::send_vfr_hud() is NOT virtual, so it cannot be overridden the way
  send_global_position_int() can. Instead try_send_message() routes MSG_VFR_HUD here.
  This is the message QGroundControl actually drives its compass rose from, so
  missing it was why pinning ATTITUDE alone changed nothing on screen.
 */
void GCS_MAVLINK_Rocket::send_vfr_hud_rocket()
{
    AP_AHRS &ahrs = AP::ahrs();

    mavlink_msg_vfr_hud_send(
        chan,
        vfr_hud_airspeed(),
        ahrs.groundspeed(),
        (int16_t)wrap_360(degrees(rocket_heading_rad())),   // magnetometer heading, deg
        abs(vfr_hud_throttle()),
        vfr_hud_alt(),
        vfr_hud_climbrate());
}

/*
  Vehicle-specific message sending.

  This override is load-bearing even though it handles a single message. The
  stream tables in GCS_MAVLink_Parameters.cpp are SHARED by every vehicle, and
  they list some messages that the base GCS_MAVLINK cannot send because their
  content is vehicle-specific. MSG_WIND is one: Plane, Copter, Rover, Sub, Blimp
  and Tracker each implement it, so the base class treats reaching it as a bug.

  Without this override a ground station that requests all data streams -- which
  is a perfectly ordinary thing for a GCS to do on connect -- queues MSG_WIND,
  it falls through to GCS_MAVLINK::try_send_message's default branch, and:

    in SITL, that branch calls AP_HAL::panic() and the vehicle DIES outright;
    on real hardware the panic is compiled out, so instead it emits
    "Sending unknown message (36)" as STATUSTEXT at the full stream rate,
    flooding the telemetry link for the rest of the flight.

  Neither is acceptable, and the failure appears only once a real GCS connects,
  which is why the scripted tests never caught it.
 */
bool GCS_MAVLINK_Rocket::try_send_message(enum ap_message id)
{
    switch (id) {

    case MSG_VFR_HUD:
        // routed here only to replace the heading field; send_vfr_hud() is not virtual
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud_rocket();
        break;

    case MSG_WIND:
        /*
          Deliberately send nothing, following ArduSub and AntennaTracker.

          A rocket carries no airspeed sensor and the EKF is given no horizontal
          velocity source (see EK3_SRC1_* in rocket.parm), so there is no wind
          estimate to report. Reporting a fabricated or stale zero would be worse
          than reporting nothing: a ground station would display it as fact.

          Returning true means "handled" -- the message is consumed, not deferred,
          so it does not retry forever.
         */
        return true;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    // reached only by cases that break: the message was sent
    return true;
}

MAV_RESULT GCS_MAVLINK_Rocket::handle_command_int_packet(const mavlink_command_int_t &packet,
                                                         const mavlink_message_t &msg)
{
    switch (packet.command) {

    case MAV_CMD_DO_AUX_FUNCTION:
        /*
          "Fin Check" trigger. param1 is the aux-function number; we own
          RKT_AUX_FUNC_FIN_CHECK and pass everything else through to the base class.

          This is deliberately NOT MAV_CMD_DO_MOTOR_TEST: these are fins, not motors,
          and DO_MOTOR_TEST surfaces in a ground station as a "Motor Test" panel,
          which is both mislabelled and buried. DO_AUX_FUNCTION is the generic
          "invoke a named action" command; a ground-station button (see the QGC
          custom-action file shipped with this vehicle) labels it "Fin Check".

          Triggering on the rail latches the arming gate; on the bench it just
          exercises the fins. See ArduRocket::trigger_fin_check().
         */
        if ((uint16_t)packet.param1 == RKT_AUX_FUNC_FIN_CHECK) {
            return rocket.trigger_fin_check() ? MAV_RESULT_ACCEPTED
                                              : MAV_RESULT_TEMPORARILY_REJECTED;
        }
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);

    case MAV_CMD_DO_MOTOR_TEST:
        /*
          Kept as a fallback for ground stations without a custom "Fin Check" button:
          their built-in Motor Test panel still runs the fin check. Same behaviour as
          the aux-function path -- on the rail it counts, on the bench it does not.
         */
        return rocket.trigger_fin_check() ? MAV_RESULT_ACCEPTED
                                          : MAV_RESULT_TEMPORARILY_REJECTED;

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}
