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
    if (rocket.stage == FlightStage::COAST) {
        // burnt out: the controller is shut down and the airframe is ballistic
        return MAV_STATE_STANDBY;
    }
    if (rocket.motors != nullptr && rocket.motors->armed()) {
        return MAV_STATE_ACTIVE;
    }
    return MAV_STATE_STANDBY;
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
  The vehicle has exactly one behaviour and no selectable modes, so there is no
  mode list to advertise.
 */
uint8_t GCS_MAVLINK_Rocket::send_available_mode(uint8_t index) const
{
    return 0;
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
}

MAV_RESULT GCS_MAVLINK_Rocket::handle_command_int_packet(const mavlink_command_int_t &packet,
                                                         const mavlink_message_t &msg)
{
    switch (packet.command) {

    case MAV_CMD_DO_MOTOR_TEST:
        /*
          Reused as "run the fin sequence" so the fins can be exercised on the
          bench from any ground station -- Mission Planner and QGC already have UI
          for this command. Parameters are ignored: the sequence is fixed, and
          running the SAME sequence as the arming check means what you verify on
          the bench is exactly what runs on the rail.

          This does NOT satisfy the arming fin-check gate (see ArduRocket.h), and
          is refused while armed.
         */
        return rocket.start_bench_fin_test() ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}
