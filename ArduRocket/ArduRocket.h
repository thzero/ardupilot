/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
/*
  ArduRocket - a hobby rocket held vertical on four steering fins under a solid
  motor, with no RC transmitter and no ground station.

  The vehicle has no flight modes. set_mode() always fails, so no MAVLink
  SET_MODE, no RC switch and no failsafe can change what it does. What it does is
  hold vertical; the only thing that varies is the flight stage, which advances
  from measured acceleration alone. See defines.h for the stage enum and
  rocket_control.cpp for the controller.

  "Hold vertical" is spelled "hold level" here: the attitude controller runs on an
  AP_AHRS_View(ROTATION_PITCH_90), which presents the nose-up airframe to it as a
  level multicopter. See AP_FinMixerRocket.h for the resulting axis mapping, which
  is the thing most likely to be got wrong.
 */

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

// Application dependencies
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <Filter/Filter.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AP_Motors/AP_FinMixerRocket.h>
#include <AP_Rocket/AP_Rocket.h>

// Configuration
#include "defines.h"
#include "config.h"

#include "GCS_MAVLink_Rocket.h"
#include "GCS_Rocket.h"
#include "AP_Arming_Rocket.h"

// Local modules
#include "Parameters.h"

class ArduRocket : public AP_Vehicle
{
public:
    friend class GCS_MAVLINK_Rocket;
    friend class GCS_Rocket;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Rocket;

    ArduRocket(void);

private:

    // Global parameters are all contained within the 'g' class.
    Parameters g;
    ParametersG2 g2;

    // Arming/Disarming management class
    AP_Arming_Rocket arming;

    // GCS selection
    GCS_Rocket _gcs; // avoid using this; use gcs()
    GCS_Rocket &gcs() { return _gcs; }

    // Control
    AP_FinMixerRocket *motors;
    AC_AttitudeControl_Multi *attitude_control;
    const struct AP_Param::GroupInfo *motors_var_info;
    const struct AP_Param::GroupInfo *attitude_control_var_info;

    /*
      The whole trick. ROTATION_PITCH_90 presents the nose-up airframe to the
      attitude controller as a level multicopter, so holding vertical is holding
      level and Copter's attitude maths applies unmodified.
     */
    AP_AHRS_View *ahrs_view;

    // Flight stage. Advanced only by rocket_control.cpp from AP_Rocket's
    // acceleration-derived stage. Nothing outside the vehicle can set it.
    FlightStage stage;

    /*
      The attitude the airframe was sitting at when it was armed, in the rotated
      view frame -- i.e. how far off vertical the launch rail points.

      This is a CALIBRATION, not the target. The target is always true vertical.
      Knowing the starting offset lets the vehicle blend from it to vertical as
      dynamic pressure builds, instead of demanding the whole correction as a step
      at rail exit where fin authority is lowest. See RKT_LEVEL_Q and the target
      blend in rocket_control.cpp. Captured once, by AP_Arming_Rocket::arm().
     */
    float rail_roll_rad;
    float rail_pitch_rad;

    /*
      Pre-arm fin check. Fin DIRECTION cannot be verified in software: with the
      airframe clamped and no airflow there is no motion to observe, and comparing
      the mixer output against measured attitude is circular. So instead of
      pretending to check it, the vehicle makes the human check unskippable.

      First ARM of a power cycle runs a known wiggle sequence -- one fin at a time,
      announced -- and does NOT arm. The operator watches, then sends ARM again to
      confirm what they saw. Only then does the vehicle arm. Latched for the power
      cycle so a scrubbed countdown does not force a repeat, and cleared by reboot.
     */
    bool     fin_check_done;         // confirmed this power cycle
    bool     fin_check_awaiting;     // sequence finished, waiting for the operator
    uint32_t fin_check_start_ms;     // 0 when the sequence is not running

    /*
      True when the running sequence is an on-demand BENCH test rather than the
      arming gate.

      This distinction is load-bearing. A bench run must NOT satisfy the arming
      confirmation, or someone could wiggle the fins in the workshop and then arm
      on the rail with the gate already cleared -- exactly the check we are trying
      to force. Bench runs finish by returning to PREP and clearing themselves.
     */
    bool     fin_check_bench;

    // rocket_control.cpp
    void run_fin_check();
    void start_fin_check(bool gating);
public:
    // Start an on-demand fin test from a ground station (MAV_CMD_DO_MOTOR_TEST).
    // Refuses while armed. Does not affect the arming fin-check gate.
    bool start_bench_fin_test();
private:

    // Battery
    AP_BattMonitor battery{MASK_LOG_CURRENT, nullptr, nullptr};

    // Altitude/speed estimate, used to schedule the fin gains on dynamic pressure
    float dynamic_pressure_pa;

    // setup the var_info table
    AP_Param param_loader;

    bool initialised;

    static const AP_Scheduler::Task scheduler_tasks[];
    static const AP_Param::Info var_info[];
    static const struct LogStructure log_structure[];

    // ArduRocket.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    void update_batt_compass(void);
    void one_hz_loop();
    void read_AHRS(void);
    void update_altitude();
    void motors_output();

    // rocket_control.cpp
    void run_rocket_control();
    void update_dynamic_pressure();
    void set_stage(FlightStage new_stage);
    const char *stage_string() const;

    /*
      Nothing may change what this vehicle is doing. There is no mode to select,
      so every request to select one fails: no MAVLink SET_MODE, no RC switch, no
      failsafe reaction. This is the whole of the vehicle's mode-change surface.
     */
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return false; }
    uint8_t get_mode() const override { return (uint8_t)stage; }

#if HAL_LOGGING_ENABLED
    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;

    // Log.cpp
    void Log_Write_Attitude();
    void Log_Write_Rocket();
    void Log_Write_Vehicle_Startup_Messages();
    void ten_hz_logging_loop();
    void full_rate_logging();
#endif
    bool should_log(uint32_t mask);

    // Parameters.cpp
    void load_parameters(void) override;

    // system.cpp
    void init_ardupilot() override;
    void allocate_motors(void);
    MAV_TYPE get_frame_mav_type();
    const char* get_frame_string();

public:
    void failsafe_check();
};

extern ArduRocket rocket;

using AP_HAL::millis;
using AP_HAL::micros;
