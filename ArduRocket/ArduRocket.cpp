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

#include "ArduRocket.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(ArduRocket, &rocket, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(ArduRocket, &rocket, func)

/*
  scheduler table - all tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine the order
  in which tasks are run.

  Note what is NOT here, and why:

  - rc_loop / read_radio: there is no RC receiver. Omitting the task removes the
    RC failsafe surface altogether rather than configuring it away.
  - failsafe_gcs_check: there is no ground station in flight. A GCS failsafe on a
    boosting rocket is a way to misfire, not a safety net.
  - ekf_check / check_vibration: in Copter these exist to trigger mode changes,
    and this vehicle has no modes to change to. A rocket is also guaranteed to be
    a high-vibration airframe, so check_vibration would fire on every flight.
  GPS is present but is a TRACKING sensor, not a control sensor. It is polled so its
  raw position reaches telemetry and the log (which is how you find the airframe after
  it lands), but EK3_SRC1_POSXY/VELXY exclude it, so it never feeds the attitude or
  velocity solution the fins fly on. GPS under high-g boost drops lock exactly when
  control matters most, which is why it is kept out of the loop.

  The GCS tasks stay despite there being no ground station: autotest drives the
  vehicle over MAVLink. Keep the transport, drop the failsafes.
 */
const AP_Scheduler::Task ArduRocket::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &rocket.ins, update),
    // send outputs to the fins immediately
    FAST_TASK(motors_output),
    // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
    // flight stage detection and the attitude controller
    FAST_TASK(run_rocket_control),

    // GPS for tracking/recovery only -- see the note above. Cheap, and it must be
    // polled or the driver produces no fixes at all.
    SCHED_TASK_CLASS(AP_GPS,               &rocket.gps,                update,          50, 200,   9),
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK(update_altitude,       10,    100,  21),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(full_rate_logging,     50,     50,  33),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &rocket.notify,             update,          50,  90,  36),
    SCHED_TASK(send_rocket_telemetry,  5,    100,  38),
    SCHED_TASK(one_hz_loop,            1,    100,  39),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&rocket._gcs,         update_receive, 400, 180,  51),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&rocket._gcs,         update_send,    400, 550,  54),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  57),
    SCHED_TASK_CLASS(AP_Logger,            &rocket.logger,             periodic_tasks, 400, 300,  63),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &rocket.ins,                periodic,       400,  50,  66),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &rocket.scheduler,          update_logging, 0.1,  75,  69),
#endif
};

void ArduRocket::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                     uint8_t &task_count,
                                     uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

// update_batt_compass - read battery and compass
void ArduRocket::update_batt_compass(void)
{
    battery.read();

    if (AP::compass().available()) {
        compass.read();
    }
}

void ArduRocket::one_hz_loop()
{
    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    AP_Notify::flags.flying = (stage != FlightStage::PREP);
}

void ArduRocket::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it above
    ahrs.update(true);

    // keep the rotated view in step with the AHRS it wraps
    if (ahrs_view != nullptr) {
        ahrs_view->update();
    }
}

// read baro
void ArduRocket::update_altitude()
{
    barometer.update();

    // Cache air density here, off the 400 Hz control path. It feeds the dynamic-
    // pressure gain scheduling but changes only with altitude, so 10 Hz is ample.
    air_density_kgm3 = AP_Baro::get_air_density_for_alt_amsl(barometer.get_altitude_AMSL());
}

void ArduRocket::motors_output()
{
    if (motors == nullptr) {
        return;
    }

    /*
      The spool state machine holds in GROUND_IDLE until the vehicle clears the
      spool-up block. That block exists for Copter's pre-takeoff ESC checks, none
      of which apply to a solid motor we do not command, so clear it every loop.
      Without this the fins would never go live. ArduPlane does the same at
      quadplane.cpp:1953.
     */
    motors->set_spoolup_block(false);

    // Convert the channel values to PWM, then cork/push so all four fins move in
    // the same output frame. The push() is essential and easy to omit: without it
    // the mixer's fin commands are computed and stored but never reach the
    // outputs, so the servos sit at zero and the rocket has no control at all.
    SRV_Channels::calc_pwm();

    auto &srv = AP::srv();
    srv.cork();

    SRV_Channels::output_ch_all();

    motors->output();

    srv.push();
}

bool ArduRocket::should_log(uint32_t mask)
{
#if HAL_LOGGING_ENABLED
    return logger.should_log(mask);
#else
    return false;
#endif
}

/*
  constructor for main ArduRocket class
 */
ArduRocket::ArduRocket(void)
    :
      motors(nullptr),
      attitude_control(nullptr),
      motors_var_info(nullptr),
      attitude_control_var_info(nullptr),
      ahrs_view(nullptr),
      stage(FlightStage::PREP),
      rail_roll_rad(0.0f),
      rail_pitch_rad(0.0f),
      fin_check_valid(false),
      fin_check_valid_ms(0),
      fin_check_start_ms(0),
      fin_check_on_rail(false),
      land_start_ms(0),
      dynamic_pressure_pa(0.0f),
      param_loader(var_info),
      initialised(false)
{
}

ArduRocket rocket;
AP_Vehicle& vehicle = rocket;

AP_HAL_MAIN_CALLBACKS(&rocket);
