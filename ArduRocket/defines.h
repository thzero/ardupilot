#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

//  Logging parameters - only 32 messages are available to the vehicle here.
enum LoggingParameters {
    LOG_ROCKET_MSG,
};

// Logging bitmask bits
#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_RCIN                   (1<<5)
#define MASK_LOG_IMU                    (1<<6)
#define MASK_LOG_CURRENT                (1<<7)
#define MASK_LOG_RCOUT                  (1<<8)
#define MASK_LOG_IMU_FAST               (1<<9)
#define MASK_LOG_IMU_RAW                (1<<10)
#define MASK_LOG_PID                    (1<<11)
#define MASK_LOG_ANY                    0xFFFF

/*
  Flight stage.

  This is deliberately NOT a flight mode. Nothing outside the vehicle can select
  it: ArduRocket::set_mode() always returns false, so there is no MAVLink SET_MODE,
  no RC switch and no failsafe path that can change what the vehicle is doing. The
  stage advances only from measured acceleration and climb rate, via AP_Rocket.
 */
enum class FlightStage : uint8_t {
    PREP     = 0,  // disarmed. On the table OR on the rail. Fins centered.
    FINCHECK = 1,  // arm requested and pre-arm passed: driving the fin wiggle so the
                   // pad crew can watch each fin move, in order, and confirm it is
                   // the fin that was announced and that it moves the right way.
                   // NOT ARMED. The operator must send ARM again to confirm.
    ARMED    = 2,  // armed on the rail, pre-launch. Attitude live (P/D), I-terms held.
    BOOST    = 3,  // launch detected, motor burning. Full fin authority.
    COAST    = 4,  // burnout recorded, still ascending. Fins KEEP steering (aero fins
                   // still have airflow). Burnout drives no control change here.
    DESCENT  = 5,  // apogee passed (climb rate negative). Steering ceases; fins centered.
};
