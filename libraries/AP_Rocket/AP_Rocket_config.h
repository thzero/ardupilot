#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// AP_Rocket holds the flight-stage detection driven by ArduRocket. Default it on
// for ArduRocket builds and off elsewhere. A board's hwdef may override.
#ifndef AP_ROCKET_ENABLED
#define AP_ROCKET_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduRocket)
#endif
