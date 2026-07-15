#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// AP_Rocket drives the QROCKET tailsitter vertical-hold mode, which only exists
// on ArduPlane. Default it on for Plane builds and off elsewhere. A board's
// hwdef may override. The additional HAL_QUADPLANE_ENABLED guard is applied in
// the ArduPlane-side files, where that macro is in scope.
#ifndef AP_ROCKET_ENABLED
#define AP_ROCKET_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#endif
