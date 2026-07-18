#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
// ArduRocket build configuration.
//
// A rocket holds vertical on fins under a solid motor. It has no mission, no
// waypoint navigation, no RC and no position control, so those subsystems are
// compiled out rather than merely left unused. See the note on AP_SIM_FRAME_CLASS
// in system.cpp for why the SITL frame binding is NOT set here.
//////////////////////////////////////////////////////////////////////////////

#ifndef MAIN_LOOP_RATE
 # define MAIN_LOOP_RATE    400
#endif

#ifndef MAIN_LOOP_SECONDS
 # define MAIN_LOOP_SECONDS (1.0f / MAIN_LOOP_RATE)
#endif

// The rocket flies a ballistic arc under a solid motor. There is nothing to
// navigate to and no mission to run.
#ifndef AP_MISSION_ENABLED
 # define AP_MISSION_ENABLED 0
#endif

#ifndef AC_WPNAV_ENABLED
 # define AC_WPNAV_ENABLED 0
#endif

#ifndef MODE_AUTO_ENABLED
 # define MODE_AUTO_ENABLED 0
#endif

// Default log bitmask: attitude and IMU are what matter for post-flight tuning.
#ifndef DEFAULT_LOG_BITMASK
 # define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_RCOUT | \
    MASK_LOG_IMU | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCIN
#endif
