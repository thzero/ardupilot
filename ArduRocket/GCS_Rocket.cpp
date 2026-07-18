#include "GCS_Rocket.h"

#include "ArduRocket.h"

const char* GCS_Rocket::frame_string() const
{
    return rocket.get_frame_string();
}

MAV_TYPE GCS_Rocket::frame_type() const
{
    return rocket.get_frame_mav_type();
}

/*
  There is no mode to report, so report the flight stage. It is the only thing
  about this vehicle's behaviour that varies, and a ground station watching a
  test flight wants to see PAD -> BOOST -> COAST.
 */
uint32_t GCS_Rocket::custom_mode() const
{
    return (uint32_t)rocket.stage;
}

bool GCS_Rocket::vehicle_initialised() const
{
    return rocket.initialised;
}
