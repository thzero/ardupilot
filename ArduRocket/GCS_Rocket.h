#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Rocket.h"

class GCS_Rocket : public GCS
{
    friend class ArduRocket; // for access to _chan in parameter declarations

public:

    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Rocket);

    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    const char* frame_string() const override;

    bool vehicle_initialised() const override;

protected:

    // minimum amount of time (in microseconds) that must remain in the main
    // scheduler loop before we are allowed to send any mavlink messages
    uint16_t min_loop_time_remaining_for_message_send_us() const override
    {
        return 250;
    }

    GCS_MAVLINK_Rocket *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override
    {
        return NEW_NOTHROW GCS_MAVLINK_Rocket(uart);
    }
};
