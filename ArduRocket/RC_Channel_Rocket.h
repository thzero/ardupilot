#pragma once

#include <RC_Channel/RC_Channel.h>

/*
  ArduRocket has no RC receiver, and nothing here reads a stick.

  The object still has to exist. Shared library code reaches for the RC_Channels
  singleton unconditionally -- AP_CRSF_Telem::queue_message() calls rc() on the
  path out of GCS::send_text(), for one -- and rc() returns a reference, so a
  missing singleton is a null dereference rather than a graceful no-op.

  So this is the smallest RC_Channels that satisfies the singleton: channels that
  exist but are never read, no flight mode channel (there are no modes), and no
  arming channel (arming is a MAVLink command or nothing).
 */
class RC_Channel_Rocket : public RC_Channel
{
};

class RC_Channels_Rocket : public RC_Channels
{
public:

    RC_Channel_Rocket obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Rocket *channel(const uint8_t chan) override
    {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    const RC_Channel_Rocket *channel(const uint8_t chan) const override
    {
        if (chan >= ARRAY_SIZE(obj_channels)) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    // there are no flight modes, so there is no mode channel
    int8_t flight_mode_channel_number() const override { return -1; }
};
