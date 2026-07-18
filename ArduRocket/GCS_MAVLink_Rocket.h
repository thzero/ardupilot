#pragma once

#include <GCS_MAVLink/GCS.h>

#include "defines.h"

class GCS_MAVLINK_Rocket : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    bool params_ready() const override;
    void send_banner() override;

    void send_nav_controller_output() const override;
    uint64_t capabilities() const override;

#if HAL_LOGGING_ENABLED
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    // Send the mode with the given index (not mode number!) return the total
    // number of modes. Index starts at 1.
    uint8_t send_available_mode(uint8_t index) const override;

    // Intercepts MAV_CMD_DO_MOTOR_TEST so a ground station can run the fin
    // sequence on the bench; everything else falls through to the base class.
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet,
                                         const mavlink_message_t &msg) override;

private:

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;
};
