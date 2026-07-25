#pragma once

#include <GCS_MAVLink/GCS.h>

#include "defines.h"

// Aux-function number that triggers the fin check via MAV_CMD_DO_AUX_FUNCTION.
// 300 is the first of ArduPilot's SCRIPTING_1.. slots -- the range reserved for
// vehicle/script-specific actions, so it collides with no standard RC option. The
// GCS "Fin Check" button sends DO_AUX_FUNCTION with param1 = this value.
#define RKT_AUX_FUNC_FIN_CHECK 300

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

    // Handles the messages the shared stream tables ask for but the base class
    // leaves to the vehicle. Omitting this is NOT harmless -- see the .cpp.
    bool try_send_message(enum ap_message id) override;

    // Reports ATTITUDE in the ROTATED VIEW rather than the raw body frame, so the
    // ground station stops sitting on the Euler singularity. Read the .cpp before
    // touching this: it deliberately changes what ATTITUDE means for this vehicle.
    void send_attitude() const override;

    // Heading is published in THREE separate messages, all of which upstream fills
    // from AHRS yaw. All three are redirected to rocket_heading_rad() below.
    void send_global_position_int() override;
    void send_vfr_hud_rocket();

    // Heading for DISPLAY ONLY, read straight off the magnetometer. See the .cpp.
    float rocket_heading_rad() const;

private:

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;
};
