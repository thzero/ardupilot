#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  Arming for a vehicle with no RC transmitter and no ground station.

  The temptation on such a vehicle is ARMING_SKIPCHK = -1, which disables every
  pre-arm check because several of them assume hardware the rocket does not have.
  That throws away the checks that DO apply -- the IMU and barometer the whole
  flight depends on -- to silence the two or three that do not. The overrides here
  are deliberately narrow instead.
 */
class AP_Arming_Rocket : public AP_Arming
{
public:
    friend class ArduRocket;

    AP_Arming_Rocket() : AP_Arming() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Rocket);

    // There is no receiver to calibrate. This is not a check being skipped for
    // convenience: there is no hardware for it to describe.
    bool rc_calibration_checks(bool display_failure) override { return true; }

    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;

protected:

    bool pre_arm_checks(bool display_failure) override;
};
