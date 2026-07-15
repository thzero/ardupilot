#include "AP_Rocket.h"

#if AP_ROCKET_ENABLED

const AP_Param::GroupInfo AP_Rocket::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Rocket vertical-hold enable
    // @Description: Enables the rocket vertical-hold behavior used by the QROCKET flight mode: forced full-authority stabilization on the pad and accel-based launch gating of the rate integrators.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Rocket, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LAUNCH_G
    // @DisplayName: Rocket launch detection threshold
    // @Description: Body-up specific-force acceleration, in g, above which launch is declared. Must be safely above the stationary 1g reading and below the expected launch acceleration. Bench-tune from a stationary vertical log.
    // @Units: gravities
    // @Range: 1.1 10.0
    // @User: Standard
    AP_GROUPINFO("LAUNCH_G", 2, AP_Rocket, _launch_g, 1.5f),

    // @Param: LAUNCH_AX
    // @DisplayName: Rocket launch detection body axis
    // @Description: Body-frame axis that points up (along the airframe, toward the nose) with the current AHRS_ORIENTATION. Used for launch detection. For a nose-up board with AHRS_ORIENTATION=ROTATION_PITCH_90 this is normally X.
    // @Values: 0:X,1:Y,2:Z
    // @User: Standard
    AP_GROUPINFO("LAUNCH_AX", 3, AP_Rocket, _launch_axis, 0),

    AP_GROUPEND
};

AP_Rocket::AP_Rocket() :
    _launched(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Rocket::update(const Vector3f &accel_body)
{
    if (!enabled()) {
        // when disabled, never gate: behave as if already launched
        _launched = true;
        return;
    }
    if (_launched) {
        return;
    }

    const uint8_t axis = constrain_int16(_launch_axis, 0, 2);
    const float up_accel = accel_body[axis];

    if (up_accel > (GRAVITY_MSS * _launch_g)) {
        _launched = true;
    }
}

#endif  // AP_ROCKET_ENABLED
