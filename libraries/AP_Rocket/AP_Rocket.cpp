#include "AP_Rocket.h"

#if AP_ROCKET_ENABLED

#include <AP_HAL/AP_HAL.h>

#if defined(APM_BUILD_TYPE)
//  - this is just here to encourage the build system to supply the "legacy build
//    defines". The actual dependency is in the AP_Rocket_config.h header.
#endif

const AP_Param::GroupInfo AP_Rocket::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Rocket flight stage detection enable
    // @Description: Enables accel-based flight stage detection: integrator gating on the rail until launch, and controller shutdown at burnout. When disabled the vehicle parks in the BOOST stage with the fins always live and no burnout shutdown, which is the bench-test configuration.
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
    // @Description: Body-frame axis that points up (along the airframe, toward the nose) with the current AHRS_ORIENTATION. Used for launch and burnout detection. Confirm on the bench: log INS/ACC while vertical and stationary, and the up-axis should read approximately +9.8 m/s/s.
    // @Values: 0:X,1:Y,2:Z
    // @User: Standard
    AP_GROUPINFO("LAUNCH_AX", 3, AP_Rocket, _launch_axis, 0),

    // @Param: LAUNCH_MS
    // @DisplayName: Rocket launch detection debounce
    // @Description: The launch acceleration threshold must be exceeded continuously for this long before launch is declared. Guards against a single noisy sample triggering launch on a vibrating pad.
    // @Units: ms
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("LAUNCH_MS", 4, AP_Rocket, _launch_ms, 50),

    // @Param: BURN_G
    // @DisplayName: Rocket burnout detection threshold
    // @Description: Body-up specific-force acceleration, in g, below which burnout is declared once boosting. After burnout the vehicle is coasting and the accelerometer reads near zero (drag only), so this should sit well below any powered acceleration.
    // @Units: gravities
    // @Range: 0.0 1.0
    // @User: Standard
    AP_GROUPINFO("BURN_G", 5, AP_Rocket, _burnout_g, 0.2f),

    // @Param: BURN_MS
    // @DisplayName: Rocket burnout detection debounce
    // @Description: The burnout threshold must be undershot continuously for this long before burnout is declared. Guards against a thrust dip mid-burn being read as burnout. Burnout is recorded only and drives no control change; the debounce just keeps the logged event clean.
    // @Units: ms
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("BURN_MS", 6, AP_Rocket, _burnout_ms, 100),

    // @Param: APOG_MS
    // @DisplayName: Rocket apogee detection debounce
    // @Description: Once launched, the climb rate must be negative continuously for this long before apogee is declared and steering ceases. Apogee -- not burnout -- is what stops the fins, so this must be robust to brief velocity-estimate noise near the top of the climb.
    // @Units: ms
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("APOG_MS", 7, AP_Rocket, _apogee_ms, 500),

    AP_GROUPEND
};

AP_Rocket::AP_Rocket() :
    _stage(Stage::PRE_LAUNCH),
    _accel_start_ms(0),
    _apogee_start_ms(0),
    _up_accel_g(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Rocket::reset()
{
    _stage = Stage::PRE_LAUNCH;
    _accel_start_ms = 0;
    _apogee_start_ms = 0;
}

bool AP_Rocket::debounce(uint32_t &start_ms, bool condition, uint16_t hold_ms)
{
    if (!condition) {
        start_ms = 0;
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (start_ms == 0) {
        start_ms = now_ms;
        // a zero hold time must still fire on the first sample
        return hold_ms == 0;
    }

    return (now_ms - start_ms) >= hold_ms;
}

void AP_Rocket::update(const Vector3f &accel_body, float climb_rate_ms)
{
    const uint8_t axis = constrain_int16(_launch_axis, 0, 2);
    _up_accel_g = accel_body[axis] / GRAVITY_MSS;

    if (!enabled()) {
        // Bench-test configuration: never gate the integrators, never detect
        // burnout or apogee. Park in BOOST so the fins stay live.
        _stage = Stage::BOOST;
        return;
    }

    switch (_stage) {
    case Stage::PRE_LAUNCH:
        if (debounce(_accel_start_ms, _up_accel_g > _launch_g, _launch_ms)) {
            _stage = Stage::BOOST;
            _accel_start_ms = 0;
        }
        break;

    case Stage::BOOST:
        // Apogee is checked in parallel with burnout: if burnout detection were ever
        // missed, a negative climb rate must still stop the fins. Apogee wins.
        if (debounce(_apogee_start_ms, climb_rate_ms < 0.0f, _apogee_ms)) {
            _stage = Stage::DESCENT;
            break;
        }
        // Burnout is recorded only -- it moves us to COAST but the caller keeps
        // steering. It does not gate control for a finned rocket.
        if (debounce(_accel_start_ms, _up_accel_g < _burnout_g, _burnout_ms)) {
            _stage = Stage::COAST;
            _accel_start_ms = 0;
        }
        break;

    case Stage::COAST:
        if (debounce(_apogee_start_ms, climb_rate_ms < 0.0f, _apogee_ms)) {
            _stage = Stage::DESCENT;
        }
        break;

    case Stage::DESCENT:
        // terminal until reset() on the next arm
        break;
    }
}

#endif  // AP_ROCKET_ENABLED
