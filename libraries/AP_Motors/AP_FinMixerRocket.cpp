/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_FinMixerRocket.cpp - motors library for fin-steered rockets
 *
 *       See AP_FinMixerRocket.h for the axis mapping, which is the thing most
 *       likely to be got wrong.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_FinMixerRocket.h"
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

const AP_Param::GroupInfo AP_FinMixerRocket::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // @Param: Q_REF
    // @DisplayName: Rocket fin reference dynamic pressure
    // @Description: The dynamic pressure, in Pascals, at which the attitude gains are tuned. Fin force scales with dynamic pressure, so the mixer scales fin deflection by Q_REF/q to keep the control authority seen by the rate controllers roughly constant across the burn. As a guide, q is about 600 Pa at 30 m/s and about 2500 Pa at 60 m/s at sea level.
    // @Units: Pa
    // @Range: 100 5000
    // @User: Standard
    AP_GROUPINFO("Q_REF", 1, AP_FinMixerRocket, _q_ref, 600.0f),

    // @Param: GAIN_MAX
    // @DisplayName: Rocket fin maximum gain boost
    // @Description: Upper bound on the Q_REF/q gain boost applied at low speed. Without this bound the scheduling diverges as dynamic pressure approaches zero and every fin sits on its stop while the vehicle is still on the rail. This does not create authority that the airflow cannot supply; it only stops the mixer asking for infinite deflection.
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("GAIN_MAX", 2, AP_FinMixerRocket, _gain_max, 4.0f),

    AP_GROUPEND
};

/// Constructor
AP_FinMixerRocket::AP_FinMixerRocket(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz),
    _q_pa(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
    memset(_fin_out, 0, sizeof(_fin_out));
    set_update_rate(speed_hz);
}

void AP_FinMixerRocket::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // fins default to the first four outputs, numbered clockwise viewed from the nose
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_rocketFin1, CH_1);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_rocketFin2, CH_2);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_rocketFin3, CH_3);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_rocketFin4, CH_4);

    SRV_Channels::set_angle(SRV_Channel::k_rocketFin1, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_rocketFin2, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_rocketFin3, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_rocketFin4, SERVO_OUTPUT_RANGE);

    // there is no MAV_TYPE for a rocket upstream
    _mav_type = MAV_TYPE_GENERIC;

    set_initialised_ok(frame_class == MOTOR_FRAME_ROCKET);
}

void AP_FinMixerRocket::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_rocketFin1, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_rocketFin2, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_rocketFin3, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_rocketFin4, speed_hz);
}

uint32_t AP_FinMixerRocket::get_motor_mask()
{
    // the fins are control surfaces, not motors, and the solid motor is not on a
    // PWM output at all, so there is nothing of our own to add here
    return AP_MotorsMulticopter::get_motor_mask();
}

void AP_FinMixerRocket::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    if (_test_fin >= 0) {
        // Pre-arm fin check: drive exactly one fin, centre the rest. This runs
        // while disarmed and deliberately ignores the spool state.
        for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
            _fin_out[i] = (i == (uint8_t)_test_fin) ? _test_deflection : 0.0f;
        }
    } else {
        switch (_spool_state) {
            case SpoolState::SHUT_DOWN:
            case SpoolState::GROUND_IDLE:
                // No authority in these states (the base class sets all the limit
                // flags to match), so centre the fins rather than leave them at
                // whatever the last mix produced.
                for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
                    _fin_out[i] = 0.0f;
                }
                break;
            case SpoolState::SPOOLING_UP:
            case SpoolState::THROTTLE_UNLIMITED:
            case SpoolState::SPOOLING_DOWN:
                // fins are driven by output_armed_stabilizing()
                break;
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_rocketFin1, _fin_out[0] * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rocketFin2, _fin_out[1] * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rocketFin3, _fin_out[2] * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rocketFin4, _fin_out[3] * SERVO_OUTPUT_RANGE);
}

void AP_FinMixerRocket::output_armed_stabilizing()
{
    // Thrust is not ours to command: a solid motor burns as it burns. So there is
    // no throttle term in the mix and no battery/air-density compensation, both of
    // which exist to correct a commanded motor thrust.
    float roll_thrust  = _roll_in + _roll_in_ff;    // view roll  -> tilt
    float pitch_thrust = _pitch_in + _pitch_in_ff;  // view pitch -> tilt
    float yaw_thrust   = _yaw_in + _yaw_in_ff;      // view yaw   -> spin about the long axis

    // Reserve authority for spin before spending it all on tilt. Tilt is the axis
    // that matters, so it wins ties, but leaving no spin authority at all lets the
    // airframe wind up about its long axis.
    const float rp_thrust_max = MAX(fabsf(roll_thrust), fabsf(pitch_thrust));
    float rp_scale = 1.0f;
    if (!is_zero(rp_thrust_max)) {
        rp_scale = constrain_float((1.0f - MIN(fabsf(yaw_thrust), (float)_yaw_headroom * 0.001f)) / rp_thrust_max, 0.0f, 1.0f);
        if (rp_scale < 1.0f) {
            limit.roll = true;
            limit.pitch = true;
        }
    }

    const float yaw_allowed = 1.0f - rp_scale * rp_thrust_max;
    if (fabsf(yaw_thrust) > yaw_allowed) {
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // Four fins, numbered clockwise viewed from the nose. Opposing pairs deflect
    // antisymmetrically to tilt; all four deflect together to spin.
    float fin[AP_FIN_MIXER_ROCKET_NUM_FINS];
    fin[0] =  rp_scale * roll_thrust  - yaw_thrust;
    fin[1] =  rp_scale * pitch_thrust - yaw_thrust;
    fin[2] = -rp_scale * roll_thrust  - yaw_thrust;
    fin[3] = -rp_scale * pitch_thrust - yaw_thrust;

    /*
      Schedule the gains on dynamic pressure.

      Fin force is roughly proportional to deflection times q, so a deflection
      tuned at _q_ref produces the demanded moment at other speeds only if it is
      scaled by _q_ref/q. NOTE this is the opposite of AP_MotorsSingle, which
      divides by thrust: its vanes sit in the prop wash, where authority tracks
      thrust. Rocket fins sit in freestream, where authority tracks airspeed, and
      the solid motor's thrust says nothing about how fast the vehicle is moving.

      The boost is bounded by _gain_max. Below that q the fins genuinely lack
      authority and no amount of deflection will supply it; the launch rail is
      what holds attitude until the vehicle is moving.
     */
    float gain = _gain_max;
    if (is_positive(_q_pa) && is_positive(_q_ref)) {
        gain = constrain_float(_q_ref / _q_pa, 0.0f, _gain_max);
    }
    for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
        fin[i] *= gain;
    }

    // If any fin is past its stop, scale the whole mix back rather than clip one
    // fin and silently distort the commanded moment. All four fins serve all three
    // axes, so honest saturation means reporting all three as limited: these flags
    // feed straight back into AC_PID::update_all() to stop the I terms winding up
    // against deflection the vehicle cannot produce.
    float fin_max = 0.0f;
    for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
        fin_max = MAX(fin_max, fabsf(fin[i]));
    }
    if (fin_max > 1.0f) {
        const float unsaturate = 1.0f / fin_max;
        for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
            fin[i] *= unsaturate;
        }
        limit.set_rpy(true);
    }

    for (uint8_t i = 0; i < AP_FIN_MIXER_ROCKET_NUM_FINS; i++) {
        _fin_out[i] = constrain_float(fin[i], -1.0f, 1.0f);
    }

    // No commanded thrust exists to report. Anything reading throttle out of this
    // mixer (notch tracking, logging) should see the truth rather than a fiction.
    _throttle_out = 0.0f;
}

// output_test_seq - deflect a fin to the pwm value specified
//  motor_seq is 1 to 4, matching the fin numbering
void AP_FinMixerRocket::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    switch (motor_seq) {
        case 1:
            SRV_Channels::set_output_pwm(SRV_Channel::k_rocketFin1, pwm);
            break;
        case 2:
            SRV_Channels::set_output_pwm(SRV_Channel::k_rocketFin2, pwm);
            break;
        case 3:
            SRV_Channels::set_output_pwm(SRV_Channel::k_rocketFin3, pwm);
            break;
        case 4:
            SRV_Channels::set_output_pwm(SRV_Channel::k_rocketFin4, pwm);
            break;
        default:
            break;
    }
}
