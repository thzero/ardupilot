/// @file	AP_FinMixerRocket.h
/// @brief	Fin mixer for ArduRocket. Drives four steering fins; drives no motor.
#pragma once

/*
  WHY THIS FILE IS NAMED AND SHAPED THE WAY IT IS
  ===============================================

  This class drives FOUR AERODYNAMIC FINS. It does not drive a motor at all --
  ArduRocket flies a solid rocket motor that the flight controller cannot command.
  The name says "FinMixer" rather than "AP_MotorsRocket" so that is obvious at a
  glance, because the code reads oddly otherwise.

  It nevertheless lives in AP_Motors/ and inherits AP_MotorsMulticopter. That is
  deliberate and not negotiable:

    * AC_AttitudeControl_Multi -- Copter's attitude controller, and the entire
      reason this vehicle can exist without forking ArduPlane -- takes an
      AP_MotorsMulticopter& in its constructor. To reuse that controller, the fin
      mixer must BE an AP_Motors subclass.
    * AP_Motors is ArduPilot's generic actuator-mixer layer, not literally a motor
      driver: AP_Motors6DOF drives Sub's thrusters, AP_MotorsSingle drives one
      motor plus four control vanes, AP_MotorsTailsitter drives two motors plus two
      tilt servos. "Mixer" is the right mental model.
    * Inheriting it also brings the spool state machine, servo slew limiting and
      output plumbing for free.

  Consequence, and the main wart when reading this file: we inherit throttle
  machinery that is meaningless here -- thrust linearisation, battery voltage
  compensation, throttle mixing. _throttle_in is ignored and _throttle_out is
  forced to zero. Do not try to make the throttle path mean anything.

  WHY NOT AP_MotorsTailsitter?
    It gets roll authority from DIFFERENTIAL THRUST between two throttled motors
    (_thrust_left/_thrust_right), which a single solid motor cannot provide, and it
    drives only two servos where we have four fins. We would lose an axis and half
    the actuators.

  WHY NOT AP_MotorsSingle?
    Closest existing analogue (one motor + four vanes) and its four-actuator
    geometry IS reused below. Not subclassed because its init() maps six motor
    channels we do not have, and its gain scheduling divides by THRUST -- correct
    for vanes sitting in prop wash, wrong for fins in freestream, which scale with
    dynamic pressure instead. It also carries a saturation bug (see the .cpp).
*/

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMulticopter.h"

#define AP_FIN_MIXER_ROCKET_NUM_FINS 4

/// @class      AP_FinMixerRocket
///
/// Mixer for a rocket held vertical by four steering fins under a solid motor.
///
/// Axis mapping. The vehicle runs the attitude controller on an
/// AP_AHRS_View(ROTATION_PITCH_90), which presents the nose-up airframe to the
/// controller as though it were a level multicopter. The axes therefore remap:
///
///     view roll   -> moment about body Z -> TILT away from vertical
///     view pitch  -> moment about body Y -> TILT away from vertical
///     view yaw    -> moment about body X -> SPIN about the long axis
///
/// So "hold vertical" is the view roll and pitch loops, and view yaw is the spin
/// the vehicle does not especially care about. Four fins have authority on all
/// three: opposing pairs deflect antisymmetrically to tilt, and all four deflect
/// together to spin. This mapping must be confirmed on the bench (tilt the
/// airframe; the fins must move to push it back toward vertical) before flight.
///
/// Thrust is NOT controlled. A solid motor burns as it burns, so the throttle
/// input from AC_AttitudeControl is ignored here and the inherited thrust
/// linearisation and battery compensation are unused. The class still derives
/// from AP_MotorsMulticopter because AC_AttitudeControl_Multi requires one, and
/// because the spool state machine is a useful armed/disarmed gate.
///
/// Fin authority scales with dynamic pressure, not thrust: the fins sit in
/// freestream, unlike AP_MotorsSingle's vanes which sit in prop wash. The vehicle
/// supplies q via set_dynamic_pressure() and the mixer divides by it, which
/// linearises the plant across the burn. At q below the reference the fins simply
/// run out of authority; that is physical, and it is why the launch rail has to
/// hold attitude until the vehicle is moving.
class AP_FinMixerRocket : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_FinMixerRocket(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class and type
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}

    // set update rate to the fin servos - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

    // output_to_motors - sends output to the fin servos
    void output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used
    uint32_t get_motor_mask() override;

    // output_motor_mask is meaningless with no controllable motor
    void output_motor_mask(float thrust, uint32_t mask, float rudder_dt) override {};

    /*
      Set the current dynamic pressure q = 1/2 rho v^2, in Pa, used to schedule
      fin gains. The vehicle computes this from the EKF velocity (baro + IMU).
      Must be called before output(); if it is never called the mixer falls back
      to the reference q, i.e. no scheduling.
     */
    void set_dynamic_pressure(float q_pa) { _q_pa = q_pa; }

    // fin deflection commands, -1..1, for logging
    const float *fin_out() const { return _fin_out; }

    /*
      Drive ONE fin directly, for the pre-arm fin check. Bypasses the attitude
      controller and the spool state, because the vehicle is deliberately disarmed
      while the pad crew watches the fins move.

      fin: 0..3 to drive that fin, or negative to leave test mode and return to
      normal mixing. deflection is -1..1. Every other fin is centred, so exactly
      one fin moves at a time and a swapped output channel is visible.
     */
    void set_fin_test(int8_t fin, float deflection) {
        _test_fin = fin;
        _test_deflection = deflection;
    }
    bool fin_test_active() const { return _test_fin >= 0; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // calculate fin outputs
    void output_armed_stabilizing() override;

    const char* _get_frame_string() const override { return "ROCKET"; }

    // deflect a fin at the pwm value specified
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // reference dynamic pressure, Pa. Gains are tuned at this q; the mixer scales
    // deflection by (_q_ref / q) so authority stays roughly constant across the burn.
    AP_Float _q_ref;

    // largest gain boost permitted at low q. Without this the 1/q scheduling runs
    // away as q -> 0 on the rail and every fin sits on its stop.
    AP_Float _gain_max;

    float _q_pa;                                // current dynamic pressure, Pa
    float _fin_out[AP_FIN_MIXER_ROCKET_NUM_FINS];  // fin deflection, -1..1

    // pre-arm fin check: which fin to drive (negative = not testing) and how far
    int8_t _test_fin = -1;
    float  _test_deflection = 0.0f;
};
