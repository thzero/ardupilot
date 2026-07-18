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
  fin-steered solid-motor rocket simulator class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  A hobby rocket on a solid motor, steered by four fins.

  This plant differs from every other model in SITL in three ways that matter,
  and they are the reasons it has to exist rather than borrowing SIM_SingleCopter:

  1. Control authority comes from dynamic pressure, not thrust. The fins sit in
     freestream, so they have NO authority on the pad and very little in the first
     moments off the rail, then far too much at burnout speed.
  2. The motor is not commandable. It ignites, burns for a couple of seconds, and
     stops. Thrust cannot be traded against attitude, and after burnout the vehicle
     is a lawn dart with control surfaces.
  3. The airframe is aerodynamically UNSTABLE by default (centre of pressure ahead
     of centre of gravity). Left alone it tumbles; the whole job of the controller
     is to hold it straight. Every other SITL model is passively stable.

  Ignition is modelled as a ground launch controller: the motor fires a fixed
  delay after arming, because ArduPilot does not control the igniter.
 */
class Rocket : public Aircraft {
public:
    Rocket(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Rocket(frame_str);
    }

private:
    // ---- motor ----
    float dry_mass = 0.80f;         // kg, airframe without propellant
    float propellant_mass = 0.20f;  // kg, burnt linearly over burn_time
    float motor_thrust = 90.0f;     // N, roughly a mid-power hobby motor
    float burn_time = 2.0f;         // s
    float ignition_delay = 3.0f;    // s after arming, i.e. the launch controller

    // ---- inertia ----
    // A slender rocket: hard to pitch/yaw, trivially easy to spin.
    float inertia_tilt = 0.083f;    // kg m^2 about body Y and Z (m*L^2/12, L~1m)
    float inertia_spin = 0.0015f;   // kg m^2 about body X

    // ---- aerodynamics ----
    // Moment produced per unit fin deflection per unit dynamic pressure [N m / Pa].
    float fin_moment_gain = 0.0035f;
    // Fin authority about the long axis is far weaker than about the tilt axes:
    // spinning the airframe uses the fins edge-on.
    float fin_spin_gain = 0.0008f;
    /*
      Destabilising moment per radian of angle of attack per unit dynamic pressure
      [N m / (Pa rad)]. POSITIVE means centre of pressure ahead of centre of
      gravity, i.e. the airframe diverges from straight flight and must be actively
      held. Set negative via the frame string for a passively stable airframe.
     */
    float instability_gain = 0.0060f;
    float drag_area = 0.0015f;      // Cd * frontal area, m^2
    float rot_damping = 0.05f;      // aerodynamic rate damping [N m / (Pa rad/s)]

    /*
      ---- launch rail ----

      Rails are routinely tilted a few degrees, into wind or away from the crowd
      (NAR and Tripoli both cap it at 20). Modelling that matters because the
      vehicle captures the rail attitude at arming and holds THAT rather than true
      vertical, and because the rail guides the airframe through exactly the phase
      where the fins have no authority.

      Selected from the frame string, e.g. "rocket-tilt10" or "rocket-tilt10-az90".
     */
    float rail_tilt_deg    = 0.0f;   // angle off vertical
    float rail_azimuth_deg = 0.0f;   // which way it leans

    /*
      Guided travel before the airframe is free. Defaults to 72 inches (1.8288 m),
      the rail this project actually launches from. Override in inches from the
      frame string, e.g. "rocket-tilt10-rail48".

      This length is not a detail: it sets the rail-exit velocity, and rail-exit
      velocity is what determines how much fin authority exists at the single worst
      moment of the flight -- the instant the airframe becomes free with the lowest
      dynamic pressure it will ever have while moving.
     */
    float rail_length_m    = 1.8288f;
    Matrix3f rail_dcm;               // body->NED attitude while on the rail

    // ---- state ----
    uint32_t arm_time_ms;
    float burn_elapsed;
    bool ignited;
    bool left_rail = false;
    Vector3d rail_start_pos;
};

} // namespace SITL
