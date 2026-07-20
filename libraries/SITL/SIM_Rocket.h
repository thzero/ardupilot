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
    /*
      ---- airframe and motor ----

      These are derived from a real OpenRocket export (see ARDUROCKET_PLAN.md §9),
      not invented. Note that export mixes units: lengths/speeds imperial, thrust in
      newtons, mass in ounces. Everything below is SI.

      Summary of the modelled vehicle: ~11.2 kg off the pad, M-class motor
      (~5400 N.s total impulse), 4.02 in diameter, Mach 1.2, ~3900 m apogee.
     */
    float dry_mass = 8.47f;         // kg, burnt out
    float propellant_mass = 2.72f;  // kg actually consumed (11.19 -> 8.47)
    float burn_time = 4.70f;        // s to zero thrust
    float ignition_delay = 3.0f;    // s after arming, i.e. the launch controller

    // ---- inertia ----
    // Slender body: hard to pitch/yaw, easy to spin. Estimated from mass and an
    // assumed ~1.5 m length (m*L^2/12); the export did not carry usable MOI columns.
    float inertia_tilt = 1.80f;     // kg m^2 about body Y and Z
    float inertia_spin = 0.015f;    // kg m^2 about body X (0.5*m*r^2, r=51mm)

    // ---- aerodynamics ----
    // Moment produced per unit fin deflection per unit dynamic pressure [N m / Pa].
    float fin_moment_gain = 0.0035f;
    // Fin authority about the long axis is far weaker than about the tilt axes:
    // spinning the airframe uses the fins edge-on.
    float fin_spin_gain = 0.0008f;
    /*
      Pitch/yaw moment per radian of angle of attack per unit dynamic pressure
      [N m / (Pa rad)].

      SIGN CONVENTION: positive = centre of pressure AHEAD of centre of gravity,
      i.e. the airframe diverges and must be actively held. NEGATIVE = CP aft of CG,
      i.e. passively STABLE and self-correcting.

      The real airframe is strongly stable -- the export shows CP roughly 3.5 to 5
      calibers aft of CG -- so the default is negative. That is a materially easier
      plant than the unstable one this model originally assumed: the fins assist a
      self-correcting airframe rather than fighting a diverging one.
        magnitude ~ Cn_alpha * A_ref * (x_cp - x_cg)
                  ~ 12 /rad * 0.0082 m^2 * 0.51 m
     */
    float instability_gain = -0.050f;
    float drag_area = 0.0045f;      // Cd(0.55) * A_ref(0.0082 m^2), subsonic
    float rot_damping = 0.35f;      // aerodynamic rate damping [N m / (Pa rad/s)]

    // Thrust curve, from the export. Linear interpolation between breakpoints.
    static constexpr uint8_t THRUST_PTS = 14;
    static const float thrust_time[THRUST_PTS];
    static const float thrust_newtons[THRUST_PTS];
    float thrust_at(float t) const;

    // Total impulse of the curve above, ~M-class. Mass depletes against this.
    static constexpr float TOTAL_IMPULSE_NS = 5414.0f;
    float impulse_used = 0.0f;

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
