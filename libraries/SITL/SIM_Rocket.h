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
    // Mass, motor and inertia are now runtime parameters -- see SIM_RKT_* in
    // SITL.cpp. A new rocket has a new motor and a new airframe, so none of these
    // are compile-time constants any more; SIM_Rocket.cpp reads them each step.

    // ---- aerodynamics ----
    // Moment produced per unit fin deflection per unit dynamic pressure [N m / Pa].
    /*
      FIN GEOMETRY, not a mixing table.

      The simulator is given four servo positions. It must NOT know how the flight
      code combined roll/pitch/yaw into them -- that convention lives in exactly one
      place, AP_FinMixerRocket. Here each fin is treated on its own: it has an angular
      position around the body, it makes a force, and that force makes a moment about
      the CG. The pair differences that look like "mixing" fall out of the geometry.

      Keeping a copy of the mixer here is what caused a real bug: the sim averaged the
      fin pair while using a per-fin gain, and so modelled HALF the tilt authority the
      airframe has.

      Each fin's lift acts TANGENTIALLY (perpendicular to the plane containing the
      body axis and the fin), so for a fin at angle th its force f contributes
          M_x -= f * fin_radius_m                 (spin, force at a radius)
          M_y -= f * fin_arm_m * cos(th)
          M_z -= f * fin_arm_m * sin(th)

      Angles below are chosen to match the airframe's fin numbering (clockwise viewed
      from the nose). Derived from the .ork geometry:
        4 trapezoidal fins, root 305 mm, tip 102 mm, semi-span 102 mm, sweep 178 mm
        -> area 0.0206 m^2, aspect ratio 1.00, CP 2.250 m from the nose
        -> 678 mm behind the mid-burn CG, 92 mm out from the body axis
     */
    float fin_angle_deg[4] = { 270.0f, 180.0f, 90.0f, 0.0f };

    /*
      Derived from the SIM_RKT_* parameters in recompute_fin_geometry(), NOT
      hardcoded. Three independent constants that could disagree with each other
      have become one function of measurable dimensions -- change the tab size and
      the force, arm and radius all move together.
     */
    float fin_force_gain;   // N per Pa per unit command, ONE fin
    float fin_arm_m;        // fin CP behind the CG
    float fin_radius_m;     // fin CP out from the body axis

    // Recompute the above from the SIM_RKT_* geometry. Called at construction and
    // whenever the parameters change, so the sim can be re-geometried at runtime.
    void recompute_fin_geometry();
    float last_geom_hash;   // cheap change detector for the parameters
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
    // 2 calibers of static margin, from the .ork design (fg.4.K-L.reversed).
    //   |Ka| = CN_alpha * A_ref * (x_cp - x_cg) = 12.6 * 0.007707 * 0.198 = 0.0192
    //
    // NOTE the earlier -0.050 came from test.csv, an OLDER revision of the design
    // that showed 3.87 calibers. Do not re-derive this from that CSV: it predates
    // the .ork and describes a different airframe. Static margin drives control
    // authority directly -- the tabs can hold Kf/|Ka| of angle of attack, which is
    // 11.1 deg at 2 calibers versus only 5.7 deg at 3.87.
    float instability_gain = -0.0192f;
    /*
      Axial drag as Cd * reference area [m^2].

      Both numbers were previously wrong, in opposite directions, which is why the
      total looked plausible: Cd was 0.55 and A_ref 0.0082 m^2. The export gives
      Cd = 0.59 subsonic, 0.68 transonic, 0.69 supersonic -- never 0.55 -- and the
      NOTE Cd and A_ref must be a consistent pair. OpenRocket normalises its Cd
      against its OWN reference diameter (4.02 in, CSV col53 -> A_ref 0.008189 m^2),
      NOT the 3.90 in body tube. The default SIM_RKT_DRAGA = 0.005376 is Cd(0.656) x
      that reference area. A Mach-varying Cd would be more accurate; this fixed value
      is why the modelled apogee runs slightly high.
     */
    // Drag area (Cd*A_ref) and rate damping are runtime parameters -- SIM_RKT_DRAGA
    // and SIM_RKT_ROTDAMP. Rate damping is M = ROTDAMP * V * omega, proportional to
    // V not q; see the derivation in the .cpp.

    // Thrust curve, loaded from a REAL RASP .eng file (thrustcurve.org format) named by the
    // SIM_RKT_ENG environment variable. There is NO default/reference curve -- with no motor loaded
    // thrust_n stays 0 and update() refuses to fly. Linear interpolation between breakpoints.
    // (Mass still depletes against SIM_RKT_IMPULSE.)
    static constexpr uint8_t THRUST_MAX = 128;        // max breakpoints a loaded .eng may have
    float thrust_time[THRUST_MAX];
    float thrust_newtons[THRUST_MAX];
    uint8_t thrust_n = 0;                             // active breakpoints (0 = no motor loaded)
    void load_thrust_curve();                         // from SIM_RKT_ENG; thrust_n=0 if unset
    float thrust_at(float t) const;

    // Mass depletes against total impulse (SIM_RKT_IMPULSE), not the integral of the
    // curve, so editing breakpoints cannot silently change the burnt mass.
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
    float rail_speed = 0.0f;         // m/s along the rail while captured
    float rail_dist_m = 0.0f;        // m travelled up the rail since ignition

    // Flight-event narration to the SITL console (apogee, landing), matching the
    // ignition / off-the-rail / burnout notes. Report-once latches, plus the peak
    // altitude so apogee prints the true maximum rather than the first step past it.
    bool  apogee_reported = false;
    bool  landed_reported = false;
    float max_alt_m       = 0.0f;
};

} // namespace SITL
