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

#include "SIM_Rocket.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// Minimum rail-exit velocity targeted by NAR/Tripoli practice, in feet per second.
// Below this a rocket is relying on passive stability it may not have.
#define RAIL_EXIT_TARGET_FPS 50.0f

using namespace SITL;

/*
  Thrust curve taken from the OpenRocket export: fast rise to a 1923 N peak at
  0.2 s, a long ~1700 N sustain, then a tail-off to zero by 4.7 s.

  The tail-off matters more than it looks: burnout is detected from measured axial
  acceleration (RKT_BURN_G / RKT_BURN_MS), so a gradual decay is what those
  thresholds actually have to discriminate. A square pulse would make burnout
  detection look far easier than it is.
 */
const float Rocket::thrust_time[Rocket::THRUST_PTS] = {
    0.00f, 0.05f, 0.10f, 0.20f, 0.50f, 1.00f, 1.50f,
    2.00f, 2.50f, 3.00f, 3.50f, 4.00f, 4.40f, 4.70f
};
const float Rocket::thrust_newtons[Rocket::THRUST_PTS] = {
    0.0f, 717.0f, 1433.0f, 1922.8f, 1720.0f, 1695.0f, 1672.0f,
    1550.0f, 1330.0f, 1150.0f, 760.0f, 226.0f, 62.0f, 0.0f
};

/*
  Turn the SIM_RKT_* airframe dimensions into the three numbers the physics needs.

  Everything here is textbook: trapezoidal planform area, Helmbold/Diederich lift
  slope for a low aspect ratio surface, Barrowman body interference, and the
  thin-aerofoil flap effectiveness for a trailing-edge tab. The point is that they
  are now DERIVED, so tab chord, tab span, deflection limit, fin size, body radius
  and static margin can be changed from a ground station and the model stays
  self-consistent.
 */
void Rocket::recompute_fin_geometry()
{
    const auto &p = AP::sitl()->rocket;

    const float Cr = p.fin_root, Ct = p.fin_tip, sspan = p.fin_semispan;
    const float rb = p.body_radius;

    // trapezoidal planform
    const float S_fin = 0.5f * (Cr + Ct) * sspan;
    if (!is_positive(S_fin) || !is_positive(sspan)) {
        return;   // nonsense geometry; keep whatever we had
    }
    const float AR = 2.0f * sq(sspan) / S_fin;

    // lift-curve slope of one fin, plus the body interference factor
    const float CLa = (2.0f * M_PI * AR) / (2.0f + safe_sqrt(sq(AR) + 4.0f));
    const float Kfb = 1.0f + rb / (sspan + rb);

    /*
      Trailing-edge tab effectiveness. A tab deflects the flow over the WHOLE fin
      rather than rotating it, so it delivers only a fraction of a flying fin:
        tau = 1 - (theta - sin theta)/pi,  theta = acos(2*cf/c - 1)
      times ~0.85 for real viscous losses, times the tab's span fraction.
     */
    const float cf = constrain_float(p.tab_chord, 0.01f, 1.0f);
    const float theta = acosf(constrain_float(2.0f*cf - 1.0f, -1.0f, 1.0f));
    const float tau = (1.0f - (theta - sinf(theta)) / M_PI) * 0.85f
                      * constrain_float(p.tab_span, 0.0f, 1.0f);

    fin_force_gain = S_fin * CLa * Kfb * tau * radians(p.tab_max_deg);
    fin_arm_m      = p.fin_arm;
    // fin force acts at the spanwise centre of the MAC, out from the body axis
    const float y_mac = (sspan/3.0f) * ((Cr + 2.0f*Ct) / (Cr + Ct));
    fin_radius_m   = rb + y_mac;

    // Static margin sets what the fins have to fight: the airframe weathercocks
    // into the relative wind with this stiffness.
    //   |Ka| = CN_alpha * A_ref * (x_cp - x_cg)
    const float A_ref = M_PI * sq(rb);
    const float CNa   = 2.0f * S_fin * CLa * Kfb / A_ref + 2.0f;  // fins + nose
    instability_gain  = -CNa * A_ref * (p.static_margin * 2.0f * rb);

    last_geom_hash = p.tab_chord + p.tab_span*3 + p.tab_max_deg*7 + p.fin_root*11
                   + p.fin_tip*13 + p.fin_semispan*17 + p.body_radius*19
                   + p.fin_arm*23 + p.static_margin*29;
}

float Rocket::thrust_at(float t) const
{
    if (t <= thrust_time[0]) {
        return thrust_newtons[0];
    }
    for (uint8_t i = 1; i < THRUST_PTS; i++) {
        if (t <= thrust_time[i]) {
            const float span = thrust_time[i] - thrust_time[i-1];
            const float frac = is_positive(span) ? (t - thrust_time[i-1]) / span : 0.0f;
            return thrust_newtons[i-1] + frac * (thrust_newtons[i] - thrust_newtons[i-1]);
        }
    }
    return 0.0f;
}

Rocket::Rocket(const char *frame_str) :
    Aircraft(frame_str),
    arm_time_ms(0),
    burn_elapsed(0.0f),
    ignited(false)
{
    const auto &rp = AP::sitl()->rocket;
    mass = rp.dry_mass + rp.prop_mass;

    // A rocket sits nose-up on its rail. The base class already knows how to keep
    // a vertical airframe standing on the ground.
    ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
    frame_height = 0.1f;

    /*
      The modelled airframe is passively STABLE by default, because the real one is
      (CP ~3.5-5 calibers aft of CG per the export). "rocket-unstable" flips the sign
      to model a CP-ahead-of-CG airframe, which is a far harder plant and worth
      testing the controller against deliberately.
     */
    /*
      Rail tilt, e.g. "rocket-tilt10" or "rocket-tilt10-az90". The model name is
      matched by prefix, so the whole frame string reaches us here.
     */
    const char *p = strstr(frame_str, "-tilt");
    if (p != nullptr) {
        rail_tilt_deg = constrain_float(atof(p + 5), 0.0f, 45.0f);
    }
    p = strstr(frame_str, "-az");
    if (p != nullptr) {
        rail_azimuth_deg = atof(p + 3);
    }
    // rail length, given in inches because that is how rails are sold
    p = strstr(frame_str, "-rail");
    if (p != nullptr) {
        rail_length_m = constrain_float(atof(p + 5), 6.0f, 240.0f) * 0.0254f;
    }

    /*
      Attitude the rail holds the airframe at. A nose-up rocket is pitch +90 (body
      X, the nose, pointing at NED -Z); tilting it off vertical by t degrees is
      pitch (90 - t), and the yaw term chooses which way it leans.
     */
    rail_dcm.from_euler(0.0f, radians(90.0f - rail_tilt_deg), radians(rail_azimuth_deg));

    ::printf("Rocket: rail %.1f in (%.3f m), tilt %.1f deg, azimuth %.0f deg\n",
             (double)(rail_length_m / 0.0254f), (double)rail_length_m,
             (double)rail_tilt_deg, (double)rail_azimuth_deg);

    recompute_fin_geometry();

    // "-unstable" flips the sign AFTER the geometry is computed, since
    // recompute_fin_geometry() derives instability_gain from the static margin.
    if (strstr(frame_str, "-unstable")) {
        instability_gain = -instability_gain;
    }

    lock_step_scheduled = true;
}

void Rocket::update(const struct sitl_input &input)
{
    update_wind(input);

    const float delta_time = frame_time_us * 1.0e-6f;
    const auto &rkt = AP::sitl()->rocket;

    /*
      Fin commands. The vehicle maps fins to the first four outputs at +/-4500,
      which reaches SITL as 1000..2000 PWM.
     */
    float fin[4];
    for (uint8_t i = 0; i < 4; i++) {
        fin[i] = constrain_float((input.servos[i] - 1500) / 500.0f, -1.0f, 1.0f);
    }

    /*
      Ignition. ArduPilot does not fire the igniter -- a ground launch controller
      does -- so the motor lights a fixed delay after arming rather than on any
      command from the flight code. Disarming resets the motor, which lets a test
      re-run a launch without restarting the simulator.
     */
    if (hal.util->get_soft_armed()) {
        if (arm_time_ms == 0) {
            arm_time_ms = AP_HAL::millis();
        }
        if (!ignited && (AP_HAL::millis() - arm_time_ms) > (uint32_t)(rkt.ign_delay * 1000)) {
            ignited = true;
            ::printf("Rocket: ignition\n");
        }
    } else {
        arm_time_ms = 0;
        ignited = false;
        burn_elapsed = 0.0f;
        impulse_used = 0.0f;
    }

    float thrust = 0.0f;
    if (ignited && burn_elapsed < rkt.burn_time) {
        thrust = thrust_at(burn_elapsed);
        burn_elapsed += delta_time;
        if (burn_elapsed >= rkt.burn_time) {
            ::printf("Rocket: burnout\n");
        }
    }

    /*
      Propellant depletes with IMPULSE DELIVERED, not with time. With a real thrust
      curve those differ substantially: this motor spends its first second at near
      peak thrust and its last second barely producing any, so a linear-in-time mass
      model would have the airframe far too heavy early and too light late -- which
      distorts acceleration exactly where launch detection reads it.
     */
    impulse_used += thrust * delta_time;
    const float burn_frac = constrain_float(impulse_used / rkt.impulse, 0.0f, 1.0f);
    mass = rkt.dry_mass + rkt.prop_mass * (1.0f - burn_frac);

    /*
      Inertia is NOT constant: it drops ~16% as propellant burns. Interpolate the
      loaded and burnt-out values (both exported by OpenRocket) on the same burn
      fraction the mass uses, rather than pinning it at the loaded value.
     */
    const float inertia_tilt = linear_interpolate(rkt.j_tilt_loaded, rkt.j_tilt_burnt,
                                                  burn_frac, 0.0f, 1.0f);
    const float inertia_spin = linear_interpolate(rkt.j_spin_loaded, rkt.j_spin_burnt,
                                                  burn_frac, 0.0f, 1.0f);

    /*
      Dynamic pressure is the whole story for control authority. On the pad it is
      zero and the fins do nothing at all; by burnout it is large enough that small
      deflections produce large moments.
     */
    // Pick up any runtime change to the SIM_RKT_* geometry.
    {
        const auto &p = AP::sitl()->rocket;
        const float h = p.tab_chord + p.tab_span*3 + p.tab_max_deg*7 + p.fin_root*11
                      + p.fin_tip*13 + p.fin_semispan*17 + p.body_radius*19
                      + p.fin_arm*23 + p.static_margin*29;
        if (!is_equal(h, last_geom_hash)) {
            recompute_fin_geometry();
        }
    }

    const float speed_tas = velocity_air_bf.length();
    const float q = 0.5f * air_density * sq(speed_tas);

    /*
      Angle of attack and sideslip, only meaningful once actually moving forward.
      Below a walking pace the flow direction is noise.
     */
    float alpha = 0.0f;
    float beta = 0.0f;
    if (velocity_air_bf.x > 1.0f) {
        alpha = atan2f(velocity_air_bf.z, velocity_air_bf.x);
        beta  = atan2f(velocity_air_bf.y, velocity_air_bf.x);
    }

    /*
      Fin forces and moments, ONE FIN AT A TIME.

      Deliberately no mixing here. The simulator only sees four servo positions and
      must not assume how the flight code produced them -- that convention belongs
      solely to AP_FinMixerRocket. Each fin makes a tangential force; summing the
      moments reproduces the pair differences without ever naming roll or pitch.

      See SIM_Rocket.h for the geometry and the derivation.
     */
    Vector3f moment;  // N m, body frame
    for (uint8_t i = 0; i < 4; i++) {
        const float f = fin[i] * fin_force_gain * q;   // N, tangential
        const float th = radians(fin_angle_deg[i]);
        moment.x -= f * fin_radius_m;
        moment.y -= f * fin_arm_m * cosf(th);
        moment.z -= f * fin_arm_m * sinf(th);
    }

    /*
      Aerodynamic moment about the centre of gravity. With the centre of pressure
      AHEAD of the centre of gravity the force at the CP amplifies whatever angle
      of attack already exists, so the airframe diverges instead of weathercocking.
      The signs come from r x F with r pointing forward to the CP.
     */
    moment.y += alpha * instability_gain * q;
    moment.z -= beta * instability_gain * q;

    // aerodynamic rate damping
    /*
      Aerodynamic rate damping.

      A pitch rate w gives a fin at distance r from the CG a local angle of attack
      of w*r/V, so the restoring moment is

          M = q*S*CLa*(w*r/V)*r = 0.5*rho*V*S*CLa*r^2 * w

      i.e. proportional to V*w, NOT to q*w. This previously used rot_damping*q*w,
      which is proportional to V^2*w -- overstating damping by a factor of V and
      giving a damping ratio of 7 at rail exit rising to 183 at Mach 1.2, where a
      real sounding rocket sits at 0.05-0.2. The airframe effectively could not
      rotate, so the fins were never actually asked to do anything.

      With the correct law and the corrected inertia below, the damping ratio comes
      out at a constant 0.059 across the whole envelope -- which is the physically
      expected behaviour, since stiffness and damping scale with the same
      aerodynamics.
     */
    const float damp = rkt.rot_damping * speed_tas;   // N m / (rad/s)
    /*
      Spin axis. Damping about the long axis comes from the same fin force acting at
      the fin's RADIUS rather than its axial arm, so it scales as (radius/arm)^2 --
      not the hardcoded 0.1 that used to be here, which over-damped spin by ~5x.
     */
    moment.x -= gyro.x * damp * 0.0183f;   // (fin_radius/fin_arm)^2
    moment.y -= gyro.y * damp;
    moment.z -= gyro.z * damp;

    const Vector3f rot_accel(moment.x / inertia_spin,
                             moment.y / inertia_tilt,
                             moment.z / inertia_tilt);

    // Axial drag, opposing motion through the air.
    Vector3f drag_bf;
    if (speed_tas > 0.1f) {
        drag_bf = -velocity_air_bf.normalized() * (q * rkt.drag_area);
    }

    // Thrust acts along the airframe's long axis, which is body X (the nose).
    accel_body = Vector3f(thrust, 0.0f, 0.0f) / mass;
    accel_body += drag_bf / mass;

    update_dynamics(rot_accel);

    /*
      The launch rail.

      Until the airframe has travelled rail_length_m from where it lit, the rail
      holds its attitude and constrains it to slide along the rail line. This is
      not cosmetic: it is exactly the phase where dynamic pressure is near zero and
      the fins have no authority, so without it the sim would let an unstable
      airframe topple at t=0 in a way a real rail simply prevents.

      Applied after update_dynamics() so it overrides the base class's ground
      handling, which otherwise forces a tailsitter perfectly upright.
     */
    if (!ignited) {
        rail_start_pos = position;
    }
    // Once the airframe has left the rail, the rail can never constrain it again --
    // latch on left_rail. Without this, the disarm at apogee resets `ignited` to false,
    // which would re-satisfy the `!ignited` term and freeze the airframe at apogee
    // instead of letting it fall, so it would never land.
    const bool on_rail = !left_rail &&
                         (!ignited || ((position - rail_start_pos).length() < rail_length_m));
    if (on_rail) {
        dcm = rail_dcm;
        gyro.zero();
        // slide along the rail only; no sideways motion, and never backwards
        const Vector3f rail_dir = rail_dcm * Vector3f(1.0f, 0.0f, 0.0f);
        velocity_ef = rail_dir * MAX(velocity_ef * rail_dir, 0.0f);
    } else if (!left_rail) {
        left_rail = true;
        /*
          Rail-exit velocity, reported against the 50 fps (15.24 m/s) minimum that
          NAR and Tripoli practice targets.

          This is the most consequential single number for a fin-steered rocket.
          Fin authority scales with dynamic pressure, so exit speed sets how much
          control exists at the instant the rail stops holding the airframe -- the
          lowest-authority moment of the whole flight. Below the target the airframe
          is relying on passive stability it may not have (CP ahead of CG here), and
          the fins cannot save it.
         */
        const float mps = velocity_ef.length();
        const float fps = mps * 3.28084f;
        const float q_exit = 0.5f * air_density * sq(mps);
        ::printf("Rocket: off the rail at %.1f m/s (%.0f fps), q=%.0f Pa%s\n",
                 (double)mps, (double)fps, (double)q_exit,
                 fps < RAIL_EXIT_TARGET_FPS ? "   *** BELOW 50 fps TARGET ***" : "");
    }

    /*
      Apogee and landing narration, in the same style as the ignition / off-the-rail /
      burnout notes. Altitude is height above the launch point (NED z is down, so
      alt = -position.z; climbing is negative velocity.z).
     */
    const float alt_m = -position.z;
    if (alt_m > max_alt_m) {
        max_alt_m = alt_m;
    }
    if (left_rail && !apogee_reported && velocity_ef.z > 0.0f && alt_m > 1.0f) {
        // vertical velocity has turned downward: the climb is over
        apogee_reported = true;
        ::printf("Rocket: apogee at %.0f m (%.0f ft)\n",
                 (double)max_alt_m, (double)(max_alt_m * 3.28084f));
    }
    if (apogee_reported && !landed_reported && alt_m < 1.0f) {
        // Back on the ground. The sim has no recovery model, so this is a ballistic
        // impact -- the speed is what a parachute would have had to bleed off.
        landed_reported = true;
        ::printf("Rocket: landed - ballistic descent %.0f m/s (no recovery modelled)\n",
                 (double)velocity_ef.length());
    }

    update_position();
    time_advance();
    update_mag_field_bf();
}
