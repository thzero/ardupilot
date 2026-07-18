# AP_Rocket

Vehicle-agnostic flight-stage detection for **autonomous vertical-hold rocket flight**.

It holds the rocket-specific *brains* — launch, burnout and apogee detection, plus the integrator-gating
policy — behind a small API, along with the `RKT_*` parameters. It is driven by the **ArduRocket**
vehicle (`ArduRocket/rocket_control.cpp`). Same structural pattern as `AP_Soaring`, `AP_Landing`,
`AP_ICEngine`: a self-contained library run by a thin vehicle-side hook.

> **History:** an earlier version of this library drove a `QROCKET` flight mode inside ArduPlane, and
> this README argued a standalone vehicle was infeasible. That claim did not survive inspection — see
> "Why a separate vehicle after all" below. QROCKET has been removed; ArduRocket replaces it.

## Goal

Keep a hobby rocket flying **vertically** (attitude hold to 0° tilt) on **four steering fins**, under a
**solid motor** (no thrust control), with **no RC transmitter and no ground station**. Holding vertical is
*stabilization*, not guidance-to-a-target.

Motor burn times vary widely: typically **0.8–8 s**, with outliers to **16 s**. Nothing here assumes a
fixed or short burn.

## The flight stages

```
PRE_LAUNCH -> on the rail. Rate integrators held: the fins have no airflow and
              cannot move the vehicle, so any I-term would wind up against the
              rail and dump into the fins the instant it flies. P/D stay live.
BOOST      -> launch detected (body-up accel over RKT_LAUNCH_G). Full authority.
COAST      -> burnout detected. RECORDED ONLY -- this drives no control change.
              Aerodynamic fins still have airflow while the rocket is ascending,
              so the caller keeps steering through COAST.
DESCENT    -> apogee (climb rate negative). Steering MUST cease: there is no
              upward airflow left to steer with, and nothing should flail on the
              way down.
```

**The important subtlety: burnout does not stop the fins — apogee does.** Fin tabs work off airflow, so
they keep authority after the motor quits, right up to apogee. A **control vane** (CV, a jet vane in the
exhaust) is the exception: it loses authority at burnout, because it works off the exhaust rather than
freestream. This library is fins-first, so burnout is merely reported; a CV airframe would additionally
stop at `burnt_out()`.

Apogee is checked in parallel with burnout during BOOST, so a missed burnout can never leave the fins
running on the way down.

## API

```cpp
void  update(const Vector3f &accel_body, float climb_rate_ms);  // feed every loop
Stage stage() const;              // PRE_LAUNCH / BOOST / COAST / DESCENT
bool  steering_active() const;    // BOOST or COAST -- keep flying the fins
bool  hold_integrators() const;   // PRE_LAUNCH -- gate the I terms
bool  burnt_out() const;          // informational; drives nothing for fins
bool  descending() const;         // apogee passed; stop
void  reset();                    // call on arm
```

Inputs are plain (a body-frame acceleration and a climb rate); outputs are plain decisions. The library
never references a vehicle type, so it stays portable.

## Parameters

| Param | Default | Purpose |
|---|---|---|
| `RKT_ENABLE` | 0 | Enable stage detection. When disabled the machine parks in BOOST: fins always live, no gating, no shutdown — the bench-test configuration. |
| `RKT_LAUNCH_G` | 1.5 | Launch threshold in g. Must sit above the stationary 1 g reading and below expected launch accel. |
| `RKT_LAUNCH_AX` | 0 | Body up-axis: 0=X, 1=Y, 2=Z. |
| `RKT_LAUNCH_MS` | 50 | Launch must hold this long. Guards against a single noisy sample on a vibrating pad. |
| `RKT_BURN_G` | 0.2 | Burnout threshold in g (coasting reads near zero — drag only). |
| `RKT_BURN_MS` | 100 | Burnout debounce, so a mid-burn thrust dip is not read as burnout. |
| `RKT_APOG_MS` | 500 | Apogee debounce. This is what stops the fins, so it must be robust to velocity-estimate noise near the top. |

## No GPS

The vehicle flies on **barometer + IMU only**. Climb rate (for apogee, and for the fin gain scheduling)
comes from `AP_AHRS::get_velocity_D(velD, true)` — the high-vibration path, which uses the baro/IMU
vertical rate rather than a GPS-backed velocity. Because there is no GPS the EKF never gets a home from
one, so `AP_Arming_Rocket::arm()` calls `ahrs.resetHeightDatum()`, referencing every altitude and climb
rate to the launch rail.

## Why a separate vehicle after all

The earlier QROCKET-in-ArduPlane approach was justified by the claim that the tailsitter/QuadPlane stack
is welded to the global `plane` singleton. Inspection showed otherwise:

- Of `quadplane.cpp`'s 455 `plane.` references, ~68% are transition/fixed-wing machinery (mode-pointer
  identity, mission, TECS, L1, airspeed) that a rocket deletes rather than ports.
- 33 of 103 QuadPlane methods — including `hold_hover()`, the actual hover primitive — have **zero**
  `plane.` references. `quadplane.h`, `tailsitter.h` and `transition.h` have zero.
- ArduSub already reuses this same attitude stack in a different medium.

ArduPlane's real contribution to a rocket is **one line**: `ahrs.create_view(ROTATION_PITCH_90)`, which
presents a nose-up airframe to the controller as though level, so "hold vertical" becomes "hold level"
and `AC_AttitudeControl_Multi` applies unmodified. Everything else worth having is a library already
shared by Copter and Sub. Building on ArduPlane would have dragged in TECS, L1, mission, airspeed and
transition state machines — precisely the misfire surface this project exists to remove.

## Arming (vehicle side, not this library)

Arming is a two-phase handshake: the first ARM runs a fin-wiggle sequence and deliberately
does **not** arm; a second ARM confirms it and captures the rail attitude. Details in
`ARDUROCKET_PLAN.md` §3b.

**Be clear about what that wiggle is.** It does **not** verify fin direction — software
cannot. A clamped airframe with no airflow produces no motion to observe, and comparing
the mixer's output against measured attitude is circular (the command is derived from
that attitude, so the signs agree even with a servo horn on backwards). A reversed servo,
a backwards linkage, or fins mounted in swapped positions **will arm and fly**. The wiggle
exists to make the human check unskippable, not to replace it.

The same sequence can be run on demand from a ground station with
`MAV_CMD_DO_MOTOR_TEST` (Mission Planner and QGC have UI for it), for bench testing
without attempting to arm. It is refused while armed, and — deliberately — a bench run
does **not** satisfy the arming gate, so it cannot be used to skip the check on the rail.

## Status

Flies the full profile in SITL: PREP → ARMED → BOOST → COAST → DESCENT, with fins verified live during
boost and coast and stopped at apogee. **Not flight-validated, and the attitude gains are placeholders
that have never been tuned.** See `ARDUROCKET_PLAN.md` in the repo root for the bench checks that must
pass before flying — in particular the manual fin-direction check, since the view-frame axis mapping
(view roll/pitch are tilt, view yaw is spin) is easy to get backwards.
