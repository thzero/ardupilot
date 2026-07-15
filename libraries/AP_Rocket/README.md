# AP_Rocket

Vehicle-agnostic support library for **autonomous vertical-hold rocket flight** on ArduPilot.

It holds the rocket-specific *brains* — launch detection and integrator-gating policy — behind a small,
Plane-free API, plus the `RKT_*` parameters. It is driven by the thin `QROCKET` flight mode in
`ArduPlane/` (`ArduPlane/mode_qrocket.cpp`). Same structural pattern as `AP_Soaring`, `AP_Landing`,
`AP_ICEngine`: a self-contained library run by a thin vehicle-side hook.

## Goal

Keep a hobby rocket flying **vertically** (attitude hold to 0° tilt) with **no RC transmitter and no
ground station**, using either a TVC gimbal or steering fins. Holding vertical is *stabilization*, not
guidance-to-a-target.

To the flight stack a TVC/finned rocket held vertical is a **single-motor vectored tailsitter that never
transitions to forward flight** — so `QROCKET` subclasses `ModeQStabilize` and reuses ArduPilot's hardened
tailsitter attitude/EKF/servo stack unchanged.

## What QROCKET does (three behaviors)

1. **Full-authority stabilization at zero throttle.** Forces air-mode ON so `hold_stabilize()` never
   relaxes the attitude controllers while the rocket sits on the pad at zero throttle.
2. **Vertical target.** Forces roll/pitch target to 0 every loop; ignores stray/absent RC input.
3. **Accel launch gate.** Holds the rate-controller integrators (P/D stay live) until measured body-up
   acceleration exceeds `RKT_LAUNCH_G`, preventing integrator wind-up against the launch rail.

## Architecture

```
libraries/AP_Rocket/          <- this library (rocket brains, RKT_ params) — vehicle-agnostic
    AP_Rocket_config.h        <- AP_ROCKET_ENABLED build gate
    AP_Rocket.h / .cpp        <- launch detector + gating policy + params
ArduPlane/
    mode.h                    <- QROCKET enum + ModeQRocket : public ModeQStabilize
    mode_qrocket.cpp          <- THIN shim: the only code that touches Plane/QuadPlane internals
    control_modes.cpp         <- mode_from_number() case
    Plane.h / quadplane.h     <- ModeQRocket friend + mode_qrocket member
    Parameters.cpp / .h       <- ParametersG2 RKT_ subgroup (index 42)
```

### Portability constraint (deliberate)

`AP_Rocket`'s public API is **vehicle-agnostic**: it takes plain inputs (a body-frame acceleration
`Vector3f`) and returns plain decisions (`launched()`, `hold_integrators()`). It must **never** reference
`Plane`/`QuadPlane` types. All ArduPlane coupling (`plane.ins`, `plane.nav_roll_cd`,
`quadplane.air_mode`, `attitude_control->…`) lives only in `ArduPlane/mode_qrocket.cpp`. This keeps the
library liftable into a standalone vehicle later, should that ever be wanted.

### Why not a separate `ArduRocket/` vehicle, or Lua?

- **Separate vehicle folder:** ArduPilot vehicle directories are not linkable libraries and the
  tailsitter/QuadPlane stack is welded to the global `plane` singleton via `friend` access — a separate
  vehicle would have to fork all of ArduPlane. The sanctioned OOP reuse is exactly
  `ModeQRocket : public ModeQStabilize`, which must live in `ArduPlane/`.
- **Lua (`AP_Scripting`):** runs on a low-priority thread with an instruction budget, off the fast loop —
  tens of Hz with jitter. The integrator gating must run in the ~400 Hz fast loop, which Lua cannot do.

## Parameters

| Param | Type | Default | Purpose |
|---|---|---|---|
| `RKT_ENABLE` | AP_Int8 | 0 | Enable the rocket vertical-hold behavior. |
| `RKT_LAUNCH_G` | AP_Float | 1.5 | Launch-detect threshold in g (× `GRAVITY_MSS`). Bench-tune. |
| `RKT_LAUNCH_AX` | AP_Int8 | 0 | Body up-axis: 0=X, 1=Y, 2=Z. Defends against orientation mismatch. |

## Companion parameters (set outside this library)

For a standalone, no-RC vertical-hold vehicle (verified names for this tree):

```
Q_ENABLE         = 1
Q_FRAME_CLASS    = 10     # tailsitter
Q_TAILSIT_ENABLE = 1
AHRS_ORIENTATION = 24     # ROTATION_PITCH_90 (nose-up) — bench-verify vs 25 (PITCH_270)
INITIAL_MODE     = 27     # QROCKET at boot, no receiver
RC_PROTOCOLS     = 0      # no RC expected; unblocks arming
THR_FAILSAFE     = 0
FS_GCS_ENABL     = 0
ARMING_SKIPCHK   = -1     # skip all pre-arm checks (note: NOT the old ARMING_CHECK)
ARMING_REQUIRE   = 0
BRD_SAFETY_DEFLT = 0      # outputs live at boot
ARSPD_USE        = 0
RKT_ENABLE       = 1
RKT_LAUNCH_G     = 1.5
```

### Servo mapping (choose on the bench via `SERVOn_FUNCTION`)

- **4 independent fins** (pitch+yaw+roll): S1..S4 = `k_elevon_left`(77), `k_elevon_right`(78),
  `k_vtail_left`(79), `k_vtail_right`(80).
- **TVC gimbal** (pitch+yaw, no roll): S1/S2 = `k_tiltMotorLeft`(75), `k_tiltMotorRight`(76); requires
  `Q_TAILSIT_VHGAIN > 0`.

## Bench-confirm before flight

- Body up-axis / sign under `AHRS_ORIENTATION`: log `INS`/`ACC` while vertical & stationary; the up-axis
  should read ≈ +9.8 m/s². Adjust `RKT_LAUNCH_AX` if it is not X.
- `RKT_LAUNCH_G`: pick a value safely above the stationary 1 g reading but below expected launch accel.
- Correct-direction check: tilt the nose; fins/gimbal must move to push it back toward vertical.

## Status

Skeleton / pre-flight. Single-sample launch detection (no debounce yet). Not flight-validated.
See the project working document for full design rationale and verification plan.
