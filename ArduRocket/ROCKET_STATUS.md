# ArduRocket — vertical-control status & handoff (2026-09-01)

Short note to pick this up cold. The vehicle **flies vertical off a realistic launch** and the
hard root cause is fixed; there is **one open bug** (off-axis / roll-plane leans).

## What works (confirmed in SITL, baked into `Tools/autotest/default_params/rocket.parm`)

Off a **pitch-plane** lean, the rocket drives the launch lean to vertical and holds it:
- **5° rail (normal launch angle): PASS** — drives 5° → 0.2°, holds **steady true tilt ~1.9°**,
  spin 0, estimate `d = 0`.
- **20° rail (legal max, pathological): PASS only with 2× tabs** — see the tab caveat below.

Test: `rocket_test.py gains` against `--model rocket-tilt5 -w` (or `-tilt20`). It grades the
high-`q` powered window only (apogee gravity turn is correctly ignored).

## The root cause we fixed (the thing that had it fundamentally broken)

The airframe spun up to **~1200°/s** about its long axis and a fast spin + a tilt = **coning**,
which systematically corrupted the DCM attitude estimate (est read a phantom 117° while true
was 30°). Every "estimator" symptom traced back to this. **Cause: an inverted sign in the spin
control** — the pitch-90 view makes view-yaw = **−**(body-X spin), so the cascade's yaw "damping"
was positive feedback and ran the spin away.

Fix chain (all in `ArduRocket/rocket_control.cpp` + `rocket.parm`, built into `bin/rocket`):
1. **Direct spin damper** — `motors->set_yaw(-RKT_SPIN_DAMP * ahrs.get_gyro().x)` (raw body-X
   rate, correct sign), overriding the ATC after `rate_controller_run()`. Killed the spin.
2. **Clean no-GPS estimate** — DCM (`AHRS_EKF_TYPE 0`), GPS off (`GPS_TYPE 0`), gyro-only
   attitude gate during boost (`ahrs.set_attitude_gyro_only()`), baro climb rate for apogee.
   (See PLAN §3c.) Only trustworthy once the spin was gone.
3. **Direct tilt controller** — `set_roll/set_pitch = -RKT_TILT_P*view_angle - RKT_TILT_D*rate`,
   also overriding the ATC (its cascade throttled the fins to ~0 on a standing lean). MatrixPilot
   style (P on the gravity vector). `RKT_TILT_P 4.0`, `RKT_TILT_D 0.3`.

## OPEN BUG — roll / off-axis leans (top priority)

`--model rocket-tilt5-az90` (lean in the roll plane instead of pitch) **FAILS**: estimate
diverges from truth by up to 15° (NOT coning — spin stays low), the fins barely move, and the
give-up fires on an estimate glitch. All testing to date was pitch-plane (az0) only, so this was
hidden. Real launches lean into the wind in an arbitrary direction, so it matters.

**First clue / where to start:** the vehicle reported `rail attitude 0.0/-5.0` (roll 0, pitch
−5) for `-az90` — the SAME as az0. So either the sim azimuth isn't rotating the tilt as expected,
or the vehicle is **mis-capturing the tilt direction at arming** (reading an off-axis lean as
pure pitch). If the captured rail attitude is on the wrong axis, the target/control references
the wrong axis — which fits "fins don't steer, estimate wanders." Steps:
1. Confirm the sim's TRUE tilt direction under `-az90` (SIMSTATE roll/pitch) vs. what the vehicle
   captured (`rail_roll_rad`/`rail_pitch_rad`).
2. If they disagree → rail-attitude capture / view-frame axis bug. If they agree → the gyro-only
   estimate drifts on the roll axis, or the direct tilt controller's roll axis is wrong.

## Cleanup TODO (on a working vehicle — none urgent)

- **Prune dead knobs**: the direct controllers bypass the ATC, so `ATC_ANG/RAT_*`, `RKT_LEVEL_Q`
  and the target blend do nothing now — leaving them is misleading.
- **Tab size is a design number, not a placeholder**: flying vertical off a full 20° needs ~4×
  the tab force, i.e. a tab bigger than the current fin can carry (154 mm span on a 102 mm fin).
  The real fins fly vertical to ~5–10°; 20° would need bigger fins. Put this in the PLAN as a spec.
- **Spin damper gain** crept to `mean 280` under the aggressive 20° fins (0 off 5°) — nudge up.
- **Delete scratch files**: `Tools/ArduRocket/sitl_tests/{hot_gains,hot_gains_i,hot_gains_i2,
  spin_test}.parm` (investigation leftovers).
- **Write the full investigation into ARDUROCKET_PLAN.md** (spin root cause + estimator chain +
  tab-authority-vs-launch-angle finding).
