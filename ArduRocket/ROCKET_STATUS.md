# ArduRocket — vertical-control status & handoff (2026-09-01)

Short note to pick this up cold. The vehicle **flies vertical off a realistic launch in any
lean direction** (pitch-plane AND roll-plane/off-axis). The off-axis case is now fixed.

> **CONFIG TRAP (this bit us for a whole session), now fixed:** the raw `--model … -w` launch
> resolves its embedded defaults from `@ROMFS/vehicleinfo.json` by **exact model-string match**.
> `rocket-tilt5-az90` was NOT registered there (only `rocket`, `rocket-tilt5/10/20` were), so the
> az90 model fell back to firmware defaults (EKF3 + GPS on) and the no-GPS DCM never ran — while
> `rocket-tilt5` DID load `rocket.parm`, which is why az0 "passed" and az90 "failed". Second bug:
> the GPS param is **`GPS1_TYPE`**, not `GPS_TYPE`, so `rocket.parm`'s `GPS_TYPE 0` was silently
> ignored and GPS stayed on even when the file loaded. EKF3-on-GPS flies pitch-plane leans but
> glitches off-axis — masquerading as a "DCM estimator bug" for eight runs.
> **Fixes:** (1) `GPS_TYPE`→`GPS1_TYPE` in rocket.parm; (2) az90 frames registered in
> `Tools/autotest/pysim/vehicleinfo.json` (needs `./waf configure --board sitl` to re-embed);
> (3) `rocket_test.py` reads AHRS_EKF_TYPE / GPS1_TYPE at startup and **refuses to run** on the
> wrong estimator. Raw `./build/sitl/bin/rocket --model rocket-tilt5-az90 -w` now works with no
> `--defaults`. New azimuth/tilt combos still need a vehicleinfo.json entry (or the guard trips).
>
> **Known intermittent (benign):** a rare true-spin spike (~285°/s, seen ONCE in ~6 runs) near
> rail departure on the tiny spin inertia; never corrupted the estimate (`d=0` through it). Normal
> runs read `mean 0 / max 1–3°/s` (damper holds it). Characterize it with
> `rocket_test.py gains --spin` (200 Hz truth window; auto-classifies isolated glitch vs real
> multi-sample event, stays quiet below 15°/s). Not worth chasing unless it starts recurring.

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

## RESOLVED — roll / off-axis leans

`--model rocket-tilt5-az90` now **PASSES**: drives the 5° roll-plane lean to vertical, holds
steady true tilt ~2.0°, estimate `d = 0` the whole flight, `omegaI` flat at 0.

**Real root cause (not what it looked like):** the off-axis test was silently running **EKF3 on
GPS**, not the DCM — because `rocket.parm` set the nonexistent `GPS_TYPE` (real name
`GPS1_TYPE`), so GPS was never turned off, and because the raw `-w` launch didn't apply
`rocket.parm` at all (needs `--defaults`). EKF3 dead-reckoning under thrust handles a pitch-plane
lean but glitches off-axis — which read exactly like a "DCM tilt divergence." Once the DCM
actually runs (EKF_TYPE=0 + GPS1_TYPE=0), the committed gyro-only gate flies it. **No changes to
`libraries/AP_AHRS/` were needed** — a session of DCM edits chasing this was reverted.

Fix = one param rename (`GPS_TYPE`→`GPS1_TYPE`) + always launch with `--defaults` + the harness
now refuses to run on the wrong estimator. See the CONFIG TRAP box at the top.

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


PLAN writeup — the spin root cause, the estimator chain, and the tab-authority-vs-launch-angle finding (flies vertical to ~5–10° on the real fins; 20° would need bigger fins).
Prune dead knobs — the direct controller bypassed the cascade, so ATC_ANG/RAT_*, RKT_LEVEL_Q, and the target blend do nothing now; leaving them is misleading.
Nudge the spin damper gain (crept to mean 280 under aggressive fins off 20°; it's 0 off 5°, so low priority).
Delete the scratch .parm files.

sure lets do this.