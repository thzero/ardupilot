# ArduRocket — status & handoff

Living "where are we" note. **Stable design** lives in [ARDUROCKET_PLAN.md](ARDUROCKET_PLAN.md);
**build/run/operate guide** in [ARDUROCKET_README.md](ARDUROCKET_README.md). This file is only the
current state + what's next — durable findings get folded into the PLAN, not left here.

## Current state (2026-09-02)

**Flies vertical from any lean orientation and any 0–5° launch angle — PROVEN in SITL.**
`Tools/ArduRocket/sitl_tests/sweep.py` (rail tilt × rail clock matrix) is all-PASS; steady true
tilt 0–2°, estimate `d = 0`. Where the details live:

- **Estimator** (DCM, no-GPS, gyro-only gate) and the **two config traps** that masqueraded as an
  estimator bug for a whole session — the `GPS1_TYPE` param name (not `GPS_TYPE`), and the
  `--defaults` / `vehicleinfo.json` frame-registration requirement → **PLAN §3c** (the ⚠️ box).
- **Orientation-sweep proof** + `sweep.py` → **PLAN §10**; the **`-roll` clock knob** → **PLAN §9**.
- `libraries/AP_AHRS/` is **stock** — a session of DCM edits chasing the (config-caused) "off-axis
  bug" was fully reverted (`git diff` on `libraries/AP_AHRS/` is empty).

## Known intermittent (benign, not chased)

A rare true-spin spike (~285°/s, seen once in ~6 runs) near rail departure on the tiny spin
inertia; it never corrupted the estimate (`d = 0` through it) and normal runs read `max 1–3°/s`.
Characterize it with `rocket_test.py gains --spin` (200 Hz window, quiet below 15°/s) if it
starts recurring — don't chase it otherwise.

## Cleanup TODO (working vehicle — none urgent)

- **Prune dead knobs.** The direct controllers bypass the ATC cascade, so `ATC_ANG/RAT_*`,
  `RKT_LEVEL_Q` and the target blend do nothing now — leaving them wired is misleading.
- **Tab size is a design number, not a placeholder.** Flying vertical off a full 20° rail needs
  ~4× the tab force — a tab bigger than the 102 mm fin can carry. Real fins fly vertical to
  ~5–10°; document as an airframe spec (bigger fins / less static margin for 20°).
- **Nudge the spin-damper gain** — crept to `mean 280` under the aggressive 20° fins (0 off 5°).
- **Delete scratch `.parm` files**:
  `Tools/ArduRocket/sitl_tests/{hot_gains,hot_gains_i,hot_gains_i2,spin_test}.parm`.
- **Fold the spin/control writeup into the PLAN** (estimator + config + orientation are already
  folded in). Summary to preserve: the ~1200°/s coning runaway was an **inverted spin-control
  sign** (the pitch-90 view makes view-yaw = −body-X spin, so the cascade's yaw "damping" was
  positive feedback); fixed by a **direct spin damper** + **direct tilt controller** that override
  the ATC cascade in `rocket_control.cpp` (`RKT_SPIN_DAMP`, `RKT_TILT_P/D`).
