# ArduRocket — status & handoff

Living "where are we" note. **Stable design** lives in [ARDUROCKET_PLAN.md](ARDUROCKET_PLAN.md);
**build/run/operate guide** in [ARDUROCKET_README.md](ARDUROCKET_README.md). This file is only the
current state + what's next — durable findings get folded into the PLAN, not left here.

## Current state (2026-09-06)

**Flies vertical off a full 20° rail, in wind, from any lean orientation — PROVEN in SITL.** A 20°
rail drives to steady true tilt ~0.4° and holds ~0.2–2° through the whole powered/coast ascent
(`rocket_test.py gains` PASSES; the 2° bar is the mission). The `RKT_TILT_I` integral term in the
direct tilt law is what drives the steady lean to zero. Gains are real params, fixed-gain
(MatrixPilot-style) is the firmware default, and gains are derived from the OpenRocket model
(`ork_to_rocket.py`) — full derivation rationale in **PLAN §9d** (the standalone gain-proposal doc
was folded in there and removed). A Monte Carlo over lean × wind maps the envelope.

**Reading the tilt metrics.** Grade the `powered-flight max TRUE tilt` (first 75% of ascent, while
the fins have authority) and the steady tilt — not the full-ascent peak. The full-ascent peak
includes the apogee nose-over (climb→0, q→0, the rocket tips over at the top of its arc), which is
unavoidable physics and not a flight-quality number; the grader/Monte Carlo label it as such.
`sitl_tests/baseline.parm` (all control gains zeroed → tabs frozen) flies the passive airframe as the
reference for airframe-vs-controller questions.

**Original 0–5° orientation sweep (2026-09-02):** `sweep.py` (rail tilt × rail clock matrix) all-PASS,
steady true tilt 0–2°, estimate `d = 0`. Where the details live:

- **Estimator** (DCM, no-GPS, gyro-only gate) and the **two config traps** that masqueraded as an
  estimator bug for a whole session — the `GPS1_TYPE` param name (not `GPS_TYPE`), and the
  `--defaults` / `vehicleinfo.json` frame-registration requirement → **PLAN §3c** (the ⚠️ box).
- **Orientation-sweep proof** + `sweep.py` → **PLAN §10**; the **`-roll` clock knob** → **PLAN §9**.
- `libraries/AP_AHRS/` is **stock** — a session of DCM edits chasing the (config-caused) "off-axis
  bug" was fully reverted (`git diff` on `libraries/AP_AHRS/` is empty).

## What's next (pending, before a real motor)

Everything above is **SITL** — nothing has flown on a real motor. Before one does:

- **First hardware flight.** The gate for retiring the fallback schedule and trusting the derived
  gains on a real airframe.
- **Wire up the real `fin_arm`** in `ork_to_rocket.py` (CG-to-fin axial distance from the imported CG
  + fin location), so `--fin-arm` need not be passed by hand and the derived gains are not flagged
  provisional. See PLAN §9d.
- **After one hardware flight, delete the `MOT_Q_REF` schedule and the `RKT_QSCHED` toggle.** Fixed
  gain is the default; `sitl_tests/scheduled.parm` keeps the `RKT_QSCHED=1` tune only until then.
- **Hardware pre-flight caveats** (from the PLAN header, still true): set the real control-tab
  dimensions (`SIM_RKT_TAB_*`), use `ARMING_SKIPCHK 21064` (not the SITL `-1`), and have a human
  verify fin direction on the wiggle — software cannot detect a reversed servo or swapped fin.

## Known intermittent (benign, not chased)

A rare true-spin spike (~285°/s, seen once in ~6 runs) near rail departure on the tiny spin
inertia; it never corrupted the estimate (`d = 0` through it) and normal runs read `max 1–3°/s`.
Characterize it with `rocket_test.py gains --spin` (200 Hz window, quiet below 15°/s) if it
starts recurring — don't chase it otherwise.

## Cleanup TODO (working vehicle — none urgent)

- **Prune dead knobs.** The direct controllers bypass the ATC cascade, so `ATC_ANG/RAT_*`,
  `RKT_LEVEL_Q` and the target blend do nothing now — leaving them wired is misleading.
- **Delete scratch `.parm` files**:
  `Tools/ArduRocket/sitl_tests/{gentle_gains,hot_gains,hot_gains_i,hot_gains_i2,hot_gains_i3,spin_test}.parm`
  — leftovers from the MOT_Q_REF/gain-tuning experiments; the tune is now baked in firmware. **Keep**
  `baseline.parm` (no-control diagnostic), `fixed_gain.parm`, and `scheduled.parm` (the `RKT_QSCHED=1`
  fallback) — those are intentional and documented.
- **Fold the spin/control writeup into the PLAN** (estimator + config + orientation + the gain
  derivation §9d are already folded in). Summary to preserve: the ~1200°/s coning runaway was an
  **inverted spin-control sign** (the pitch-90 view makes view-yaw = −body-X spin, so the cascade's
  yaw "damping" was positive feedback); fixed by a **direct spin damper** + **direct tilt controller**
  that override the ATC cascade in `rocket_control.cpp` (`RKT_SPIN_DAMP`, `RKT_TILT_P/D`). Spin is now
  clean (peak ~0–1°/s at the baked `RKT_SPIN_DAMP 0.006`); no gain nudge outstanding.
