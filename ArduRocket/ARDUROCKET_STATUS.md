# ArduRocket — status & handoff

Living "where are we" note. **Stable design** lives in [ARDUROCKET_PLAN.md](ARDUROCKET_PLAN.md);
**build/run/operate guide** in [ARDUROCKET_README.md](ARDUROCKET_README.md). This file is only the
current state + what's next — durable findings get folded into the PLAN, not left here.

## Current state (2026-09-08)

**Rockets are data-driven — no built-in reference airframe.** To fly a rocket:
`python3 Tools/ArduRocket/ork_to_sim.py DESIGN.csv --ork X.ork --eng M.eng --tab-chord C --tab-span S
--name NAME` → `sitl_tests/NAME.parm`, then `Tools/ArduRocket/sitl_tests/sitl.sh NAME [tiltN]`. The
converter reads the labeled design CSV (+ .ork fins + RASP `.eng` motor) and writes the whole sim
airframe (`SIM_RKT_*`) + controller params (`RKT_*`); `sitl.sh` loads it and wires the motor. Bare
`--model rocket` with nothing loaded refuses ("NO ROCKET LOADED", sits inert). `sitl_tests/test1.parm`
(+ `motors/H128W.eng`) is the bundled example and the harness default. See **PLAN §9c**.

**Flies vertical off a full 20° rail, in wind, from any lean orientation — PROVEN in SITL.** A 20°
rail drives to steady true tilt ~0.4° and holds ~0.2–2° through the whole powered/coast ascent
(`rocket_test.py gains` PASSES; the 2° bar is the mission). The `RKT_TILT_I` integral term in the
direct tilt law is what drives the steady lean to zero. Gains are real params, fixed-gain
(MatrixPilot-style) is the firmware default, and gains are derived from the OpenRocket model
(`ork_to_rocket.py`) — full derivation rationale in **PLAN §9d** (the standalone gain-proposal doc
was folded in there and removed). A Monte Carlo over lean × wind maps the envelope.

**On-vehicle gain computation from field measurements — the real-rocket path (2026-09-06).** Gains are
computed on the flight controller from the physical airframe, no OpenRocket-at-runtime required. The
fixed airframe constants (`RKT_BODY_D`, fin root/tip/span, tab chord/span/max, `RKT_NOSE_FIN`, inertia)
are set **once** — ideally from the OpenRocket design export via `design_to_qgc.py`; **mass and CG**
(`RKT_MASS`, `RKT_NOSE_CG` → `fin_arm`) are motor-dependent and entered during **prep**; the pad is
verify-and-arm (read back the logged gains, fin check). (`RKT_LENGTH` is only the rod-`J_tilt` fallback
input, unused when real `RKT_JTILT` is given.) If
`RKT_MASS>0`, `compute_airframe_gains()` (system.cpp) estimates inertia (rod `m·L²/12`, cylinder
`½·m·r²`), computes the fin force and `fin_arm`, and overrides `RKT_TILT_P/D/I`+`RKT_SPIN_DAMP` at
boot (logged; clamped). `RKT_MASS=0` (default) keeps the direct gains, so the SITL default airframe is
unchanged. **If you know the real inertia** (e.g. from an OpenRocket design export), set the optional
`RKT_JTILT`/`RKT_JSPIN` and it's used directly — exact, no rod error (which measured +25% tilt / −12%
to −34% spin on real samples). `Tools/ArduRocket/design_to_qgc.py <design.csv> --ork <file.ork>` reads
a labeled OpenRocket design export and prints the `RKT_*` values (real inertia included) to paste into
QGC. **Flight-speed correction (`RKT_VMAX`):** fixed-gain loop frequency is `omega_n^2 = q*C`, so a
small/slow rocket (low q) is under-gained; set `RKT_VMAX` (OpenRocket max velocity) and the gains
scale by `(390/vmax)^2` to hold the loop frequency. Confirmed on test1 (H128W, ~100 m/s): without it,
the physics gains held the lean even at 3°; with `RKT_VMAX 101` (×14.9) it flies vertical at 3°
(steady 0.5°). test1 is authority-limited only at an unrealistic 20° rail. Full design + honest limits:
**PLAN §9e**. The gain derivation round-trips to the flown 2.5/0.5/2.0/0.006 for its reference numbers
and scales exactly (2× inertia → 2× gains). No swing test needed; OpenRocket helps if you have it but
isn't required.

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

## What's next

Everything above is **SITL** — nothing has flown on a real motor. The real gate is a **first
hardware flight**. Actual outstanding work is small:

**Pending code:**
- **(RESOLVED for the real rocket) `fin_arm`** is no longer a hand-set offline value — the on-vehicle
  path (§9e) computes it from measured `RKT_NOSE_FIN − RKT_NOSE_CG`. The offline `ork_to_rocket.py`
  `--fin-arm` remains a **sim-only** convenience (auto-computing it there is still not worth it: the
  CG column in the OpenRocket CSV is version-dependent and deliberately unmapped, and the fin axial
  position isn't parsed — both would need a real sample export to do safely).
- Nothing else outstanding in code. The next gate is the first hardware flight (below).
- **Optional, after the first hardware flight:** delete the `MOT_Q_REF` gain schedule and the
  `RKT_QSCHED` toggle. Fixed gain is the default and works in SITL; the schedule is kept only as an
  in-tree fallback (`scheduled.parm`) until a real flight confirms it isn't needed — then it's dead
  code to remove.

**Flight-time config (not code — already built/documented):**
- The pre-arm **fin check is implemented and QGC-tested** (the "Fin Check" action +
  `Tools/ArduRocket/qgc/ArduRocket.json`). A human must still watch the wiggle — software cannot
  detect a reversed servo, swapped fins, or a backwards linkage.
- Flip `ARMING_SKIPCHK` from the SITL `-1` to `21064` for real flight (value documented in
  `rocket.parm`).
- Model the real airframe's control-tab dimensions in the import so the derived gains + sim match —
  inherent to any new/real airframe (which also wants a verification flight, §9d).

## Known intermittent (benign, not chased)

A rare true-spin spike (~285°/s, seen once in ~6 runs) near rail departure on the tiny spin
inertia; it never corrupted the estimate (`d = 0` through it) and normal runs read `max 1–3°/s`.
Characterize it with `rocket_test.py gains --spin` (200 Hz window, quiet below 15°/s) if it
starts recurring — don't chase it otherwise.

## Cleanup TODO (working vehicle — none urgent)

- **Prune dead knobs.** The direct controllers bypass the ATC cascade, so `ATC_ANG/RAT_*`,
  `RKT_LEVEL_Q` and the target blend do nothing now — leaving them wired is misleading.
- (Done 2026-09-08) Scratch `.parm` files removed. `sitl_tests/` now holds only `test1.parm` (the
  rocket), `baseline.parm` (no-control diagnostic), and `scheduled.parm` (the `RKT_QSCHED=1` fallback).
- **Fold the spin/control writeup into the PLAN** (estimator + config + orientation + the gain
  derivation §9d are already folded in). Summary to preserve: the ~1200°/s coning runaway was an
  **inverted spin-control sign** (the pitch-90 view makes view-yaw = −body-X spin, so the cascade's
  yaw "damping" was positive feedback); fixed by a **direct spin damper** + **direct tilt controller**
  that override the ATC cascade in `rocket_control.cpp` (`RKT_SPIN_DAMP`, `RKT_TILT_P/D`). Spin is now
  clean (peak ~0–1°/s at the baked `RKT_SPIN_DAMP 0.006`); no gain nudge outstanding.
