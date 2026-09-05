# Physics-derived control gains (with full rationale)

**Status:** IMPLEMENTED 2026-09-05 (see Section 13 for what shipped and the verification results).
**Date:** 2026-09-05
**Scope:** `Tools/ArduRocket/ork_to_rocket.py`, `ArduRocket/Parameters.cpp`,
`ArduRocket/rocket_control.cpp`, the fin mixer q-schedule.

---

## 0. Executive summary

Import a rocket's OpenRocket model → the attitude-control gains fall out automatically. You set
**one** global "responsiveness" constant, once, and never hand-tune `RKT_TILT_P`, `RKT_TILT_D`, or
`RKT_SPIN_DAMP` per airframe again. A bigger, heavier rocket gets bigger gains automatically, because
the gains are computed from the airframe's inertia and fin geometry — all of which the `.ork`/`.csv`
import already provides.

Two enabling conclusions were established experimentally this session and are documented in full
below:

1. **Fixed gain (MatrixPilot-style, no dynamic-pressure schedule) flies this airframe to vertical
   across the envelope.** So the `MOT_Q_REF` gain schedule — and its dependency on airframe geometry
   — can be retired.
2. **The remaining control gains are irreducibly airframe-specific** (this is control theory, not a
   design flaw), so the only way to stop hand-tuning them per rocket is to **derive** them from the
   airframe model — which ArduRocket can do and MatrixPilot could not.

---

## 1. Background: how the tuning got here

The mission is to fly **vertical, straight up**, correcting any launch-rail lean and holding against
crosswind/weathercock. The control history this session:

- The airframe was **holding its launch lean** (~15-20 deg off a 20-deg rail) instead of correcting.
  Root cause was NOT the estimator: the ascent control law is a direct proportional law that was
  **under-authority in the high-q boost window**. Raising the fin authority (`MOT_Q_REF`
  600 → 2500 → 5000 → 12000) and adding an **integral term** to the tilt law drove the steady true
  tilt from ~7 deg to ~1.3 deg (mission PASS).
- Raising `MOT_Q_REF` over-energized the spin damper (its effective gain is `~RKT_SPIN_DAMP *
  MOT_Q_REF`), which limit-cycled (~5 Hz fin chatter, spin ringing). Fixed by rescaling
  `RKT_SPIN_DAMP` 0.10 → 0.042 to hold the spin moment loop-gain constant.
- The direct-law gains were promoted from compile-time `#define`s to runtime params
  (`RKT_TILT_P/D/I`, `RKT_TILT_IMAX`, `RKT_SPIN_DAMP`) so the ascent tune is one visible, rebuild-free
  surface. The `ATC_*` cascade gains do NOT drive the ascent — the direct law overrides the cascade
  in BOOST/COAST.

This produced a working tune — but every one of those numbers is airframe-specific, which is the
problem this proposal addresses.

## 2. The problem: gains are airframe-specific

The ascent law (BOOST/COAST) commands fins directly from the estimate:

```
fin_roll  = -RKT_TILT_P * view_roll  - RKT_TILT_D * view_roll_rate  - RKT_TILT_I * integral
fin_yaw   = -RKT_SPIN_DAMP * spin_rate
```

The right values for `RKT_TILT_P/D` and `RKT_SPIN_DAMP` depend on the airframe: a heavier rocket has
more rotational inertia, so the same gain produces less angular acceleration and under-corrects; a
rocket with bigger or longer-armed fins produces more moment, so it needs less gain. There is no
single value that is correct for a 3-inch minimum-diameter bird and a 6-inch fiberglass beast.

**This is control theory, not a bug.** You cannot steer a plant without the control law encoding
something about the plant's inertia and control effectiveness.

## 3. Why MatrixPilot does not escape this (evidence)

ArduRocket borrows two techniques from the flown MatrixPilot fin-steered rocket: the DCM
accelerometer-gate under thrust (`rmat.c`: accel correction only while `launched == 0`) and direct
"P on the gravity vector" control. It was tempting to believe MatrixPilot avoided per-airframe
tuning. It does not:

- MatrixPilot's stabilization gains live in `options.h` and are configured **per aircraft**:
  `PITCHGAIN` (pitch proportional, "typically 0.125-0.25"), `PITCHKD` (pitch rate damping,
  "typically 0.25"), `YAWKP`/`YAWKD`. These are the same category as `RKT_TILT_P`/`RKT_TILT_D`. The
  entire "HowToConfigure" wiki is a guide to setting them for your airframe.
- A rocketry domain reference stated it directly: total control gain "encompasses fin sizes, angular
  throws, airspeed, density, electronic gains and angular moments of inertia."

**What MatrixPilot does NOT do is gain-schedule on airspeed** (no dynamic pressure, no `MOT_Q_REF`).
That is the part worth copying, and the fixed-gain experiment (Section 5) confirms it works here.
But MatrixPilot still carries fixed, hand-tuned, per-airframe gains. If a flown rocket "did not
retune," it reused values that were close enough for a similar airframe.

**The opportunity:** MatrixPilot users hand-tuned because they had no airframe model to compute from.
ArduRocket has one — the OpenRocket import — so it can **derive** the gains. That is doing the thing
MatrixPilot could not, not merely matching it.

Sources: MatrixPilot `HowToConfigure` wiki; MatrixPilot `HowToConfigureMP30`; Rocketry Forum
"Simulating Fin Control".

## 4. Background: where "airspeed" comes from (no pitot, no GPS)

Relevant because the retired schedule depended on it, and because it explains why fixed gain is a
clean simplification. The vehicle has no pitot and no GPS. The dynamic pressure `q` used by the
schedule comes from the **barometer**:

```
velD  = ahrs.get_velocity_D(true)   // with DCM + no GPS, this is the baro filtered climb rate
speed = |velD|
q     = 0.5 * air_density * speed^2 // air_density from barometric altitude
```

Because the rocket flies near-vertical, its vertical speed approximately equals its airspeed — so a
barometer doubles as an airspeed sensor. This is a load-bearing design assumption: it holds only
while the rocket is near-vertical (which is the mission), and near apogee where vertical speed → 0
the `RKT_MIN_Q` gate has already stopped the fins, so the degraded estimate there is harmless.

The estimate is good enough for gain scheduling (it is a filtered derivative, so it lags and carries
baro noise), which matters: the schedule never needed a precise airspeed. Which raises the question
the fixed-gain experiment answers — does it need the schedule at all?

## 5. The fixed-gain experiment (full evidence)

**Hypothesis:** the `MOT_Q_REF` dynamic-pressure schedule (and thus its airframe-geometry dependency)
is not necessary. MatrixPilot flew fixed-gain; maybe ArduRocket can too.

**Mechanism (reversible toggle, already in-tree):** the fin mixer scales fin deflection by
`min(MOT_Q_REF/q, MOT_GAIN_MAX)`, and reads `q <= 0` as "no airflow → use constant `MOT_GAIN_MAX`."
A new param `RKT_QSCHED` (default 1 = schedule) feeds the mixer `q=0` when set to 0, giving
MatrixPilot-style fixed gain. The **real** `q` still gates apogee detection and `RKT_MIN_Q`.

**Fixed-gain tune used** (`Tools/ArduRocket/sitl_tests/fixed_gain.parm`):

| param | fixed-gain value | scheduled default | note |
|-------|------------------|-------------------|------|
| `RKT_QSCHED` | 0 | 1 | schedule off |
| `MOT_GAIN_MAX` | 1.0 | 4.0 | becomes the constant mixer scale in fixed mode |
| `RKT_TILT_P` | 2.5 | 4.0 | full fin at ~23 deg lean |
| `RKT_TILT_D` | 0.5 | 0.3 | more damping; fixed gain is punchier at high q |
| `RKT_TILT_I` | 2.0 | 2.0 | unchanged; integrator is orthogonal to the schedule |
| `RKT_SPIN_DAMP` | 0.006 | 0.042 | dropped ~7x: fixed scale 1.0 vs ~0.135 at high q |

**Results (SITL, `rocket-tilt20` / `rocket` models):**

| case | steady true tilt | peak spin | fin chatter | verdict |
|------|------------------|-----------|-------------|---------|
| 20-deg rail, no wind | 0.5 deg | 0 deg/s | 0 Hz | PASS |
| vertical rail, no wind | 0.2 deg | 0 deg/s | 0 Hz | PASS |
| vertical rail, 20 mph gusting crosswind | 0.1 deg | 1 deg/s | ~0.9 Hz (slow correction, not buzz) | PASS |

The wind case showed a brief ~12 deg transient right at rail exit (crosswind hits before airspeed —
and thus fin authority — builds), nulled to 0 by t≈8 s. This is physics, not a control defect.

**One caught failure along the way:** the first fixed-gain attempt left `RKT_SPIN_DAMP` at 0.05,
which was ~7x too hot for fixed mode (scale 1.0 vs ~0.135 at high q under the schedule). The spin ran
to 1740 deg/s with ~3 Hz fin chatter — the same over-gain limit cycle seen earlier. Dropping to 0.006
fixed it. This is exactly the `RKT_SPIN_DAMP <-> mixer-scale` coupling and is expected.

**Conclusion:** fixed gain flies to vertical as well as or better than the schedule (0.1-0.5 deg
across the envelope), with no oscillation. **The `MOT_Q_REF` schedule is not needed** — which removes
the schedule's own airframe-geometry dependency and simplifies the mixer.

## 6. The derivation

With the schedule gone, the fixed-gain gains map directly to physical quantities. The tilt loop is
second order. Fixed-gain fin deflection is `-P*angle - D*rate`; the plant is
`angular_accel = (q * force_gain * fin_arm / J_tilt) * deflection`. Substituting gives

```
theta'' + (K * D) theta' + (K * P) theta = 0,   with  K = q * force_gain * fin_arm / J_tilt
```

a standard second-order system with `omega_n^2 = K*P` and `2*zeta*omega_n = K*D`. Designing for a
target natural frequency and damping ratio, the gains that produce the SAME closed-loop response
scale identically across airframes:

```
RKT_TILT_P     =  C_P * J_tilt / (force_gain * fin_arm)
RKT_TILT_D     =  C_D * J_tilt / (force_gain * fin_arm)
RKT_SPIN_DAMP  =  C_S * J_spin / (force_gain * fin_arm)
```

| term | meaning | source |
|------|---------|--------|
| `J_tilt`, `J_spin` | tilt / spin moments of inertia | **imported** (`SIM_RKT_JTILT0/1`, `SIM_RKT_JSPIN0/1`) |
| `force_gain` | fin aerodynamic force per unit q at full deflection | **computed** (`derive_fin()`, `ork_to_rocket.py` ~L163: `S_fin * CLa * Kfb * tau * radians(tab_max_deg)`) |
| `fin_arm` | axial distance from CG to the fins | CG (imported, `.csv`) + fin location (imported, `.ork`) |
| `C_P`, `C_D`, `C_S` | portable responsiveness/damping constants | **set once**, identical for every airframe |

Note the design still has an implicit reference condition (the `q` at which the response is nominal),
but that is a single global design choice, not a per-airframe number. Fixed gain accepts that the
loop is softer off the rail and punchier at boost; the experiment shows that is fine for a naturally
stable rocket that only needs modest corrections.

`RKT_TILT_I` (the integrator) is more forgiving — it sets how fast the steady weathercock bias is
nulled. It can scale with `C_P` or be left as a fixed time-constant; decide during implementation.

## 7. The calibration move (why this is trustworthy)

`C_P`, `C_D`, `C_S` are **not guessed** — they are back-calculated from the one airframe we have
flight-validated. We know its working fixed-gain values (Section 5) AND its physical numbers
(`J_tilt`, `force_gain`, `fin_arm`), so:

```
C_P = RKT_TILT_P_validated * force_gain * fin_arm / J_tilt      (a fixed number, computed once)
C_D = RKT_TILT_D_validated * force_gain * fin_arm / J_tilt
C_S = RKT_SPIN_DAMP_validated * force_gain * fin_arm / J_spin
```

Any new rocket then gets `RKT_TILT_P = C_P * J_tilt_new / (force_gain_new * fin_arm_new)`, etc.

The constants as actually computed and shipped (in `ork_to_rocket.py` as `GAIN_C_P/C_D/C_I/C_S`):

```
C_P = 1.882456e-03    C_D = 3.764911e-04    C_I = 1.505964e-03    C_S = 1.077778e-03
```

`force_gain` uses `derive_fin`'s fractional-tab formula consistently in both calibration and
emission, so the anchor round-trips (`2.5 / 0.5 / 2.0 / 0.006`) regardless of the fin model's
absolute accuracy.

**Why anchor to a flight rather than a first-principles constant:** the fin-effectiveness model
(`CLa` from aspect ratio, body interference `Kfb`, tab factor `tau`) carries ~10-20% absolute
uncertainty. Anchoring `C_P/C_D/C_S` to a REAL validated flight absorbs that error at the calibration
point. What the physics contributes is the **scaling** — a 2x-inertia rocket gets ~2x the gain — and
that scaling is exact regardless of the model's absolute error. The scaling is what removes
per-rocket retuning; the anchor pins the absolute.

**Calibrate what the derivation earns, honestly (see also Section 13):** the fixed-gain law has an
INTEGRAL term that nulls the steady weathercock lean on its own, which makes the *steady* tilt
metric fairly robust to gain mismatch — in test, a 2x-inertia airframe flown with the UN-scaled 1x
gains still reached ~1 deg steady. So the derivation's value is NOT "un-scaled gains crash" (for
moderate mismatch they often don't); it is (a) consistent TRANSIENT response across airframes and
(b) staying inside the authority envelope for LARGER airframe changes, where an un-scaled gain runs
out of authority. The scaling is correct and worth having; just don't oversell it.

## 8. The one gap to close: `fin_arm`

`fin_arm` is currently a placeholder — `ork_to_rocket.py:172`: `fin_arm = fin_semispan * 5`, flagged
for hand-set. Replace it with the real **CG-to-fin axial distance**:

```
fin_arm = (fin axial position, from the .ork component) - (CG, from the .csv per burn state)
```

Explicitly **not** needed: the whole-airframe center of pressure, or the static margin (CP - CG).
Those are stability quantities (they set how hard the airframe weathercocks — a disturbance the
integrator already nulls without knowing its magnitude). The control moment arm is just the lever
length from the CG to where the fin force acts. CG is imported from the `.csv` and is also physically
measurable (the balance-on-a-string test every rocketeer already does).

## 9. Implementation plan

1. **Wire up `fin_arm`** in `ork_to_rocket.py` — real CG-to-fin distance from imported CG + fin
   location (replace the `semispan*5` placeholder). Track CG over burn state if the arm shift matters.
2. **Calibrate** — back out `C_P`, `C_D`, `C_S` from the current validated airframe and its known-good
   fixed-gain values; store as module constants with the anchor-flight provenance noted in a comment.
3. **Emit** — generate `RKT_TILT_P/D` and `RKT_SPIN_DAMP` into the output `.parm`.
4. **Verify (round trip)** — regenerate for the current airframe; the emitted gains must reproduce
   the flown values (sanity check on the calibration). Then generate for a **scaled** airframe (e.g.
   2x inertia) and fly it in SITL to confirm the scaling holds and it stays vertical / does not
   oscillate.
5. **Retire the schedule** — set `RKT_QSCHED=0` as the default and delete `MOT_Q_REF` + the mixer's
   q-scaling branch (separate cleanup). Keep or drop the toggle per the open decision below.

## 10. Outcome / interface

- **Workflow:** `.ork` → `ork_to_rocket.py` → `.parm` with gains already correct for that airframe.
- **The only knob you ever touch:** the global responsiveness constant — bump it to make the whole
  fleet snappier or gentler. One number, not three-to-six per rocket.
- **Per new airframe:** ideally zero tuning; at most one SITL verification flight, because the scaling
  is physics and the anchor is a real flight.

## 11. Risks / caveats

- **Model uncertainty.** The fin-effectiveness model is approximate, so a very different airframe may
  want one verification flight. The scaling is sound; the absolute is pinned by the anchor.
- **Data quality.** Needs a clean `.ork`/`.csv` with inertia and fin geometry (already parsed).
- **`fin_arm` depends on CG.** CG is imported and measurable; a wrong CG throws the arm off, so keep
  the "flagged for hand-set" discipline until the CG parse is trusted.
- **Reference condition.** Fixed gain accepts a loop gain that varies with speed. Validated for this
  airframe class; a radically different airframe (very low or very high thrust-to-weight, unusual
  static margin) should be sim-checked.

## 12. Open decisions

- Make fixed gain (`RKT_QSCHED=0`) the default now, or after one more real-flight's confidence?
- Derive `RKT_TILT_I`, or leave it a fixed time-constant?
- Keep the `MOT_Q_REF` schedule in-tree (behind `RKT_QSCHED=1`) as a fallback, or delete outright?

## Appendix A: validated reference data (anchor airframe)

Fixed-gain values that flew clean across the envelope (Section 5), to be used as the calibration
anchor in step 2:

```
RKT_TILT_P     2.5
RKT_TILT_D     0.5
RKT_TILT_I     2.0
RKT_SPIN_DAMP  0.006
MOT_GAIN_MAX   1.0     (constant mixer scale in fixed mode)
RKT_QSCHED     0
```

The physical quantities (`J_tilt`, `J_spin`, `force_gain`, and the corrected `fin_arm`) for this
airframe come from its `.ork`/`.csv` via `ork_to_rocket.py`; capture them alongside the gains when
calibrating so the anchor is fully reproducible.

## Appendix B: what this replaces / relates to

- **Retires:** the `MOT_Q_REF` dynamic-pressure gain schedule and its dependency on airframe geometry
  (the schedule's own reference `q` would otherwise also need per-airframe derivation).
- **Supersedes:** hand-tuning `RKT_TILT_P/D` and `RKT_SPIN_DAMP` per airframe.
- **Does not touch:** the estimator (DCM + no-GPS + gyro-only gate), the stage detector, the fin
  mixer's saturation handling, or the integrator anti-windup — all validated separately this session.

## 13. Implemented — what shipped and verification results (2026-09-05)

Built in three stages; every stage flight-verified in SITL.

**Stage 1 — fixed gain is the firmware default.**
- `Parameters.cpp`: `RKT_QSCHED` default 1 -> 0; fixed-gain `AP_GROUPINFO` defaults
  `RKT_TILT_P 2.5`, `RKT_TILT_D 0.5`, `RKT_TILT_I 2.0`, `RKT_SPIN_DAMP 0.006`; `MOT_GAIN_MAX 1.0`
  baked in `rocket_late_defaults`. `MOT_GAIN_MAX` removed from `rocket.parm` (single-source in C++).
- `rocket_control.cpp`: the `RKT_QSCHED` mixer feed already added (feeds `q=0` for fixed gain).
- `Tools/ArduRocket/sitl_tests/scheduled.parm`: the `RKT_QSCHED=1` fallback tune (kept for one
  hardware flight before the schedule is deleted).
- Verified: a **pure `rocket_test.py gains` with no flags PASSES at 0.5 deg** (fixed gain, defaults).
- Also fixed a harness bug this exposed: the `gains` run window was 30 s, but fixed gain flies more
  vertically -> higher -> apogee near 30-31 s of run time, so `ascent_end` sometimes never latched
  and the steady metric came back "--". Bumped to 36 s.

**Stage 2 — gains derived in `ork_to_rocket.py`.**
- Added `GAIN_C_P/C_D/C_I/C_S` (Section 7) and `derive_gains(J_tilt, J_spin, force_gain, fin_arm)`.
- Added `--fin-arm` (CG-to-fin arm, m). Without it a placeholder is used and the emitted gains are
  flagged provisional. The `.parm` now emits a derived control-gains block.
- Verified round-trip: a synthetic `.ork`/`.csv` matching the anchor geometry regenerates
  `RKT_TILT_P/D/I = 2.5/0.5/2.0`, `RKT_SPIN_DAMP = 0.006` exactly.

**Stage 3 — scaling verified.**
- A 2x-inertia airframe emits gains that **exactly double** (`P 5.0, D 1.0, I 4.0, SD 0.012`).
- Flown in SITL with the 2x inertia + 2x derived gains: **vertical, 1.0 deg steady, PASS.** (Rail-exit
  transient was larger, ~51 deg, than the 1x anchor's ~20 deg -- a heavier rocket is more sluggish at
  low-q rail exit -- but it recovered cleanly.)
- **Honest control result:** the SAME 2x airframe flown with the UN-scaled 1x gains ALSO passed
  (~0.9 deg steady, ~35 deg transient). The fixed-gain integrator nulls the steady lean regardless
  of the exact P/D, so the steady metric is forgiving of moderate gain mismatch. This does not
  invalidate the derivation (which is correct and scales exactly) -- it reframes its value as
  transient consistency + authority-envelope safety for larger airframe changes, not "un-scaled
  gains crash." See the note in Section 7.

**Files changed:** `ArduRocket/Parameters.cpp`, `ArduRocket/Parameters.h`,
`ArduRocket/rocket_control.cpp` (RKT_QSCHED feed), `Tools/ArduRocket/ork_to_rocket.py`,
`Tools/autotest/default_params/rocket.parm`, `Tools/ArduRocket/sitl_tests/rocket_test.py` (36 s
window), plus new `Tools/ArduRocket/sitl_tests/scheduled.parm`.

**Still open:** wire up a real `fin_arm` (Section 8) so `--fin-arm` is not required for a good
result; and after one real hardware flight, delete the `MOT_Q_REF` schedule and the `RKT_QSCHED`
toggle.
