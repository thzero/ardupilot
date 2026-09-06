# ArduRocket SITL test harness

> Tooling notes, not spec. The authoritative docs are `ArduRocket/ARDUROCKET_*.md` (README = operate,
> PLAN = design record, STATUS = current state + what's next); this defers to them.

Repeatable, scripted SITL runs for tuning the attitude gains (ARDUROCKET_PLAN.md **B.2.1**)
and for exercising the tilt give-up backstop (**B.3**).

## In one picture

Two terminals — one runs SITL, the other runs the test:
```
Terminal 1 (SITL):  build/sitl/bin/rocket --model rocket -w      # or rocket-tilt20
Terminal 2 (test):  python3 Tools/ArduRocket/sitl_tests/rocket_test.py step
```
`rocket` and `rocket-tilt20` are the two **literal** model names — type one of them after
`--model` (no angle brackets, no `|`). The script **force-arms** (skipping the pre-arm fin
check — a human step, irrelevant to a plant test), lets it fly, and prints **max tilt,
final tilt, whether the fins hit the stops, the stage transitions, and give-up PASS/FAIL**.

## The two setup levers

| Situation set by | How |
|---|---|
| **Rail tilt** | the **model string at launch** — `--model rocket` (vertical) or `--model rocket-tilt20` (20° rail, the NAR/Tripoli max). Cannot be changed at runtime. |
| **Wind** | runtime params `SIM_WIND_SPD/DIR/TURB`, set by the script. |

The script **force-arms** (past the pre-arm fin check — that's a human step, not part of a
plant test). Ignition follows ~3 s later, as in a normal flight.

## Running

**Terminal 1 — SITL** (pick the model the scenario needs):
```bash
build/sitl/bin/rocket --model rocket        -w      # vertical rail
build/sitl/bin/rocket --model rocket-tilt20 -w      # 20° rail (max legal launch angle)
```

**Terminal 2 — the test:**
```bash
python3 Tools/ArduRocket/sitl_tests/rocket_test.py <scenario>
```
Add `--url tcp:<WSL-IP>:5760` if you run the script from Windows instead of inside WSL.

## The schedule

| Scenario | SITL model | What it checks |
|---|---|---|
| `quiet` | `rocket` | Stage 0 — on the rail, vertical, no wind: fins quiet, not saturated |
| `step` | `rocket-tilt20` | Stage 1 — null a 20° rail to vertical; fins ~half at 20° (gentle), ≤1 overshoot |
| `wind --mph N` | `rocket` | Stage 2 — crosswind disturbance rejection; holds while fast, no oscillation |
| `flight` | `rocket` | Stage 3 — nominal full flight; tilt small through boost, degrades near apogee |
| `sweep` | `rocket` | Stage 4 — prints the per-wind commands for the 0→20 mph envelope |
| `giveup` | `rocket-tilt20-unstable` | Backstop — an unstable airframe genuinely tumbles past `RKT_GIVEUP_DEG=45`; the hardened backstop needs a real *rotating* departure (gyro-corroborated), not a steady lean; PASS/FAIL |
| `gains` | `rocket-tilt20` | Checks the airframe is driven to and held **VERTICAL** (median true tilt < 2°) through the powered flight — the mission. **PASSES** on the baked config (steady ~0.4°). Pass `--gains <file>` only to experiment with `RKT_*`/`MOT_*` knobs (`ATC_*` gains are inert for ascent); `--wind N` adds a crosswind. PASS/FAIL |

## The `gains` mission grade — fly VERTICAL

The `gains` scenario reports **PASS/FAIL** against **the mission — fly VERTICAL.** It passes only
if the airframe is actually driven to and held near 0° off vertical (median TRUE tilt < 2° over the
steady window — 20–60% of ascent, skipping the launch transient and the apogee tail), doesn't tumble
during powered flight, and the fins have authority without being pinned. On the baked config it
**PASSES** — a 20° rail drives to steady ~0.4° and holds ~0.2–2° through the whole powered/coast
ascent. The `RKT_TILT_I` integral term in the direct tilt law is what drives the steady lean to zero.

The grade also prints two peak numbers — read them correctly:
- **`powered-flight max TRUE tilt`** (first 75% of ascent, apogee excluded) — the real flight-quality
  peak while the fins have authority. It is ~= the launch lean; the rocket never overshoots it.
- **`peak tilt (FULL ascent)`** — includes the unavoidable **apogee nose-over** (climb→0, q→0, the
  rocket tips over at the top of its arc). This is NOT a flight-quality number and NOT a fault; every
  rocket does it. Do not read it as a rail-exit or wind excursion.

`ATC_*` gains are **inert for ascent** (the direct tilt/spin law in `rocket_control.cpp` overrides the
cascade during BOOST/COAST), so the real ascent knobs are `RKT_TILT_P/D/I`, `RKT_SPIN_DAMP`, and
`MOT_Q_REF`/`MOT_GAIN_MAX`. Pass `--gains <file>` only to experiment with those. To fly the PASSIVE
airframe with the tabs frozen (no control, for airframe-vs-controller diagnosis), load `baseline.parm`.
Do **not** relax the 2° bar to make a tune pass; holding the launch lean is a FAIL.

## Tuning loop

Gains are derived from the OpenRocket model (`ork_to_rocket.py`) and baked as the firmware default,
so per-rocket hand-tuning should not be needed. When you do want to experiment, tune the **ascent**
knobs — `RKT_TILT_P/D/I`, `RKT_SPIN_DAMP`, `MOT_Q_REF`/`MOT_GAIN_MAX` — NOT `ATC_*` (the direct law
overrides the cascade in BOOST/COAST, so `ATC_*` is inert for ascent).

1. `gains` on `rocket-tilt20` (optionally `--gains <file>`, `--wind 20`) — the mission grade; want
   PASS with steady TRUE tilt < 2°.
2. Adjust `RKT_TILT_P/D/I` for the transient/steady balance; keep `RKT_SPIN_DAMP` scaled with
   `MOT_Q_REF` (spin moment gain ~ `RKT_SPIN_DAMP·MOT_Q_REF` — move them together).
3. `wind --mph 20` and `flight` on `rocket` — confirm it holds while fast, no oscillation.
4. **Cross-check** the final gains in MATLAB-in-the-loop (stages 0/1/3 — the MATLAB plant has
   no wind, so skip wind cases there). Same behaviour on both plants ⇒ not overfit.

## Notes

- Metrics come from `ATTITUDE` (tilt = `acos(cos(roll)·cos(pitch))`, matching
  `tilt_from_vertical_deg()`), `SERVO_OUTPUT_RAW` (fin saturation), and `STATUSTEXT` (stage
  transitions, the give-up message).
- `sweep` needs a fresh launch per wind (arming is one-shot per flight), so it prints the
  individual `wind --mph N` commands rather than re-flying in one process.
- Real time only — do **not** run SITL with `--speedup` (it changes the physics; see the plan).
