# ArduRocket SITL test harness

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
| `gains` | `rocket-tilt20` | Loads `gentle_gains.parm` and checks the airframe is driven to and held **VERTICAL** (median true tilt < 8°) through the powered flight — the mission. Currently FAILS (holds ~20°); fins have authority, not pinned; PASS/FAIL |

## Starting gains (`gentle_gains.parm`)

`gentle_gains.parm` next to this script holds the Appendix B.2 gentle starting set
(`ATC_ANG_*_P 3.0`, `ATC_RAT_*_P 0.05`, `D 0.002`, `I 0`). Load it into any scenario with
`--gains`, so `step`/`wind` start from those values without hand-typing:
```bash
python3 rocket_test.py step --gains Tools/ArduRocket/sitl_tests/gentle_gains.parm
```
The `gains` scenario loads it automatically (no `--gains` needed) and reports **PASS/FAIL**
against **the mission — fly VERTICAL.** It passes only if the airframe is actually driven to
and held near 0° off vertical (median TRUE tilt < 8° over the steady window, skipping the
launch transient and the apogee tail), doesn't tumble, and the fins have authority without
being pinned. **It currently FAILS:** the airframe holds ~20° because the steering-induced
attitude-estimate error fools the controller into thinking it is already near vertical and
easing the fins off. Making the *true* attitude (not just the estimate) reach vertical is the
open work — see PLAN §3c. Do **not** relax this bar to make it pass; holding the launch lean
is a FAIL.

## Tuning loop (Proposal 1)

1. `gains` on `rocket-tilt20` — loads `gentle_gains.parm` and checks the gentle baseline (PASS/FAIL).
2. `step` on `rocket-tilt20` — confirm fins reach ~half at 20°, not saturated (gentle = full at ~40°).
3. `wind --mph 20` and `flight` on `rocket` — confirm it holds while fast, no oscillation.
4. Raise `ATC_ANG_*_P` until `step` first shows overshoot, back off ~30%.
5. **Cross-check** the final gains in MATLAB-in-the-loop (stages 0/1/3 — the MATLAB plant has
   no wind, so skip 2/4 there). Same behaviour on both plants ⇒ not overfit.

## Notes

- Metrics come from `ATTITUDE` (tilt = `acos(cos(roll)·cos(pitch))`, matching
  `tilt_from_vertical_deg()`), `SERVO_OUTPUT_RAW` (fin saturation), and `STATUSTEXT` (stage
  transitions, the give-up message).
- `sweep` needs a fresh launch per wind (arming is one-shot per flight), so it prints the
  individual `wind --mph N` commands rather than re-flying in one process.
- Real time only — do **not** run SITL with `--speedup` (it changes the physics; see the plan).
