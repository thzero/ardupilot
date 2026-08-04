# ArduRocket

A standalone ArduPilot vehicle for a **fin-steered, solid-motor hobby rocket**. Four
trailing-edge steering tabs hold the airframe **vertical** off the launch rail through
boost and coast. There is **no RC transmitter and no in-flight ground station** — the
rocket flies itself; a ground station is used only on the pad and for post-flight
telemetry.

The job is **stabilization, not guidance**: hold vertical to 0° tilt for as long as the
fins have authority. Recovery (parachute deployment) is deliberately **off-board**, on a
dedicated altimeter.

> **Why a separate vehicle and not an ArduPlane mode?** To shrink the reachable-behavior
> surface: no modes, no missions, no RC, no failsafes that can misfire. See
> [`ARDUROCKET_PLAN.md`](ARDUROCKET_PLAN.md) "Context" for the full argument. This README
> is the how-to; `ARDUROCKET_PLAN.md` is the design rationale and decision log.

**Status:** implemented and flying in SITL. Open items: attitude (`ATC_*`) gain tuning,
and the real control-tab dimensions (currently assumed).

---

## 1. How it works

### One mode, nothing selectable
There are no flight modes. `set_mode()` always returns false, so no MAVLink `SET_MODE`, no
RC switch, and no failsafe can change what the vehicle does. What it does is inferred from
measured acceleration and climb rate — never commanded. (Rationale: PLAN §2.)

### Flight stages
```
PREP → FINCHECK → ARMED → BOOST → COAST → DESCENT → LANDED → (manual disarm) → PREP
                          │ignition      │burnout   │apogee   │touchdown
```

| Stage | Meaning |
|---|---|
| `PREP` | disarmed, on the table or the rail. Fins centered. |
| `FINCHECK` | the pre-arm fin wiggle is running (not armed). |
| `ARMED` | armed on the rail, pre-launch. Attitude live (P/D), integrators held. |
| `BOOST` | launch detected, motor burning. Full fin authority, gain-scheduled on q. |
| `COAST` | burnout detected. Still steering (fins have airflow while ascending). |
| `DESCENT` | apogee passed. Fins centered, controller shut down. **Stays armed.** |
| `LANDED` | touchdown detected. Terminal; waits for a **manual disarm** after recovery. |

**Disarm is manual, by design.** The flight computer never disarms itself — not at apogee,
not at touchdown — so it keeps logging and reporting all the way down. You disarm once the
rocket is recovered. (PLAN §3.)

### The view rotation (why "vertical" reads as "level")
Attitude runs through `AP_AHRS_View(ROTATION_PITCH_90)`, so a nose-up airframe looks level
to the controller and "hold vertical" becomes "hold level" — Copter's attitude math works
unmodified, away from the pitch-90° Euler singularity. Consequences you'll see in a GCS:
- **Roll/pitch report tilt-from-vertical**, not raw body angles.
- **Heading** comes straight off the magnetometer (the EKF's nose-up yaw is meaningless),
  display-only — the compass is never in the flight solution.

(PLAN §2–§3 and the heading section.)

---

## 2. Hardware

Everything below is **config-only on the ArduPilot side** — the sensors were designed out
of the control path from the start (GPS and compass never steer). The one part that drove a
real decision is the IMU G-range. (Full detail: PLAN §0.)

| Role | Part | Notes |
|---|---|---|
| Flight controller | **iFlight BLITZ Wing H743** (ICM-45686) | H7 flash headroom; **±32 g** IMU; servo-first layout; onboard baro |
| GPS + compass | u-blox **M10 combo** (Holybro M10 / Matek M10Q-5883) | recovery position + pad heading; never control |
| Telemetry (air) | **Matek mR24-30** (mLRS RX) | on a MAVLink UART |
| Telemetry (ground) | **Matek mR24-30-TX** + 2.4 GHz antennas | bound pair; USB or WiFi to the laptop → QGC |
| Actuators | **4× servos** on `SERVO1–4` | the steering tabs |
| Recovery / deployment | **OFF-BOARD** — Featherweight BlueRaven, or 2× Eggtimer Quantum | dedicated rocketry altimeter |

**IMU G-range matters for a rocket.** This airframe peaks at ~17.7 g on boost; most FC IMUs
(ICM-42688 etc.) saturate at ±16 g and clip. The ICM-45686 runs at ±32 g (ArduPilot
configures it there, clip limit 29.5 g). A ±16 g board still *flies fine* — control is
gyro-driven and apogee detection fires near 0 g — it just clips the logged boost data.

---

## 3. Ground station setup (QGroundControl)

One-time QGC app setup, shared by real flights and simulation. **How QGC connects differs by
context:** a real flight uses the **mLRS telemetry radio** (§2, §4); simulation connects over
**TCP `127.0.0.1:5760`** to SITL (§5). For TCP, use an explicit link — not QGC's auto-connect
(UDP), which doesn't traverse WSL cleanly.

### Install the "Fin Check" button
QGC has no dialog for this — it loads action files at **startup** from a `MavlinkActions`
folder in its Documents directory. Copy `Tools/ArduRocket/qgc/ArduRocket.json` there and
restart QGC:
- Windows Daily: `Documents\QGroundControl Daily\MavlinkActions\` (OneDrive may redirect
  `Documents` — check `OneDrive\Documents` too).
- The file **must** contain `"fileType": "MavlinkActions"` and only the documented keys, or
  QGC silently ignores it.

Fallback if the button won't appear: the fin check also fires from **Motor Test**
(`MAV_CMD_DO_MOTOR_TEST`).

### Known quirks (not bugs in the vehicle)
- **"Unknown mode."** We advertise all stages by name via the full standard-modes protocol,
  but QGC receives and does not render custom modes — an open QGC bug
  ([#12549](https://github.com/mavlink/qgroundcontrol/issues/12549)) that hits PX4 too. The
  plain-language STATUSTEXT narration carries the flight state instead.
- **Flying / on-ground** is reported correctly (`landed_state`), so QGC stops saying
  "Flying" once the rocket is down even though it stays armed.

(PLAN §3d and the "what you see in QGC" table.)

---

## 4. Operating at the launch site (real flight)

You're on the pad, hardware powered. Your laptop's QGroundControl connects to the rocket
over the **mLRS telemetry radio** (§2) — the ground module on USB or WiFi. (TCP `5760` is the
*simulator* link, §5; a real flight uses the radio.) The one-time QGC app setup — the Fin
Check button — is in §3.

Then, on the rail:

1. **Fin Check.** Tap the **Fin Check** button in the GCS (§3). Each fin is driven in turn
   — full one way, full the other, then center, each held ~1.5 s — while the GCS announces
   which fin (`Rocket: testing fin N (+/- then center)`). **Watch each fin** and confirm it
   deflects the way that would push the nose back toward the rail line.
   - The check takes ~18 s (4.5 s per fin). On the rail (vertical & still) completing it
     **latches the arming gate**; off the rail it just exercises the fins and will not arm.
   - There is no software "pass": the code can't see the fins, only that the motion ran.
     **Your ARM press is the attestation** that they moved correctly.
2. **ARM.** One press. Pre-arm must pass (below); then `Rocket: armed, on the rail`.
3. **Flight.** Ignition, then the GCS narrates it in plain language:
   `liftoff` → `burnout` → `apogee at X m` → `landed - disarm when recovered`.
4. **Recover, then disarm** manually.

### Pre-arm gates (all must pass to arm)
- **Fin check latched on the rail** — the unskippable human step above.
- **Tilt within a 20° cone** of vertical (NAR/Tripoli rail-tilt limit).
- **Stationary** — gyro < 15 °/s.
- **All four fins assigned** to outputs (`SERVO1–4_FUNCTION` = 190–193).

### What software cannot check
Fin **direction**. A reversed servo horn moves the correct channel the wrong way and is
invisible to software (a clamped airframe produces no motion to observe, and comparing the
mixer command to attitude is circular). This is why the fin check puts the fins in front of
a human. Confirm direction by eye before every flight. (PLAN §3b, §3d.)

---

## 5. Running the simulator

### 5.1 — Built-in physics (default, fastest)
ArduPilot's own `SIM_Rocket` C++ plant. Use this for quick iteration.

1. **Build** (after any code or `rocket.parm` change):
   ```bash
   ./waf configure --board sitl     # only needed after editing rocket.parm
   ./waf rocket
   ```
2. **Start SITL** — real time, no `--speedup`:
   ```bash
   build/sitl/bin/rocket --model rocket -w
   ```
   It prints `SERIAL0 on TCP port 5760`, then `Waiting for connection ...`.
3. **Connect QGC** — add a **TCP** comm link to **`127.0.0.1:5760`** (or the WSL IP if
   localhost fails) and Connect. Use an explicit TCP link, **not** QGC's auto-connect
   (which is UDP and doesn't traverse WSL cleanly). The one-time Fin Check button install
   is in §3.
4. **Fly it** — Fin Check → ARM → ignition (3 s) → flight.

### 5.2 — MATLAB-in-the-loop (physics in MATLAB)
MATLAB owns the plant; ArduPilot runs the **real flight code** unchanged. State and PWM are
exchanged over UDP; the loop closes through the actual controller.

```
MATLAB (thrust, mass, aero)          ArduPilot SITL (ArduRocket)
        |--- JSON sensor state (UDP 9002) ---->|   real mixer + controller
        |<-- binary packet, 16× PWM -----------|
```

1. **Networking** (MATLAB on Windows, SITL in WSL) — the physics UDP link crosses the
   WSL↔Windows boundary. Enable **WSL2 mirrored networking** (`.wslconfig`:
   `networkingMode=mirrored`, then `wsl --shutdown`) so localhost is shared. Verify with the
   two-line UDP test in `Tools/ArduRocket/matlab/README.md`.
2. **Start MATLAB** — `cd` to `Tools/ArduRocket/matlab`, run `rocket_sim` (listens on 9002).
   Needs the **Instrument Control Toolbox** (`udpport`).
3. **Start SITL** — `build/sitl/bin/rocket --model JSON --speedup 1 -w` (lockstep: MATLAB
   paces it, no real-time race).
4. **Connect QGC** — TCP 5760, as usual.
5. **Fly it** — Fin Check → ARM → ignition (3 s). Watch it in **QGC** *and* the live readout
   / end-of-run plots in **MATLAB**.

**When to use which:** built-in = fast iteration; MATLAB = a plant you can inspect, plot and
tune against — the rig for the ATC gain work.

### 5.3 — What's modeled: validated vs assumed
`SIM_Rocket` and the MATLAB plant are derived from a real OpenRocket export (geometry,
mass, inertia, thrust, drag, static margin), validated against it (max velocity within
~1.4 %, apogee within a few percent). **The one remaining assumption is the control-tab
size** (chord/span/max deflection); fin authority scales linearly with it. Treat any gain
tuned before the tabs are measured as provisional. (PLAN §9/§9b, `matlab/README.md`.)

### 5.4 — Adding a new rocket (the extraction tool)
Every airframe constant comes from two OpenRocket exports, and neither alone is enough:
- the **`.ork`** design file has fin geometry and component masses but **not inertia**
  (OpenRocket computes inertia at runtime and does not store it);
- the **CSV** export has the computed inertia, mass and trajectory but **not fin geometry**.

So a new rocket needs both. Generate its config deterministically:

```bash
python3 Tools/ArduRocket/ork_to_rocket.py DESIGN.ork FLIGHT.csv --name NAME
```

This emits:
- `NAME.parm` — the SITL config (`SIM_RKT_*` + `RKT_*` defaults),
- `NAME_params.m` — the MATLAB params struct,

running the **same fin-geometry derivation as the C++**, so the built-in and MATLAB sims
agree. It deliberately does **not** guess what it cannot know: the **control-tab dimensions,
fin arm and static margin** are left at defaults and flagged **SET BY HAND** — edit those and
re-run so the derived fin force and stability update. The tool exists because hand
transcription is exactly where the early physics errors crept in (an inertia off by ~3×, a
stability figure from a stale export, a drag area with two compensating unit errors); it
makes the extraction repeatable.

---

## 6. Configuration

Key parameter groups (all visible in the GCS parameter editor):

| Prefix | What |
|---|---|
| `RKT_*` | flight-stage detection — launch/burnout/apogee thresholds and debounces (`RKT_LAUNCH_G`, `RKT_BURN_G`, `RKT_APOG_MS`, …); `RKT_LEVEL_Q` / `RKT_MIN_Q` blend-to-vertical and low-q cutoff |
| `MOT_*` | fin mixer — `MOT_Q_REF` (gain-schedule reference, 600 by measurement), `MOT_GAIN_MAX` |
| `ATC_*` | attitude controller (rate/angle PIDs) — **untuned placeholders** |
| `SIM_RKT_*` | the SITL airframe (mass, motor, inertia, fin geometry, tab dimensions, drag) |

### Before any real flight
- **Fin check** must be run on the rail (unskippable via pre-arm), and fin **direction**
  confirmed by eye.
- Set the **real control-tab dimensions** (`SIM_RKT_TAB_C/TAB_SPAN/TAB_MAX`) once measured.
- Set **`AHRS_ORIENTATION`** to how the board is actually mounted.
- **Tune `ATC_*`** against the corrected plant (open item).
- Review the arming checklist (`ARMING_CHECK`) per your airframe.

---

## 7. Architecture (for developers)

- **Vehicle:** `ArduRocket/` (19 files) — scaffolding modeled on Blimp, control objects on
  Copter. No `mode*.cpp`, no RC, no mission. Key trick:
  `ahrs.create_view(ROTATION_PITCH_90)` in `system.cpp`.
- **Mixer:** `libraries/AP_Motors/AP_FinMixerRocket` — subclasses `AP_MotorsMulticopter`,
  four fins summed per-fin, gain-scheduled on dynamic pressure (`Q_REF/q`, not thrust).
- **Stage detection:** `libraries/AP_Rocket` — launch/burnout/apogee from accel + climb
  rate, all debounced.
- **Scheduler:** 400 Hz loop; fast tasks (INS, fin output, AHRS, control) + scheduled tasks
  (GPS 50 Hz logging, baro/density 10 Hz, telemetry 5 Hz, GCS 400 Hz). See PLAN §4 for the
  full table and the air-density caching note.
- **Registration:** `APM_BUILD_ArduRocket` (AP_Vehicle_Type.h), `AP_PARAM_FRAME_ROCKET`,
  SITL model table, autotest entries. (PLAN §1, §4–§8.)

---

## 8. Safety

- **Fin direction is a human check** — unverifiable in software; confirm by eye every flight.
- **Manual disarm** — the vehicle never disarms itself; you disarm after recovery.
- **Recovery is off-board** on a dedicated altimeter, so deployment never depends on the
  flight computer's apogee estimate.
- **No RC, no mission, no GCS failsafe** — the misfire surface this project exists to remove.
  A telemetry radio (mLRS) is a downlink only and does not reintroduce a control path.

---

## 9. Reference & status

- **[`ARDUROCKET_PLAN.md`](ARDUROCKET_PLAN.md)** — the design rationale and decision log:
  why each choice was made, the physics corrections, the bug history, verification results.
  This README links into its §-numbers for the "why."
- **`Tools/ArduRocket/matlab/README.md`** — the MATLAB plant and JSON-bridge details.
- A **thrust-vector-control (TVC)** variant is scoped but not implemented (PLAN Appendix A).

**Open items:** `ATC_*` gain tuning (hold vertical as long as possible against the corrected
plant), and setting the real control-tab dimensions.
