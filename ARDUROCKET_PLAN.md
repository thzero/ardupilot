# ArduRocket — a standalone vehicle folder

> ## STATUS: IMPLEMENTED AND FLYING IN SITL (2026-07-18)
>
> The design below was implemented on the `ardurocket` branch and **passes a full
> simulated flight**:
>
> ```
> PREP -> ARMED -> BOOST -> COAST -> DESCENT -> PREP
> fin travel:  0us     1us    39us    400us     stop
>              ignition ....... burnout ..... apogee
> ```
>
> Verified: fins centred when disarmed, quiet on the rail (integrator gating),
> live under boost, **still steering through coast after burnout**, and stopped at
> apogee followed by auto-disarm. ArduPlane still builds clean with QROCKET removed.
>
> **NOT verified — required before any real motor:**
> - **Gains are placeholders, never tuned.** Coast deflection hit ~400us, which is
>   ~80% of full fin travel — near saturation. Tune `MOT_Q_REF` / `MOT_GAIN_MAX`
>   and the `ATC_*` gains in SITL first.
> - **`ARMING_SKIPCHK -1`** in the defaults is a SITL bring-up shortcut. Use
>   `21064` for real flight (see `Tools/autotest/default_params/rocket.parm`).
> - **FIN DIRECTION IS NOT VERIFIED BY SOFTWARE — IT CANNOT BE.** The pre-arm fin
>   check wiggles the fins and blocks arming until a human confirms, but it does
>   **not** detect a reversed servo, a backwards linkage, or fins mounted in swapped
>   positions. Those will arm and fly. A human must watch the wiggle and check each
>   fin. See §3b.
>
> Run the SITL flight test with the scripts in the session scratchpad
> (`run_fly.sh` / `flytest.py`), the bench fin test with (`run_bench.sh` /
> `bench_fin_test.py`), or drive `bin/rocket --model rocket` yourself.
>
> ### Bugs found during bring-up (each silently broke everything downstream)
>
> | Bug | Symptom |
> |---|---|
> | Missing `ins.init()` in `init_ardupilot()` | IMU sample rate never set; main loop blocked forever in `wait_for_sample()`. Vehicle appeared to boot but nothing ran. **The big one.** |
> | Missing `srv.push()` in `motors_output()` | Mixer computed fin commands that never reached the outputs; servos sat at 0 PWM. |
> | Missing armed propagation in `AP_Arming_Rocket::arm()` | `motors->armed()` and `hal.util->set_soft_armed()` never set, so the stage machine stayed in PREP and the sim never ignited. |
> | Wrong param name `ARMING_CHECK` | The parameter is **`ARMING_SKIPCHK`** in this tree. Wrong names fail silently. |
> | `AP_Arming` not registered in `Parameters.cpp` | All `ARMING_*` params simply did not exist. |
> | Loop rate defaulted to 50 Hz | ArduRocket was not in the 400 Hz group in `AP_Scheduler.cpp`. |
> | Apogee used GPS-backed velocity | Switched to `get_velocity_D(velD, true)` (baro+IMU) since there is no GPS. |
>
> Lesson for next time: **diff a new vehicle's startup against a known-working
> minimal vehicle (Blimp) before instrumenting anything.** Most of the above were
> visible by comparison and cost far more to find by probing.

## Context

The goal is a hobby rocket held **vertical** (attitude hold to 0° tilt) by **4 steering fins**, on a
**solid motor** (no thrust control), with **no RC transmitter and no ground station**. Holding vertical
is stabilization, not guidance. The drivers are **safety surface** (fewer reachable modes and failsafes
that can misfire) and **simplicity/readability**. Flash is not a constraint.

**Motor burn time** varies widely: typically **0.8–8 s**, with outliers up to **16 s**. Nothing in the
firmware or the simulator may assume a fixed or short burn.

**Actuator note (fins vs CV).** Aerodynamic **fin tabs** work off airflow, so they keep authority for as
long as the rocket is moving upward through the air — *including after burnout*. A **control vane** (CV,
a jet vane in the exhaust) only has authority while the motor is firing, so a CV loses control at
burnout. This design is fins-first; the stage machine below is written so burnout is merely *recorded*,
and the real "stop steering" trigger is **apogee** (vertical velocity going negative), which is correct
for fins. A CV variant would additionally stop at burnout.

The prior approach (commit `3484b723a3`) added a `QROCKET` mode inside ArduPlane, subclassing
`ModeQStabilize`. Its README argued a separate vehicle was infeasible because the tailsitter/QuadPlane
stack is "welded to the global `plane` singleton." **That claim does not survive inspection**, and this
plan replaces the approach.

Evidence gathered:

- Of `quadplane.cpp`'s 455 `plane.` references, **68% are transition/fixed-wing machinery** (mode-pointer
  identity 116, mission/L1/WP 110, TECS 13, airspeed ~25) that a rocket deletes rather than ports. Only
  ~9% are essential to attitude hold.
- **33 of 103 QuadPlane methods have zero `plane.` references**, including `hold_hover()` — the actual
  hover primitive. `quadplane.h`, `tailsitter.h`, `transition.h` have zero.
- The minimum hover object set (`quadplane.cpp:763-785`) is four objects, and its construction chain
  contains no `plane.` at all.
- **ArduSub is the existence proof**: `Sub.h:261-338` reuses this same attitude stack in a different
  medium via a per-vehicle `AC_AttitudeControl` subclass.

**ArduPlane's real contribution to a rocket is one line** — `ahrs.create_view(ROTATION_PITCH_90)` at
`quadplane.cpp:763`. Everything else worth having is a library already shared by Copter and Sub.
Basing on ArduPlane would drag in TECS, L1, mission, airspeed and transition state machines — precisely
the misfire surface this project exists to remove.

### Axis mapping (get this right; it reaches the pad)

Under `AP_AHRS_View(ROTATION_PITCH_90)`, a nose-up airframe looks level to the controller, so
"hold vertical" becomes "hold level" and Copter's math works unmodified. The axes remap:

| View axis | Body axis | Physical meaning for a rocket |
|---|---|---|
| view **roll** | about body Z | **tilt** away from vertical |
| view **pitch** | about body Y | **tilt** away from vertical |
| view **yaw** | about body X | **spin** about the long axis |

Four fins give authority on all three. Write this table into a header comment; verify on the bench.

### Build and run

```
./waf configure --board sitl     # required after ANY change to rocket.parm
./waf rocket                     # builds build/sitl/bin/rocket
build/sitl/bin/rocket --model rocket --speedup 5 -w
```

Note the reconfigure: `Tools/autotest/default_params/rocket.parm` is embedded into ROMFS at
**configure** time (`Tools/ardupilotwaf/boards.py` globs `default_params/`), so editing it and only
running `./waf rocket` silently keeps the old values.

---

## 1. New vehicle: `ArduRocket/` (~14 files)

Scaffolding from **Blimp** (the only modern minimal vehicle); control objects from **Copter**.
Waf auto-discovers vehicle folders and derives `-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduRocket` from the
directory name (`Tools/ardupilotwaf/ardupilotwaf.py:136`) — no build registration needed.

| File | Contents | Model on |
|---|---|---|
| `wscript` | `ap_stlib` + `ap_program(program_name='rocket')`. Libraries: `ap_common_vehicle_libraries()` + `AC_AttitudeControl`, `AP_Motors`, `AC_PID`, `AP_Rocket`. Omit `AP_Mission`, `AC_WPNav`, `AP_Avoidance`. | `Blimp/wscript` |
| `version.h` | `THISFIRMWARE`, `FIRMWARE_VERSION`. Mandatory — defines `AP_FWVersion::fwver`; linker error without it. | `Blimp/version.h` |
| `config.h` | Loop rate, and `AP_MISSION_ENABLED 0` / `AC_WPNAV_ENABLED 0`. **The SITL frame macros do NOT go here** — the original plan was wrong about that: `libraries/AP_HAL/SIMState.cpp` is a library and never includes a vehicle's `config.h`, so its `AP_SIM_FRAME_CLASS`/`_STRING` ladders must be edited directly (§8). | `Blimp/config.h` |
| `ArduRocket.h` | `class ArduRocket : public AP_Vehicle`. Members: `Parameters g; ParametersG2 g2; AP_FinMixerRocket *motors; AC_AttitudeControl_Multi *attitude_control; AP_AHRS_View *ahrs_view; AP_Arming_Rocket arming; AP_Rocket rocket;` + `stage`. **No `AC_PosControl`, `AC_WPNav`, `AC_Loiter`.** | `Blimp/Blimp.h` shape; `ArduCopter/Copter.h:474-478` member set |
| `ArduRocket.cpp` | Scheduler table (§4), `get_scheduler_tasks()`, `const AP_HAL::HAL& hal`, and the globals block: `ArduRocket rocket; AP_Vehicle& vehicle = rocket; AP_HAL_MAIN_CALLBACKS(&rocket);` — `AP_Vehicle& vehicle` is **load-bearing** (`AP_Vehicle.cpp:307` `extern AP_Vehicle& vehicle;`, consumed at static-init by `SCHED_TASK_CLASS`). `motors_output()` must do `calc_pwm()` → `cork()` → `output_ch_all()` → `motors->output()` → **`srv.push()`**; without the push the fin commands never reach the outputs and the servos sit at 0 PWM. | `Blimp/Blimp.cpp`, `Blimp/motors.cpp` |
| `Parameters.h/.cpp` | `const AP_Param::Info ArduRocket::var_info[]` — **mandatory**; `load_parameters()`/`check_var_info()` run unconditionally in `AP_Vehicle::setup()`. Minimum `FORMAT_VERSION` + `AP_VAREND`. `AP_SUBGROUPINFO` for `RKT_`, `ATC_`, `MOT_`, `SERVO_`, and **`GOBJECT(arming, "ARMING_", AP_Arming_Rocket)`** — omit that and the `ARMING_*` params silently do not exist. | `Blimp/Parameters.cpp` |
| `system.cpp` | `init_ardupilot()`, `allocate_motors()`, **`ahrs_view = ahrs.create_view(ROTATION_PITCH_90);`** ← the key trick, and **`ahrs.init()` + `ins.init(scheduler.get_loop_rate_hz())`** — mandatory; without the IMU init the main loop blocks forever in `wait_for_sample()` and the vehicle never runs. | `ArduCopter/system.cpp:358-431`, `Blimp::startup_INS_ground()` |
| `rocket_control.cpp` | The whole flight controller (§3). ~150 lines. Replaces the entire `mode*.cpp` family. | `ArduCopter/mode_stabilize.cpp:9-60` (spool-state switch) |
| `AP_Arming_Rocket.h/.cpp` | `arm()` must reset the stage detector AND propagate the armed state: `hal.util->set_soft_armed(true)`, `motors->armed(true)`, logger/notify, and `ahrs.resetHeightDatum()` when there is no home (there never is — no GPS). Omitting the propagation leaves the stage machine stuck in PREP. Also overrides `rc_calibration_checks() → true` (no RC) and enforces vertical-and-still on the rail. | `Blimp/AP_Arming_Blimp.*` |
| `GCS_Rocket.*`, `GCS_MAVLink_Rocket.*` | Required — `GCS::create_gcs_mavlink_backend()` is pure virtual under `HAL_GCS_ENABLED`. `frame_type()` → `MAV_TYPE_GENERIC` (no upstream `MAV_TYPE_ROCKET`). `custom_mode()` → flight stage. `base_mode()` must OR in `MAV_MODE_FLAG_SAFETY_ARMED` or every GCS shows the vehicle disarmed while it is live. Overrides `handle_command_int_packet()` to catch `MAV_CMD_DO_MOTOR_TEST` for the bench fin test (§3b). | `Blimp/GCS_*` |
| `Log.cpp` | `RKT` message: stage, tilt error, fin commands, spin rate, body-up accel, dynamic pressure. | `Blimp/Log.cpp` |
| `RC_Channel_Rocket.h/.cpp` | **(added during implementation, not in original plan)** A minimal `RC_Channels` subclass. The vehicle has no receiver, but shared code (`AP_CRSF_Telem::queue_message`, reached from `GCS::send_text`) dereferences the `rc()` singleton unconditionally, so the object must exist or the process segfaults on the first status text. | `Blimp/RC_Channel_Blimp.*` |

`AP_Vehicle` has exactly **five pure virtuals**: `set_mode`, `get_mode`, `get_scheduler_tasks`,
`init_ardupilot`, `load_parameters`. `setup()`/`loop()` are `override final`.

Absent vs Blimp: all `mode*.cpp`, `radio.cpp`, `commands.cpp`, `ekf_check.cpp`.

## 2. No modes at all

Implement the two mode virtuals as:

```cpp
bool ArduRocket::set_mode(uint8_t, ModeReason) override { return false; }  // nothing can ever change mode
uint8_t ArduRocket::get_mode() const override { return (uint8_t)stage; }
```

`set_mode()` hard-`false` means no MAVLink `SET_MODE`, no RC switch, no failsafe and no GCS can change
behavior — the safety requirement expressed in three lines, and provably exhaustive: there is no
reachable alternate behavior to audit. A one-mode `Mode` hierarchy gives identical runtime behavior but
retains the plumbing and implies modes exist.

## 3. Flight stage machine (internal state, not a settable mode)

```
PREP    -> disarmed. May be on the table OR on the rail. Fins centered.
ARMED   -> armed, on the rail, pre-launch. Attitude live (P/D), I-terms held. Rail holds attitude.
BOOST   -> launch detected. Full fin authority, gain-scheduled on q.
COAST   -> burnout detected. Burnout is RECORDED ONLY -- no control change. Fins keep steering,
           because aerodynamic fins still have airflow while the rocket is ascending.
DESCENT -> apogee reached (vertical velocity negative). Stop ALL fin activity; do not steer on the
           way down.
```

Key points, per the airframe owner:

- **PREP vs ARMED.** PREP is the getting-ready state and covers both the bench/table and the rail.
  Arming is only permitted on the rail, so ARMED always implies "on the rail" (enforce this in the
  pre-arm checks, e.g. require a level/vertical attitude and stillness).
- **Burnout does NOT stop the controller.** For fin tabs, burnout is just an event to detect and log.
  The rocket is still moving upward through the air, so the fins still steer. (A CV/jet-vane variant is
  the exception: it *would* stop at burnout, because a jet vane has no authority once the exhaust stops.)
- **Apogee is the stop trigger.** Detect the top of the climb — simplest robust signal is vertical
  velocity going negative — and cease all fin activity. Nothing should flail on descent. Recovery
  deployment (pyro/altimeter) stays out of scope.

This revises the earlier draft, which incorrectly shut the controller down at burnout.

## 3b. Arming sequence, rail capture, and the fin check

Arming is a **two-phase handshake**, not a single command:

```
ARM (1st)  -> pre-arm checks run
           -> FINCHECK stage: fins wiggle one at a time, each announced
           -> returns FAILED. The vehicle is NOT armed.
ARM (2nd)  -> operator confirming what they saw
           -> rail attitude captured, vehicle arms
```

Latched for the power cycle: a disarm/re-arm after a scrubbed countdown does not
repeat the wiggle. A reboot does. 60 s confirmation timeout falls back to PREP so it
cannot sit half-committed.

### What the fin check DOES NOT do

**It does not verify fin direction. Software cannot verify fin direction on the pad.**
This is the single most important thing to understand about this feature, because the
failure it does not catch is the one most likely to destroy the rocket.

Verifying direction means confirming that a commanded deflection produces the *correct
motion*. On a clamped rail there is no motion to observe, and with no airflow a fin
deflection produces no moment at all. Comparing the mixer's output against measured
attitude is **circular**: the command is derived from that attitude, so the signs always
agree by construction -- including with a servo horn fitted backwards.

Therefore the following are **NOT** detected, and will arm and fly:

- a reversed servo (`SERVOn_REVERSED` wrong)
- a control horn or linkage fitted backwards
- fins physically mounted in swapped positions
- an incorrectly signed mixer

### What it DOES do

It refuses to let the check be skipped, and puts the fins in front of a human:

- Drives **one fin at a time**, full one way, full the other, centre.
- **Announces each fin as it starts moving** (`Rocket: fin 3`), so the crew can confirm
  the fin that moves is the fin that was named -- this is what makes a swapped output
  channel visible.
- **Blocks arming** until a human sends ARM a second time to confirm.

The operator's job during the wiggle is to check, for each announced fin:
1. the fin that moves is the one named, and
2. it deflects in the direction that would push the nose **back toward** the rail line.

If either is wrong, do not send the second ARM.

### What IS genuinely automated

| Check | Catches | Result if it fails |
|---|---|---|
| All four fin functions assigned to outputs | forgot `SERVOn_FUNCTION`, typo | arm refused |
| Tilt within 20 deg of vertical | mis-mounted airframe, bad attitude solution | arm refused |
| Stationary (gyro < 15 deg/s) | being carried or shaken | arm refused |

20 degrees is the maximum rail tilt permitted by both **NAR and Tripoli** safety codes,
so anything beyond it is either mis-mounted or a bad attitude solution.

### Bench fin test (on demand, from a ground station)

`MAV_CMD_DO_MOTOR_TEST` triggers the same fin sequence on demand, so the fins can be
exercised on the bench without attempting to arm. Mission Planner and QGC already have
UI for this command, so no custom tooling is needed.

- **Command parameters are ignored.** The sequence is fixed, and is the *identical code
  path* used by the arming check -- so what is verified on the bench is exactly what
  runs on the rail.
- **Refused while armed** (`MAV_RESULT_TEMPORARILY_REJECTED`). Control surfaces are not
  moved on a live vehicle.
- **A bench run does NOT satisfy the arming fin-check gate.** This is deliberate and
  load-bearing: otherwise a wiggle in the workshop would clear the gate, and the rocket
  could then be armed on the rail without anyone having watched the fins there. Bench
  runs finish by returning to PREP and clearing themselves; the next ARM still runs its
  own check.

What the bench test can and cannot catch is the same as on the rail: pairing an
announced fin with the channel that actually moved catches **wiring and output-assignment
errors**, but a reversed servo moves the correct channel the wrong way and remains
invisible to software. Direction is still a human judgement.

### Rail attitude capture — a calibration, not the target

**The target is always true vertical.** The rocket corrects to vertical once it is
flying; that is the whole point of the vehicle.

What the second ARM records is *how far off vertical the rail points*. That measurement
exists so the vehicle can reach vertical without demanding the entire correction as a
step input at the worst possible moment. Rails are routinely tilted a few degrees (into
wind, away from the crowd; NAR/Tripoli cap it at 20°), and commanding vertical from
ignition would ask for the full correction while still on the rail and again at rail exit
— precisely where dynamic pressure, and therefore fin authority, is lowest.

So the attitude target **blends from the measured rail attitude to true vertical as
dynamic pressure rises**:

```
target = rail_attitude * (1 - k),   k = clamp(q / RKT_LEVEL_Q, 0, 1)
```

- On the rail: `q ≈ 0` → target is the rail attitude → no error, no windup, and the fins
  have no authority anyway.
- Climbing out: the target sweeps toward vertical exactly as fast as the fins gain the
  authority to follow it.
- At `q ≥ RKT_LEVEL_Q` (default 600 Pa, ≈31 m/s): fully vertical for the rest of the flight.

Set `RKT_LEVEL_Q` lower to level out sooner and more aggressively, higher to level out
more gently, or `0` to command vertical immediately with no blend.

**So fin travel SHOULD rise with rail tilt** — a rocket launched 20° off vertical has more
correcting to do than one launched vertical. That is correct behaviour, not a failure.

## 4. Scheduler table

`Blimp/Blimp.cpp:50-95` is the shape. Entries **must be priority-ordered**; the table is interleaved
with `AP_Vehicle::get_common_scheduler_tasks()`.

```cpp
FAST_TASK_CLASS(AP_InertialSensor, &rocket.ins, update),
FAST_TASK(motors_output),        // fins first, minimum latency
FAST_TASK(read_AHRS),
FAST_TASK(run_rocket_control),   // launch detect + attitude, 400 Hz
SCHED_TASK(update_batt_compass, 10, 120, 12),
SCHED_TASK(update_altitude,     10, 100, 21),   // baro: speed estimate + apogee cross-check
SCHED_TASK(one_hz_loop,          1, 100, 39),
SCHED_TASK_CLASS(GCS, ..., update_receive/update_send, 400, ..., 51/54),
// + logging tasks
```

Omitted, with reasons: `rc_loop`/`read_radio` (no RC — removes the RC failsafe surface entirely);
`three_hz_loop`/`failsafe_gcs_check` (a GCS failsafe on a rocket is a misfire source);
`ekf_check`/`check_vibration` (these trigger *mode changes* in Copter; inert given §2, and a rocket is
guaranteed high-vibration — `check_vibration` would fire every flight); `AP_GPS::update` (attitude hold
needs no position; add at 50 Hz/prio 9 only if position logging is wanted).

**Keep the GCS tasks** despite "no ground station" — autotest drives the vehicle over MAVLink. Keep the
transport, delete the failsafes.

## 5. `AP_FinMixerRocket` — new mixer (`libraries/AP_Motors/`)

Subclass `AP_MotorsMulticopter`. `AP_MotorsMulticopter::output()`
(`AP_MotorsMulticopter.cpp:269-300`) is a template method — implement only `output_armed_stabilizing()`
and `output_to_motors()`, plus `init()`, `set_frame_class_and_type()`, `get_motor_mask()`,
`_get_frame_string()`, `_output_test_seq()`. Target `AP_MotorsTailsitter.h`'s 60 lines / 245 `.cpp`.

**Geometry — reuse `AP_MotorsSingle`'s 4-vane mix** (`AP_MotorsSingle.cpp:175-183`), which maps exactly
onto 4 rocket fins (opposing pairs antisymmetric for tilt; all four together for spin):

```cpp
actuator[0] =  rp_scale * roll_thrust  - yaw_thrust;   // fin 1
actuator[1] =  rp_scale * pitch_thrust - yaw_thrust;   // fin 2
actuator[2] = -rp_scale * roll_thrust  - yaw_thrust;   // fin 3
actuator[3] = -rp_scale * pitch_thrust - yaw_thrust;   // fin 4
```

**Gain scheduling — do NOT copy `AP_MotorsSingle`'s `/ thrust_out_actuator`** (`:227`). Its vanes sit in
prop wash, so authority ∝ thrust. Rocket fins sit in **freestream**, so authority ∝ q = ½ρv².
Scale by `1/q` using EKF vertical velocity (baro+IMU; no GPS or pitot needed — for a vertical rocket
|v| ≈ |v_z|), with a floor to bound gain at low speed. Vehicle supplies q via `set_dynamic_pressure()`.

**Do NOT inherit the `AP_MotorsSingle` bug** at `:208-212`: `actuator_max` starts at `0.0f` and the
comparison is inverted (`>` should be `<`), so the body never executes, `actuator_max` stays 0, and the
saturation branch at `:213` is unreachable dead code — saturation is never detected. Consider a separate
upstream PR to fix it.

**Set `limit.roll/pitch/yaw` honestly.** They feed straight back into `AC_PID::update_all()`
(`AC_AttitudeControl_Multi.cpp:474-482`) to stop I-windup. Fins have genuine 3-axis authority, so all
three are set on saturation (unlike a TVC frame, which would pin `limit.yaw` permanently).

**Throttle is a fiction** (solid motor). `AC_AttitudeControl_Multi`'s ctor requires an
`AP_MotorsMulticopter&`, so the subclass exists partly to satisfy that. `_throttle_in` is ignored by the
mixer; thrust linearization and battery compensation are dead paths. **Risk: the inherited spool state
machine transitions on throttle and may never reach `THROTTLE_UNLIMITED`** — verify early. (Implementation
resolved this by calling `motors->set_spoolup_block(false)` every loop, as ArduPlane does at
`quadplane.cpp:1953`, and driving the desired spool state from the flight stage.)

Frame: the implementation added `MOTOR_FRAME_ROCKET = 18` to `AP_Motors_Class.h` and four new
`k_rocketFin1..4` (190–193) SRV_Channel functions rather than reusing elevon/vtail names.

Use `AC_AttitudeControl_Multi`, **not** `AC_AttitudeControl_TS` — the TS variant's only two additions
(`relax_attitude_controllers(exclude_pitch)`, `input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad()`)
exist solely to handle the hover↔forward transition.

## 6. `AP_Rocket` — API changes

The vehicle-agnostic design was correct and pays off here: the library lifts into `ArduRocket/` almost
unchanged. Required changes:

- **Stage detector.** Replace the single latched `launched()` boolean with a `stage()` returning
  `{PRE_LAUNCH, BOOST, COAST, DESCENT}`. Transitions, all debounced:
  - PRE_LAUNCH → BOOST: body-up accel above the launch threshold (default ~1.5 g).
  - BOOST → COAST: **burnout, detected and recorded only** (body-up accel drops below ~0.2 g while
    launched). This causes **no control change** for fins; it exists so it can be logged and reported.
  - COAST → DESCENT: **apogee** — vertical velocity goes negative. This is the trigger that stops fin
    activity. (Detecting via velocity means the library needs a vertical-velocity input from the vehicle,
    or the vehicle owns the apogee test and the library just carries the flag.)
  - `hold_integrators()` stays `stage() == PRE_LAUNCH`. `steering_active()` = `stage() ∈ {BOOST, COAST}`.
- **Add debounce to launch detection.** The README already flags "single-sample launch detection". A
  single accel sample crossing 1.5 g is a hair-trigger on a vibrating pad; require N consecutive samples.
- Burn time varies 0.8–8 s (up to 16 s), so any timeout guards must span that range, not assume ~2 s.
- `AP_Rocket_config.h:11`: change `APM_BUILD_TYPE(APM_BUILD_ArduPlane)` → `APM_BUILD_TYPE(APM_BUILD_ArduRocket)`.

## 7. Delete the ArduPlane QROCKET code — DONE

Remove `ArduPlane/mode_qrocket.cpp`, `ModeQRocket` + `QROCKET=27` from `mode.h`, the case in
`control_modes.cpp:77`, the friend+member in `Plane.h:169,319` and `quadplane.h:62`, and the `RKT_`
`ParametersG2` subgroup (index 42). QROCKET's whole value proposition was "reuse rather than fork";
once `ArduRocket/` exists it is a second, divergent implementation reachable from a vehicle with a large
failsafe surface. Git history retains it.

## 8. Registration checklist

1. **`libraries/AP_Vehicle/AP_Vehicle_Type.h`** — add `#define APM_BUILD_ArduRocket 14` (max is
   `APM_BUILD_Heli 13`) **before the `// @LoggerEnumEnd` marker** (scraped by
   `Tools/autotest/logger_metadata/enum_parse.py`).
   **Do this first.** `APM_BUILD_TYPE(x)` is `((type) == APM_BUILD_DIRECTORY)`; a missing define makes
   **every** `APM_BUILD_TYPE` check in every library silently evaluate false, with no diagnostic. This
   is the single most dangerous gotcha in the plan.
2. **`libraries/AP_Param/AP_Param.h:122`** — add `#define AP_PARAM_FRAME_ROCKET (1<<7)` (bits 0-6 used;
   `BLIMP` is `(1<<6)` — bit 7 is the last that fits in the uint16 flags field). Call
   `AP_Param::set_frame_type_flags(AP_PARAM_FRAME_ROCKET)` in `load_parameters()`.
3. **SITL** — `libraries/AP_HAL_SITL/SITL_cmdline.cpp` model table (`{ "rocket", Rocket::create }`) and
   `vehicle_str` ladder, `SITL_State_common.h` `vehicle_type` enum, **and the `AP_SIM_FRAME_CLASS` /
   `AP_SIM_FRAME_STRING` ladders in `libraries/AP_HAL/SIMState.cpp` directly** (the original plan's idea
   of setting these from `ArduRocket/config.h` does not work — that file is never included there).
4. **Build command** — add `'rocket'` to the `vehicles` list in the top-level `wscript` (the Explore
   pass wrongly reported this list as tasklist-only; it also generates `./waf rocket`).
5. **Per-vehicle lib compile** — add a no-op `#if defined(APM_BUILD_TYPE)` block to `AP_Rocket.cpp`
   (the `AP_LandingGear.cpp:15` idiom) so waf compiles it with `APM_BUILD_DIRECTORY` defined.
6. **GCS stream rates** — add `APM_BUILD_ArduRocket` to the zero-default branch in
   `GCS_MAVLink_Parameters.cpp` or the build errors with `#error Need to set streamrates`.
7. **Autotest** — `Tools/autotest/pysim/vehicleinfo.json`, `Tools/autotest/default_params/rocket.parm`
   (**absence of this file breaks the ROMFS build**; it is embedded at *configure* time, so a fresh
   `./waf configure` is required after creating it), `Tools/autotest/sim_vehicle.py` `vehicle_map`
   (`"Rocket": "ArduRocket"`), `Tools/autotest/param_metadata/param_parse.py`.
8. **MAV_TYPE** — nothing to register; runtime return from `GCS_MAVLINK_Rocket::frame_type()`.
9. **Loop rate** — add `APM_BUILD_TYPE(APM_BUILD_ArduRocket)` to the 400 Hz branch of
   `SCHEDULER_DEFAULT_LOOP_RATE` in `libraries/AP_Scheduler/AP_Scheduler.cpp`. Anything not in that
   list silently defaults to **50 Hz**, which is far too slow for a rocket.
10. **New SRV_Channel functions** — `k_rocketFin1..4` (190–193) added in `SRV_Channel.h` before the
    `k_nr_aux_servo_functions` sentinel, plus a `@Values{Rocket}` doc line in `SRV_Channel.cpp`. Named
    fin functions beat reusing `k_elevon_*`/`k_vtail_*`.
11. **Motor frame class** — `MOTOR_FRAME_ROCKET = 18` in `AP_Motors_Class.h`.

## 9. `SIM_Rocket` (`libraries/SITL/SIM_Rocket.{h,cpp}`)

**There is no SITL rocket model** — closest is `SIM_SingleCopter.cpp`. This is the largest novel item
and the only realistic tuning surface: a burn is ~2 s and you get one shot per motor.

`class Rocket : public Aircraft`, ~250 lines, modeling: thrust curve with **burnout** (mandatory to
exercise `COAST`), fin force ∝ q (**zero at liftoff** — the defining characteristic), mass depletion,
and **CP-ahead-of-CG aerodynamic instability** — the tilt plant is open-loop unstable, unlike every
existing SITL model. Ignition is a ground launch controller: the motor fires a fixed delay after arming,
because ArduPilot does not control the igniter. Use the base class's `GROUND_BEHAVIOR_TAILSITTER` for a
vehicle that stands nose-up on the pad.

### The launch rail is modelled, and matters

Until the airframe has travelled `rail_length_m` from ignition, the rail holds its attitude
and constrains it to slide along the rail line. Applied **after** `update_dynamics()`, so it
overrides the base class's tailsitter ground handling (which otherwise forces the airframe
perfectly upright and silently ignores any configured tilt).

This is not cosmetic. It covers exactly the phase where dynamic pressure is near zero and
the fins have no authority; without it the sim lets an unstable airframe topple at t=0 in a
way a real rail simply prevents.

**Default rail length is 72 in (1.8288 m)** — the rail this project launches from.

The sim prints, and flags against the target:

```
Rocket: off the rail at 15.2 m/s (50 fps), q=142 Pa
Rocket: off the rail at 11.0 m/s (36 fps), q=74 Pa   *** BELOW 50 fps TARGET ***
```

**Rail exit is the most consequential number for a fin-steered rocket.** Fin authority
scales with dynamic pressure, so exit speed sets how much control exists at the instant the
rail stops holding the airframe — the lowest-authority moment of the flight. Below target
the rocket is relying on passive stability it does not have (CP is ahead of CG here), and
the fins cannot save it. **Target is 50 fps (15.24 m/s)**, per NAR/Tripoli practice.

### This sets the gain scheduling reference

The rail-exit condition is what `MOT_Q_REF` should be derived from:

```
50 fps = 15.24 m/s   ->   q = 0.5 * 1.225 * 15.24^2 = 142 Pa
```

`MOT_Q_REF = 142` makes the fin gains nominal (scale = 1.0) exactly at rail exit, so the
gains are tuned for the worst case and scale *down* as the rocket accelerates — correct,
since at high q small deflections suffice.

The original 600 Pa default was a guess (≈31 m/s) and was actively wrong: it demanded a
4.22× boost at rail exit against a 4× `MOT_GAIN_MAX`, leaving the scheduling **clamped at
precisely the moment it mattered most**. With `Q_REF` at the rail-exit value, `GAIN_MAX`
now only engages *below* target — i.e. when the rocket is already leaving the rail slower
than it should.

Frame strings select the geometry (matched by prefix, so the whole string reaches the model):

| Frame | Meaning |
|---|---|
| `rocket` | 72 in rail, vertical |
| `rocket-stable` | CP behind CG — passively stable, for isolating controller bugs |
| `rocket-tilt10` | 10° off vertical (NAR/Tripoli cap is 20°) |
| `rocket-tilt10-az90` | 10°, leaning toward 90° azimuth |
| `rocket-tilt10-rail48` | 10°, on a 48 in rail |

## 10. Verification

**SITL — status:**
1. ✅ Full stage sequence `PREP → ARMED → BOOST → COAST → DESCENT → PREP`.
2. ✅ Launch detection with debounce.
3. ✅ Burnout recorded **without** stopping the fins (400us travel during COAST).
4. ✅ Apogee stops the fins and auto-disarms.
5. ⬜ **Gain tuning — not done.** Coast deflection reached ~400us of a ±500us range
   (~80% of full travel), i.e. close to saturation. Tune `MOT_Q_REF`, `MOT_GAIN_MAX`
   and the `ATC_*` gains before flying.
6. ⬜ Disturbance rejection with CP ahead of CG — not characterised.
7. ⬜ Low-q behaviour in the first ~0.5 s off the rail — not characterised.

**Bench (no SITL needed):**
- **FIN DIRECTION — HUMAN CHECK, MANDATORY, NOT AUTOMATED.** Two parts, and neither is
  performed by the firmware:
  1. *Off the rail:* tilt the airframe by hand with the vehicle powered and disarmed;
     the fins must deflect so as to push the nose **back toward** vertical. This is the
     only way to catch a reversed servo or a backwards linkage, because it is the only
     time you can observe deflection against a known attitude change.
  2. *On the rail, during the pre-arm wiggle (§3b):* confirm the fin that moves is the
     fin that was announced, and that it moves the expected way. Do not send the second
     ARM if either is wrong.

  Software cannot do this: a clamped airframe with no airflow produces no motion to
  observe, and comparing mixer output to measured attitude is circular. Sign errors are
  the most likely fatal bug, and this check plus the §0 axis table are the only defence.
- Body up-axis under `AHRS_ORIENTATION` — log `INS`/`ACC` stationary and vertical; up-axis ≈ +9.8 m/s².
  Adjust `RKT_LAUNCH_AX`.
- `RKT_LAUNCH_G` — pick above the stationary 1 g reading, below expected launch accel; validate against
  logged drop/jerk tests.
- Spool state reachability (§5 risk) — confirm fins go live with no throttle input.

## Risks

- **`APM_BUILD_ArduRocket` silent-false** — compiles and links; subtly wrong everywhere. Step 8.1 first.
- **Axis-mapping confusion** — the view-roll/view-yaw remap (§0) is exactly the error class that reaches
  the pad. My own first analysis got this backwards. Bench-verify.
- **Mistaking the pre-arm fin check for a direction check.** It is not one, and cannot be
  (§3b). It wiggles the fins and blocks arming until a human confirms; it does not
  detect a reversed servo, backwards linkage, or swapped fin positions. If the culture
  around it drifts to "the firmware checks the fins", that is a latent fatal bug.
- **Spool state machine with no throttle** (§5) — may never reach `THROTTLE_UNLIMITED`.
- **Zero fin authority at liftoff** — inherent to fins-only; the launch rail must hold attitude until q
  builds. This is what makes `PAD` integrator-gating load-bearing rather than incidental.
- **One shot per motor.** SITL is not optional; it is the only place gains can be tuned.

## Effort

~60% mechanical copy-paste (scaffolding, GCS, params, arming, logging, registration).
Genuinely novel: `SIM_Rocket` (largest), `AP_FinMixerRocket` q-scheduling + `limit.*` honesty,
`AP_Rocket` burnout detection.

---

# APPENDIX A — Scope: thrust-vector control (TVC) variant

**Status: SCOPED, NOT BUILT.** The fins variant is what flies today. This is the plan
for a gimballed-motor variant, to be built after the fin gains are tuned.

## A.1 Why TVC is a different plant, not just different actuators

Fin authority scales with **dynamic pressure** (½ρv²). Gimbal authority scales with
**thrust** (`moment ≈ thrust × sin(deflection) × moment_arm`). That inverts the
control window end to end:

| | Fins (built) | TVC (scoped) |
|---|---|---|
| Authority source | dynamic pressure | **thrust** |
| At liftoff, v≈0 | **near zero** — the rail holds attitude | **full authority immediately** |
| At burnout | still strong (fast, coasting) | **falls to zero** |
| Control window | rail → apogee | **the burn only** (0.8–8 s) |
| Steering ends at | **apogee** | **burnout** |
| Controllable axes | all three | **tilt only; spin is impossible** |

The two are complementary — TVC has authority exactly where fins don't. A combined
boost-TVC / coast-fins airframe is the natural end state, and is out of scope here.

**Spin is uncontrollable with a 2-axis gimbal.** Thrust acts along the airframe's long
axis, so deflecting it produces moments about body Y and Z (both *tilt* axes, i.e.
view pitch and view roll) but never about body X. So `limit.yaw` is pinned true and the
yaw target must be reset every loop so no error accumulates. See the axis-mapping table
in the Context section.

## A.2 New class: `AP_TVCMixerRocket` (`libraries/AP_Motors/`)

A **separate** `AP_MotorsMulticopter` subclass, not a frame option on
`AP_FinMixerRocket`. ArduPilot's convention is one class per frame class
(`Single`/`Coax` are separate despite shared geometry), the differing parts are exactly
the ones you would override anyway, and sharing one class would leave half the
parameters meaningless for whichever frame was selected.

| Member | Behaviour |
|---|---|
| `init()` | Two servos, `k_rocketGimbalPitch`/`k_rocketGimbalRoll`, `set_angle(±4500)`. `set_initialised_ok(frame_class == MOTOR_FRAME_ROCKET_TVC)`. |
| `output_armed_stabilizing()` | **Decoupled** (unlike the fins' elevon-style mix): `gimbal_pitch = pitch_thrust`, `gimbal_roll = roll_thrust`. Scale by the thrust proxy (A.3). Clamp to the mechanical limit and set `limit.roll/pitch` on clip. **`limit.yaw = true` unconditionally.** |
| `output_to_motors()` | Write the two channels; centre them in `SHUT_DOWN`/`GROUND_IDLE`. |
| `var_info` | `MOT_ACC_REF` (axial accel, g, at which gains are tuned), `MOT_GAIN_MAX`, `MOT_GMB_LIM` (mechanical gimbal limit, degrees — typically 5–15°). |
| `_get_frame_string()` | `"ROCKET_TVC"` |

Carry over from the fins class: throttle is still a fiction (solid motor), so
`_throttle_in` is ignored and `_throttle_out` forced to 0; and the header must open
with the same "why this is an AP_Motors subclass" explanation.

## A.3 Gain scheduling — the thrust proxy is already measured

Thrust is not instrumented, but it does not need to be: the **body-axial accelerometer
reading is specific force ≈ thrust/mass**, and `AP_Rocket::up_accel_g()` already
computes it.

```
fins:  scale = MOT_Q_REF   / q               (q from baro-derived vertical speed)
TVC:   scale = MOT_ACC_REF / up_accel_g      (already available, no new sensor)
```

Caveat: axial accel is thrust *minus drag*, so at high speed it slightly underestimates
thrust and the scheduling runs a little hot. Acceptable; note it when tuning.

## A.4 Shared base class — the one real refactor

The vehicle must feed a different scheduling input per frame, and apply a different
stop policy. Rather than branch on a frame enum in `rocket_control.cpp`, introduce:

```cpp
class AP_MotorsRocketBase : public AP_MotorsMulticopter {
public:
    // q in Pa for fins, axial accel in g for TVC; each mixer interprets it
    // against its own reference parameter.
    virtual void set_authority_measure(float measure) = 0;

    // true when control authority dies with the motor (TVC/jet vane), so the
    // vehicle must stop steering at BURNOUT rather than at apogee.
    virtual bool authority_requires_thrust() const = 0;
};
```

This keeps tuning parameters with the mixer, and puts the burnout-vs-apogee policy
*with the frame that owns it* instead of in a vehicle-side conditional.
`AP_FinMixerRocket` returns `false`; `AP_TVCMixerRocket` returns `true`.

`ArduRocket::motors` becomes an `AP_MotorsRocketBase*`.

## A.5 Registration checklist (additions)

1. `AP_Motors_Class.h` — `MOTOR_FRAME_ROCKET_TVC = 19` (fins keep 18).
2. `SRV_Channel.h` — `k_rocketGimbalPitch = 194`, `k_rocketGimbalRoll = 195`, before the
   `k_nr_aux_servo_functions` sentinel; plus a `@Values{Rocket}` line in `SRV_Channel.cpp`.
3. **`FRAME_CLASS` parameter — ALREADY ADDED.** `0=Fins` (default, implemented),
   `1=ThrustVectoring`, `@RebootRequired: True`, GCS-settable. It already drives the
   switch in `ArduRocket::allocate_motors()`, which currently raises
   `config_error("FRAME_CLASS=1 (TVC) not implemented")` for TVC — the vehicle refuses
   to boot rather than silently falling back to fins and flying the wrong actuator.
   Implementing TVC means replacing that `config_error` with the real allocation, and
   passing `MOTOR_FRAME_ROCKET_TVC` to `motors->init()`.
4. `Tools/autotest/default_params/rocket-tvc.parm`, and a `rocket-tvc` frame in
   `vehicleinfo.json`. Remember `./waf configure` after adding it.

## A.6 `rocket_control.cpp` change

One policy line, driven by the mixer rather than a frame check:

```cpp
const bool steer = motors->authority_requires_thrust()
                     ? (rkt.stage() == AP_Rocket::Stage::BOOST)      // TVC: dies at burnout
                     : rkt.steering_active();                        // fins: BOOST or COAST
```

Plus feeding the right measure: `motors->set_authority_measure(q_pa)` for fins,
`up_accel_g()` for TVC.

`AP_Rocket` itself needs **no change** — it already reports the stages, and the actuator
policy belongs to the vehicle/mixer.

## A.7 `SIM_Rocket` change

Add a gimbal model behind the `rocket-tvc` frame string: moment about body Y/Z equal to
`thrust × sin(deflection) × moment_arm`, clamped to the mechanical limit, and **zero
moment about body X** (no spin authority). Existing fin path untouched. ~40 lines.

## A.8 Risks specific to TVC

- **Short moment arm.** Gimbal-to-CG distance is small on a hobby airframe, so real
  authority is lower than the maths suggests. Measure it, do not assume.
- **Mechanical limit is hard.** ±5–15° typical. The mixer must clamp and report
  saturation honestly, or the rate PIDs wind up against a stop.
- **Servo slew rate matters far more than with fins**, because the entire control window
  is one short burn.
- **Zero authority after burnout** — the vehicle is ballistic and uncontrolled from that
  moment. There is no fallback unless fins are also fitted.
- **No spin control at all**, so a spinning airframe will gyroscopically cross-couple the
  tilt axes. Log spin rate; do not attempt to control it with the gimbal.

## A.9 Effort

Mostly mechanical: ~200 lines for the new mixer, ~40 for the SIM gimbal, a handful of
registration edits, and the `AP_MotorsRocketBase` refactor (which also tidies the fins
class). The genuinely new work is tuning against a plant whose authority collapses to
zero mid-flight.
