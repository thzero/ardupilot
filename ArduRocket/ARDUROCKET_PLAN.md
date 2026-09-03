# ArduRocket — a standalone vehicle folder

> ## STATUS: IMPLEMENTED AND FLYING IN SITL (2026-07-18)
>
> The design below was implemented on the `ardurocket` branch and **passes a full
> simulated flight**:
>
> ```
> PREP -> FINCHECK -> ARMED -> BOOST -> COAST -> DESCENT -> LANDED -> (manual disarm) -> PREP
> fin travel:  0us      0us      0us    4us    400us    stop      stop
>                                ignition ..... burnout ..... apogee ..... touchdown
> ```
>
> Verified: fins centred when disarmed, quiet on the rail (integrator gating),
> live under boost, **still steering through coast after burnout**, and stopped at
> apogee. The vehicle then stays ARMED through descent and touchdown -- disarm is a
> manual operator action after recovery, not automatic. ArduPlane still builds clean
> with QROCKET removed.
>
> **NOT verified — required before any real motor:**
> - **`ATC_*` gains are the day-one placeholders, never tuned.** `MOT_Q_REF` has been
>   tuned (600, by measurement — see §9), but the attitude gains have not. NOTE the
>   plant they must be tuned against was corrected substantially (inertia, damping,
>   drag, fin authority — §9); any gain work predating that is void. For a principled
>   starting point (start gentle — full fin at ~40° tilt) and a tilt-based give-up
>   backstop, see **Appendix B**, drawn from a flown fin-steered rocket.
> - **Wind-limited, not gain-limited, near apogee.** At the 20 mph operational ceiling
>   the airframe reaches ~15 deg off vertical late in coast because the tabs cannot
>   out-muscle the wind at low q, not because the controller is failing. In calm air
>   it holds within ~0.3 deg. This is a static-margin/tab-authority limit, not a
>   tuning one; see §9's MOT_Q_REF sweep.
> - **Control tab dimensions are ASSUMED.** The tab is defined in **mm** on the trailing
>   edge — width (flap depth), height (spanwise length), root offset, hinge inset
>   (`SIM_RKT_TAB_W/H/RT/AX`, `TAB_MAX` deg; MATLAB `P.tab`). Not in either OpenRocket
>   file; defaults are the old 25%/75%/20° assumption in mm. Measure and set the real tab;
>   fin authority is derived from it (same formula in both sims: `recompute_fin_geometry`
>   / `rocket_fin_gain.m`).
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
> | **AHRS / compass / EKF3 never registered in `Parameters.cpp`** | The worst of the session. `AP_Vehicle` does not register these for you; Copter and Blimp each do it themselves. Result: **zero** `AHRS_*`, `EK3_*` and `COMPASS_*` parameters existed (param count 901; after the fix 1071). Every such line in `rocket.parm` was silently discarded, so **`AHRS_ORIENTATION` and the `EK3_SRC1_*` GPS exclusion had never once taken effect** — and §3c documented that exclusion as a safety property. Same silent-failure mode as `ARMING_CHECK`. |
> | `AHRS_ORIENTATION 24` was wrong, and hidden by the bug above | It describes how the BOARD is rotated in the airframe, not that the rocket stands vertical — the `AP_AHRS_View(ROTATION_PITCH_90)` does that. Setting both rotates twice. The moment registration was fixed, tilt read **88.8° on a vertical rail**. Correct value is **0**. A wrong setting concealed by a dead one. |
> | `try_send_message()` not overridden | The **shared** stream tables list `MSG_WIND`, which every other vehicle implements and the base class refuses. A GCS requesting all data streams triggered `PANIC: Sending unknown ap_message 36`. In SITL that **kills the vehicle**; on hardware the panic is compiled out and it instead floods the link with `Sending unknown message (36)` at the stream rate. Only reachable with a real GCS attached, so the scripted tests never saw it. |
>
> ### Ground station connection (WSL2)
>
> Connect the GCS over **TCP to `127.0.0.1:5760`**; WSL2 forwards Windows localhost
> into WSL. Verified with ArduDeck, and with pymavlink.
>
> Do **not** use `--serial0 udpclient:<windows-host>:14550`. Measured: SITL sends
> heartbeats correctly to `udpclient:127.0.0.1`, but aimed at the WSL gateway IP it
> emits **zero** datagrams — confirmed against `/proc/net/snmp` OutDatagrams with a
> control send on the same path proving the counter works, and re-confirmed after the
> `MSG_WIND` crash above was fixed, so the two faults are independent. Ping to the
> gateway is clean, so it is neither routing nor Windows Firewall. Unexplained; TCP
> works, so it was not pursued further.
>
> Note SITL **blocks on startup** until the first TCP client connects (`Waiting for
> connection ....`) and accepts **one** client at a time — GCS or test script, never both.
>
> ### Heading and attitude reporting — SOLVED, but not the obvious way
>
> **Current behaviour:** heading comes STRAIGHT OFF THE MAGNETOMETER, computed in
> `GCS_MAVLINK_Rocket::rocket_heading_rad()`, and the compass is kept out of the
> flight solution entirely (`COMPASS_USE/2/3 = 0`). Verified against a SITL truth of
> 353°: **357.3° with 12° peak-to-peak**, versus 95.7–167° from the EKF.
>
> The split is the design: the flight code never uses heading at all (attitude hold
> works on tilt; spin is held by commanding yaw RATE zero off the gyro), while the
> ground station still gets a usable compass rose.
>
> #### Why the EKF's yaw cannot be used
>
> `ATTITUDE` normally reports Euler angles in the **body** frame, and a nose-up rocket
> sits at body pitch = 90° — the Euler singularity, where roll and yaw describe the
> same rotation and neither is individually defined. Measured, stationary, 35 s:
>
> | quantity | mean | sd | peak-to-peak |
> |---|---|---|---|
> | **tilt of nose from vertical** | 0.086° | 0.043° | **0.156°** |
> | direction the nose leans | 85.96° | 49.4° | 166.5° |
> | raw Euler yaw | 85.96° | 49.4° | 166.5° |
>
> Euler yaw tracks the lean *direction* exactly (85.961 vs 85.962). At 0.086° of tilt
> that direction is meaningless — like asking which way you face standing on the North
> Pole.
>
> #### The three fixes, in the order they were needed
>
> 1. **Report `ATTITUDE` in the view, not the body frame** (`send_attitude()`). In the
>    `ROTATION_PITCH_90` view a vertical rocket reads level, so roll/pitch become tilt
>    and are well conditioned. Cut the swing 167° → 96°. `ATTITUDE_QUATERNION` is
>    deliberately left untouched as a truthful ground-truth channel.
> 2. **Heading is published in THREE messages, not one.** `ATTITUDE.yaw`,
>    `VFR_HUD.heading`, and `GLOBAL_POSITION_INT.hdg` all get filled from AHRS yaw
>    upstream. QGC drives its compass rose from `VFR_HUD`, so fixing only `ATTITUDE`
>    changed nothing on screen. `send_vfr_hud()` is **not virtual** — route
>    `MSG_VFR_HUD` through `try_send_message()` instead.
> 3. **Compute heading from the magnetometer**, bypassing the EKF entirely.
>
> #### Traps, all of which caught me
>
> - *Do not* judge drift from the angle between successive quaternions. That metric
>   lumps tilt together with the degenerate lean direction, and reported 78.8° of
>   apparent motion on an airframe holding within 0.16°.
> - *Do not* pass the view's rotation matrix to `Compass::calculate_heading()`. That
>   helper pairs the matrix with the field vector in the **raw body** frame; mixing
>   frames produced a 347° swing, worse than doing nothing. Rotate the field into the
>   view frame first, then tilt-compensate.
> - `EK3_MAG_CAL=4` (3-axis mag fusion always) genuinely helps the EKF — measured
>   95.7° → 9.4° drift — so this is **not** purely structural, as first claimed. It is
>   still **not** enabled: 3-axis fusion assumes a magnetically stable environment, and
>   a steel motor casing plus igniter current is the opposite. It would also improve a
>   number nothing reads.
> - Distrust suspiciously perfect results. One run of this reported 0.067° drift, from
>   a SITL process that had already died. Confirm the process is live before believing
>   a number.
>
> **Also worth putting on screen:** `send_rocket_telemetry()` publishes
> `NAMED_VALUE_FLOAT` named **`TILT`** at 5 Hz — the real angle from vertical, from
> the same `tilt_from_vertical_deg()` the arming gate uses. Verified: 0.068° on a
> vertical rail, **10.035° on `rocket-tilt10`**, agreeing with an independent
> quaternion-derived tilt to **0.0009°**. That is the number that decides arming.
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
build/sitl/bin/rocket --model rocket -w    # real time -- do NOT add --speedup
```

Run in **real time** (no `--speedup`). A short burn integrated at a coarser step under speedup changes
the results measurably (a worst-case tilt read 19° at `--speedup 5` vs 15° at real time), so any run you
draw numbers from must be real-time.

Note the reconfigure: `Tools/autotest/default_params/rocket.parm` is embedded into ROMFS at
**configure** time (`Tools/ardupilotwaf/boards.py` globs `default_params/`), so editing it and only
running `./waf rocket` silently keeps the old values.

---

## 0. Target hardware

The physical build this vehicle is aimed at. Everything here is **config-only on the ArduPilot side** —
no code changes — because the sensors were designed out of the control path from the start (§3c: GPS and
compass never steer). The one part that drove a real decision is the IMU G-range.

| Role | Part | Notes |
|---|---|---|
| Flight controller | **iFlight BLITZ Wing H743** (ICM-45686) | H7 flash headroom; ±32 g IMU; servo-first "Wing" layout; onboard baro |
| GPS + compass | u-blox **M10 combo** (Holybro M10 / Matek M10Q-5883) | recovery position + pad heading; never control |
| Telemetry — air | **Matek mR24-30** (mLRS RX) | on a MAVLink UART, in the airframe |
| Telemetry — ground | **Matek mR24-30-TX** + 2× 2.4 GHz antennas | bound pair; USB or WiFi to the laptop → QGC |
| Actuators | **4× servos** on `SERVO1-4` (`k_rocketFin1-4`) | the four steering tabs |
| Recovery / deployment | **OFF-BOARD** — Featherweight BlueRaven, or 2× Eggtimer Quantum | dedicated rocketry altimeter, sized to the rocket |
| Ground station | QGroundControl + `Tools/ArduRocket/qgc/ArduRocket.json` | Fin Check action + the pre-arm/arm flow |

### Flight controller: iFlight BLITZ Wing H743 (ICM-45686)

- **H7, not F4.** ArduPilot 4.x no longer fits a full build in 1 MB F4 flash; the H743 (2 MB) has headroom
  and runs EKF3 comfortably.
- **±32 g IMU — the reason this board, not a smaller ±16 g one.** A rocket boosts far harder than a drone.
  This airframe peaks at **173.7 m/s² ≈ 17.7 g** (the SIM/OpenRocket reference), and most FC IMUs
  (ICM-42688, BMI270, MPU6000) saturate at **±16 g** — so they clip at peak boost. The ICM-45686 runs at
  ±32 g and clears it with margin to ~29 g. This is **verified in the driver, not assumed**:
  `AP_InertialSensor_Invensensev3.cpp:342` sets `accel_scale = ACCEL_SCALE_32G` for the ICM-45686, and the
  per-chip clip limit at `:1140` is **29.5 g** for the ±32 g parts (vs 15.5 g for the ±16 g parts) — the
  whole pipeline, including clip detection, treats it as a 32 g sensor.
- **What the clip would and wouldn't break** (if you ever fall back to a ±16 g board): control is
  unaffected — it is gyro-driven. Apogee *detection* is unaffected — it is a climb-rate-sign test that
  fires near apogee where accel is ~0 g, nowhere near a clip. The **only** casualty is logged peak
  accel/velocity during boost. So ±16 g flies fine; ±32 g just keeps the boost data honest.
- **"Wing" layout** gives servo outputs and UARTs with no bundled 4-in-1 ESC — the right shape for a
  servo-driven, no-ESC rocket.
- **Verify before flight:** confirm the exact board/rev is a listed ArduPilot target and enumerates its
  IMU as `ICM45686` (boot messages / a log's IMU device-type). That is the only board-specific unknown —
  the ±32 g scaling itself is definitively correct in the driver.

### GPS + compass: u-blox M10 combo

- Carried for **recovery** (last-known position) and the **pad heading** number on the GCS — **never for
  control** (`EK3_SRC1_POSXY/VELXY = 0`; compass display-only). Scheduled at 50 Hz for logging (§4).
- A rocket is a friendly place for both: **solid motor → no compass current interference**, and a
  **fiberglass airframe is GPS-transparent** (mount the module high with sky view; carbon would block it).
- The GPS **drops lock under boost and reacquires on coast** — harmless, because it is never in the
  control loop, and coast/descent is exactly the phase recovery cares about. Set the GPS **dynamic model to
  airborne**. This airframe's ~400 m/s is under the COCOM 515 m/s cutoff; a higher-impulse build could hit it.

### Telemetry link (recovery downlink): mLRS pair

- **Air** `Matek mR24-30` (RX) on a MAVLink UART; **ground** `Matek mR24-30-TX` → laptop by USB (COM) or
  WiFi (UDP) → QGC. Bind the pair once, matching mLRS params on both ends.
- **Why mLRS, not the alternatives:** SiK/RFD900 are physically too big for the airframe. ELRS-MAVLink has
  the smallest air unit (~1 g dual-band nano) but routes MAVLink through an **RC handset** on the ground —
  odd for a vehicle that has no RC. mLRS's ground side is a **standalone module**, which is the clean fit
  for a telemetry-only, no-RC build; the RX is still tiny next to a SiK.
- The telemetry payload is trivial (position + a few status messages at 1–5 Hz), so **bandwidth never drove
  the choice** — size and range did. Ground-side TX power is configurable 13–30 dBm; set it to what your
  region allows (EU ~100 mW on 2.4 GHz) and what range actually needs, not the 1 W max.
- FC side is identical to every other link: that UART's `SERIALx_PROTOCOL = 2`, matched baud. The TX kit
  ships **without antenna or case** (adapter cables + a cooling fan included), so budget those.

### Recovery / deployment is off-board, on purpose

Pyro/chute deployment is **not** the flight controller's job — it is out of scope, and keeping it separate
means deployment never rides on an apogee estimate we would rather not trust for a pyro event. It runs on a
dedicated commercial rocketry altimeter: a **Featherweight BlueRaven**, or **two Eggtimer Quantums**
(redundant dual-deploy) depending on rocket size.

### Deliberately absent

- **No RC receiver/transmitter.** No RC by design. The mLRS link is telemetry-only and does **not**
  reintroduce a control surface: the UART is MAVLink protocol with no RC input functions mapped, and
  `set_mode()` returns false so any stray RC frame is inert.
- **No ESC.** Solid motor — nothing electric to drive.

---

## 1. New vehicle: `ArduRocket/` (19 files)

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
| `Parameters.h/.cpp` | `const AP_Param::Info ArduRocket::var_info[]` — **mandatory**; `load_parameters()`/`check_var_info()` run unconditionally in `AP_Vehicle::setup()`. Minimum `FORMAT_VERSION` + `AP_VAREND`. `AP_SUBGROUPINFO` for `RKT_`, `ATC_`, `MOT_`, `SERVO_`, and **`GOBJECT(arming, "ARMING_", AP_Arming_Rocket)`** — omit that and the `ARMING_*` params silently do not exist. **Also mandatory: `GOBJECT(compass, "COMPASS_", Compass)`, `GOBJECT(ahrs, "AHRS_", AP_AHRS)` and `GOBJECTN(ahrs.ekf3.EKF3, NavEKF3, "EK3_", NavEKF3)`.** `AP_Vehicle` does NOT register these; omitting them made `AHRS_ORIENTATION` and the `EK3_SRC1_*` GPS exclusion silently dead. See the bug table. | `Blimp/Parameters.cpp` |
| `system.cpp` | `init_ardupilot()`, `allocate_motors()`, **`ahrs_view = ahrs.create_view(ROTATION_PITCH_90);`** ← the key trick, and **`ahrs.init()` + `ins.init(scheduler.get_loop_rate_hz())`** — mandatory; without the IMU init the main loop blocks forever in `wait_for_sample()` and the vehicle never runs. | `ArduCopter/system.cpp:358-431`, `Blimp::startup_INS_ground()` |
| `rocket_control.cpp` | The whole flight controller (§3). ~150 lines. Replaces the entire `mode*.cpp` family. | `ArduCopter/mode_stabilize.cpp:9-60` (spool-state switch) |
| `AP_Arming_Rocket.h/.cpp` | `arm()` must reset the stage detector AND propagate the armed state: `hal.util->set_soft_armed(true)`, `motors->armed(true)`, logger/notify, and `ahrs.resetHeightDatum()` when there is no home (there never is — no GPS). Omitting the propagation leaves the stage machine stuck in PREP. Also overrides `rc_calibration_checks() → true` (no RC) and enforces vertical-and-still on the rail. | `Blimp/AP_Arming_Blimp.*` |
| `GCS_Rocket.*`, `GCS_MAVLink_Rocket.*` | Required — `GCS::create_gcs_mavlink_backend()` is pure virtual under `HAL_GCS_ENABLED`. `frame_type()` → `MAV_TYPE_GENERIC` (no upstream `MAV_TYPE_ROCKET`). `custom_mode()` → flight stage. `base_mode()` must OR in `MAV_MODE_FLAG_SAFETY_ARMED` or every GCS shows the vehicle disarmed while it is live. Overrides `handle_command_int_packet()` to catch `MAV_CMD_DO_AUX_FUNCTION` (func 300) for the fin check (§3b), with `MAV_CMD_DO_MOTOR_TEST` as a fallback. Overrides `send_attitude()` (view frame), `send_global_position_int()` and — via `try_send_message()` because `send_vfr_hud()` is not virtual — `MSG_VFR_HUD`, so that **all three** heading fields carry the magnetometer heading from `rocket_heading_rad()` instead of the EKF's meaningless nose-up yaw. **Must also override `try_send_message()`** to consume `MSG_WIND`: the stream tables are shared across all vehicles and every other vehicle implements it, so omitting it kills the vehicle in SITL and floods the link on hardware the moment a GCS requests all streams. Overrides `landed_state()` (EXTENDED_SYS_STATE) → IN_AIR only for BOOST/COAST/DESCENT, ON_GROUND otherwise; without it the GCS shows "Flying" whenever armed, which — since the vehicle stays armed from rail through touchdown — means it never stops saying "Flying" after landing (or before launch on the rail). `vehicle_system_status()` **must agree**: it reports `MAV_STATE_ACTIVE` only when airborne (same three stages), not merely when armed — reporting ACTIVE on the ground while `landed_state()` says ON_GROUND makes the GCS flicker between "armed" and "flying". | `Blimp/GCS_*` |
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
           way down. Announces apogee altitude to the GCS. STAYS ARMED.
LANDED  -> touchdown detected (vertical speed settled to ~zero and held, debounced -- NOT altitude,
           since it may drift under chute onto a hill or ditch).
           Announces "landed" to the GCS. Terminal and STILL ARMED: the vehicle waits here until the
           operator disarms after recovery. Nothing auto-disarms.
```

**Disarm is manual, by design.** The flight computer never disarms itself -- not at apogee, not at
touchdown. It stays armed (controller shut down, fins centered) the whole way down so it keeps logging
and reporting, and the operator disarms once the rocket is recovered. Pyro/chute deployment is off-board
on a dedicated altimeter (§0), so nothing here depends on the disarm.

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

Arming is a **single ARM press, gated on a separately-triggered fin check**:

```
Fin Check  -> GCS "Fin Check" button (MAV_CMD_DO_AUX_FUNCTION, param1 = 300)
           -> FINCHECK stage: fins wiggle one at a time, each announced
           -> on the rail (vertical & still) this LATCHES fin_check_valid
           -> returns to PREP. "Rocket: fin check done - if fins moved correctly, ARM"
              (NOT "OK": the code cannot see the fins, only that the wiggle ran; the
              verdict is the operator's, recorded by the ARM press below)
ARM        -> single press. Pre-arm requires fin_check_valid, so until the check
              is run, ARM fails with "fin check required" (the standard GCS pre-arm
              reason line) rather than a confusing error. Once latched, ARM simply
              succeeds; rail attitude captured. The arm press is the attestation.
```

The latch is invalidated by: disarm, reboot, a ~5-minute timeout, or a gyro spike
(the airframe disturbed after the check) — so you cannot check, bump the rail, then
arm on a stale confirmation. A fin check run OFF the rail (held in hand) still drives
the fins so you can bench-test, but does NOT latch the gate.

### Why a dedicated aux-function, not Motor Test

The trigger is `MAV_CMD_DO_AUX_FUNCTION` (func 300, the SCRIPTING_1 slot), NOT
`MAV_CMD_DO_MOTOR_TEST`. These are fins, not motors, and DO_MOTOR_TEST surfaces as a
mislabelled, buried "Motor Test" panel. A QGC custom-action file
(`Tools/ArduRocket/qgc/ArduRocket.json`) adds a one-tap button literally labelled
**"Fin Check"**. DO_MOTOR_TEST is still accepted as a fallback for ground stations
without the custom button.

**Installing the action file in QGC Daily (Windows 11) -- the procedure that actually
works.** The upstream QGC docs say action files auto-load from a `MavlinkActions` folder
and there is no UI for it; that was NOT true for the Daily build tested -- it has a
browse field and the file must be selected there. Steps that worked:

1. Copy `Tools/ArduRocket/qgc/ArduRocket.json` into the QGC save directory's
   `MavlinkActions` folder. QGC's save dir is shown in Application Settings and, with
   OneDrive redirecting `Documents`, was:
   `C:\Users\<user>\OneDrive\Documents\QGroundControl Daily\MavlinkActions\`.
2. In QGC, click the **QGC logo (top-left)** to open Settings.
3. Open **Fly View Settings** (labelled "Fly View" on some Daily iterations).
4. Scroll the Fly View options to the **MAVLink Actions / Custom Action File** field.
5. Click **Browse**, go to that `MavlinkActions` folder, and select `ArduRocket.json`.
6. **Fully restart QGC** -- and check Task Manager for a lingering `qgroundcontrol.exe`
   background process, which keeps the old config if not killed.

Two file-level gotchas that silently make QGC ignore the file:
- It **must** contain `"fileType": "MavlinkActions"`.
- It must contain **only** the documented keys. QGC validates against a fixed schema
  and rejects the whole file if it sees an unexpected key (e.g. a top-level `comment`).
  Keep it to `fileType`, `version`, and `actions` with `label`/`description`/`mavCmd`/
  `compId`/`param1`/`param2`.

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
- **Announces each fin** (`Rocket: testing fin 3 (+/- then center)`) as it starts its throws, so the crew can confirm
  the fin that moves is the fin that was named -- this is what makes a swapped output
  channel visible.
- **Blocks arming** until the check has been run on the rail (pre-arm requires the
  latched `fin_check_valid`), so it cannot be skipped.

The operator's job during the wiggle is to check, for each announced fin:
1. the fin that moves is the one named, and
2. it deflects in the direction that would push the nose **back toward** the rail line.

If either is wrong, do not ARM -- disarm/re-run. The ARM press is your attestation
that every fin moved correctly.

### What IS genuinely automated

| Check | Catches | Result if it fails |
|---|---|---|
| All four fin functions assigned to outputs | forgot `SERVOn_FUNCTION`, typo | arm refused |
| Tilt within 20 deg of vertical | mis-mounted airframe, bad attitude solution | arm refused |
| Stationary (gyro < 15 deg/s) | being carried or shaken | arm refused |

20 degrees is the maximum rail tilt permitted by both **NAR and Tripoli** safety codes,
so anything beyond it is either mis-mounted or a bad attitude solution.

#### The tilt limit is a CONE, not a per-axis check

The limit describes a **20° cone around vertical**, and the check asks exactly one
question: is the nose inside that cone? `ArduRocket::tilt_from_vertical_deg()` is the
single source of truth, computing the real geometric angle
`acos(cos(view roll) × cos(view pitch))`. Both the arming gate and the `TILT`
telemetry call it, so the number the pad crew reads is by construction the number
that decides arming.

Getting this wrong is easy and it failed in **both** directions before landing here:

| scheme | asks | at roll 14 / pitch 14 | at roll 20 / pitch 20 |
|---|---|---|---|
| `\|roll\| + \|pitch\|` *(original, wrong)* | is the sum ≤ 20? | sum 28 → **refused at 19.7° real tilt** | 40 → refused |
| per-axis `\|roll\| ≤ 20 && \|pitch\| ≤ 20` | is each ≤ 20? | accepted | **accepted at 28.0° real tilt** |
| **`acos(cos r × cos p)`** *(current)* | how far is the nose from vertical? | 19.7° → accepted | 28.0° → refused |

The sum is **too strict**: it treats a diagonal lean as worse than a cardinal one
purely as an artifact of the decomposition, tightening a 20° limit to as little as
**14.1°** and refusing rails that NAR and Tripoli both permit.

Per-axis is **too loose, in the unsafe direction**: 20° on each axis is 28° of real
tilt, well outside the safety-code limit, and it would arm.

Leaning 20° north, 20° east or 20° north-east are all exactly 20° of tilt. Only the
cone treats them identically, which is what the safety codes actually mean.

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

What the ARM press records is *how far off vertical the rail points*. That measurement
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

### Steering stops when there is no authority left

`RKT_MIN_Q` (default 50 Pa, ≈9 m/s) stops driving the fins below a dynamic pressure at
which they cannot produce a useful moment however far they deflect. Without it the
scheduling (`Q_REF/q`, capped at `GAIN_MAX`) would drive them hard against a plant that
cannot respond, wasting travel and winding up integrators just before shutdown.

`ARMED` is deliberately exempt: q is ~0 on the rail too, but the fins must already be
tracking attitude when the rail releases.

**Measured caveat, worth knowing before tuning:** adding this did **not** reduce observed
coast fin travel (400 µs before and after). That peak occurs at q well *above* the
threshold, where the fins genuinely have authority and are correcting a real tip-over as
speed bleeds off. It is control effort, not scheduling runaway. `RKT_MIN_Q` guards only
the true zero-authority tail near apogee.

## 3c. Attitude estimator: DCM on pure IMU, no GPS

The flight controller has **no GPS**, and attitude is estimated by **DCM, not EKF3**. This
was the hardest single problem in the vehicle, resolved by a SITL investigation (the
`Tools/ArduRocket/sitl_tests/` harness records estimate-vs-truth as a `d` column), and the
reasoning is worth keeping.

**The problem.** With no GPS there is no velocity reference, and without one an estimator
cannot separate the vehicle's *own* acceleration from gravity. During boost the motor pulls
tens of g along the nose, swamping the 1 g of gravity used to find "down". An estimator that
assumes "big steady acceleration ≈ down" concludes the nose points more straight-up than it
does, and **under-reads the tilt exactly when the fins have the most authority to correct it.**

**EKF3 cannot do it without GPS.** Measured in SITL (est vs. sim truth):
- Default EKF3: the tilt estimate glitched by up to **90°** — a phantom **117° tilt while the
  true tilt was 30°** — which also tripped the give-up backstop on nothing.
- Retuning its process noise (`EK3_ACC_P_NSE`→1.0, `EK3_GYRO_P_NSE`→0.003) removed the glitches
  but left a **standing ~12° bias it could not shed**: trusting the accelerometer less to
  survive boost also stops it re-levelling in coast. One static knob cannot do both.

The failure is **structural** — no velocity aiding to make tilt observable — not tuning.

**DCM holds it**, the way the flown MatrixPilot fin-steered rocket did (`RollPitchYaw/main.c`:
pure gyro + accelerometer; its GPS callback only blinks an LED). `AHRS_EKF_TYPE 0`. In SITL
its tilt tracked truth within **~1–2°** for most of the flight with no glitches, because it
corrects toward gravity *slowly* — it coasts on the gyro through the burn and re-levels after.

**What makes it work is a code gate, not just a parameter.** `AHRS_EKF_TYPE 0` selects DCM,
but DCM alone still gets pulled off by the accelerometer under thrust and — worse — under fin
steering, both of which the accelerometer reads as a tilt of gravity. The fix is the flown
MatrixPilot rocket's exact technique: **stop using the accelerometer for attitude once
launched, and coast on the gyro.** `rocket_control.cpp` calls `ahrs.set_attitude_gyro_only(true)`
from BOOST through DESCENT; `AP_AHRS_DCM::drift_correction()` then zeroes the accel error term
(killing both the P and I accelerometer corrections), leaving pure gyro. On the pad the gate is
off, so DCM aligns to true vertical from gravity first. This mirrors `rmat.c`'s
`if (acceleration < GRAVITY/4 && launched == 0)` gate. (A parameter cannot do this: ArduPilot
clamps `AHRS_RP_P` to a 0.05 floor and the integral term isn't gated by it at all.)

| Param | Value | Why |
|---|---|---|
| `AHRS_EKF_TYPE` | **0** (DCM) | Attitude from gyro+accel; EKF3 stays as a dormant fallback. |
| `AHRS_RP_P` | **0.2** (default) | Only matters on the pad now — the gate removes the accelerometer entirely in flight — so it is left at the default for good pad alignment. (An earlier `0.05`, to slow the thrust pull, became unnecessary once the gate was added.) |

**The limit found (and why it is partly a sim artifact).** With the accelerometer gated out,
the estimate is clean when the fins are quiet but still shows ~14° between the FC's DCM and the
*sim's* truth while the fins steer hard. That residual is **gyro-side** (confirmed: the gate is
active, and SITL gyro noise is zero by default), and it is partly a measurement artifact: the
`d` column compares two independent integrators of the same gyro — the FC's DCM and the sim's
own attitude — which drift apart during fast dynamics. **On hardware there is no second
integrator**; DCM *is* the attitude, it integrates the real gyro, and the airframe tracks it.
So the real-flight steering error is expected to be smaller than the sim's `d` suggests.

**GPS stays OFF (`GPS1_TYPE 0`), not merely out of the estimator.** Two reasons:
1. It isn't fitted, and a receiver loses lock under high-g boost anyway.
2. **A present GPS corrupts DCM.** Its velocity feeds DCM's centripetal-acceleration
   correction, which pollutes the tilt once the fins start steering — in SITL, GPS-present
   made the estimate **diverge past 100°** while GPS-off held it within a few degrees.
   `AHRS_GPS_USE=0` gates GPS *navigation* but not that correction, so it is not enough; the
   receiver must be off. (`EK3_SRC1_*` are kept for the dormant EKF3 but are moot with GPS off.)

**Vertical velocity** — needed by apogee detection and the fin gain schedule — then comes
from the **barometer's filtered climb rate** automatically: with DCM primary and no GPS,
`get_velocity_D(_, true)` → `get_vert_pos_rate_D()` falls back to `AP_Baro::get_climb_rate()`
(a 7-point derivative filter). No code change was needed to feed it.

> ⚠️ **Two ways this exact config has silently NOT taken effect — verify it at runtime, do not
> trust the `.parm` file.** `rocket_test.py` now reads `AHRS_EKF_TYPE` and `GPS1_TYPE` at startup
> and refuses to run if either is wrong; do the equivalent (dump params, grep) on hardware.
> 1. **Wrong param name.** The GPS type param is **`GPS1_TYPE`**, not the older `GPS_TYPE`. A
>    `GPS_TYPE 0` line is silently discarded, GPS stays ON, and the DCM runs *with* GPS — which
>    corrupts the tilt off-axis and looks exactly like an estimator bug. (Same silent-discard mode
>    as the historically-dead `EK3_SRC1_*` before the EKF param group was registered — see the bug
>    table. Grep the live param list for what you depend on.)
> 2. **Defaults not loaded at all.** The raw `./build/sitl/bin/rocket --model <frame> -w` launch
>    resolves `rocket.parm` from `@ROMFS/vehicleinfo.json` by **exact model-string match**. A frame
>    not listed there (e.g. a new `-az`/`-roll` variant) falls back to firmware defaults
>    (**EKF3 + GPS on**) and the no-GPS DCM never runs. Either register the frame in
>    `Tools/autotest/pysim/vehicleinfo.json` (then `./waf configure --board sitl` to re-embed) or
>    launch with `--defaults Tools/autotest/default_params/rocket.parm`.
>
> The compass is separately excluded via `COMPASS_USE/2/3 = 0`; it is read only for a display
> heading (see the heading section near the top).

**Consequences, all expected:**
- No position solution and therefore **no home** — normal for this vehicle.
- `GLOBAL_POSITION_INT` is not valid. Finding the airframe after landing is an **off-board**
  job (a separate recovery tracker), not the flight controller's.
- Arming resets the height datum to the rail, so altitude and climb rate are rail-relative.

**Pre-arm warning.** If the estimator ever *does* produce a horizontal position solution,
arming emits `Rocket: EKF has a horizontal position solution - GPS should be tracking only`
— a warning, not a refusal, because the in-flight symptom would otherwise be baffling.

## 3d. Pad procedure (from a ground station)

Operational checklist for driving the vehicle from ArduDeck / QGroundControl / Mission
Planner. Every claim below was re-verified against the build; the measured values are
from SITL runs, not from memory.

Connect over **TCP `127.0.0.1:5760`** (see the WSL2 note near the top — do not use UDP).

### a) The fin check (bench or rail)

Triggered by `MAV_CMD_DO_AUX_FUNCTION` param1 = 300 — the **"Fin Check"** button from
the QGC custom-action file (`Tools/ArduRocket/qgc/ArduRocket.json`). `MAV_CMD_DO_MOTOR_TEST`
also works as a fallback (built-in Motor Test panel) for a GCS without the button.

The **same** command serves bench and rail: on the rail (vertical & still) completing
it latches the arming gate; held in the hand it just exercises the fins and announces
`(bench, will NOT arm)`. You get `Rocket: FIN CHECK - watch the fins`, then, **per fin**,
`Rocket: testing fin N (+/- then center)` -- while that fin visibly does all three
throws, each held ~1.5 s:

| | |
|---|---|
| per fin | 1500 ms one way, 1500 ms the other, 1500 ms center = 4500 ms |
| total | 4 fins × 4500 ms = **18 s** |

**Why one GCS message per fin, not per throw.** The firmware sends STATUSTEXT in real
time, but a ground station throttles its on-screen notifications (a queued toaster with
a minimum display time). Announcing every throw -- 12 messages -- makes that toaster
fall seconds behind the actual movement (you see fin 4 moving while it still shows fin
1). One message per fin keeps the GCS in step. The **throw-by-throw** detail
(`fin N -> +100% / -100% / center`) still goes to the **local SITL console**, which is
real-time and unthrottled, for the case where there is no physical fin to watch. The
1.5 s hold is also deliberately generous so the fin dwells at each position well after
the per-fin message has rendered.

Two deliberate properties:
- It is the **identical code path** as the arming check, so what you verify on the bench
  is exactly what runs on the rail.
- It does **not** satisfy the arming gate, and is refused outright while armed. A bench
  run in the workshop must never let anyone skip the on-the-rail check.

> **This is the step that catches the one fatal bug software cannot detect.** A reversed
> servo, a backwards linkage, or swapped fins will arm and fly happily. Physically watch
> each announced fin move the direction you expect. See "What the fin check DOES NOT do".

### b) Prep — params and telemetry

**The one change that matters before a real flight:** `ARMING_SKIPCHK` is `-1` (skip
everything), a SITL bring-up shortcut because the simulated board has no accel
calibration. Set it to **`21064`** for flight — skips only absent hardware (GPS, RC,
airspeed, mission) and keeps baro + INS. Apogee detection rides on the baro; a dead baro
means the fins never stop.

Things that look broken in the GCS but are correct:

| What you see | Why |
|---|---|
| Throttle always 0 | Solid motor. There is no throttle. |
| Roll/pitch read as tilt, not body angles | `ATTITUDE` is reported in the rotated view, so the artificial horizon reads "am I vertical" rather than raw body Euler angles. Deliberate — see the heading section near the top. `ATTITUDE_QUATERNION` still carries true body attitude. |
| Heading is steady and points north | Taken straight off the magnetometer, not the EKF (whose yaw is meaningless nose-up). Verified 357.3° against a 353° truth, 12° peak-to-peak. The compass is NOT in the flight solution (`COMPASS_USE=0`). |
| `GLOBAL_POSITION_INT` lat/lon = 0 | No horizontal fix in the EKF, by design. **Its `relative_alt` IS valid** — verified −0.232 m on the pad. For *position*, read `GPS_RAW_INT`. |
| Mode shows as "unknown" / a bare number 0–6 | The stages are PREP/FINCHECK/ARMED/BOOST/COAST/DESCENT/LANDED; `set_mode()` refuses everything by design so no mode-change control does anything. **The vehicle advertises all seven by NAME** via the full standard-modes protocol (`send_available_mode()` → `AVAILABLE_MODES`, plus `AVAILABLE_MODES_MONITOR` and a matching heartbeat `custom_mode`). QGC *receives* them but does **not render** them — this is an **open QGC bug** ([qgroundcontrol#12549](https://github.com/mavlink/qgroundcontrol/issues/12549)), not a vehicle problem, and it hits PX4 custom modes too. Not fixable from firmware: reporting `MAV_AUTOPILOT_GENERIC` to dodge QGC's ArduPilot plugin would (a) likely still not render, per the same bug, and (b) disable QGC's ArduPilot param handling. Left as-is; the plain-language STATUSTEXT narration (liftoff/burnout/apogee/landed) and the flying/on-ground status carry the actual flight state. |

Worth watching:
- **`TILT`** (`NAMED_VALUE_FLOAT`, 5 Hz) — angle from vertical, the same number the arming
  gate uses. 0.08° on a vertical rail; verified 10.035° on `rocket-tilt10`; tracked to
  161.4° during descent, so it stays meaningful for the whole flight.
- **`VFR_HUD` altitude** — barometer, rail-relative after arming.
- **`VFR_HUD` airspeed** — EKF speed, peaked at 408.6 m/s in flight. **NOT the fin gain
  scheduling input**, despite what an earlier comment claimed: this is
  `get_velocity_NED()`, the scheduler uses `get_velocity_D(velD, true)`. They track each
  other only because the motion is near-vertical (408.6 vs 407.9 m/s).

### c) Fin check, then a single-press arm

Pre-arm gates: **tilt within a 20° cone** of vertical, gyro < 15 °/s, all four fins
assigned to `SERVO1-4_FUNCTION` = 190–193, and **the fin check latched on the rail**.

1. **Tap "Fin Check"** (the QGC custom-action button, or the Motor Test panel as a
   fallback). The 18 s wiggle runs — **watch each announced throw** move the direction
   you expect. On the rail it ends with `Rocket: fin check done - if fins moved correctly, ARM`.
2. **ARM.** One press → `Rocket: rail attitude X/Y deg` then `Rocket: armed, on the rail`.

> Until you run the fin check, ARM stays blocked with `PreArm: fin check required (run
> it on the rail)` in the GCS's normal pre-arm readout — **not** an error popup. The
> arm press is your attestation that the fins were correct.

## 4. Scheduler table

`Blimp/Blimp.cpp:50-95` is the shape. Entries **must be priority-ordered**; the table is interleaved
with `AP_Vehicle::get_common_scheduler_tasks()`. The main loop runs at **400 Hz** (ArduRocket is in the
400 Hz group in `AP_Scheduler.cpp`; see §8.9 — a vehicle left out of that branch silently drops to 50 Hz).
A `FAST_TASK` runs once per loop, i.e. at the loop rate.

This is the complete table as implemented (`ArduRocket/ArduRocket.cpp`):

| Task | Rate | Prio | What it does |
|---|---|---|---|
| `AP_InertialSensor::update` | 400 Hz (fast) | — | read the IMU; gates the loop via `wait_for_sample()` |
| `motors_output` | 400 Hz (fast) | — | push fin PWM. **First**, for minimum actuation latency |
| `read_AHRS` | 400 Hz (fast) | — | update the attitude estimate |
| `run_rocket_control` | 400 Hz (fast) | — | stage detection + attitude control + dynamic-pressure gain schedule |
| `AP_GPS::update` | 50 Hz | 9 | position **logging only**, never control (§3c) |
| `update_batt_compass` | 10 Hz | 12 | battery monitor + compass (compass feeds GCS heading only, never control) |
| `update_altitude` | 10 Hz | 21 | `barometer.update()` **and refresh the cached air density** (see below) |
| `full_rate_logging` | 50 Hz | 33 | fast attitude / IMU log messages |
| `AP_Notify::update` | 50 Hz | 36 | LEDs / buzzer |
| `send_rocket_telemetry` | 5 Hz | 38 | `TILT` NAMED_VALUE_FLOAT to the GCS |
| `one_hz_loop` | 1 Hz | 39 | housekeeping |
| `GCS::update_receive` | 400 Hz | 51 | MAVLink in |
| `GCS::update_send` | 400 Hz | 54 | MAVLink out |
| `ten_hz_logging_loop` | 10 Hz | 57 | medium-rate log messages |
| `AP_Logger::periodic_tasks` | 400 Hz | 63 | logger buffer flush |
| `AP_InertialSensor::periodic` | 400 Hz | 66 | INS periodic housekeeping |
| `AP_Scheduler::update_logging` | 0.1 Hz | 69 | scheduler performance log |

Omitted, with reasons: `rc_loop`/`read_radio` (no RC — removes the RC failsafe surface entirely);
`three_hz_loop`/`failsafe_gcs_check` (a GCS failsafe on a rocket is a misfire source);
`ekf_check`/`check_vibration` (these trigger *mode changes* in Copter; inert given §2, and a rocket is
guaranteed high-vibration — `check_vibration` would fire every flight).

**Keep the GCS tasks** despite "no ground station" — autotest drives the vehicle over MAVLink. Keep the
transport, delete the failsafes.

### Air density is cached at 10 Hz, deliberately

`run_rocket_control` computes dynamic pressure `q = ½·ρ·v²` every loop to schedule the fin gains. `v`
(EKF vertical speed) genuinely changes every loop and is read at 400 Hz. **`ρ` is not**: it is refreshed
in `update_altitude` at 10 Hz and cached in `air_density_kgm3`, and the 400 Hz path just reads the cached
value. This keeps the `powf` inside `AP_Baro::get_air_density_for_alt_amsl`
(`AP_Baro_atmosphere.cpp:227`, the ISA gradient-layer branch — where all hobby-rocket flight happens) out
of the hot loop: ~400 `powf`/s becomes ~10.

**Why 10 Hz is enough, quantified.** The cache is at most one 0.1 s interval stale. The fastest point of
the whole flight is peak velocity, 403.6 m/s (the `validate_trajectory` reference), so the worst-case
altitude staleness is `400 × 0.1 = 40 m`. Density change over 40 m in the ISA troposphere:

```
ρ(h)/ρ0 = (T/T0)^(g/(L·R) − 1),  T0=288.15 K, L=0.0065 K/m, g=9.80665, R=287.05
  exponent = 5.2559 − 1 = 4.2559
  at 40 m:  T/T0 = 287.89/288.15 = 0.999098
  ρ/ρ0     = 0.999098^4.2559 = 0.996166  →  0.38% change
```

(Cross-check, log-derivative `d(lnρ)/dh = −g/(RT) + L/T = −9.60e−5 /m`, × 40 m = 0.384%. Agrees.)

That worst-case 0.38% lands at **peak speed**, where `q` is ~96 kPa and the mixer gain `Q_REF/q ≈
600/96000 ≈ 0.006` — the fins are barely deflecting, far below the `GAIN_MAX` cap, so a 0.38% error is
invisible. Where the gain schedule *is* delicate (rail exit, near apogee), the airframe is moving slowly
by definition, so altitude — and density — barely change between updates. It is also well under the fixed-Cd
modelling error we already accept (which runs the modelled apogee several percent high). A faster airframe
scales this linearly: ~600 m/s would give ~0.57%, still negligible. So 10 Hz is below the noise floor here,
not a compromise; raising it during boost would spend the reclaimed hot-path CPU to shave an error nothing
downstream can feel, at the one phase where it matters least.

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

  **`RKT_APOG_MS` is the load-bearing parameter here.** Climb rate measures **1.25 m/s
  peak-to-peak while stationary**, so its sign flips negative constantly; only the 500 ms
  continuous-negative debounce stands between that and a spurious apogee — and it is
  thinnest exactly at the top, where the true rate passes slowly through zero. If fins ever
  stop early, look at this parameter first, not at the stage machine, which is only a
  debounced sign test. Full detail and the tuning direction are in
  `libraries/AP_Rocket/README.md`.

  Climb rate is an **EKF3 state fed by baro + IMU, not a differentiated barometer** — that
  is what makes the detection viable at all. Do not replace it with a hand-rolled 1D filter:
  EKF3 already fuses the accelerometer, which a baro-only filter cannot.
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

The SITL plant. `class Rocket : public Aircraft`. Models a thrust curve with burnout,
mass depletion by impulse, altitude-varying air density (from the base class), fin
forces summed per-fin, aerodynamic stability, rate damping, and the launch rail.

> **This section was substantially rewritten. The model as first built was wrong in
> several ways that all flattered the controller; every one was found by measurement,
> not review. If you are reading the git history, do not trust the original numbers.**

### The airframe is a parameter set, not hardcoded constants

Everything that describes the airframe is a runtime `SIM_RKT_*` parameter (SITL.cpp),
so a different rocket is a different `.parm` file with no rebuild. A new **motor**
still needs a rebuild (the thrust curve stays a compile-time array) — an accepted
limit, since motors are not shared between airframes here.

| group | params |
|---|---|
| geometry | `FIN_ROOT FIN_TIP FIN_SPAN BODY_R FIN_ARM TAB_W TAB_H TAB_RT TAB_AX TAB_MAX MARGIN` |
| mass / motor | `DRYMASS PRPMASS IMPULSE BRNTIME IGNDLY` |
| inertia | `JTILT0/JTILT1 JSPIN0/JSPIN1` (loaded/burnt pair, interpolated on burn fraction) |
| aero | `DRAGA ROTDAMP` |

The fin **force**, **arm**, **radius** and **stability** are DERIVED from the geometry
in `recompute_fin_geometry()`, not stored — change a tab dimension and all four move
together. Runtime edits are picked up via a change hash.

### Fin forces are summed per-fin — no mixing in the sim

The sim sees four servo positions and treats each fin independently: a tangential
force at its angular position, `M = p x F`. The pair differences fall out of the
geometry. The sim therefore does NOT encode `AP_FinMixerRocket`'s roll/pitch/yaw
convention — that lives in exactly one place. An earlier version DID copy the mixer
here, and averaged the pair while using a per-fin gain, modelling **half** the tilt
authority; the per-fin form makes that class of bug impossible.

### The airframe is passively STABLE

`instability_gain` is negative (CP aft of CG). The real Quattro is ~2 calibers stable.
`rocket-unstable` flips the sign to model a CP-ahead-of-CG airframe for adversarial
testing. NOTE the original model had this backwards (unstable by default) AND used a
stale 3.87-caliber figure from an older CSV export; both corrected.

### Numbers that were wrong, and how they were caught

| was | is | how found |
|---|---|---|
| `inertia_tilt` 1.80 (assumed 1.5 m airframe) | 4.96 (CSV MOI column) | 2.8x error; the .ork says 2.44 m |
| `rot_damping` 0.35, law `q*omega` | 0.0458, law `V*omega` | damping ratio was 7-183; real is 0.05-0.2 |
| spin damping factor 0.1 (invented) | 0.0183 = (r/arm)^2 | over-damped spin 5x |
| `drag_area` Cd 0.55 x tube A_ref | Cd 0.656 x OR reference A_ref | Cd/A_ref must be a consistent pair; ~7% low |
| fixed sea-level density | ISA vs altitude | 1.48x too dense at apogee |

### Validated against OpenRocket

Flown zero-wind and compared to the export (an independent implementation of the same
airframe): **max velocity within 1.4%, peak thrust/mass exact, apogee +3.7%** (the
apogee margin is the single fixed Cd vs the real transonic rise). See
`scratchpad/validate_trajectory.py`.

### The launch rail is modelled, and matters

Until the airframe has travelled `rail_length_m` from ignition, the rail holds attitude
and constrains motion to the rail line. Applied AFTER `update_dynamics()`, overriding
the base class tailsitter ground handling. It covers exactly the phase where q is near
zero and the fins have no authority. Default 72 in (1.8288 m). Rail exit target is
50 fps (15.24 m/s), per NAR/Tripoli.

### Flight-event narration (SITL console)

The sim `::printf`s the flight's milestones to the SITL console (stdout) -- NOT MAVLink,
so these appear in the terminal running `bin/rocket`, not in the ground station:

```
Rocket: ignition
Rocket: off the rail at 23.2 m/s (76 fps), q=311 Pa
Rocket: burnout
Rocket: apogee at 3900 m (12795 ft)
Rocket: landed - ballistic descent 120 m/s (no recovery modelled)
```

`apogee` prints the tracked peak altitude; `landed` reports the impact speed and flags
that the sim has **no recovery model** -- the descent is ballistic, so the number is
what a parachute would have to bleed off, not a real landing speed.

The sim flies the descent to the ground because `on_rail` latches off once `left_rail`
is set: without that, a disarm (which resets the sim's `ignited` flag) would re-satisfy
the on-rail clamp and freeze the airframe. Since the firmware now stays armed all the
way down (§3), the disarm only happens later -- manually, on the ground -- but the latch
still correctly stops that from re-railing a landed vehicle.

The firmware narrates the flight to the ground station too (so it shows in QGC on real
hardware, not just the SITL console). `set_stage()` sends a plain-language callout on
each key transition, matching the console vocabulary rather than only the terse stage
name (QGC was showing "COAST" for burnout, which is easy to miss):
- **Liftoff**: `Rocket: liftoff`, on ->BOOST.
- **Burnout**: `Rocket: burnout`, on ->COAST.
- **Apogee**: `Rocket: apogee at X m`, on ->DESCENT (the edge IS apogee -- climb rate
  went negative). Height is launch-referenced.
- **Landing**: `Rocket: landed - disarm when recovered`, on ->LANDED, when vertical
  speed has settled to ~zero and held (debounced). Keyed on speed, NOT altitude -- the
  rocket drifts under chute and may land on a hill or in a ditch, so its resting baro
  altitude need not match the pad, but its vertical speed at rest is zero wherever it
  comes down. The vehicle stays ARMED; the message reminds the operator to disarm.

The SITL-console `landed` line is separate -- it narrates the sim's *ballistic* impact
speed (no chute modelled), whereas the firmware LANDED is a settled-on-the-ground
detection.

### Gain scheduling reference — MOT_Q_REF = 600, by measurement

`MOT_Q_REF` sets the maximum control moment the mixer will command, because the fin
output clips at +/-1. It was swept against an 8 m/s crosswind:

| MOT_Q_REF | worst tilt (>25 m/s) | note |
|---|---|---|
| 142 (rail-exit q) | 19.8 deg | fins at 10% travel — authority left unused |
| **600** | **13-16 deg** | fins at 38% |
| 1200 | 12.4 deg | but low-speed band degrades and fins near saturation |

NOTE the earlier text here argued for 142 on the theory that gains should be nominal
at the lowest-authority moment. That is sound for loop STABILITY but throttles
disturbance rejection everywhere above rail exit, because the weathercock moment grows
with q while the scheduled control moment is held constant. 600 is the measured balance.

### Frame strings

| Frame | Meaning |
|---|---|
| `rocket` | 72 in rail, vertical, stable |
| `rocket-unstable` | CP ahead of CG — the hard plant, for adversarial testing |
| `rocket-tilt10` | 10 deg off vertical (NAR/Tripoli cap is 20) |
| `rocket-tilt10-az90` | 10 deg leaning toward 90 deg azimuth (earth heading — a no-op with GPS off) |
| `rocket-tilt10-roll45` | 10 deg lean **clocked 45 deg** — falls on the DIAGONAL between fin pairs, not head-on onto one. This is the orientation variable that matters for a fin-steered rocket; `roll` = rail roll euler. |
| `rocket-tilt10-rail48` | 10 deg, on a 48 in rail |

Suffixes stack (`rocket-tilt5-roll45-az90`). Any `-az`/`-roll` combo is fine, but a combo not
listed in `vehicleinfo.json` needs `--defaults …/rocket.parm` at launch (see §3c ⚠️).

## 9b. MATLAB plant, and the C++/MATLAB split

`Tools/ArduRocket/matlab/` is a second plant that drives the SAME ArduPilot flight code
through the **JSON external-simulator backend** (`--model JSON`): MATLAB owns the
physics, ArduPilot runs unmodified, closed over UDP. See `run_gcs`/`rocket_sim.m` and
the README there for the protocol gotchas (reply to the sender port, newline-wrapped
JSON, specific-force accelerometer convention).

Two independent plants is deliberate — disagreements between them have caught real bugs
(the fin-authority 2x, an inverted initial attitude). But they must be kept identical
where they overlap: both use the per-fin summation, the same density model, and the
SAME fin-geometry derivation.

`rocket_selftest.m` checks the hand-written quaternion and atmosphere maths against
known answers (18 assertions, all also verified in Python). It found the inverted
attitude bug on first run. Run it whenever the MATLAB maths changes.

## 9c. Adding a new rocket — `Tools/ArduRocket/ork_to_rocket.py`

A rocket needs BOTH OpenRocket exports, because neither alone is sufficient:
- the `.ork` has fin geometry and component masses but NOT inertia (OpenRocket computes
  inertia at runtime and does not store it);
- the CSV export has the computed inertia, mass and trajectory but NOT fin geometry.

```
python3 Tools/ArduRocket/ork_to_rocket.py DESIGN.ork FLIGHT.csv --name NAME
```

emits `NAME.parm` (SITL) and `NAME_params.m` (MATLAB), running the same fin-geometry
derivation as the C++ so both sims agree. It deliberately does NOT guess what it cannot
know — control-tab dimensions, fin arm and static margin are left at defaults and
flagged **SET BY HAND** (re-run after editing so the derived force/stability update).
This script exists because hand transcription is exactly where the errors above came
from; it makes the extraction deterministic and repeatable.

## 10. Verification

**Orientation & launch-angle proof (`Tools/ArduRocket/sitl_tests/sweep.py`).** A headless-SITL
matrix of **rail tilt × rail clock** (`-roll`, §9): clock 0 = lean onto a fin pair, 45 = the
diagonal between pairs; 0→90° covers all 360° by 4-fold fin symmetry, and tilt spans the 0–5°
realistic launch angle. Result — **every cell drives to vertical and holds ~0–2° true tilt**
(incl. the diagonal, where both control axes correct at once). Run `python3 sweep.py`; it
auto-retries and marks `P*` for a cell that only passes on retry (the known intermittent
rail-departure spin blip, characterised by `rocket_test.py gains --spin`), `F` for a persistent
failure. Azimuth is not swept: with GPS off the estimate and control are pure body-frame, so
earth heading cannot affect the result.

**SITL — status:**
1. ✅ Full stage sequence `PREP → ARMED → BOOST → COAST → DESCENT → LANDED`, then manual disarm → PREP.
2. ✅ Launch detection with debounce.
3. ✅ Burnout recorded **without** stopping the fins (400us travel during COAST).
4. ✅ Apogee stops the fins (announced to GCS). Vehicle stays ARMED; touchdown detected and announced;
   disarm is manual.
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

# APPENDIX B — Lessons from a flown fin-steered rocket (MatrixPilot RollPitchYaw)

A cross-check against a controller that has actually flown: the **MatrixPilot / UAV
DevBoard "RollPitchYaw" demo, repurposed for rocket fin stabilization**. A group flew it
on boards SN1–SN6 between 2015 and 2019. It runs on a dsPIC (UDB4/5) at 40 Hz on a DCM
attitude solution, IMU-only, GPLv3. Source read: `Rocket_Stabilization/RollPitchYaw/`
(`main.c`, `mainRocket.c`, `options.h`).

It is a much simpler controller than ArduRocket, but it is a real flown one, so where it
agrees it is reassuring and where it differs it is worth a look.

## B.1 What it confirms about our design

| Aspect | MatrixPilot (flown) | ArduRocket | Verdict |
|---|---|---|---|
| Tilt sensing | DCM gravity-vector components `rmat[6..8]` fed straight in — never Euler angles | `AP_AHRS_View(ROTATION_PITCH_90)`, tilt read in the rotated view | **Same idea**, arrived at independently — singularity-free tilt |
| Controller | pure **P** on tilt, plus **P** on spin rate; **no integrator** | P/D on attitude; integrators HELD on the rail, near-zero in flight | Aligned — a flown rocket needs no I term |
| Fin mixing | `roll±pitch` / `roll±yaw` across the four fins | `AP_FinMixerRocket` antisymmetric pairs (tilt) + common (spin) | **Identical** — ours is the standard mix |
| Sensors in the loop | IMU only: accel launch, tilt apogee, DCM+gyro steer; no baro/GPS | IMU + baro; GPS excluded from control by config | Aligned (we add baro only for climb-rate apogee) |
| Spin | only damped above `MAX_SPIN_RATE = 500 °/s` | low spin priority, light damping | Aligned — don't fight spin hard |
| At apogee | center the fins and stop steering | DESCENT centers fins, controller shut down (stays armed) | Aligned |

The takeaway: our two most unusual choices — measuring tilt in a rotated view and running
essentially P/D with no flight integrator — are exactly what a repeatedly-flown fin rocket
does.

## B.2 Proposal 1 — start the tilt gain GENTLE (full fin at ≈ 40° tilt)

Their `MAX_TILT_ANGLE` is the tilt at which the fins reach full deflection. The instructive
part is the **evolution across real flights**: early boards used **7.5°** (aggressive — full
authority at a tiny tilt), and by the flown SN4–SN6 boards they had backed it all the way off
to **40°**. That is a group de-tuning the gain over many flights because a high tilt gain
over-controls and oscillates, and a gentle one holds.

**Proposal:** when tuning `ATC_*`, **start gentle** — size the loop so the fins do not reach
full deflection until the airframe is roughly **40° off vertical**, then tighten only if the
response is sluggish. This matches the project's stated goal (hold vertical as long as
possible; gentle is stable) and gives the tuning a principled starting point instead of a
guess.

Caveat on mapping: their controller is single-loop (`fin = gain × tilt`), while ArduRocket's
is a cascade (`angle error → ATC_ANG_*_P → target rate → ATC_RAT_* → fin`), so there is no
one-to-one gain to copy. The **saturation principle** is what transfers: in SITL, command a
tilt step and check at what angle the fins saturate — aim for ~40°, not a few degrees. Turn
`ATC_ANG_*_P` (and the rate gains) down until that holds, then work up.

**Concrete starting point** (sized for full fin at ~40°; NO flight integrator). The cascade
gives `fin ≈ ATC_ANG_P · angle_err · ATC_RAT_P`, so the `ANG_P·RAT_P` product ≈ 1.4 puts full
deflection near 40° tilt (≈ half fin at a 20° lean):
```
ATC_ANG_RLL_P  6.0     ATC_ANG_PIT_P  6.0     (tilt angle -> target rate)
ATC_RAT_RLL_P  0.24    ATC_RAT_PIT_P  0.24
ATC_RAT_RLL_D  0.01    ATC_RAT_PIT_D  0.01
ATC_RAT_RLL_I  0       ATC_RAT_PIT_I  0        (held in flight)
```
An earlier draft used `ANG_P 3.0 / RAT_P 0.05` — that product (0.15) yields only ~6% fin at a
20° lean, far too weak to hold the gravity-turn, so the airframe tilted past the give-up angle
and tumbled. These values are ~9× hotter to match the full-fin-at-40° intent.

**Dialing procedure:**
1. Arm on a **20°-tilted rail** (`rocket-tilt20`) — the maximum NAR/Tripoli launch angle, no
   wind. Watch the fin outputs (`SERVO_OUTPUT_RAW`, or the `RKT` log fin field).
2. Adjust so the fins sit at **~half deflection at the 20° lean** and are **not saturated**. A
   gentle gain (full fin at ~40°) gives ~half at 20°; if they slam to the stops at 20° or a few
   degrees, the gains are too high — come down. The actual full-fin-at-40° point is an *in-flight*
   upset beyond the legal rail — probe it with the wind runs (stage 2/4) or `rocket-unstable`,
   not by tilting the rail past 20°.
3. Raise `ATC_ANG_*_P` until a step response first shows overshoot / oscillation, then back off
   ~30%. That is the gentle baseline.
4. Only then chase the wind-envelope numbers (§9): the goal is holding vertical as long as q
   lasts, not a fast step.

This is a **tuning task, no code change** — the `ATC_*` gains are runtime parameters.

## B.2.1 Testing schedule (built-in SITL and MATLAB-in-the-loop)

Both sims run the **same flight code**, so tune fast in one and cross-check the final gains in
the other: if a gain set behaves the same on both plants it is not overfit to one plant's
quirks — that is the whole reason to keep both.

**Roles:**
- **Built-in SITL** (`--model rocket`) — fast, real-time. Do the bulk of the tuning here:
  parameter sweeps, step responses, wind runs.
- **MATLAB-in-the-loop** (`--model JSON` + `rocket_sim`) — same controller, a plant you can
  plot and inspect (the four live plots: tilt / q / fins / altitude). Use it to **confirm** the
  final gains and to see *why* a run behaves as it does.

**Stages — run each in built-in SITL first, then re-run the final candidate in MATLAB:**

| # | Test | Setup | Pass criterion |
|---|---|---|---|
| 0 | Quiet on the rail | armed, vertical, no wind | fins centered, no chatter; integrators held |
| 1 | Step response | arm on `rocket-tilt20` (MATLAB: `P.rail_tilt_deg = 20`) — max legal rail, no wind | fins ~half at the 20° lean (gentle = full at ~40°), not saturated; settles to vertical with ≤1 overshoot |
| 2 | Disturbance rejection | vertical, 8 m/s crosswind (`SIM_WIND_*`) | holds within a few degrees while q is high; no oscillation |
| 3 | Full flight | launch → boost → coast → apogee | tilt small through boost; degrades gracefully as q drains late in coast |
| 4 | Wind envelope | sweep wind 0 → 20 mph (operational ceiling) | worst tilt tracks the §9 table (~15° at 20 mph); no divergence |

**Wind caveat:** the MATLAB plant has **no wind model yet** (`rocket_step.m`: `vel_air = vel`),
so stages **2 and 4 are built-in-SITL only**. Cross-check the wind-free stages (**0, 1, 3**) in
MATLAB.

**Cross-check rule:** once stage 4 passes in built-in SITL, re-run stages 0/1/3 in MATLAB with
the same gains. Expect the same qualitative behaviour and similar worst-case tilt. Divergence
means one plant is wrong (or the gains exploit a plant artefact) — investigate before trusting
them; the MATLAB live plots show the mechanism.

**"Done" looks like:** vertical held to a few degrees while dynamic pressure lasts, wind within
the 20 mph limit, fins neither chattering nor saturating early — and the same result on both
plants.

## B.3 Proposal 2 — a tilt-angle apogee / give-up backstop

Their apogee detection is **not** climb-rate — it is **tilt > 60° after launch → lock apogee,
center the fins, stop steering** (`DETECT_APOGEE`; `rmat[7] < 8256` ≈ 60°). The reasoning: once
a fin-steered rocket is more than ~60° off vertical it has lost the plot, so stop flailing the
fins against a plant it can no longer control.

ArduRocket detects apogee from **negative climb rate** (EKF), which is the right primary
trigger. But it has **no backstop for losing control before apogee** — if the airframe departs
and tumbles while still nominally ascending, climb-rate may stay positive for a while and the
fins keep driving uselessly.

**IMPLEMENTED (hardened).** A tilt-angle backstop: while steering (BOOST/COAST), if
`tilt_from_vertical_deg()` exceeds **`RKT_GIVEUP_DEG`** (default **60°**, `0` = disabled) the
controller announces `Rocket: tilt N deg - giving up` and moves to DESCENT — stop steering,
centre the fins. It catches the "control lost before apogee" case the climb-rate test misses,
and complements, not replaces, the climb-rate apogee.

The trip is **corroborated**, because without GPS the EKF attitude estimate can briefly glitch
to a huge tilt during the high-thrust boost or the low-speed apogee transition (a spike in the
*estimate* with no matching motion on the raw gyro — see SITL, where a phantom 117° tilt tripped
the original backstop while the true tilt was 30°). So the backstop now requires **both**:
- the over-tilt held for **`RKT_GIVEUP_MS` = 1000 ms** (was 200 ms), and
- the raw gyro showing the airframe is genuinely rotating, `|gyro| > RKT_GIVEUP_RATE_DPS` (25 °/s).

A frozen estimate glitch fails the gyro test; a brief one fails the long debounce; a real
tumble satisfies both. This is a **band-aid over the symptom** — the root cause is the
attitude estimate degrading under high axial thrust with no GPS (accelerometer swamped by the
motor, so it cannot serve as the gravity reference), which is a separate state-estimation task.

Implementation note: the give-up DESCENT has to be made **sticky**. The ascent detector
(`AP_Rocket`) only knows stages up to DESCENT, so while climb rate is still positive it would
map the vehicle back to BOOST/COAST and undo the give-up. The stage-mapping guard in
`run_rocket_control()` therefore skips remapping once `stage` is DESCENT **or** LANDED — both
are terminal until touchdown / a manual disarm.

## B.4 Proposal 3 (optional, future) — margin-based authority allocation

Their `roll_feedback()` is more sophisticated than our mixer in one respect: it computes the
fin-throw **left over** after the tilt commands and hands the remainder to spin, per axis, with
explicit cases for which axis has margin. `AP_FinMixerRocket` does a simpler version (`rp_scale`
reserves a fixed `yaw_headroom`; tilt wins ties). Ours is adequate — tilt is what matters and it
gets priority — but if we ever see spin authority starved when tilt saturates, their
margin-sharing scheme is the reference to copy.

## B.5 Their flown configuration, for reference (SN5/SN6)

`MAX_TILT_ANGLE 40°`, `MAX_TILT_PULSE_WIDTH 500 µs`, `MAX_SPIN_RATE 500 °/s`,
`MAX_SPIN_PULSE_WIDTH 500 µs`, `GYRO_RANGE 1000 °/s`, 40 Hz loop, `DETECT_APOGEE` on,
`NO_MIXING` (SN5 drove three separate control channels rather than mixing four fins). Note the
gyro range: a rocket spins fast, so they ran ±1000 °/s — a reminder that gyro full-scale, like
the ±32 g accel (§0), is a real hardware selection point.
