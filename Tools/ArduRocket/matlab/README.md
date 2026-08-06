# ArduRocket — MATLAB physics driving the real flight code

MATLAB owns the rocket physics. ArduPilot runs **our actual flight code**,
unmodified, and decides the fin deflections. The loop closes through the real
controller, and you watch the reaction in MATLAB.

```
MATLAB (thrust, mass, aero)          ArduPilot SITL (ArduRocket)
        |                                       |
        |--- JSON sensor state (UDP) ---------->|   real mixer +
        |                                       |   real controller
        |<-- binary packet, 16x PWM ------------|
```

No C++ changes are needed. This uses ArduPilot's built-in **JSON external
simulator backend**.

## Running it

```bash
build/sitl/bin/rocket --model JSON --speedup 1 -w
```

```matlab
rocket_sim
```

Then arm from your ground station (TCP 5760 — a separate link from the physics
one). Ignition follows 3 s later.

## Networking (MATLAB on Windows, SITL in WSL)

If MATLAB runs on Windows and SITL runs in WSL, the physics UDP link (port 9002)
crosses the WSL↔Windows boundary — the same boundary that trips up UDP telemetry.
Two ways to make it work:

- **WSL2 mirrored networking (recommended, Windows 11).** Put this in
  `C:\Users\<you>\.wslconfig`, then `wsl --shutdown` and reopen:
  ```
  [wsl2]
  networkingMode=mirrored
  ```
  WSL and Windows then share `localhost` for TCP **and** UDP, so `--model JSON`
  (default `127.0.0.1:9002`) reaches MATLAB, QGC connects to TCP `5760` on
  localhost, and there are no IPs to juggle.
- **Explicit IP.** Otherwise launch `--model JSON:<windows-ip>` so SITL sends the
  servo packets to MATLAB's interface. `rocket_sim.m` already replies to the packet's
  sender, so the return path needs no configuration.

**Toolbox:** `rocket_sim.m` uses `udpport`, which is in the **Instrument Control
Toolbox**. Without it the bridge will not run as written — a Java `DatagramSocket`
version can replace it with no toolbox dependency (ask if you need it).

| file | role |
|---|---|
| `rocket_sim.m` | UDP bridge + main loop + live/summary plots |
| `rocket_params.m` | airframe / motor / aero constants |
| `rocket_init.m` | initial state on the rail |
| `rocket_step.m` | one physics timestep |

## Aerospace Toolbox and Blockset

**Aerospace Toolbox** — no separate version needed. The helpers `rocket_air_density`,
`rocket_quat_to_dcm` and `rocket_quat_from_euler` **auto-detect** the toolbox and use
`atmosisa` / `quat2dcm` / `angle2quat` when it is installed, falling back to the
hand-rolled math when it is not. So `rocket_sim` runs identically with or without the
toolbox; install it and the same script simply uses the validated library functions.
Run `rocket_selftest` after installing to confirm the conventions.

To **force the hand-rolled math even when the toolbox is installed** (to A/B the two, or
keep a run reproducible): `rocket_use_toolbox(false)`. This is a pure override — the
default is unchanged, so without it the helpers still use the toolbox whenever it is
installed. `rocket_use_toolbox(true)` re-enables it.

**Aerospace Blockset** — the Simulink version of the plant lives in `simulink/` (the
6DOF block + reusable MATLAB Function blocks). See `simulink/README.md`. Note it is a
scaffold: a `.slx` cannot be authored outside MATLAB, so that folder holds the function-block
sources, a wiring guide, and a programmatic builder to assemble the model in MATLAB.

## Protocol details (verified against `libraries/SITL/SIM_JSON.{h,cpp}`)

These are the things that will silently waste your afternoon:

- **Reply to the sender, not to a fixed port.** SITL sends servo packets to
  `127.0.0.1:9002` from an *ephemeral* source port and listens on that same
  socket. `rocket_sim.m` replies to `dg.SenderAddress/SenderPort`.
- **Wrap the JSON in newlines** — `"\n{...}\n"`. `SIM_JSON` converts `'\n'` to nul
  and parses between the *last two* nuls, so a message with no leading newline is
  never parsed and you get no error.
- **Servo packet is 40 bytes**: magic `uint16 = 18458`, `frame_rate uint16`,
  `frame_count uint32`, `pwm[16] uint16`.
- **Required JSON fields**: `timestamp`, `imu.gyro`, `imu.accel_body`, `velocity`,
  plus one of `attitude`/`quaternion`. Miss any and the frame is dropped.
- **`accel_body` is specific force** — thrust and aero only, gravity **excluded**.
  A stationary vertical rocket reads about **+9.81 on the nose axis**, not zero.
  Get this wrong and launch detection and the EKF both misbehave in ways that
  look like control bugs.
- **Lockstep**: SITL waits for our reply, so MATLAB sets the pace. Run as slowly
  as the model needs.

Body X points out the **nose**, so a vertical rocket has body X pointing up.

## ⚠️ One assumption remains: the control-tab dimensions

The physics that used to be wrong here have been corrected, so **do not** re-apply
the old warnings:
- Rate damping is now the correct `V·ω` law at a realistic magnitude
  (`rot_damping_coeff = 0.0458`, damping ratio ≈ 0.06, in the 0.05–0.2 sounding-rocket
  band) — **not** the old `0.35` with the `q·ω` (`V²·ω`) law.
- Fin force is **derived from the .ork fin geometry**, not the old `0.0035` guess.
- Drag, inertia, mass, thrust and static margin all come from the OpenRocket export.

**The one number still assumed is the control-tab size.** The airframe steers with
trailing-edge **tabs**, and the .ork's `tabheight`/`tablength` describe the
*structural* through-the-wall mounting tab, not a control surface. So the tab chord
(25% of fin chord), span (75% of fin span) and max deflection (20°) are assumptions,
and `P.fin_force_gain` (currently `0.005456` N/Pa per fin) scales **linearly** with
all three. This is the single number to update once the real tab dimensions are
measured — until then, treat any gain tuned here as provisional.

## Not verified

**These scripts have never been executed** — there is no MATLAB or Octave in the
development environment they were written in. The protocol details above were
read directly from the ArduPilot source and are reliable; the MATLAB syntax is
not. Expect to fix errors on the first run.
