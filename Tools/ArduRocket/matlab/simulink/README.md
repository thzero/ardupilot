# ArduRocket — Simulink / Aerospace Blockset plant

> Tooling notes, not spec. The authoritative docs are `ArduRocket/ARDUROCKET_*.md` (README = operate,
> PLAN = design record, STATUS = current state + what's next); this defers to them.

The Simulink version of the plant. MATLAB owns the physics as a Simulink model;
ArduPilot runs the **real flight code** and returns fin commands over UDP, exactly as in
the script bridge (`../rocket_sim.m`) — this just swaps the hand-integrated `rocket_step.m`
for an Aerospace Blockset **6DOF** block plus the reusable MATLAB Function blocks in this
folder.

> **Reality check.** A Simulink model is a binary `.slx` that cannot be authored or tested
> outside MATLAB, so what lives here is: (1) the MATLAB Function-block **sources** (correct
> MATLAB, reusable either way), (2) this **wiring guide** to assemble the model by hand,
> and (3) `build_rocket_model.m`, a **programmatic builder** that assembles it for you.
> None of it has been run — expect to tune the UDP timing and the rail constraint on the
> first pass. The physics function blocks mirror `rocket_step.m` and are the trustworthy part.

## Files
| File | Role (MATLAB Function block) |
|---|---|
| `rocket_aero_fcn.m` | body-frame aero force & moment (fins + stability + damping + drag) |
| `rocket_servo_decode.m` | 40-byte servo packet → 4 fin deflections + `ok` flag |
| `rocket_json_encode.m` | sensor state → newline-wrapped JSON reply bytes |
| `build_rocket_model.m` | builds the `.slx` programmatically (option b) |

## Block diagram
```
 [UDP Receive :9002] --bytes--> [rocket_servo_decode] --fin[4]--> [rocket_aero_fcn] --F_aero,M_aero--+
                                        |ok (enable/lockstep)                                         |
 [ISA Atmosphere Model] --rho------------------------------------> [rocket_aero_fcn]                  |
                                                                                                      v
 [Thrust & mass subsystem] --thrust(bodyX), mass, dm/dt, I, dI/dt ----> (+thrust) --> [Custom Variable
                                                                                        Mass 6DOF
   fin, states -----------------------------------------------------------------------> (Quaternion)] --+
                                                                                                         |
   6DOF outputs: pos(NED), vel(NED), quat[w x y z], body rates (pqr), accel ------> [rocket_json_encode] |
                                                                                            |            |
                                                                       [UDP Send :sender] <-+            |
   live scopes on tilt / altitude / fins <-----------------------------------------------------------------+
```

## Blocks to place
- **Aerospace Blockset → Equations of Motion → 6DOF → `Custom Variable Mass 6DOF (Quaternion)`.**
  This is the core. Inputs: body Forces, body Moments, Mass, dMass/dt, Inertia (3×3),
  dInertia/dt. Outputs: NED position & velocity, quaternion, body rates, DCM, accels.
  Initialise its attitude to the rail attitude (nose up + rail tilt) — the same
  `angle2quat(azimuth, 90-tilt, 0)` the script's `rocket_init.m` uses.
- **Aerospace Blockset → Environment → Atmosphere → `ISA Atmosphere Model`** (or COESA).
  Feed it altitude `= P.origin_alt - pos_ned(3)`; take the density output.
- **The three MATLAB Function blocks** from this folder.
- **UDP Receive / UDP Send** (Instrument Control Toolbox, or DSP System Toolbox "UDP
  Receive/Send"). Receive on **9002**; Send back to the **packet's sender** (same as the
  script — SITL sends from an ephemeral port and listens on it).
- A **Thrust & mass** subsystem: `thrust = interp1(thrust_time, thrust_newtons, t_burn)`,
  mass depleted by impulse (`mass = dry + prop*(1 - impulse_used/total_impulse)`), and the
  loaded/burnt inertia interpolated on burn fraction → the 6DOF mass/inertia inputs.

## Parameters
Set a model parameter `P = rocket_params();` (Model Explorer → Model Workspace, or a
`PreLoadFcn`). The MATLAB Function blocks take `P` and read the same fields as the script.

## The two hard parts (expect to iterate)
1. **Lockstep timing.** SITL waits for our reply each step, so the model must step once per
   received packet, not on a free-running solver clock. Drive the model from the UDP
   Receive block (blocking/enabled), or run a fixed-step discrete solver at `P.dt` paced by
   the receive. The `ok` output of the decode block gates a real step.
2. **Rail constraint.** The 6DOF block integrates freely; the launch rail (no rotation, slide
   along the rail only until `rail_length`) has no native block. Replicate `rocket_step.m`'s
   clamp: while on the rail, zero the moments and body rates and project acceleration onto the
   rail direction. Easiest as an **enabled subsystem** that overrides the 6DOF during the
   on-rail phase, or by feeding large constraint forces. This is the fiddliest bit.

## Build it
- **By hand:** place the blocks above, paste the function-block sources, wire per the diagram.
- **Programmatically:** run `build_rocket_model` (see that file). It creates the model and
  wires the main signals; you finish the thrust/mass subsystem and the rail constraint.

Then: start the model, start SITL with `--model JSON --speedup 1 -w`, arm from QGC. The
model plays the role `rocket_sim.m` plays in the script version.
