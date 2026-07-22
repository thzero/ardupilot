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

| file | role |
|---|---|
| `rocket_sim.m` | UDP bridge + main loop + plots |
| `rocket_params.m` | airframe / motor / aero constants |
| `rocket_init.m` | initial state on the rail |
| `rocket_step.m` | one physics timestep |

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

## ⚠️ Two constants are not physically justified

Carried over from `SIM_Rocket.h` for parity, and worth fixing before trusting any
gain tuned here:

1. **`rot_damping = 0.35` is roughly 70× too large.** It gives a damping ratio of
   7.0 at rail exit rising to 183 at Mach 1.2; real sounding rockets are 0.05–0.2.
   At a realistic 0.2 rad/s the damping moment is **20× what the fins produce at
   full deflection**. An airframe that can barely rotate makes any controller look
   excellent. For ζ ≈ 0.1 at rail exit it should be ≈ **0.005**.

2. **The damping law is dimensionally wrong.** Aerodynamic pitch damping goes as
   `V·ω`; the code uses `rot_damping * q * ω`, which is `V²·ω`. That overstates
   damping by a further ~4× at 60 m/s and ~26× at 400 m/s.

Set `P.use_realistic_damping = true` in `rocket_params.m` to switch to the
corrected law and magnitude, and see how much harder the real problem is.

3. **`fin_moment_gain = 0.0035` is a guess.** It is the single constant deciding
   whether the fins can fly the airframe, and OpenRocket does not export it.
   Fitting it from a SITL log is **circular** — that log used this same guess. It
   needs Barrowman geometry: `Kf ≈ Cn_delta × A_fin × moment_arm`.

## Not verified

**These scripts have never been executed** — there is no MATLAB or Octave in the
development environment they were written in. The protocol details above were
read directly from the ArduPilot source and are reliable; the MATLAB syntax is
not. Expect to fix errors on the first run.
