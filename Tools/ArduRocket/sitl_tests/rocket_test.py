#!/usr/bin/env python3
"""ArduRocket SITL test harness -- the tuning schedule from ARDUROCKET_PLAN.md B.2.1,
plus the tilt give-up backstop (B.3), as repeatable scripted runs.

It connects to a RUNNING SITL over MAVLink, sets parameters, force-arms (bypassing the
pre-arm fin check, which is a human step not relevant to a plant test), lets the sim fly,
and reports metrics: worst tilt, whether it settled, fin saturation, stage transitions,
and whether the give-up backstop fired.

Two SITL levers set up each situation:
  * RAIL TILT is chosen at LAUNCH by the model string -- you must start SITL with the
    right one for the scenario (this script cannot change it at runtime):
        vertical rail : build/sitl/bin/rocket --model rocket        -w
        20 deg rail   : build/sitl/bin/rocket --model rocket-tilt20  -w   (max legal launch angle)
  * WIND is runtime, set by this script via SIM_WIND_*.

USAGE (start SITL first, in another terminal):
    python3 rocket_test.py quiet                 # stage 0  (start SITL: --model rocket)
    python3 rocket_test.py step                  # stage 1  (start SITL: --model rocket-tilt20)
    python3 rocket_test.py wind   --mph 20        # stage 2  (--model rocket)
    python3 rocket_test.py flight                # stage 3  (--model rocket)
    python3 rocket_test.py sweep                 # stage 4  (--model rocket) 0..20 mph
    python3 rocket_test.py giveup                # backstop (--model rocket-tilt20-unstable)

Connect: tcp:127.0.0.1:5760 by default (--url to change, e.g. the WSL IP from Windows).
"""
import argparse
import math
import os
import time
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
GENTLE = os.path.join(HERE, "gentle_gains.parm")

MPH = 0.44704   # mph -> m/s


def connect(url):
    m = mavutil.mavlink_connection(url, source_system=254, source_component=190)
    m.wait_heartbeat(timeout=30)
    m._sys = m.target_system
    m._comp = m.target_component
    # ATTITUDE (est) and SIMSTATE (truth, incl. TRUE body rates) at 25 Hz; servo outputs 25 Hz.
    for msgid, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 25),
                      (mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 25),
                      (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 25)):
        m.mav.command_long_send(m._sys, m._comp,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msgid, int(1e6 / hz), 0, 0, 0, 0, 0)
    return m


def set_param(m, name, value, ptype=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    m.mav.param_set_send(m._sys, m._comp, name.encode(), float(value), ptype)
    time.sleep(0.2)


def load_params(m, path):
    n = 0
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) >= 2:
                set_param(m, parts[0], float(parts[1]))
                n += 1
    print(f"loaded {n} params from {os.path.basename(path)}")


def reboot_fc(m, url):
    """Reboot the flight computer so boot-time params (GPS_TYPE, AHRS_EKF_TYPE) take
    effect, then reconnect. Params set before this persist in SITL storage across the
    soft reboot (only the launch-time -w wipes storage), so they come back applied."""
    print("rebooting FC to apply boot-time params ...")
    m.mav.command_long_send(m._sys, m._comp,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                            1, 0, 0, 0, 0, 0, 0)
    try:
        m.close()
    except Exception:
        pass
    time.sleep(4)
    for _ in range(20):
        try:
            m2 = connect(url)
            print("reconnected after reboot")
            return m2
        except Exception:
            time.sleep(1)
    raise SystemExit("could not reconnect after reboot")


def set_wind(m, mph, direction=90.0):
    set_param(m, "SIM_WIND_SPD", mph * MPH)
    set_param(m, "SIM_WIND_DIR", direction)
    set_param(m, "SIM_WIND_TURB", mph * MPH * 0.25)


def force_arm(m):
    # param2 = 21196 forces arming past the pre-arm checks (the fin check is a human step).
    m.mav.command_long_send(m._sys, m._comp,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 21196, 0, 0, 0, 0, 0)


def tilt_from_attitude(att):
    # ATTITUDE is reported in the rotated view (vertical reads as level), so tilt from
    # vertical = acos(cos(roll)*cos(pitch)) -- the same as tilt_from_vertical_deg().
    c = math.cos(att.roll) * math.cos(att.pitch)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def run(m, seconds):
    """Fly for `seconds`, collecting tilt/fin/stage. Returns a metrics dict."""
    t_end = time.time() + seconds
    tilt, stages, gaveup = [], [], False
    fin_vs_tilt = []          # (tilt_deg, max |fin deflection| in -1..1) paired samples
    traj = []                 # (t_rel, est_tilt, true_tilt, fin) time series
    servo_sat = False
    last_tilt = None
    last_true = None
    last_fin = 0.0
    truerates = []            # (t_rel, spin_x deg/s, transverse_yz deg/s) from SIMSTATE truth
    t0 = None
    ascent_end = None         # abs time control stopped (apogee / give-up); metrics scope
    last_hb = 0
    while time.time() < t_end:
        if time.time() - last_hb > 0.5:
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb = time.time()
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "ATTITUDE":
            now = time.time()
            if t0 is None:
                t0 = now
            last_tilt = tilt_from_attitude(msg)
            tilt.append((now, last_tilt))
            traj.append((now - t0, last_tilt, last_true, last_fin))
        elif t == "SIMSTATE":
            # sim TRUTH. SIMSTATE is the RAW airframe attitude (nose-up = pitch +90 deg),
            # NOT the vehicle's ROTATION_PITCH_90 view that ATTITUDE uses. Tilt from vertical
            # is the angle of the nose (body X) from up, which in this convention is
            # acos(sin(pitch)) -- roll- and yaw-independent. Using the view-frame cos*cos
            # formula here would add a bogus 90 deg offset.
            last_true = math.degrees(math.acos(max(-1.0, min(1.0, math.sin(msg.pitch)))))
            # TRUE body rates (rad/s). Split into spin about the long axis (x = the nose) and
            # transverse tilt rate (y,z). A large spin with a tilt is coning, which first-order
            # DCM integration turns into a systematic tilt error.
            if t0 is not None:
                spin = abs(math.degrees(msg.xgyro))
                trans = math.degrees(math.sqrt(msg.ygyro ** 2 + msg.zgyro ** 2))
                truerates.append((time.time() - t0, spin, trans))
        elif t == "SERVO_OUTPUT_RAW":
            chans = [c for c in (msg.servo1_raw, msg.servo2_raw,
                                 msg.servo3_raw, msg.servo4_raw) if c]
            if not chans:
                continue
            defl = max(abs(c - 1500) / 500.0 for c in chans)   # 0..1, full = 1
            last_fin = defl
            if any(c <= 1010 or c >= 1990 for c in chans):
                servo_sat = True
            if last_tilt is not None:
                fin_vs_tilt.append((last_tilt, defl))
        elif t == "STATUSTEXT":
            s = msg.text
            if s.startswith("Rocket:"):
                stages.append((round(time.time(), 2), s))
                if "giving up" in s:
                    gaveup = True
                # Control ends at apogee (or give-up). Latch the first such marker so all
                # metrics below are scored on the ASCENT only -- the post-apogee tumble
                # (fins already centered) is ballistic junk, not control performance.
                if ascent_end is None and ("apogee" in s or "giving up" in s):
                    ascent_end = time.time()

    # Restrict every metric to the ascent window [rail, apogee/give-up].
    ascent_rel = (ascent_end - t0) if (ascent_end is not None and t0 is not None) else None
    asc = [(tr, est, tru, fn) for (tr, est, tru, fn) in traj
           if ascent_rel is None or tr <= ascent_rel]
    est_vals = [est for _, est, _, _ in asc]
    true_vals = [tru for _, _, tru, _ in asc if tru is not None]
    hi = [fn for _, est, _, fn in asc]                     # peak fin deflection over the ascent
    sat_lo = any(fn > 0.98 for _, est, _, fn in asc if est < 8)   # pinned near vertical?
    servo_sat = any(fn >= 0.98 for _, _, _, fn in asc)
    # TRUE spin and transverse rates over the ascent (deg/s).
    rr = [(s, tv) for tr, s, tv in truerates if ascent_rel is None or tr <= ascent_rel]
    spins = [s for s, _ in rr]
    transs = [tv for _, tv in rr]
    return dict(tilt=tilt, traj=traj, ascent_rel=ascent_rel,
                max_tilt=max(est_vals) if est_vals else None,
                final_tilt=est_vals[-1] if est_vals else None,
                true_max=max(true_vals) if true_vals else None,
                true_final=true_vals[-1] if true_vals else None,
                servo_sat=servo_sat, stages=stages, gaveup=gaveup,
                fin_hi=max(hi) if hi else 0.0, sat_lo=sat_lo,
                spin_mean=(sum(spins) / len(spins)) if spins else None,
                spin_max=max(spins) if spins else None,
                trans_max=max(transs) if transs else None)


def print_trajectory(traj, step_s=0.4, ascent_rel=None):
    """Downsample the (t, est_tilt, true_tilt, fin) series to ~step_s spacing. Printing the
    EKF estimate next to the sim truth exposes state-estimation divergence: if `est` jumps
    while `true` stays smooth, the controller is chasing a bad estimate, not mis-tuned. A
    marker line shows apogee/give-up; rows after it are the ballistic tumble, not control."""
    if not traj:
        return
    print("   t(s)   est   true   d    fin        (bar = est tilt)")
    next_t = 0.0
    marked = False
    for tr, est, tru, fn in traj:
        if ascent_rel is not None and not marked and tr > ascent_rel:
            print("   ---- apogee / control ends; below is the ballistic tumble ----")
            marked = True
        if tr + 1e-9 < next_t:
            continue
        next_t = tr + step_s
        ts = f"{tru:5.1f}" if tru is not None else "  -- "
        dd = f"{abs(est - tru):4.0f}" if tru is not None else "  --"
        bar = "#" * min(50, int(est / 3.0))
        print(f"   {tr:4.1f}  {est:5.1f}  {ts}  {dd}  {fn:4.2f}  {bar}")


def report(name, r, extra=""):
    print(f"\n=== {name} ===  {extra}")
    for t, s in r["stages"]:
        print(f"   {s}")
    print("   (metrics below are ASCENT ONLY: rail -> apogee/give-up)")
    if r["max_tilt"] is not None:
        print(f"   max tilt   est {r['max_tilt']:6.1f}   true {fmt(r['true_max'])} deg")
        print(f"   at apogee  est {r['final_tilt']:6.1f}   true {fmt(r['true_final'])} deg")
    print(f"   fins hit the stops: {'YES' if r['servo_sat'] else 'no'}")
    if r.get("spin_mean") is not None:
        print(f"   TRUE spin (roll about nose): mean {r['spin_mean']:.0f}  max {r['spin_max']:.0f} deg/s"
              f"   | transverse (tilt) rate max {r['trans_max']:.0f} deg/s")
        print(f"     (fast spin + tilt = coning -> systematic DCM tilt error; want spin small)")
    if r["gaveup"]:
        print("   GIVE-UP backstop fired (during ascent)")
    if r.get("traj"):
        print_trajectory(r["traj"], ascent_rel=r.get("ascent_rel"))


def fmt(v):
    return f"{v:6.1f}" if v is not None else "   -- "


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario",
                    choices=["quiet", "step", "wind", "flight", "sweep", "giveup", "gains"])
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--mph", type=float, default=8.0, help="crosswind for 'wind'")
    ap.add_argument("--gains", metavar="FILE",
                    help="load this .parm before flying (any scenario)")
    ap.add_argument("--reboot", action="store_true",
                    help="reboot the FC after loading params (needed for boot-time params "
                         "like AHRS_EKF_TYPE / GPS_TYPE to take effect), then reconnect")
    args = ap.parse_args()

    m = connect(args.url)
    print(f"connected to {args.url}")

    if args.gains:
        load_params(m, args.gains)

    if args.reboot:
        m = reboot_fc(m, args.url)

    if args.scenario == "quiet":
        # Stage 0: on the rail, vertical, no wind. Keep it on the rail (long ignition
        # delay) and check the fins sit quiet and centered. Start SITL: --model rocket.
        set_param(m, "SIM_WIND_SPD", 0.0)
        set_param(m, "SIM_RKT_IGNDLY", 60.0)   # do not launch during the check
        force_arm(m)
        report("QUIET ON RAIL", run(m, 6), "expect: tilt ~0, fins do NOT hit the stops")

    elif args.scenario == "step":
        # Stage 1: 20 deg rail (max legal NAR/Tripoli), no wind -- the vehicle must null the
        # 20 deg lean to vertical. Start SITL: --model rocket-tilt20.
        set_param(m, "SIM_WIND_SPD", 0.0)
        force_arm(m)
        report("STEP (20 deg rail)", run(m, 30),
               "expect: fins ~half at 20 deg (gentle), drive to vertical with <=1 overshoot")

    elif args.scenario == "wind":
        # Stage 2: vertical, crosswind. Start SITL: --model rocket.
        set_wind(m, args.mph)
        force_arm(m)
        report(f"WIND {args.mph:.0f} mph", run(m, 30),
               "expect: holds within a few deg while fast; no oscillation")

    elif args.scenario == "flight":
        # Stage 3: nominal full flight, no wind. Start SITL: --model rocket.
        set_param(m, "SIM_WIND_SPD", 0.0)
        force_arm(m)
        report("FULL FLIGHT", run(m, 40),
               "expect: tilt small through boost, degrades as q drains near apogee")

    elif args.scenario == "sweep":
        # Stage 4: wind envelope 0..20 mph. Requires re-arming each pass, so the sim must
        # be re-launched between passes -- here we just report; re-run per wind manually,
        # or watch that each pass disarms. Start SITL: --model rocket, and disarm between.
        print("Run per-wind: set the wind, arm, fly. Re-launch SITL between passes.")
        for mph in (0, 5, 10, 15, 20):
            print(f"  -> for {mph} mph: python3 rocket_test.py wind --mph {mph}")

    elif args.scenario == "gains":
        # THE MISSION IS TO FLY VERTICAL. Grade whether the airframe was actually driven to and
        # HELD near 0 deg off vertical through the powered flight -- NOT whether it merely held
        # its launch lean without tumbling. Measure the median TRUE tilt over the steady window
        # (skip the launch transient and the apogee tail, where q collapses and the fins lose
        # authority to the gravity turn). PASS requires that median below RKT_VERT_TARGET_DEG.
        # Right now this FAILS: the airframe holds ~20 deg because the steering-induced estimate
        # error fools the controller into thinking it is near vertical and easing off. Fixing
        # that (so true, not just est, reaches vertical) is the open work -- do not paper over
        # it by relaxing this bar. Start SITL: --model rocket-tilt20.
        VERT_TARGET_DEG = 8.0
        if not args.gains:
            load_params(m, GENTLE)          # default to gentle_gains.parm next to this script
        set_param(m, "SIM_WIND_SPD", 0.0)
        force_arm(m)
        r = run(m, 30)
        report("GENTLE GAINS (20 deg rail)", r)
        print(f"   peak fin deflection: {r['fin_hi']:.2f}   (want > 0.25 = fins actually steered)")
        print(f"   fins pinned near vertical (<8 deg): {'YES' if r['sat_lo'] else 'no'}   (want no)")
        # Grade ONLY the high-q powered window, where the fins actually have authority. Skip
        # the launch transient AND everything near apogee: up there the airframe is slow, q has
        # collapsed, the fins can do nothing, and the gravity turn noses it over -- vertical
        # control is pointless and must not count against the tune. Window: 20%..60% of ascent.
        traj = r.get("traj") or []
        a = r.get("ascent_rel")
        steady = sorted(tru for (tr, _e, tru, _f) in traj
                        if tru is not None and a and 0.20 * a <= tr <= 0.60 * a)
        steady_tilt = steady[len(steady) // 2] if steady else None
        # "Tumbled" only counts a runaway DURING the powered phase (up to 75% of ascent). The
        # near-apogee gravity-turn nose-over is expected and pointless -- q has collapsed, the
        # fins have no authority -- so it must NOT count against the tune.
        powered_max = max((tru for (tr, _e, tru, _f) in traj
                           if tru is not None and a and tr <= 0.75 * a), default=0.0)
        tumbled = r["gaveup"] or powered_max > 60.0
        flew_vertical = steady_tilt is not None and steady_tilt < VERT_TARGET_DEG
        print(f"   steady TRUE tilt (MISSION: drive to < {VERT_TARGET_DEG:.0f}): {fmt(steady_tilt)} deg")
        print(f"   tumbled / gave up: {'YES' if tumbled else 'no'}")
        ok = flew_vertical and (not tumbled) and (r["fin_hi"] > 0.25) and (not r["sat_lo"])
        print("   RESULT:", "PASS" if ok else "FAIL",
              "" if ok else f"(mission: fly VERTICAL -- true tilt driven below {VERT_TARGET_DEG:.0f} deg "
                            "and held through the powered flight; holding the launch lean is a FAIL)")

    elif args.scenario == "giveup":
        # Backstop: needs a REAL rotating departure, because the hardened give-up requires
        # the raw gyro to corroborate the tilt (a steady lean or an estimate glitch will not
        # trip it -- that is the point). Fly an UNSTABLE airframe (CP ahead of CG) so it
        # genuinely pitches over past the angle while actually rotating.
        # Start SITL: --model rocket-tilt20-unstable   (or --model rocket-unstable).
        set_param(m, "SIM_WIND_SPD", 0.0)
        set_param(m, "RKT_GIVEUP_DEG", 45.0)   # a real departure past 45 deg, gyro-corroborated
        force_arm(m)
        r = run(m, 15)
        report("GIVE-UP BACKSTOP", r, "expect: 'giving up' fires on the real tumble -> DESCENT")
        print("   RESULT:", "PASS" if r["gaveup"] else "FAIL (give-up did not fire)")


if __name__ == "__main__":
    main()
