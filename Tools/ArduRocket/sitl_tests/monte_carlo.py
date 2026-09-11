#!/usr/bin/env python3
"""
ArduRocket MONTE CARLO -- fly random (rail lean x crosswind) samples and map the vertical-flight
envelope. Each sample launches a headless SITL on the rocket-tilt<lean> frame, sets a crosswind,
runs the graded `gains` flight, and records PASS/FAIL + the steady TRUE tilt it settled to.

Rail lean is a launch-time property (the model string), so every sample is a fresh SITL. Wind is
runtime (SIM_WIND_*), set per flight. One SITL at a time on port 5760.

  python3 monte_carlo.py --n 50                       # 50 random samples, lean 0-20, wind 0-20 mph
  python3 monte_carlo.py --n 100 --max-wind 25 --speedup 10 --seed 7 --plot
  python3 monte_carlo.py --grid --leans 0,5,10,15,20 --winds 0,10,20   # structured grid instead

Reports pass rate, the worst samples, and tilt percentiles; with --plot writes a lean-vs-tilt
scatter (coloured by wind) to montecarlo.png -- one panel for the steady attitude-hold tilt (with
the 2 deg mission bar) and one for the powered-flight max tilt (apogee nose-over excluded).
"""
import argparse
import os
import random
import re
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))
BIN = os.path.join(ROOT, "build", "sitl", "bin", "rocket")
# Fly the test1 rocket (the harness default now the reference airframe is gone): rocket.parm for the
# estimator/tune + test1.parm for the airframe (SIM_RKT_* + RKT_*), and its real motor via SIM_RKT_ENG.
ROCKET = os.path.join(HERE, "test1.parm")   # self-contained: airframe + embedded motor curve
DEFAULTS = os.path.join(ROOT, "Tools", "autotest", "default_params", "rocket.parm") + "," + ROCKET
TEST = os.path.join(HERE, "rocket_test.py")
PORT = 5760


def wait_port(port, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        try:
            socket.create_connection(("127.0.0.1", port), 1).close()
            return True
        except OSError:
            time.sleep(0.3)
    return False


def run_sample(lean, wind, wdir, speedup, retries=1):
    """Fly one (lean, wind, dir) sample. Returns (result, steady_tilt, powered_max) -- steady is the
    attitude-hold metric (median true tilt, 20-60% of ascent), powered_max is the worst true tilt
    while the fins still have authority (first 75% of ascent, apogee nose-over EXCLUDED). We do NOT
    use the grader's full-ascent 'peak tilt': that is dominated by the apogee gravity-turn nose-over
    (climb -> 0, q -> 0), which every rocket does and which is not a flight-quality number. Retries
    on a no-data run (harness missed apogee), since a real graded flight is what we want."""
    model = "rocket-tilt%g" % round(lean, 1)
    res = "NO-DATA"
    for _ in range(retries + 1):
        sitl = subprocess.Popen(
            [BIN, "--model", model, "--defaults", DEFAULTS, "-w", "--speedup", str(speedup)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=ROOT,
            env={**os.environ, "SIM_RKT_ENG": ROCKET},   # motor curve embedded in the rocket file
            start_new_session=True)
        try:
            if not wait_port(PORT):
                continue
            time.sleep(1.0)
            try:
                out = subprocess.run(
                    [sys.executable, TEST, "gains",
                     "--wind", "%g" % wind, "--wind-dir", "%g" % wdir],
                    capture_output=True, text=True, cwd=ROOT, timeout=180).stdout
            except subprocess.TimeoutExpired:
                continue
            res = ("PASS" if "RESULT: PASS" in out
                   else "FAIL" if "RESULT: FAIL" in out else "?")
            ms = re.search(r"steady TRUE tilt.*?:\s*(-?[\d.]+)\s*deg", out)
            mm = re.search(r"powered-flight max TRUE tilt.*?:\s*(-?[\d.]+)\s*deg", out)
            steady = float(ms.group(1)) if ms else None
            mx = float(mm.group(1)) if mm else None
            if steady is not None:          # got a real graded flight; done
                return (res, steady, mx)
        finally:
            _kill(sitl)
            time.sleep(0.5)
    return (res, None, None)


def _kill(sitl):
    try:
        os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
        try:
            sitl.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def make_samples(a, rng):
    if a.grid:
        leans = [float(x) for x in a.leans.split(",")]
        winds = [float(x) for x in a.winds.split(",")]
        dirs = [float(x) for x in a.dirs.split(",")]
        return [(t, w, d) for t in leans for w in winds for d in dirs]
    return [(rng.uniform(0, a.max_lean),
             rng.uniform(0, a.max_wind),
             rng.uniform(0, 360)) for _ in range(a.n)]


def plot(results, path):
    # Two panels: STEADY tilt (attitude hold -- median true tilt over 20-60% of ascent, the mission
    # metric) and POWERED-FLIGHT MAX tilt (worst true tilt over the first 75% of ascent, while the
    # fins have authority -- the apogee gravity-turn nose-over is EXCLUDED). Plotting the powered max
    # vs lean coloured by wind shows the real wind envelope during controllable flight; it is NOT a
    # "rail-exit transient" and it does NOT include the unavoidable apogee nose-over.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(15, 6))
    for ax, idx, ylab, title in ((a1, 4, "steady TRUE tilt (deg)", "attitude hold (mission metric)"),
                                  (a2, 5, "powered-flight max TRUE tilt (deg)",
                                   "worst tilt in powered flight (apogee nose-over excluded)")):
        ok = [(t, w, r[idx]) for r in results for (t, w, d) in [r[:3]]
              if r[idx] is not None and r[3] == "PASS"]
        bad = [(t, w, r[idx]) for r in results for (t, w, d) in [r[:3]]
               if r[idx] is not None and r[3] != "PASS"]
        if ok:
            sc = ax.scatter([x[0] for x in ok], [x[2] for x in ok],
                            c=[x[1] for x in ok], cmap="viridis", s=40, label="PASS")
            fig.colorbar(sc, ax=ax, label="crosswind (mph)")
        if bad:
            ax.scatter([x[0] for x in bad], [x[2] for x in bad], c="red", marker="x",
                       s=70, label="FAIL")
        if idx == 4:   # the 2 deg pass bar is on the steady attitude only; the transient has none
            ax.axhline(2.0, color="red", ls="--", lw=1, label="steady bar (2 deg)")
        ax.set_xlabel("rail lean (deg)")
        ax.set_ylabel(ylab)
        ax.set_title(title)
        ax.legend()
        ax.grid(alpha=0.3)
    fig.suptitle("ArduRocket vertical-flight envelope: lean x crosswind", fontweight="bold")
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"\nwrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=50, help="random samples (ignored with --grid)")
    ap.add_argument("--max-lean", type=float, default=20.0, dest="max_lean")
    ap.add_argument("--max-wind", type=float, default=20.0, dest="max_wind")
    ap.add_argument("--speedup", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", action="store_true", help="write montecarlo.png")
    ap.add_argument("--grid", action="store_true", help="structured grid instead of random")
    ap.add_argument("--leans", default="0,5,10,15,20")
    ap.add_argument("--winds", default="0,10,20")
    ap.add_argument("--dirs", default="0,90,180,270")
    a = ap.parse_args()

    if not os.path.exists(BIN):
        sys.exit("no binary at %s -- build first: ./waf rocket" % BIN)
    if wait_port(PORT, timeout=0.5):
        sys.exit("port %d in use -- kill any running SITL first" % PORT)

    rng = random.Random(a.seed)
    samples = make_samples(a, rng)
    print(f"MONTE CARLO: {len(samples)} samples  (lean 0-{a.max_lean:g} deg, "
          f"wind 0-{a.max_wind:g} mph, dir 0-360)  speedup {a.speedup}\n")
    # steady = attitude hold (mission metric, median true tilt 20-60% of ascent); pmax = worst true
    # tilt in powered flight (first 75% of ascent, apogee nose-over excluded) -- the real wind-driven
    # excursion while the fins have authority. Report both.
    print("   #   lean   wind   dir   result   steady   pow_max")

    results = []
    for i, (t, w, d) in enumerate(samples):
        res, steady, mx = run_sample(t, w, d, a.speedup)
        results.append((t, w, d, res, steady, mx))
        ss = f"{steady:.1f}" if steady is not None else "  --"
        xs = f"{mx:.1f}" if mx is not None else "  --"
        print(f"  {i:3d}  {t:5.1f}  {w:5.1f}  {d:4.0f}   {res:6}   {ss:>5}   {xs:>5}")

    graded = [r for r in results if r[4] is not None]
    passes = [r for r in graded if r[3] == "PASS"]
    print("\n" + "=" * 60)
    print(f"graded {len(graded)}/{len(results)} samples;  "
          f"PASS {len(passes)}/{len(graded)} "
          f"({100*len(passes)/len(graded) if graded else 0:.0f}%)")

    def pctiles(vals, label):
        vals = sorted(v for v in vals if v is not None)
        if not vals:
            return
        n = len(vals)
        p = lambda q: vals[min(n - 1, int(q * n))]
        print(f"{label}: median {p(0.5):.1f}  90th {p(0.9):.1f}  max {vals[-1]:.1f} deg")

    if graded:
        pctiles([r[4] for r in graded], "steady tilt (attitude hold)")
        pctiles([r[5] for r in graded], "powered-flight max tilt (apogee nose-over excluded)")
        # rank by the wind-sensitive metric: the biggest excursion during controllable (high-q) flight
        worst = sorted((r for r in graded if r[5] is not None), key=lambda r: -r[5])[:5]
        print("worst 5 (highest powered-flight max tilt):")
        for t, w, d, r, st, mx in worst:
            print(f"   lean {t:4.1f}  wind {w:4.1f} @ {d:3.0f}   {r}   pow_max {mx:4.1f}  steady {st:.1f}")
    if a.plot:
        plot(results, os.path.join(ROOT, "montecarlo.png"))
    fails = [r for r in graded if r[3] != "PASS"]
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
