#!/usr/bin/env python3
"""
ArduRocket ORIENTATION SWEEP -- proof that the vehicle flies vertical from any lean orientation.

For each (tilt, clock) cell it launches a headless SITL on the `rocket-tilt<T>-roll<C>` frame,
runs the `gains` test, and tabulates PASS/FAIL + the steady TRUE tilt it settled to.

  clock (= rail roll) rotates the lean around the four body-fixed fins:
      clock 0  -> lean falls straight onto ONE fin pair (head-on; the only case tested before)
      clock 45 -> lean falls on the DIAGONAL, between two pairs (both axes correcting at once)
  Sweeping clock 0..90 covers all 360 deg by 4-fold fin symmetry. Sweeping tilt covers the
  0-5 deg launch-rail-angle range. If every cell drives true tilt to vertical (< 8 deg) and
  holds it, the vehicle is PROVEN orientation- and angle-independent (to the sweep resolution).

Runs one SITL at a time on port 5760, with --defaults so the no-GPS DCM config is applied to
every frame (the roll variants are not registered in vehicleinfo.json). rocket_test.py's startup
guard still verifies AHRS_EKF_TYPE=0 / GPS1_TYPE=0 per cell.

  python3 sweep.py
  python3 sweep.py --tilts 0,2,5 --clocks 0,45,90 --speedup 10
"""
import argparse
import os
import re
import signal
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))
BIN = os.path.join(ROOT, "build", "sitl", "bin", "rocket")
DEFAULTS = os.path.join(ROOT, "Tools", "autotest", "default_params", "rocket.parm")
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


def run_cell(tilt, clock, speedup):
    """Launch a headless SITL for one frame, run `gains`, return (result, steady_tilt_str)."""
    model = "rocket-tilt%g-roll%g" % (tilt, clock)
    sitl = subprocess.Popen(
        [BIN, "--model", model, "--defaults", DEFAULTS, "-w", "--speedup", str(speedup)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=ROOT,
        start_new_session=True)
    try:
        if not wait_port(PORT):
            return ("NO-SITL", None)
        time.sleep(1.0)   # let SITL settle after the port opens
        try:
            out = subprocess.run([sys.executable, TEST, "gains"],
                                 capture_output=True, text=True, cwd=ROOT,
                                 timeout=120).stdout
        except subprocess.TimeoutExpired:
            return ("TIMEOUT", None)
        res = ("PASS" if "RESULT: PASS" in out
               else "FAIL" if "RESULT: FAIL" in out else "?")
        m = re.search(r"steady TRUE tilt.*?:\s*([-\d.]+)", out)
        return (res, m.group(1) if m else "?")
    finally:
        try:
            os.killpg(os.getpgid(sitl.pid), signal.SIGTERM)
            try:
                sitl.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(sitl.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        time.sleep(0.5)


def cell_str(res, tilt_v, width=9):
    mark = {"PASS": "P", "RETRY": "P*", "FAIL": "F"}.get(res, "?")
    body = tilt_v if (tilt_v not in (None, "?")) else "--"
    return "%*s" % (width, "%s %s" % (mark, body))


def run_cell_retry(tilt, clock, speedup, retries):
    """Run a cell, retrying on non-PASS. Returns ('PASS'|'RETRY'|'FAIL', tilt, attempts)."""
    last = ("FAIL", None)
    for attempt in range(retries + 1):
        res, tv = run_cell(tilt, clock, speedup)
        if res == "PASS":
            return ("PASS" if attempt == 0 else "RETRY", tv, attempt + 1)
        last = (res, tv)
    return ("FAIL", last[1], retries + 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tilts", default="0,1,2,3,5", help="rail tilt angles, deg (comma list)")
    ap.add_argument("--clocks", default="0,22,45,67,90", help="rail clock angles, deg (comma list)")
    ap.add_argument("--speedup", type=int, default=5, help="SITL speedup (default 5)")
    ap.add_argument("--retries", type=int, default=2,
                    help="re-run a non-PASS cell this many times before calling it FAIL")
    a = ap.parse_args()
    tilts = [float(x) for x in a.tilts.split(",")]
    clocks = [float(x) for x in a.clocks.split(",")]

    if not os.path.exists(BIN):
        sys.exit("no binary at %s -- build first: ./waf rocket" % BIN)
    if wait_port(PORT, timeout=0.5):
        sys.exit("port %d is already in use -- kill any running SITL first (Ctrl-C Terminal 1)"
                 % PORT)

    print("ArduRocket ORIENTATION SWEEP  (tilt deg  x  clock deg)")
    print("clock = lean direction vs fins: 0 = onto a fin pair, 45 = diagonal between pairs")
    print("cell  = P/F  +  steady TRUE tilt (deg); want 'P' with tilt < 8\n")

    head = "  tilt\\clk |" + "".join("%9g" % c for c in clocks)
    print(head)
    print("  " + "-" * (len(head) - 2))

    allpass = True
    flaky = 0
    for t in tilts:
        row = "  %7g  |" % t
        for c in clocks:
            if t == 0 and c != clocks[0]:
                row += "%9s" % "(vert)"      # no lean -> clock irrelevant
                continue
            sys.stdout.write("  ... tilt=%g clock=%g       \r" % (t, c))
            sys.stdout.flush()
            res, tilt_v, _ = run_cell_retry(t, c, a.speedup, a.retries)
            if res == "RETRY":
                flaky += 1
            if res == "FAIL":
                allpass = False
            row += cell_str(res, tilt_v)
        print(row)

    print()
    print("=" * 62)
    print("legend: P = pass first try   P* = passed only after a retry (intermittent)   "
          "F = failed every try")
    if allpass:
        msg = "RESULT: drives to vertical from every orientation & angle tested."
        if flaky:
            msg += (" %d cell(s) needed a retry (the known rail-departure transient) --" % flaky
                    + " orientation-independent, but that intermittent is real.")
        else:
            msg += " No retries needed -- clean."
        print(msg)
    else:
        print("RESULT: at least one cell FAILED every retry (F above) -- a real, persistent "
              "problem at that orientation. Investigate that exact tilt/clock.")
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
