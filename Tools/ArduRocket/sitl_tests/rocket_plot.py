#!/usr/bin/env python3
"""Plot an ArduRocket flight from a dataflash .BIN -- the real thing, not ASCII.

Reads the truth trajectory (SIM2 PN/PE/PD), the all-axis attitude commanded-vs-
achieved (ANG), and the rocket-specific channels (RKT: tilt, spin, four fins) and
lays them out as one figure:

  * a 3D flight path through space (East/North/Up), coloured by flight stage
  * top-down ground track + altitude and speed vs time
  * roll / pitch / yaw, each as achieved (solid) over commanded (dashed)
  * tilt-from-vertical, nose spin, and the four fin channels

Flight stages (RKT.Stage) are shaded on every time plot and legended on the path:
  2 ARMED (on rail)   3 BOOST (motor)   4 COAST (ascending)   5 DESCENT (ballistic)

Usage:
    rocket_plot.py                     # newest logs/*.BIN, pop up a window
    rocket_plot.py logs/00000133.BIN   # a specific log
    rocket_plot.py --save flight.png   # write a PNG (still shows unless --no-show)
    rocket_plot.py --no-show --save flight.png   # headless

The matplotlib 3D toolkit on this box is shadowed by a stale system package; we
repair mpl_toolkits.__path__ from matplotlib's own location before importing
Axes3D, and fall back to a 2D-only layout if that still fails.
"""

import argparse
import glob
import os
import sys

import numpy as np
from pymavlink import mavutil


# --- rescue the 3D toolkit (venv matplotlib vs. stale system mpl_toolkits) -----
def _enable_3d():
    try:
        import matplotlib
        import mpl_toolkits
        sp = os.path.dirname(os.path.dirname(matplotlib.__file__))  # site-packages
        cand = os.path.join(sp, "mpl_toolkits")
        if os.path.isdir(os.path.join(cand, "mplot3d")) and cand not in mpl_toolkits.__path__:
            mpl_toolkits.__path__.insert(0, cand)
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        return True
    except Exception as e:  # pragma: no cover - environment dependent
        print(f"   (3D unavailable: {e}; falling back to 2D projections)")
        return False


HAVE_3D = _enable_3d()
import matplotlib

if not sys.stdout.isatty() and os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

STAGE = {0: "PREP", 1: "FINCHECK", 2: "ARMED", 3: "BOOST",
         4: "COAST", 5: "DESCENT", 6: "LANDED"}
STAGE_COLOR = {0: "#adb5bd", 1: "#ced4da", 2: "#8a8a8a", 3: "#e8590c",
               4: "#1c7ed6", 5: "#7048e8", 6: "#495057"}


def newest_log():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", ".."))  # repo root
    logs = sorted(glob.glob(os.path.join(root, "logs", "*.BIN")), key=os.path.getmtime)
    if not logs:
        logs = sorted(glob.glob("logs/*.BIN"), key=os.path.getmtime)
    return logs[-1] if logs else None


def load(path):
    """Pull the series we plot out of the log into numpy arrays keyed by message."""
    m = mavutil.mavlink_connection(path)
    cols = {
        "SIM2": ("TimeUS", "PN", "PE", "PD", "VN", "VE", "VD"),
        "ANG": ("TimeUS", "DesRoll", "Roll", "DesPitch", "Pitch", "DesYaw", "Yaw"),
        "RKT": ("TimeUS", "Stage", "Tilt", "Spin", "Fin1", "Fin2", "Fin3", "Fin4"),
    }
    acc = {k: {f: [] for f in fs} for k, fs in cols.items()}
    while True:
        msg = m.recv_match(type=list(cols.keys()), blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        for f in cols[t]:
            acc[t][f].append(getattr(msg, f))
    out = {}
    for k, d in acc.items():
        out[k] = {f: np.asarray(v, dtype=float) for f, v in d.items()}
    return out


def rel_t(d, t0):
    return (d["TimeUS"] - t0) / 1e6 if len(d["TimeUS"]) else d["TimeUS"]


def stage_spans(rkt, t0):
    """Contiguous (t_start, t_end, stage) spans from the RKT.Stage step signal."""
    if not len(rkt["TimeUS"]):
        return []
    t = rel_t(rkt, t0)
    s = rkt["Stage"].astype(int)
    spans, start, cur = [], t[0], s[0]
    for i in range(1, len(s)):
        if s[i] != cur:
            spans.append((start, t[i], cur))
            start, cur = t[i], s[i]
    spans.append((start, t[-1], cur))
    return spans


def shade_stages(ax, spans):
    for a, b, st in spans:
        ax.axvspan(a, b, color=STAGE_COLOR.get(st, "#cccccc"), alpha=0.10, lw=0)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("log", nargs="?", help="path to a .BIN (default: newest logs/*.BIN)")
    ap.add_argument("--save", metavar="PNG", help="write the figure to this file")
    ap.add_argument("--no-show", action="store_true", help="do not open a window")
    ap.add_argument("--no-3d", action="store_true", help="force the 2D top/side layout")
    ap.add_argument("--full", action="store_true",
                    help="do not trim the post-landing stationary tail (show whole log)")
    args = ap.parse_args()

    path = args.log or newest_log()
    if not path or not os.path.exists(path):
        sys.exit("no log found (pass a path or put a .BIN in logs/)")
    print(f"reading {path}")
    d = load(path)
    if not len(d["SIM2"]["TimeUS"]):
        sys.exit("no SIM2 truth trajectory in this log")

    t0 = d["SIM2"]["TimeUS"][0]
    sim, ang, rkt = d["SIM2"], d["ANG"], d["RKT"]
    spans = stage_spans(rkt, t0)

    # NED -> East/North/Up for a natural view
    east, north, up = sim["PE"], sim["PN"], -sim["PD"]
    ts = rel_t(sim, t0)
    spd = np.sqrt(sim["VN"] ** 2 + sim["VE"] ** 2 + sim["VD"] ** 2)

    # A disarmed rocket keeps logging for minutes after touchdown; that stationary
    # tail crushes every time-plot into the first fifth. Trim to the actual flight:
    # up to the last moment it is off the ground OR still moving, plus a short margin.
    # --full keeps everything.
    t_end = ts[-1]
    if not args.full:
        moving = (up > 1.0) | (spd > 1.0)
        if moving.any():
            t_end = ts[int(np.nonzero(moving)[0][-1])] + 3.0

    def trim(d):
        """Slice a message dict (and return its relative-time axis) to t <= t_end."""
        if not len(d["TimeUS"]):
            return d, np.array([])
        t = (d["TimeUS"] - t0) / 1e6
        keep = t <= t_end
        return {k: v[keep] for k, v in d.items()}, t[keep]

    sim, ts = trim(sim)
    east, north, up = sim["PE"], sim["PN"], -sim["PD"]
    ang, ta = trim(ang)
    rkt, tr = trim(rkt)
    spans = [(a, b, st) for (a, b, st) in spans if a <= t_end]
    apogee_i = int(np.argmax(up))
    use_3d = HAVE_3D and not args.no_3d

    # THE mission metric: steady ATTITUDE tilt -- how far the nose is off vertical while it has
    # authority (median RKT.Tilt over 20-60% of ascent, skipping the launch transient and the
    # apogee tail). This is what "flying vertical" means. Do NOT headline the downrange path angle
    # (below): off a tilted rail the body banks sideways velocity in the first seconds and coasts
    # on it, so the launch->apogee straight line reads several degrees even at ~0 attitude -- that
    # confused readers into thinking a passing flight had failed.
    apo_t = ts[apogee_i]
    if len(tr):
        w = (tr > 0.2 * apo_t) & (tr < 0.6 * apo_t)
        steady_tilt = float(np.median(rkt["Tilt"][w])) if np.any(w) else float("nan")
    else:
        steady_tilt = float("nan")
    horiz = np.hypot(east[apogee_i] - east[0], north[apogee_i] - north[0])  # downrange at apogee
    tilt_str = (f"steady tilt ~{steady_tilt:.1f}deg off vertical"
                if np.isfinite(steady_tilt) else "no steady ascent to grade")
    fig = plt.figure(figsize=(16, 19))
    fig.suptitle(
        f"ArduRocket flight  --  {os.path.basename(path)}   "
        f"(apogee {up[apogee_i]:.0f} m @ {ts[apogee_i]:.1f} s, flight {ts[-1]:.1f} s,  "
        f"{tilt_str},  {horiz:.0f} m downrange)",
        fontsize=13, fontweight="bold",
    )
    gs = fig.add_gridspec(5, 3, hspace=0.5, wspace=0.28)

    # --- the flight path -------------------------------------------------------
    if use_3d:
        axp = fig.add_subplot(gs[0:2, 0:2], projection="3d")
        for a, b, st in spans:
            mask = (ts >= a) & (ts <= b)
            axp.plot(east[mask], north[mask], up[mask],
                     color=STAGE_COLOR.get(st, "#888"), lw=1.6,
                     label=STAGE.get(st, str(st)))
        axp.scatter([east[0]], [north[0]], [up[0]], c="k", s=30, label="launch")
        axp.scatter([east[apogee_i]], [north[apogee_i]], [up[apogee_i]],
                    c="red", marker="^", s=45, label="apogee")
        # East/North share one scale so a near-vertical climb reads honestly
        # (no lateral exaggeration); Up keeps its own tall scale.
        hc_e, hc_n = (east.min() + east.max()) / 2, (north.min() + north.max()) / 2
        half = max(np.ptp(east), np.ptp(north), 10.0) / 2
        axp.set_xlim(hc_e - half, hc_e + half)
        axp.set_ylim(hc_n - half, hc_n + half)
        # THE key fix: force the visual box to match real distances, else matplotlib
        # squashes 4000 m of climb into a cube and every flight looks like a 45deg
        # diagonal. With this, a vertical flight looks vertical and a 20deg lean
        # looks like 20deg. Up is 2*half wide horizontally vs its full altitude tall.
        up_span = max(np.ptp(up), 1.0)
        axp.set_box_aspect((2 * half, 2 * half, up_span))
        axp.view_init(elev=8, azim=-75)
        axp.set_xlabel("East (m)")
        axp.set_ylabel("North (m)")
        axp.set_zlabel("Up (m)")
        axp.set_title("flight path (drag to rotate; axes to scale)")
        axp.legend(loc="upper left", fontsize=8)
    else:
        axp = fig.add_subplot(gs[0:2, 0:2])
        for a, b, st in spans:
            mask = (ts >= a) & (ts <= b)
            axp.plot(east[mask], up[mask], color=STAGE_COLOR.get(st, "#888"),
                     lw=1.6, label=STAGE.get(st, str(st)))
        axp.scatter([east[0]], [up[0]], c="k", s=30, label="launch")
        axp.scatter([east[apogee_i]], [up[apogee_i]], c="red", marker="^",
                    s=45, label="apogee")
        axp.set_xlabel("East (m)")
        axp.set_ylabel("Up (m)")
        axp.set_title("flight path -- side view (East vs Up)")
        axp.legend(loc="upper left", fontsize=8)
        axp.grid(alpha=0.3)

    # --- top-down ground track -------------------------------------------------
    axg = fig.add_subplot(gs[0, 2])
    sc = axg.scatter(east, north, c=ts, cmap="viridis", s=4)
    axg.scatter([east[0]], [north[0]], c="k", s=25)
    axg.scatter([east[apogee_i]], [north[apogee_i]], c="red", marker="^", s=35)
    axg.set_aspect("equal", "datalim")
    axg.set_xlabel("East (m)")
    axg.set_ylabel("North (m)")
    axg.set_title("ground track (colour = time)")
    fig.colorbar(sc, ax=axg, label="t (s)", shrink=0.8)
    axg.grid(alpha=0.3)

    # --- velocity vs time ------------------------------------------------------
    axv = fig.add_subplot(gs[1, 2])
    shade_stages(axv, spans)
    axv.plot(ts, -sim["VD"], color="#1c7ed6", lw=1.2, label="climb (-VD)")
    axv.plot(ts, np.hypot(sim["VN"], sim["VE"]), color="#e8590c", lw=1.1,
             label="horizontal")
    axv.axvline(ts[apogee_i], color="red", ls="--", lw=1)
    axv.set_xlabel("t (s)")
    axv.set_ylabel("speed (m/s)")
    axv.set_title("velocity vs time")
    axv.legend(fontsize=8)
    axv.grid(alpha=0.3)

    # --- POSITION per axis: East (X), North (Y), Up (Z) each vs time -----------
    for col, (series, name, axis, color) in enumerate(
        ((east, "East", "X", "#e8590c"),
         (north, "North", "Y", "#2f9e44"),
         (up, "Up", "Z", "#1c7ed6"))
    ):
        ax = fig.add_subplot(gs[2, col])
        shade_stages(ax, spans)
        ax.plot(ts, series, color=color)
        ax.axvline(ts[apogee_i], color="red", ls="--", lw=1)
        ax.set_xlabel("t (s)")
        ax.set_ylabel(f"{name} (m)")
        ax.set_title(f"{axis}: {name} position vs time")
        ax.grid(alpha=0.3)

    # --- attitude: roll / pitch / yaw, achieved (solid) vs commanded (dashed) --
    ta = rel_t(ang, t0)
    for col, (des, act, name) in enumerate(
        (("DesRoll", "Roll", "roll"),
         ("DesPitch", "Pitch", "pitch"),
         ("DesYaw", "Yaw", "yaw"))
    ):
        ax = fig.add_subplot(gs[3, col])
        shade_stages(ax, spans)
        if len(ta):
            ax.plot(ta, ang[act], color="#1c7ed6", lw=1.3, label="actual")
            ax.plot(ta, ang[des], color="#e8590c", lw=1.1, ls="--", label="commanded")
        ax.set_xlabel("t (s)")
        ax.set_ylabel(f"{name} (deg)")
        ax.set_title(f"{name}: actual vs commanded")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

    # --- tilt / spin / fins ----------------------------------------------------
    tr = rel_t(rkt, t0)
    axt = fig.add_subplot(gs[4, 0])
    shade_stages(axt, spans)
    if len(tr):
        axt.plot(tr, rkt["Tilt"], color="#7048e8")
    axt.set_xlabel("t (s)")
    axt.set_ylabel("tilt (deg)")
    axt.set_title("tilt from vertical")
    axt.grid(alpha=0.3)

    axs = fig.add_subplot(gs[4, 1])
    shade_stages(axs, spans)
    if len(tr):
        axs.plot(tr, rkt["Spin"], color="#2f9e44")
    axs.set_xlabel("t (s)")
    axs.set_ylabel("spin")
    axs.set_title("nose spin (roll about long axis)")
    axs.grid(alpha=0.3)

    axf = fig.add_subplot(gs[4, 2])
    shade_stages(axf, spans)
    for i, c in enumerate(("#e8590c", "#1c7ed6", "#2f9e44", "#7048e8"), start=1):
        if len(tr):
            axf.plot(tr, rkt[f"Fin{i}"], color=c, lw=1.1, label=f"fin{i}")
    axf.set_xlabel("t (s)")
    axf.set_ylabel("fin cmd")
    axf.set_title("fin channels")
    axf.legend(fontsize=8, ncol=2)
    axf.grid(alpha=0.3)

    if args.save:
        fig.savefig(args.save, dpi=130, bbox_inches="tight")
        print(f"wrote {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
