#!/usr/bin/env python3
"""Generate a rocket's simulator config from its OpenRocket files.

    ork_to_rocket.py DESIGN.ork FLIGHT.csv --name quattro

Writes two files describing ONE rocket:
    <name>.parm         SIM_RKT_* + RKT_* defaults for SITL
    <name>_params.m     the MATLAB struct for the JSON-bridge sim

WHY THIS EXISTS
Every value in the sim comes from one of two OpenRocket exports, and neither
file alone is enough:
    the .ork  has geometry and component masses, but NOT inertia (OpenRocket
              computes inertia at runtime and does not store it)
    the .csv  has the computed inertia, mass and trajectory over time, but NOT
              the fin geometry
So a rocket needs BOTH. Pulling the numbers out by hand -- which is how the first
airframe was set up -- is exactly where transcription errors crept in (an inertia
off by 2.8x, a stability figure read from a stale export, a drag area with two
compensating unit errors). This script does the same extraction deterministically.

WHAT IT CANNOT DO
  - The control-tab dimensions are not in either file (OpenRocket has no concept
    of a deflecting tab). They default to the assumed 25% chord / 75% span / 20 deg
    and must be set to the real values by hand.
  - The thrust curve is left to the C++ (a new motor still needs a rebuild). The
    MATLAB output does include it, since MATLAB reads it from the struct.

CSV COLUMN MAP (0-indexed) -- an OpenRocket CSV with NO header row, "everything"
selected. Verified against the reference export; if OpenRocket changes its column
order these must be rechecked.
"""
import argparse
import csv
import math
import re
import sys
import zipfile

# --- unit conversions ---
FT = 0.3048
OZ = 0.0283495
LBFT2 = 0.0421401
IN = 0.0254
CM = 0.01

# --- CSV columns (0-indexed), no header, "all" fields selected ---
C_TIME = 0
C_ALT = 1
C_VEL_TOTAL = 4
C_MASS_OZ = 21
C_JLONG_LBFT2 = 23      # longitudinal (tilt) moment of inertia
C_JROT_LBFT2 = 24       # rotational (spin) moment of inertia
C_CP_CM = 23 - 0        # placeholder; CP/CG resolved from the .ork below
C_THRUST_N = 29
C_CD = 32
C_DIA_IN = 53


def load_csv(path):
    rows = [r for r in csv.reader(open(path)) if r and r[0]]

    def g(r, i):
        try:
            return float(r[i])
        except (ValueError, IndexError):
            return float('nan')
    return rows, g


def extract_csv(rows, g):
    """Everything the CSV can give: mass, inertia, impulse, burn time, drag."""
    powered = [r for r in rows if g(r, C_THRUST_N) > 0]
    if not powered:
        sys.exit("no powered rows (thrust column all zero) -- wrong column map?")

    first, last_pwr = rows[0], powered[-1]
    liftoff_mass = g(first, C_MASS_OZ) * OZ
    dry_mass = g(rows[-1], C_MASS_OZ) * OZ
    prop_mass = liftoff_mass - dry_mass

    # total impulse = integral of thrust dt
    impulse = 0.0
    for i in range(len(rows) - 1):
        f0, f1 = g(rows[i], C_THRUST_N), g(rows[i + 1], C_THRUST_N)
        dt = g(rows[i + 1], C_TIME) - g(rows[i], C_TIME)
        impulse += 0.5 * (f0 + f1) * dt

    burn_time = g(last_pwr, C_TIME)

    j_tilt_loaded = g(first, C_JLONG_LBFT2) * LBFT2
    j_tilt_burnt = g(last_pwr, C_JLONG_LBFT2) * LBFT2
    j_spin_loaded = g(first, C_JROT_LBFT2) * LBFT2
    j_spin_burnt = g(last_pwr, C_JROT_LBFT2) * LBFT2

    # v^2-weighted mean Cd over the fast part of flight (drag impulse lives there)
    num = den = 0.0
    for r in rows:
        v = g(r, C_VEL_TOTAL) * FT
        cd = g(r, C_CD)
        if v > 30 and 0.1 < cd < 2.0:      # reject the low-speed q->0 blow-up
            num += v * v * cd
            den += v * v
    cd_mean = num / den if den else 0.65

    dia = g(first, C_DIA_IN) * IN
    a_ref = math.pi / 4 * dia * dia

    return dict(dry_mass=dry_mass, prop_mass=prop_mass, impulse=impulse,
                burn_time=burn_time, j_tilt_loaded=j_tilt_loaded,
                j_tilt_burnt=j_tilt_burnt, j_spin_loaded=j_spin_loaded,
                j_spin_burnt=j_spin_burnt, cd=cd_mean, a_ref=a_ref, dia=dia,
                liftoff_mass=liftoff_mass)


def extract_ork(path):
    """Fin geometry and body radius from the .ork (zipped XML)."""
    with zipfile.ZipFile(path) as z:
        xml = z.read(z.namelist()[0]).decode('utf-8', 'ignore')

    fin = re.search(r'<trapezoidfinset>.*?</trapezoidfinset>', xml, re.S)
    if not fin:
        sys.exit("no <trapezoidfinset> in the .ork -- only trapezoidal fins are supported")
    fb = fin.group(0)

    def f(tag, blk=fb):
        m = re.search(r'<%s>([^<]*)</%s>' % (tag, tag), blk)
        return float(m.group(1)) if m else float('nan')

    n_fins = int(f('fincount'))
    root = f('rootchord')
    tip = f('tipchord')
    semispan = f('height')

    # body radius: the largest bodytube outer radius
    radii = [float(m) for m in re.findall(r'<outerradius>([-0-9.]+)</outerradius>', xml)]
    body_r = max(radii) if radii else float('nan')

    return dict(n_fins=n_fins, fin_root=root, fin_tip=tip, fin_semispan=semispan,
                body_r=body_r)


def compute_derived(csvd, orkd):
    """Fin arm and static margin need CG/CP, which come from the CSV per-row."""
    # NOTE: CG and CP columns vary by OpenRocket version; leave the .parm at the
    # defaults for FIN_ARM and MARGIN and print a reminder rather than guess a
    # column and get it silently wrong.
    return {}


def derive_fin(ork, tab_chord=0.25, tab_span=0.75, tab_max_deg=20.0,
               static_margin=2.0):
    """Mirror SIM_Rocket::recompute_fin_geometry() so the MATLAB struct gets the
    SAME derived fin constants the C++ sim computes internally. If this drifts from
    the C++, the two sims disagree -- keep them identical."""
    Cr, Ct, sspan, rb = ork['fin_root'], ork['fin_tip'], ork['fin_semispan'], ork['body_r']
    S_fin = 0.5 * (Cr + Ct) * sspan
    AR = 2.0 * sspan**2 / S_fin
    CLa = 2.0 * math.pi * AR / (2.0 + math.sqrt(AR**2 + 4.0))
    Kfb = 1.0 + rb / (sspan + rb)
    theta = math.acos(max(-1.0, min(1.0, 2.0*tab_chord - 1.0)))
    tau = (1.0 - (theta - math.sin(theta)) / math.pi) * 0.85 * tab_span
    force_gain = S_fin * CLa * Kfb * tau * math.radians(tab_max_deg)
    y_mac = (sspan/3.0) * ((Cr + 2.0*Ct) / (Cr + Ct))
    fin_radius = rb + y_mac
    A_ref = math.pi * rb**2
    CNa = 2.0 * S_fin * CLa * Kfb / A_ref + 2.0
    stability = -CNa * A_ref * (static_margin * 2.0 * rb)
    return dict(force_gain=force_gain, fin_radius=fin_radius, stability=stability,
                fin_arm=ork['fin_semispan']*5)   # arm placeholder, flagged for hand-set



PARM_TEMPLATE = """# {name}: SITL simulator config, generated by ork_to_rocket.py
# from {ork} and {csv}
#
# NOT hand-edited. Re-run the script to regenerate. The two values it CANNOT know
# are left at their defaults and flagged below -- set them by hand.

# --- airframe geometry (from the .ork) ---
SIM_RKT_FIN_ROOT {fin_root:.5f}
SIM_RKT_FIN_TIP  {fin_tip:.5f}
SIM_RKT_FIN_SPAN {fin_semispan:.5f}
SIM_RKT_BODY_R   {body_r:.5f}

# --- mass and motor (from the .csv) ---
SIM_RKT_DRYMASS  {dry_mass:.4f}
SIM_RKT_PRPMASS  {prop_mass:.4f}
SIM_RKT_IMPULSE  {impulse:.1f}
SIM_RKT_BRNTIME  {burn_time:.3f}

# --- inertia, loaded / burnt pair (from the .csv) ---
SIM_RKT_JTILT0   {j_tilt_loaded:.4f}
SIM_RKT_JTILT1   {j_tilt_burnt:.4f}
SIM_RKT_JSPIN0   {j_spin_loaded:.5f}
SIM_RKT_JSPIN1   {j_spin_burnt:.5f}

# --- drag (from the .csv, v^2-weighted mean Cd = {cd:.3f}, A_ref = {a_ref:.6f}) ---
SIM_RKT_DRAGA    {drag_area:.6f}

# ============================ SET THESE BY HAND ============================
# Not in either OpenRocket file. Left at defaults; edit to the real airframe.
#
# Control tab: OpenRocket has no concept of a deflecting tab.
SIM_RKT_TAB_C    0.25
SIM_RKT_TAB_SPAN 0.75
SIM_RKT_TAB_MAX  20.0
#
# Fin arm (CP behind CG) and static margin: depend on CG/CP columns whose position
# varies by OpenRocket version, so this script does not guess them. Read them off
# the OpenRocket stability display (margin in calibers) and CG/CP positions.
SIM_RKT_FIN_ARM  {fin_arm_default:.4f}
SIM_RKT_MARGIN   2.0
# ==========================================================================
"""



MATLAB_TEMPLATE = """function P = {name}_params()
% {name}: MATLAB sim config, generated by ork_to_rocket.py
% from {ork} and {csv}. NOT hand-edited -- re-run the script.
%
% The derived fin constants (force_gain, arm, radius, stability) are computed the
% SAME way SIM_Rocket::recompute_fin_geometry() does, so the two sims stay in step.
% Two things this cannot know are left at defaults: the control-tab dimensions
% (baked into force_gain here) and the fin arm / static margin. Set them by hand,
% then RE-RUN so force_gain and stability pick up the change.
P = struct();
P.dt = 0.0025;

P.dry_mass    = {dry_mass:.4f};
P.prop_mass   = {prop_mass:.4f};
P.total_impulse = {impulse:.1f};
P.ignition_delay = 3.0;

% thrust curve: NOT in either OpenRocket file at breakpoint resolution -- copy from
% the motor's .eng/RASP data or the C++ SIM_Rocket thrust_time/thrust_newtons.
P.thrust_time    = [];   % <-- FILL IN
P.thrust_newtons = [];   % <-- FILL IN

% inertia, interpolated loaded->burnt in rocket_step by burn fraction
P.J_tilt = {j_tilt_loaded:.4f};   % loaded; burnt = {j_tilt_burnt:.4f}
P.J_spin = {j_spin_loaded:.5f};  % loaded; burnt = {j_spin_burnt:.5f}

P.rho_sl = 1.225;
P.fin_angle_deg  = [270 180 90 0];
P.fin_force_gain = {force_gain:.6f};   % DERIVED from tab {tab_c}/{tab_span}/{tab_max} deg
P.fin_arm_m      = {fin_arm:.4f};       % <-- SET BY HAND (CP behind CG), then re-run
P.fin_radius_m   = {fin_radius:.4f};
P.stability_gain = {stability:.4f};   % DERIVED from margin {margin} cal

P.rot_damping_coeff = {rot_damp:.4f};
P.drag_area  = {drag_area:.6f};   % Cd({cd:.3f}) * A_ref({a_ref:.6f})

P.rail_length = 1.8288;
P.rail_tilt_deg = 0.0;
P.rail_azimuth_deg = 0.0;
P.g = 9.80665;
P.origin_lat = -35.363262;
P.origin_lon = 149.165237;
P.origin_alt = 584.0;
end
"""

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ork', help='OpenRocket design file (.ork)')
    ap.add_argument('csv', help='OpenRocket flight export (.csv, no header, all fields)')
    ap.add_argument('--name', required=True, help='rocket name (output file prefix)')
    ap.add_argument('--outdir', default='.', help='where to write the outputs')
    args = ap.parse_args()

    rows, g = load_csv(args.csv)
    c = extract_csv(rows, g)
    o = extract_ork(args.ork)

    if o['n_fins'] != 4:
        print(f"WARNING: {o['n_fins']} fins; the sim mixer assumes 4.", file=sys.stderr)

    drag_area = c['cd'] * c['a_ref']

    print(f"\n=== {args.name} ===")
    print(f"  liftoff mass   {c['liftoff_mass']:.2f} kg  (dry {c['dry_mass']:.2f} + prop {c['prop_mass']:.2f})")
    print(f"  total impulse  {c['impulse']:.0f} N.s")
    print(f"  burn time      {c['burn_time']:.2f} s")
    print(f"  J_tilt         {c['j_tilt_loaded']:.3f} -> {c['j_tilt_burnt']:.3f} kg.m^2")
    print(f"  J_spin         {c['j_spin_loaded']:.4f} -> {c['j_spin_burnt']:.4f} kg.m^2")
    print(f"  diameter       {c['dia']/IN:.2f} in  (A_ref {c['a_ref']:.6f} m^2)")
    print(f"  mean Cd        {c['cd']:.3f}  -> drag_area {drag_area:.6f}")
    print(f"  fins           {o['n_fins']}x root {o['fin_root']*1000:.0f} tip {o['fin_tip']*1000:.0f} span {o['fin_semispan']*1000:.0f} mm")
    print(f"  body radius    {o['body_r']*1000:.1f} mm")
    print("  NOT extracted (set by hand): tab dimensions, FIN_ARM, MARGIN")

    parm = PARM_TEMPLATE.format(
        name=args.name, ork=args.ork, csv=args.csv,
        fin_root=o['fin_root'], fin_tip=o['fin_tip'], fin_semispan=o['fin_semispan'],
        body_r=o['body_r'], dry_mass=c['dry_mass'], prop_mass=c['prop_mass'],
        impulse=c['impulse'], burn_time=c['burn_time'],
        j_tilt_loaded=c['j_tilt_loaded'], j_tilt_burnt=c['j_tilt_burnt'],
        j_spin_loaded=c['j_spin_loaded'], j_spin_burnt=c['j_spin_burnt'],
        cd=c['cd'], a_ref=c['a_ref'], drag_area=drag_area,
        fin_arm_default=o['fin_semispan'] * 5)   # rough placeholder, flagged for hand-set

    outp = f"{args.outdir}/{args.name}.parm"
    open(outp, 'w').write(parm)
    print(f"\n  wrote {outp}")

    d = derive_fin(o)
    rot_damp = 0.5 * 1.225 * (4 * 0.5 * (o['fin_root'] + o['fin_tip']) * o['fin_semispan']) \
               * (2 * math.pi * (2 * o['fin_semispan']**2 / (0.5 * (o['fin_root'] + o['fin_tip']) * o['fin_semispan']))
                  / (2 + math.sqrt((2 * o['fin_semispan']**2 / (0.5 * (o['fin_root'] + o['fin_tip']) * o['fin_semispan']))**2 + 4))) \
               * (o['body_r'] + (o['fin_semispan']/3) * ((o['fin_root'] + 2*o['fin_tip'])/(o['fin_root'] + o['fin_tip'])))**2
    matlab = MATLAB_TEMPLATE.format(
        name=args.name, ork=args.ork, csv=args.csv,
        dry_mass=c['dry_mass'], prop_mass=c['prop_mass'], impulse=c['impulse'],
        j_tilt_loaded=c['j_tilt_loaded'], j_tilt_burnt=c['j_tilt_burnt'],
        j_spin_loaded=c['j_spin_loaded'], j_spin_burnt=c['j_spin_burnt'],
        force_gain=d['force_gain'], fin_arm=d['fin_arm'], fin_radius=d['fin_radius'],
        stability=d['stability'], rot_damp=rot_damp, drag_area=drag_area,
        cd=c['cd'], a_ref=c['a_ref'], tab_c=0.25, tab_span=0.75, tab_max=20.0, margin=2.0)
    outm = f"{args.outdir}/{args.name}_params.m"
    open(outm, 'w').write(matlab)
    print(f"  wrote {outm}")


if __name__ == '__main__':
    main()
