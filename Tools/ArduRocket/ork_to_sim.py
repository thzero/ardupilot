#!/usr/bin/env python3
"""Convert an OpenRocket design + motor into a COMPLETE, self-contained ArduRocket rocket file.

    ork_to_sim.py samples/test1 --tab-chord 0.025 --tab-span 0.030
    Then:  Tools/ArduRocket/sitl_tests/sitl.sh test1 [tiltN]

The positional is a shared name/stem: `samples/test1` uses samples/test1.ork (and, only if that .ork
has no <designinfo>, samples/test1-design.csv) and names the output test1. Override any with
--ork / --design / --name. Only the control-tab dims (--tab-chord/--tab-span) are always required.

Emits <name>.parm (in Tools/ArduRocket/sitl_tests by default) holding the WHOLE rocket in one file:
  * SIM_RKT_*  -- the airframe the simulator flies (mass, inertia, fins, drag, motor, tab, rail damp)
  * RKT_*      -- the flight-controller params (measured airframe + real inertia + RKT_VMAX)
  * # MOTOR:   -- the real thrust curve embedded (so there is no separate .eng to carry)

Where the numbers come from:
  * design stats (mass, CG, inertia, diameter, Cd, stability, control-fin nose-to-root position):
    from the .ork's <designinfo> block (SI) AND/OR a LABELED design CSV (Scope,Field,Value,Unit --
    robust, no column-index guessing, via --design / <base>-design.csv). Both are used and merged --
    the .ork wins where both carry a field, the CSV fills any gaps. If a required stat is in NEITHER,
    the tool bails and lists exactly what is missing.
  * fin planform: the .ork's <trapezoidfinset>.
  * motor: the .ork's SELECTED (default="true") configuration, looked up on thrustcurve.org by
    designation automatically -- override with --motor, or --eng FILE to work offline from a RASP .eng.
  * control tab: not in OpenRocket -- given on the command line (--tab-chord/--tab-span).
There is no reference/default rocket or motor -- you always fly a real one produced by this.
"""
import argparse
import csv
import json
import math
import os
import re
import sys
import urllib.request
import zipfile

IN = 0.0254                       # inch  -> m
OZ = 0.0283495                    # ounce -> kg
OZIN2 = OZ * IN * IN              # oz.in^2 -> kg.m^2

# Reference calibration for rot-damping: the flown reference airframe used SIM_RKT_ROTDAMP 0.0458 at
# S_fin*CLa*fin_arm^2 = 0.01408, so ROTDAMP = 3.254 * S_fin*CLa*fin_arm^2 reproduces that scaling.
ROTDAMP_K = 0.0458 / 0.01408


def load_design(path):
    """Labeled design CSV -> {Field: value_float}."""
    d = {}
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if len(row) < 3 or row[0] == 'Scope':
                continue
            try:
                d[row[1].strip()] = float(row[2])
            except ValueError:
                pass
    return d


def need(d, field):
    if field not in d:
        sys.exit("design CSV missing required field: %r (have: %s)" % (field, ', '.join(d)))
    return d[field]


def extract_ork_fins(path):
    """Fin root/tip/span (m) and fin count from the first trapezoidfinset."""
    with zipfile.ZipFile(path) as z:
        xml = z.read(z.namelist()[0]).decode('utf-8', 'ignore')
    import re
    fin = re.search(r'<trapezoidfinset>(.*?)</trapezoidfinset>', xml, re.S)
    if not fin:
        sys.exit("no <trapezoidfinset> in the .ork -- only trapezoidal fins are supported")
    blk = fin.group(0)

    def f(tag):
        m = re.search(r'<%s>([^<]*)</%s>' % (tag, tag), blk)
        return float(m.group(1)) if m else float('nan')
    return dict(n=int(f('fincount')), root=f('rootchord'), tip=f('tipchord'), span=f('height'))


def extract_ork_motors(path):
    """Read the .ork's motors. Returns (default_designation, {designation: manufacturer}).
    default_designation is the motor of the configuration marked default="true" -- i.e. the one
    OpenRocket has selected -- so the caller can use it automatically (no --motor needed)."""
    with zipfile.ZipFile(path) as z:
        xml = z.read(z.namelist()[0]).decode('utf-8', 'ignore')
    dm = re.search(r'<motorconfiguration\b[^>]*default="true"[^>]*>', xml)
    dcid = None
    if dm:
        cid = re.search(r'configid="([^"]+)"', dm.group(0))
        dcid = cid.group(1) if cid else None
    motors, default_desig = {}, None
    for mo in re.finditer(r'<motor configid="([^"]+)">(.*?)</motor>', xml, re.S):
        cid, blk = mo.group(1), mo.group(2)
        d = re.search(r'<designation>([^<]*)</designation>', blk)
        if not d:
            continue
        desig = d.group(1).strip()
        mf = re.search(r'<manufacturer>([^<]*)</manufacturer>', blk)
        motors.setdefault(desig, mf.group(1).strip() if mf else None)
        if dcid and cid == dcid:
            default_desig = desig
    return default_desig, motors


def _tc_post(endpoint, body):
    req = urllib.request.Request("https://www.thrustcurve.org/api/v1/" + endpoint,
                                 data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(req, timeout=30))


def fetch_motor(manufacturer, designation):
    """Look a motor up on thrustcurve.org -> prop/total mass (kg), impulse (N.s), burn (s), curve."""
    query = {"designation": designation, "maxResults": 5}
    if manufacturer:
        query["manufacturer"] = manufacturer
    try:
        results = _tc_post("search.json", query).get("results", [])
    except Exception as e:                                   # network / API failure
        sys.exit("thrustcurve.org lookup failed (%s) -- use --eng with a downloaded .eng instead" % e)
    if not results:
        sys.exit("no thrustcurve.org match for %s %s -- check the name or pass --eng"
                 % (manufacturer or "", designation))
    r = results[0]
    dl = _tc_post("download.json", {"motorIds": [r["motorId"]], "format": "RASP", "data": "samples"})
    samples = (dl.get("results") or [{}])[0].get("samples", [])
    pts = [(float(p["time"]), float(p["thrust"])) for p in samples if "time" in p and "thrust" in p]
    if len(pts) < 2:
        sys.exit("thrustcurve.org returned no curve for %s -- pass --eng" % designation)
    impulse = r.get("totImpulseNs") or sum(0.5 * (pts[i][1] + pts[i + 1][1]) * (pts[i + 1][0] - pts[i][0])
                                           for i in range(len(pts) - 1))
    return dict(prop=(r.get("propWeightG") or 0) / 1000.0, total=(r.get("totalWeightG") or 0) / 1000.0,
                impulse=impulse, burn=r.get("burnTimeS") or pts[-1][0], pts=pts,
                name="%s %s" % (r.get("manufacturer", ""), r.get("designation", designation)))


def extract_ork_designinfo(path):
    """If the .ork carries a <designinfo> block (SI stats + control-fin position), return the design
    values in SI: {mass, empty, body_d, cg, fin_top, jtilt, jspin, cd, margin}. Else None. This lets
    the .ork be the single source -- no separate design CSV needed."""
    with zipfile.ZipFile(path) as z:
        xml = z.read(z.namelist()[0]).decode('utf-8', 'ignore')
    m = re.search(r'<designinfo>(.*?)</designinfo>', xml, re.S)
    if not m:
        return None
    blk = m.group(1)
    stats = {}
    for s in re.finditer(r'<stat\s+field="([^"]+)"\s+value="([^"]+)"', blk):
        try:
            stats[s.group(1)] = float(s.group(2))
        except ValueError:
            pass
    ft = re.search(r'<nosetoroottop[^>]*>([-0-9.]+)</nosetoroottop>', blk)
    return dict(mass=stats.get('Mass (Loaded)'), empty=stats.get('Mass (Empty)'),
                body_d=stats.get('Max Diameter'), cg=stats.get('CG (Loaded)'),
                fin_top=float(ft.group(1)) if ft else None,
                jtilt=stats.get('Pitch Inertia (Loaded)'), jspin=stats.get('Roll Inertia (Loaded)'),
                cd=stats.get('Drag Coeff. (Ma 0.3)'), margin=stats.get('Stability (on pad)'))


def csv_designvalues(path):
    """Design values in SI from the labeled (imperial) design CSV. Missing fields come back None
    (no exit) so the caller can merge sources and report everything that's absent at once."""
    d = load_design(path)

    def g(field, conv=1.0):
        v = d.get(field)
        return v * conv if v is not None else None
    return dict(mass=g('Mass (Loaded)', OZ), empty=g('Mass (Empty)', OZ),
                body_d=g('Max Diameter', IN), cg=g('CG (Loaded)', IN),
                fin_top=g('CONTROL: Nose to top of fin root', IN),
                jtilt=g('Pitch Inertia (Loaded)', OZIN2), jspin=g('Roll Inertia (Loaded)', OZIN2),
                cd=g('Drag Coeff. (Ma 0.3)'), margin=g('Stability (on pad)'))


def load_eng(path):
    """RASP .eng -> prop mass, total mass (kg), total impulse (N.s), burn time (s)."""
    prop = total = None
    pts = []
    header_seen = False
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith(';'):
                continue
            if not header_seen:
                header_seen = True
                tok = s.split()
                # designation dia len delays propMass totalMass manufacturer
                prop, total = float(tok[4]), float(tok[5])
                continue
            tok = s.split()
            if len(tok) >= 2:
                try:
                    pts.append((float(tok[0]), float(tok[1])))
                except ValueError:
                    pass
    if prop is None or len(pts) < 2:
        sys.exit("could not parse .eng %s" % path)
    impulse = sum(0.5 * (pts[i][1] + pts[i + 1][1]) * (pts[i + 1][0] - pts[i][0])
                  for i in range(len(pts) - 1))
    return dict(prop=prop, total=total, impulse=impulse, burn=pts[-1][0], pts=pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('base', nargs='?', help='shared name/stem: uses <base>.ork and <base>-design.csv '
                    '(or <base>.csv), and names the output <base>. e.g. `ork_to_sim.py samples/test1`. '
                    'Override any of the three with --ork/--design/--name.')
    ap.add_argument('--ork', help='.ork path (default <base>.ork) -- fin planform + motor config')
    ap.add_argument('--design', help='labeled design CSV path (default <base>-design.csv or <base>.csv)')
    ap.add_argument('--motor', help="motor designation to look up on thrustcurve.org (e.g. H128W); "
                    "default is the .ork's selected (default) motor config")
    ap.add_argument('--eng', help='use a local RASP .eng instead of the thrustcurve.org lookup (offline)')
    ap.add_argument('--name', help='output rocket name (default: basename of <base>) -> <name>.parm')
    ap.add_argument('--tab-chord', type=float, required=True, dest='tab_chord',
                    help='control-tab chord/depth, m (not in OpenRocket -- measure it)')
    ap.add_argument('--tab-span', type=float, required=True, dest='tab_span',
                    help='control-tab spanwise length, m')
    ap.add_argument('--tab-max', type=float, default=20.0, dest='tab_max',
                    help='control-tab max deflection, deg (default 20)')
    ap.add_argument('--vmax', type=float, default=None,
                    help='max airspeed m/s (OpenRocket flight sim); default estimates impulse/mass')
    ap.add_argument('--outdir', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     'sitl_tests'),
                    help='where to write <name>.parm (default: Tools/ArduRocket/sitl_tests, where '
                         'sitl.sh finds rockets by name)')
    a = ap.parse_args()

    # Resolve files from the shared <base> stem, with per-file overrides. The .ork is required; the
    # design CSV is only needed when the .ork has no <designinfo> block.
    base = a.base
    ork_path = a.ork or ((base + ".ork") if base else None)
    design_path = a.design or (next((c for c in (base + "-design.csv", base + ".csv")
                                     if os.path.exists(c)), None) if base else None)
    name = a.name or (os.path.basename(base) if base else None)
    missing = [n for n, v in (("--ork (or <base>.ork)", ork_path),
                              ("--name (or <base>)", name)) if not v]
    if missing:
        sys.exit("missing: " + ", ".join(missing) + " -- give a <base> stem, or the explicit flags")
    if not os.path.exists(ork_path):
        sys.exit("ork not found: " + ork_path)

    fins = extract_ork_fins(ork_path)
    # Motor: a local .eng (offline), or the .ork's selected motor looked up on thrustcurve.org.
    if a.eng:
        eng = load_eng(a.eng)
        motor_src = os.path.basename(a.eng)
    else:
        default_desig, motors = extract_ork_motors(ork_path)
        desig = a.motor or default_desig          # --motor overrides the .ork's selected motor
        if not desig:
            avail = ", ".join(sorted(motors)) or "none"
            sys.exit("no selected motor in the .ork (mark one default in OpenRocket) -- pass "
                     "--motor DESIGNATION (available: %s) or --eng FILE" % avail)
        mfr = motors.get(desig)                   # manufacturer from the .ork (None if --motor is new)
        eng = fetch_motor(mfr, desig)
        motor_src = "thrustcurve.org: " + eng['name']
        if not a.motor:
            print(f"  motor: {desig} ({mfr}) -- the .ork's selected (default) config")
    if fins['n'] != 4:
        print("# WARNING: %d fins in the .ork; the mixer assumes 4." % fins['n'], file=sys.stderr)

    # --- design values (SI): use the .ork's <designinfo> and/or a labeled design CSV, MERGED (the
    # .ork wins where both carry a field; the CSV fills any gaps). Bail listing everything missing. ---
    dv, sources = {}, []
    di = extract_ork_designinfo(ork_path)
    if di is not None:
        dv = {k: v for k, v in di.items() if v is not None}
        sources.append(os.path.basename(ork_path) + " <designinfo>")
    if design_path and os.path.exists(design_path):
        for k, v in csv_designvalues(design_path).items():
            if v is not None and dv.get(k) is None:
                dv[k] = v
        sources.append(os.path.basename(design_path))
    if not sources:
        sys.exit("no design data: the .ork has no <designinfo> and no design CSV was found "
                 "(<base>-design.csv) -- add a <designinfo> block in the .ork, or pass --design FILE")
    LABELS = {'mass': 'Mass (Loaded)', 'body_d': 'Max Diameter', 'cg': 'CG (Loaded)',
              'fin_top': 'nose-to-top-of-fin-root', 'jtilt': 'Pitch Inertia (Loaded)',
              'jspin': 'Roll Inertia (Loaded)', 'cd': 'Drag Coeff.', 'margin': 'Stability (on pad)'}
    missing = [LABELS[k] for k in LABELS if dv.get(k) is None]
    if missing:
        sys.exit("missing design values (checked %s): %s -- add them to the .ork <designinfo> or the "
                 "design CSV" % (" + ".join(sources), ", ".join(missing)))
    design_src = " + ".join(sources)

    mass, body_d, cg = dv['mass'], dv['body_d'], dv['cg']
    rb = body_d / 2.0
    fin_top, jtilt, jspin, cd, margin = dv['fin_top'], dv['jtilt'], dv['jspin'], dv['cd'], dv['margin']
    fin_arm = fin_top - cg
    a_ref = math.pi / 4.0 * body_d * body_d
    drag_area = cd * a_ref
    dry = mass - eng['prop']
    vmax = a.vmax if a.vmax is not None else eng['impulse'] / mass   # rough burnout-speed estimate

    # Sanity: the design's implied motor mass (loaded - empty) should match the chosen motor's total
    # mass. A mismatch means the design was captured with a DIFFERENT motor than this one, so its
    # loaded mass/CG/inertia won't be consistent with the motor being flown.
    empty = dv.get('empty')
    if empty is not None and eng.get('total', 0) > 0:
        implied = mass - empty                       # both kg
        if abs(implied - eng['total']) > 0.10 * eng['total']:
            print("WARNING: design shows a %.0f g motor (loaded-empty) but %s is %.0f g -- the design "
                  "and the motor may not match; re-capture the design with this motor selected."
                  % (implied * 1000, eng.get('name', motor_src).strip(), eng['total'] * 1000),
                  file=sys.stderr)

    # --- rate damping, scaled from the reference (S_fin*CLa*fin_arm^2) ---
    Cr, Ct, sspan = fins['root'], fins['tip'], fins['span']
    S_fin = 0.5 * (Cr + Ct) * sspan
    AR = 2.0 * sspan * sspan / S_fin
    CLa = 2.0 * math.pi * AR / (2.0 + math.sqrt(AR * AR + 4.0))
    rotdamp = ROTDAMP_K * S_fin * CLa * fin_arm * fin_arm

    # Embed the motor thrust curve so the file is self-contained (one file per rocket+engine). The
    # sim reads these `# MOTOR:` lines when SIM_RKT_ENG points at this .parm; sitl.sh does that.
    motor_block = "\n".join(f"# MOTOR: {t:.4f} {thr:.3f}" for t, thr in eng['pts'])

    out = f"""# {name}: complete, self-contained ArduRocket rocket+motor definition, generated by
# ork_to_sim.py from {os.path.basename(ork_path)} (design: {design_src}) + motor [{motor_src}].
# One file = one airframe + one motor (loaded mass/CG/inertia depend on the motor). Load it:
#   Tools/ArduRocket/sitl_tests/sitl.sh {name} [tiltN]
#
# ---- motor thrust curve (embedded; {len(eng['pts'])} points from {motor_src}) ----
{motor_block}
#
# ---- SIM airframe (what the simulator flies) ----
SIM_RKT_DRYMASS  {dry:.4f}
SIM_RKT_PRPMASS  {eng['prop']:.5f}
SIM_RKT_IMPULSE  {eng['impulse']:.1f}
SIM_RKT_BRNTIME  {eng['burn']:.3f}
SIM_RKT_JTILT0   {jtilt:.5f}
SIM_RKT_JTILT1   {jtilt:.5f}
SIM_RKT_JSPIN0   {jspin:.6f}
SIM_RKT_JSPIN1   {jspin:.6f}
SIM_RKT_DRAGA    {drag_area:.6f}
SIM_RKT_FIN_ROOT {Cr:.5f}
SIM_RKT_FIN_TIP  {Ct:.5f}
SIM_RKT_FIN_SPAN {sspan:.5f}
SIM_RKT_BODY_R   {rb:.5f}
SIM_RKT_FIN_ARM  {fin_arm:.4f}
SIM_RKT_MARGIN   {margin:.2f}
SIM_RKT_ROTDAMP  {rotdamp:.5f}
SIM_RKT_TAB_W    {a.tab_chord * 1000:.0f}
SIM_RKT_TAB_H    {a.tab_span * 1000:.0f}
SIM_RKT_TAB_RT   0
SIM_RKT_TAB_AX   0
SIM_RKT_TAB_MAX  {a.tab_max:.1f}
#
# ---- flight-controller airframe (on-vehicle gain computation, PLAN 9e) ----
RKT_MASS       {mass:.4f}
RKT_JTILT      {jtilt:.5f}
RKT_JSPIN      {jspin:.6f}
RKT_BODY_D     {body_d:.5f}
RKT_FIN_ROOT   {Cr:.4f}
RKT_FIN_TIP    {Ct:.4f}
RKT_FIN_SPAN   {sspan:.4f}
RKT_TAB_CHORD  {a.tab_chord:.4f}
RKT_TAB_SPAN   {a.tab_span:.4f}
RKT_TAB_MAX    {a.tab_max:.1f}
RKT_NOSE_CG    {cg:.4f}
RKT_NOSE_FIN   {fin_top:.4f}
RKT_VMAX       {vmax:.1f}
"""
    outp = os.path.join(a.outdir, f"{name}.parm")
    open(outp, 'w').write(out)
    print(f"wrote {outp}")
    print(f"  mass {mass:.3f} kg (dry {dry:.3f} + prop {eng['prop']:.3f}), motor {eng['impulse']:.0f} N.s"
          f" / {eng['burn']:.2f} s, fin_arm {fin_arm:.3f} m, vmax {vmax:.0f} m/s")


if __name__ == '__main__':
    main()
