#!/usr/bin/env python3
"""Turn an OpenRocket *design* export (+ the .ork for fin planform) into the ArduRocket QGC
airframe params (RKT_*) for on-vehicle gain computation -- PLAN §9e, the real-rocket path.

    design_to_qgc.py test1-design.csv --ork test1.ork

The design CSV is the LABELED static-measurements export (Scope,Field,Value,Unit) -- robust, no
column-index guessing. It carries mass, length, diameter, CG, and the real inertias. The .ork adds
the fin root/tip/span (the design CSV has no fin planform). The control TAB is not in either file
(OpenRocket has no deflecting tab) -- measure it and set the three RKT_TAB_* by hand.

This prints the RKT_* param lines to type/load into QGC. Real inertia is emitted as RKT_JTILT/JSPIN
so the firmware uses it directly (exact) rather than the slender-rod estimate.
"""
import argparse
import csv
import os
import sys

IN = 0.0254                       # inch  -> m
OZ = 0.0283495                    # ounce -> kg
OZIN2 = OZ * IN * IN              # oz.in^2 -> kg.m^2


def load_design(path):
    """Parse the labeled design CSV into {Field: (value_float, unit)}."""
    d = {}
    with open(path, encoding='utf-8-sig') as f:
        for row in csv.reader(f):
            if len(row) < 3 or row[0] == 'Scope':
                continue
            field = row[1].strip()
            try:
                d[field] = (float(row[2]), row[3].strip() if len(row) > 3 else '')
            except ValueError:
                pass
    return d


def need(d, field):
    if field not in d:
        sys.exit("design CSV missing required field: %r (have: %s)" % (field, ', '.join(d)))
    return d[field][0]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('design_csv', help='OpenRocket design export (labeled Scope,Field,Value,Unit)')
    ap.add_argument('--ork', help='the .ork, for fin root/tip/span (else set RKT_FIN_* by hand)')
    a = ap.parse_args()

    d = load_design(a.design_csv)

    mass = need(d, 'Mass (Loaded)') * OZ                     # oz -> kg
    body_d = need(d, 'Max Diameter') * IN                    # in -> m
    cg = need(d, 'CG (Loaded)') * IN                         # nose -> CG, in -> m
    fin_top = need(d, 'CONTROL: Nose to top of fin root') * IN
    jtilt = need(d, 'Pitch Inertia (Loaded)') * OZIN2        # oz.in^2 -> kg.m^2
    jspin = need(d, 'Roll Inertia (Loaded)') * OZIN2
    fin_arm = fin_top - cg

    fin_root = fin_tip = fin_span = None
    if a.ork:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from ork_to_rocket import extract_ork
        o = extract_ork(a.ork)
        fin_root, fin_tip, fin_span = o['fin_root'], o['fin_tip'], o['fin_semispan']
        if o['n_fins'] != 4:
            print("# WARNING: %d fins in the .ork; the mixer assumes 4." % o['n_fins'])

    print("# ArduRocket airframe params (PLAN 9e) from %s%s" %
          (os.path.basename(a.design_csv), " + " + os.path.basename(a.ork) if a.ork else ""))
    print("# Real inertia emitted (RKT_JTILT/JSPIN) -> used directly, no rod estimate.")
    print("RKT_MASS       %.4f" % mass)
    print("RKT_BODY_D     %.5f" % body_d)
    # RKT_LENGTH is not emitted: it only feeds the rod J_tilt fallback, and we give real RKT_JTILT.
    print("RKT_JTILT      %.5f" % jtilt)
    print("RKT_JSPIN      %.6f" % jspin)
    print("RKT_NOSE_CG    %.4f" % cg)
    print("RKT_NOSE_FIN   %.4f     # nose -> top(front) of fin root  (fin_arm = %.4f m)" %
          (fin_top, fin_arm))
    if fin_root is not None:
        print("RKT_FIN_ROOT   %.4f" % fin_root)
        print("RKT_FIN_TIP    %.4f" % fin_tip)
        print("RKT_FIN_SPAN   %.4f" % fin_span)
    else:
        print("# RKT_FIN_ROOT/TIP/SPAN: pass --ork, or measure and set by hand")
    print("# --- SET BY HAND (measure the real control tab; not in OpenRocket) ---")
    print("# RKT_TAB_CHORD  <m>   RKT_TAB_SPAN <m>   RKT_TAB_MAX <deg>")


if __name__ == '__main__':
    main()
