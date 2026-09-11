#!/usr/bin/env bash
# Launch ArduRocket SITL flying a REAL rocket you produced with ork_to_sim.py. There is no built-in
# reference rocket -- you always name one. This loads that rocket's SIM_RKT_*/RKT_* params, wires its
# motor thrust curve (from the file's `# SIM_RKT_ENG:` line), and bakes the DCM+no-GPS estimator/tune
# (rocket.parm) so you never hit the wrong-estimator refusal.
#
# Usage (from anywhere in the repo):
#   Tools/ArduRocket/sitl_tests/sitl.sh test1                 # test1, vertical rail
#   Tools/ArduRocket/sitl_tests/sitl.sh test1 tilt3           # test1, 3 deg rail
#   Tools/ArduRocket/sitl_tests/sitl.sh test1 tilt20 --speedup 10   # extra sim args pass through
#   Tools/ArduRocket/sitl_tests/sitl.sh path/to/myrocket.parm tilt5
# <rocket> is a name (-> sitl_tests/<name>.parm) or a path to a .parm from ork_to_sim.py.
# Make a rocket:  Tools/ArduRocket/ork_to_sim.py DESIGN.csv --ork X.ork --eng M.eng --tab-* --name NAME
set -euo pipefail

root="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$root"
here="Tools/ArduRocket/sitl_tests"
bin="build/sitl/bin/rocket"
defs="Tools/autotest/default_params/rocket.parm"
[[ -x "$bin"  ]] || { echo "no $bin -- build it: ./waf build --target bin/rocket" >&2; exit 1; }
[[ -f "$defs" ]] || { echo "missing $defs" >&2; exit 1; }

if [[ $# -lt 1 ]]; then
  echo "usage: sitl.sh <rocket> [tiltN] [extra sim args]" >&2
  echo "available rockets:" >&2
  # list rocket files, hiding the non-rocket overlays (baseline = no-control diagnostic,
  # scheduled = the RKT_QSCHED=1 gain-schedule fallback tune).
  ls "$here"/*.parm 2>/dev/null | sed "s#$here/##;s#\.parm##" | grep -vE '^(baseline|scheduled)$' | sed 's/^/  /' >&2
  exit 1
fi

# Resolve the rocket definition file (a path, or a name in sitl_tests/).
rocket="$1"; shift
if   [[ -f "$rocket" ]];               then rfile="$rocket"
elif [[ -f "$here/$rocket.parm" ]];    then rfile="$here/$rocket.parm"
else echo "no rocket '$rocket' (looked for $here/$rocket.parm) -- make one with ork_to_sim.py" >&2; exit 1; fi

# Optional rail model as the next arg (tiltN / rocket-...); everything after passes to the sim.
model="rocket"
if [[ "${1:-}" == tilt* || "${1:-}" == rocket* || "${1:-}" == az* || "${1:-}" == roll* ]]; then
  model="$1"; [[ "$model" == rocket* ]] || model="rocket-$model"; shift
fi

# The motor thrust curve is embedded in the rocket file (`# MOTOR:` lines). The sim reads it when
# SIM_RKT_ENG points at that file, so just point it at the rocket file itself.
if grep -q '^# MOTOR:' "$rfile"; then
  export SIM_RKT_ENG="$rfile"
  echo "motor: embedded ($(grep -c '^# MOTOR:' "$rfile") points) in $rfile"
else
  echo "WARNING: no embedded motor curve in $rfile -- sim will refuse (NO MOTOR)" >&2
fi

echo "launching $model  rocket=$rfile  (DCM + no-GPS, tune from $defs)"
exec "$bin" --model "$model" -w --defaults "$defs,$rfile" "$@"
