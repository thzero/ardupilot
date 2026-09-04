#!/usr/bin/env bash
# Launch ArduRocket SITL with the CORRECT estimator + tune baked in, so you never hit the
# "wrong estimator (AHRS_EKF_TYPE=3 / GPS1_TYPE=1)" refusal again.
#
# The trap: `build/sitl/bin/rocket --model X -w` wipes params to FIRMWARE defaults, which are
# EKF3 + GPS. The rocket needs DCM + no-GPS + its servo/mixer/gain tune -- all of which live in
# Tools/autotest/default_params/rocket.parm. The raw binary does NOT auto-load that (only
# sim_vehicle.py does, by model-string match), so it must be passed with --defaults. This script
# always does that.
#
# Usage (from anywhere in the repo):
#   Tools/ArduRocket/sitl_tests/sitl.sh                 # vertical rail  (rocket-tilt0)
#   Tools/ArduRocket/sitl_tests/sitl.sh tilt20          # 20 deg rail
#   Tools/ArduRocket/sitl_tests/sitl.sh rocket-tilt20-unstable
#   Tools/ArduRocket/sitl_tests/sitl.sh tilt20 --wind-direction 90   # extra sim args pass through
set -euo pipefail

root="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$root"

bin="build/sitl/bin/rocket"
defs="Tools/autotest/default_params/rocket.parm"
[[ -x "$bin"  ]] || { echo "no $bin -- build it: ./waf build --target bin/rocket" >&2; exit 1; }
[[ -f "$defs" ]] || { echo "missing $defs" >&2; exit 1; }

# First arg = model. Accept a bare suffix ("tilt20") or the full name ("rocket-tilt20").
model="${1:-rocket-tilt0}"
[[ "$model" == rocket* ]] || model="rocket-$model"
shift || true   # remaining args pass straight through to the sim

echo "launching $model  (DCM + no-GPS, tune from $defs)"
exec "$bin" --model "$model" -w --defaults "$defs" "$@"
