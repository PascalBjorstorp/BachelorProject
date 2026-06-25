#!/usr/bin/env bash
# Closed-loop plant fit: search plant parameters so the REAL controller (with the
# sanctioned `*1.4` max-accel removed) drives the simulated car around the lap
# matching the recorded ICP position and speed, without crashing.
#
# Prerequisite: run_prepare.sh + run_fit.sh have produced output/observed_run.csv
# and output/lateral_fit.json (the open-loop physical seed).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MPC_DIR="$(cd "$ROOT/../../MPC" && pwd)"
OUT="${1:-$ROOT/output}"
RACELINE="${2:-$ROOT/input/raceline.csv}"

export PYTHONPATH="$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

# Build the simulator. NOTE: this requires the one sanctioned controller change —
# the *1.4 multiplier removed from VP_MAX_ACCEL_MPS2 in include/mpc_types.h so the
# controller matches the bag run.
BIN="$OUT/test_sim_drive"
gcc -D_GNU_SOURCE -O3 -std=c99 -ffast-math -Wno-unused-variable -Wno-unused-but-set-variable \
  -I"$MPC_DIR/include" "$MPC_DIR/test/test_sim_drive.c" "$MPC_DIR/src/mpc.c" \
  "$MPC_DIR/src/riccati_solver.c" "$MPC_DIR/src/vehicle_model.c" "$MPC_DIR/src/util_math.c" \
  -o "$BIN" -lm

python3 "$ROOT/scripts/fit_closed_loop.py" \
  --observed "$OUT/observed_run.csv" --base-params "$OUT/lateral_fit.json" \
  --raceline "$RACELINE" --binary "$BIN" --output "$OUT/closed_loop_fit.json" \
  --duration 50 --target-laps 4 --n-bins 60 --w-ey 4.0 \
  --maxiter "${MAXITER:-100}" --popsize "${POPSIZE:-24}" --workers -1

python3 "$ROOT/scripts/export_sim_env.py" "$OUT/closed_loop_fit.json" --output "$OUT/fitted_sim_env.sh"
echo "Closed-loop fit complete. Source $OUT/fitted_sim_env.sh before running test_sim_drive."
