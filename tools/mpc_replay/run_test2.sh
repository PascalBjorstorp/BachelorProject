#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Test 2: low-velocity-weight ("stop around corner")
# One-shot driver. Edit the variables in the EDIT-ME section below, then run.
#
# What it does:
#   1. Builds the cost-replay binary (skippable).
#   2. Replays the bag with baseline weights and with the chosen low weight.
#   3. Plots the spatial maps (1, 2, 4) for the selected lap.
#   4. Optionally runs the cost-landscape sweep (Map 3) on an auto-picked
#      snapshot from the low-vel run.
#
# Outputs land in: tools/output/test2_low_vel_<timestamp>/
# =============================================================================

# ============================ EDIT-ME ========================================
LOW_VEL_WEIGHT="50.0"   # weight_velocity for the low-weight run (baseline=200)
LAP="average"           # integer lap number (1=warmup; 2..11=race laps),
                        # or 'average' to spatially average across kept laps
MAX_LAP="11"            # discard laps strictly after this
CELL_SIZE="0.1"        # min spacing (m) between reference points in 'average' mode
                        # (smaller = finer continuous line; line stays connected)
BAG=""                  # bag .mcap; empty = first .mcap in tools/input/MPC_10Laps
RUN_LANDSCAPE="0"       # 1 = also run Map 3 (cost-landscape sweep)
LANDSCAPE_W_MIN="0.001" # sweep range for Map 3
LANDSCAPE_W_MAX="500.0"
LANDSCAPE_N_STEPS="40"
SKIP_COMPILE="0"        # 1 = skip rebuilding cost-replay binaries
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HELPER_DIR="${REPO_ROOT}/tools/mpc_replay/helper"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

CLI overrides for the variables at the top of the script:
  --low-vel-weight VALUE   Override LOW_VEL_WEIGHT
  --lap LAP                Override LAP (integer or 'average')
  --max-lap N              Override MAX_LAP
  --cell-size METERS       Override CELL_SIZE (for 'average' mode)
  --bag PATH               Override BAG
  --landscape              Set RUN_LANDSCAPE=1
  --skip-compile           Set SKIP_COMPILE=1
  -h, --help               Show this help

You can also just edit the EDIT-ME block at the top of this file and re-run.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --low-vel-weight) LOW_VEL_WEIGHT="$2"; shift 2 ;;
    --lap) LAP="$2"; shift 2 ;;
    --max-lap) MAX_LAP="$2"; shift 2 ;;
    --cell-size) CELL_SIZE="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --landscape) RUN_LANDSCAPE="1"; shift ;;
    --skip-compile) SKIP_COMPILE="1"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

# ---- 1. Run the replay harness ----------------------------------------------
HARNESS_ARGS=( --low-vel-weight "${LOW_VEL_WEIGHT}" )
[[ -n "${BAG}" ]] && HARNESS_ARGS+=( --bag "${BAG}" )
[[ "${SKIP_COMPILE}" == "1" ]] && HARNESS_ARGS+=( --skip-compile )

echo "=== Running replay harness (LOW_VEL_WEIGHT=${LOW_VEL_WEIGHT}) ==="
HARNESS_OUT=$("${SCRIPT_DIR}/run_test2_low_velocity_weight.sh" "${HARNESS_ARGS[@]}")
echo "${HARNESS_OUT}"

OUT_DIR=$(echo "${HARNESS_OUT}" | awk -F'out_dir:' '/out_dir:/ {print $2}' | tr -d ' ')
if [[ -z "${OUT_DIR}" || ! -d "${OUT_DIR}" ]]; then
  echo "ERROR: could not determine out_dir from harness output" >&2
  exit 3
fi
echo
echo "Harness output dir: ${OUT_DIR}"

# ---- 2. Plot spatial maps ---------------------------------------------------
echo
echo "=== Plotting spatial maps (lap ${LAP}, max-lap ${MAX_LAP}) ==="
python3 "${HELPER_DIR}/plot_test2_maps.py" \
  --out-dir "${OUT_DIR}" \
  --lap "${LAP}" \
  --max-lap "${MAX_LAP}" \
  --cell-size "${CELL_SIZE}"

# ---- 3. Optional landscape sweep --------------------------------------------
if [[ "${RUN_LANDSCAPE}" == "1" ]]; then
  echo
  echo "=== Running cost-landscape sweep (Map 3) ==="
  LANDSCAPE_ARGS=(
    --prior-out-dir "${OUT_DIR}"
    --w-min "${LANDSCAPE_W_MIN}"
    --w-max "${LANDSCAPE_W_MAX}"
    --n-steps "${LANDSCAPE_N_STEPS}"
  )
  [[ "${SKIP_COMPILE}" == "1" ]] && LANDSCAPE_ARGS+=( --skip-compile )
  "${SCRIPT_DIR}/run_test2_landscape.sh" "${LANDSCAPE_ARGS[@]}"
fi

echo
echo "=== Done ==="
echo "Plots:  ${OUT_DIR}/plots/"
[[ "${RUN_LANDSCAPE}" == "1" ]] && echo "Landscape plots are in: ${OUT_DIR}/landscape_idx*/plots/"
