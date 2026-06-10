#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Test 3: augmented-state ablation comparison.
# One-shot driver. Edit the variables in the EDIT-ME section below, then run.
#
# What it does:
#   1. Extracts iter_count + pose + steer + speed from each bag.
#   2. Classifies each run as "working" (>= MIN_LAPS race laps) or "crashed".
#   3. Plots:
#        - trajectory overlay (crashed runs end abruptly)
#        - boxplot of per-solve iteration counts across all runs
#        - per-metric track-aligned comparisons of the working runs
#          (steering jitter, commanded speed, iteration count)
#
# Outputs land in: tools/output/test3_ablation_<timestamp>/
# =============================================================================

# ============================ EDIT-ME ========================================
# Each entry: LABEL=DIR (script picks the first .mcap inside DIR).
# The first WORKING entry becomes the reference path for averaging.
RUNS=(
  "Baseline=tools/input/MPC10LapBaseline"
  "SteerOff=tools/input/SteerOff"
  "AccelOff=tools/input/AccelOff"
  "AccelOffSteerOff=tools/input/AccelOffSteerOff"
)
MAX_LAP="11"        # keep race laps 2..MAX_LAP for the average
MIN_LAPS="5"        # < this many race laps -> classify as crashed
CELL_SIZE="0.1"     # min spacing (m) between reference points along the path
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HELPER_DIR="${REPO_ROOT}/tools/mpc_replay/helper"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --max-lap N           Override MAX_LAP
  --min-laps N          Override MIN_LAPS
  --cell-size METERS    Override CELL_SIZE
  -h, --help            Show this help

To change which runs are compared, edit the RUNS array at the top of this file.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --max-lap) MAX_LAP="$2"; shift 2 ;;
    --min-laps) MIN_LAPS="$2"; shift 2 ;;
    --cell-size) CELL_SIZE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

OUT_DIR="${DEFAULT_OUTPUT_PARENT}/test3_ablation_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT_DIR}"
echo "Output dir: ${OUT_DIR}"

echo "[1/3] Sourcing ROS"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u; source "${REPO_ROOT}/install/setup.bash"; set -u
fi

echo "[2/3] Extracting per-run CSVs"
declare -a RUN_ARGS=()
for entry in "${RUNS[@]}"; do
  label="${entry%%=*}"
  dir="${entry#*=}"
  # Resolve dir relative to REPO_ROOT if not absolute
  case "${dir}" in
    /*) abs_dir="${dir}" ;;
    *)  abs_dir="${REPO_ROOT}/${dir}" ;;
  esac
  bag=$(ls -1 "${abs_dir}"/*.mcap 2>/dev/null | head -1 || true)
  if [[ -z "${bag}" || ! -f "${bag}" ]]; then
    echo "  WARNING: no .mcap found in ${abs_dir} for ${label} -- skipping"
    continue
  fi
  csv="${OUT_DIR}/ablation_${label}.csv"
  echo "  ${label}: ${bag}"
  python3 "${HELPER_DIR}/export_ablation_csv.py" --bag "${bag}" --out "${csv}"
  RUN_ARGS+=( --run "${label}=${csv}" )
done

if [[ ${#RUN_ARGS[@]} -eq 0 ]]; then
  echo "ERROR: no valid bags found" >&2
  exit 3
fi

echo "[3/3] Plotting"
python3 "${HELPER_DIR}/plot_test3_ablation.py" \
  "${RUN_ARGS[@]}" \
  --out-dir "${OUT_DIR}/plots" \
  --max-lap "${MAX_LAP}" \
  --min-laps "${MIN_LAPS}" \
  --cell-size "${CELL_SIZE}"

echo
echo "=== Done ==="
echo "  per-run CSVs:  ${OUT_DIR}/ablation_*.csv"
echo "  plots:         ${OUT_DIR}/plots/"
