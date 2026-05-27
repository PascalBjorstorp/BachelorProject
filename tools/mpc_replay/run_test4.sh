#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Test 4: warm-start vs cold-start convergence comparison.
# One-shot driver. Edit the variables in the EDIT-ME section below, then run.
#
# What it does:
#   1. Extracts /mpc/timing/iteration_count joined with /ekf_pose from each
#      bag (iter count + nearest pose by bag timestamp).
#   2. Renders a boxplot of per-solve iteration counts.
#   3. Renders spatial heatmaps showing per-cell mean iterations for each
#      run, and the cold-minus-warm difference on the track.
#
# Outputs land in: tools/output/test4_warm_vs_cold_<timestamp>/
# =============================================================================

# ============================ EDIT-ME ========================================
BASELINE_BAG=""    # .mcap with warm-start (default MPC config); empty = first
                   # .mcap in tools/input/MPC10LapBaseline
COLD_BAG=""        # .mcap with cold-start (controller resets every solve);
                   # empty = first .mcap in tools/input/ColdStartOnly
CELL_SIZE="0.05"    # spatial bin size (meters) for the heatmaps
MIN_CELL_COUNT="3" # drop heatmap cells with fewer samples than this per run
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HELPER_DIR="${REPO_ROOT}/tools/mpc_replay/helper"
DEFAULT_BASELINE_DIR="${REPO_ROOT}/tools/input/MPC10LapBaseline"
DEFAULT_COLD_DIR="${REPO_ROOT}/tools/input/ColdStartOnly"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

CLI overrides for the variables at the top of the script:
  --baseline-bag PATH       Override BASELINE_BAG
  --cold-bag PATH           Override COLD_BAG
  --cell-size METERS        Override CELL_SIZE
  --min-cell-count N        Override MIN_CELL_COUNT
  -h, --help                Show this help

You can also edit the EDIT-ME block at the top of this file and re-run.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline-bag) BASELINE_BAG="$2"; shift 2 ;;
    --cold-bag) COLD_BAG="$2"; shift 2 ;;
    --cell-size) CELL_SIZE="$2"; shift 2 ;;
    --min-cell-count) MIN_CELL_COUNT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BASELINE_BAG}" ]]; then
  BASELINE_BAG=$(ls -1 "${DEFAULT_BASELINE_DIR}"/*.mcap 2>/dev/null | head -1 || true)
fi
if [[ -z "${COLD_BAG}" ]]; then
  COLD_BAG=$(ls -1 "${DEFAULT_COLD_DIR}"/*.mcap 2>/dev/null | head -1 || true)
fi
for label in "baseline:${BASELINE_BAG}" "cold:${COLD_BAG}"; do
  name="${label%%:*}"
  path="${label#*:}"
  if [[ -z "${path}" || ! -f "${path}" ]]; then
    echo "ERROR: ${name} bag not found (path='${path}')" >&2
    exit 2
  fi
done

OUT_DIR="${DEFAULT_OUTPUT_PARENT}/test4_warm_vs_cold_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT_DIR}"
echo "Output dir: ${OUT_DIR}"

echo "[1/3] Sourcing ROS"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u; source "${REPO_ROOT}/install/setup.bash"; set -u
fi

BASELINE_CSV="${OUT_DIR}/iters_baseline_warm.csv"
COLD_CSV="${OUT_DIR}/iters_cold_start.csv"

echo "[2/3] Extracting iteration counts"
echo "  baseline (warm): ${BASELINE_BAG}"
python3 "${HELPER_DIR}/export_iteration_count_csv.py" \
  --bag "${BASELINE_BAG}" --out "${BASELINE_CSV}"
echo "  cold-start:      ${COLD_BAG}"
python3 "${HELPER_DIR}/export_iteration_count_csv.py" \
  --bag "${COLD_BAG}" --out "${COLD_CSV}"

echo "[3/4] Plotting boxplot"
python3 "${HELPER_DIR}/plot_test4_iterations.py" \
  --baseline-csv "${BASELINE_CSV}" \
  --cold-csv "${COLD_CSV}" \
  --out-dir "${OUT_DIR}/plots"

echo "[4/4] Plotting spatial mismatch heatmaps"
python3 "${HELPER_DIR}/plot_test4_iter_diff_heatmap.py" \
  --baseline-csv "${BASELINE_CSV}" \
  --cold-csv "${COLD_CSV}" \
  --out-dir "${OUT_DIR}/plots" \
  --cell-size "${CELL_SIZE}" \
  --min-cell-count "${MIN_CELL_COUNT}"

echo
echo "=== Done ==="
echo "  iteration CSVs: ${BASELINE_CSV}"
echo "                  ${COLD_CSV}"
echo "  plots:          ${OUT_DIR}/plots/"
