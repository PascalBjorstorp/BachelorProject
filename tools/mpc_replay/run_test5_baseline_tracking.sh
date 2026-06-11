#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Test 5: baseline tracking error maps.
# One-shot driver for the MPC 10-lap baseline bag.
#
# What it does:
#   1. Extracts /mpc_state into state_replay.csv.
#   2. Optionally stores bag info if `ros2` is available.
#   3. Renders track-aligned SVG maps for |e_y|, |e_psi|, and e_vx.
#
# Outputs land in: tools/output/test5_baseline_tracking_<timestamp>/
# =============================================================================

# ============================ EDIT-ME ========================================
# Empty = first .mcap inside tools/input/MPC10LapBaseline.
BASELINE_BAG=""
CELL_SIZE="0.1"     # min spacing (m) between reference points along the path
MAX_LAP="11"        # keep race laps 2..MAX_LAP for the average
HORIZON="20"        # fixed-width replay horizon in exported CSV
STEERING_SOURCE="auto"
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HELPER_DIR="${REPO_ROOT}/tools/mpc_replay/helper"
DEFAULT_BASELINE_DIR="${REPO_ROOT}/tools/input/MPC10LapBaseline"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

CLI overrides for the variables at the top of the script:
  --baseline-bag PATH    Override BASELINE_BAG
  --cell-size METERS     Override CELL_SIZE
  --max-lap N            Override MAX_LAP
  --horizon N            Override HORIZON
  --steering-source SRC  Override STEERING_SOURCE for synthesized CPU export
  -h, --help             Show this help

You can also edit the EDIT-ME block at the top of this file and re-run.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline-bag) BASELINE_BAG="$2"; shift 2 ;;
    --cell-size) CELL_SIZE="$2"; shift 2 ;;
    --max-lap) MAX_LAP="$2"; shift 2 ;;
    --horizon) HORIZON="$2"; shift 2 ;;
    --steering-source) STEERING_SOURCE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

resolve_path() {
  case "$1" in
    /*) echo "$1" ;;
    *)  echo "${REPO_ROOT}/$1" ;;
  esac
}

if [[ -z "${BASELINE_BAG}" ]]; then
  BASELINE_BAG=$(ls -1 "${DEFAULT_BASELINE_DIR}"/*.mcap 2>/dev/null | head -1 || true)
fi
BASELINE_BAG=$(resolve_path "${BASELINE_BAG}")
if [[ ! -f "${BASELINE_BAG}" ]]; then
  echo "ERROR: baseline bag not found (got '${BASELINE_BAG}')" >&2
  exit 2
fi

OUT_DIR="${DEFAULT_OUTPUT_PARENT}/test5_baseline_tracking_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT_DIR}"
echo "Output dir: ${OUT_DIR}"

echo "[1/3] Sourcing ROS"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u; source "${REPO_ROOT}/install/setup.bash"; set -u
fi

STATE_CSV="${OUT_DIR}/state_replay.csv"
PLOT_SVG="${OUT_DIR}/baseline_tracking_maps.svg"
BAG_INFO_TXT="${OUT_DIR}/bag_info.txt"

echo "[2/3] Exporting state replay CSV from ${BASELINE_BAG}"
if command -v ros2 >/dev/null 2>&1; then
  ros2 bag info "${BASELINE_BAG}" > "${BAG_INFO_TXT}" 2>/dev/null || true
fi

if [[ -f "${BAG_INFO_TXT}" ]] && grep -q "Topic: /mpc_state |" "${BAG_INFO_TXT}"; then
  echo "  /mpc_state present -> direct export"
  python3 "${HELPER_DIR}/export_mpc_state_csv.py" \
    --bag "${BASELINE_BAG}" \
    --out "${STATE_CSV}" \
    --horizon "${HORIZON}"
else
  echo "  /mpc_state absent -> synthesizing from CPU topics"
  python3 "${HELPER_DIR}/export_mpc_state_csv_from_cpu_bag.py" \
    --bag "${BASELINE_BAG}" \
    --out "${STATE_CSV}" \
    --horizon "${HORIZON}" \
    --steering-source "${STEERING_SOURCE}"
fi

echo "[3/3] Plotting SVG maps"
python3 "${HELPER_DIR}/plot_baseline_tracking_from_state.py" \
  --state-csv "${STATE_CSV}" \
  --out-dir "${OUT_DIR}" \
  --cell-size "${CELL_SIZE}" \
  --max-lap "${MAX_LAP}"

echo
echo "=== Done ==="
echo "  state CSV:     ${STATE_CSV}"
echo "  plot SVG:      ${PLOT_SVG}"
if [[ -f "${BAG_INFO_TXT}" ]]; then
  echo "  bag info:      ${BAG_INFO_TXT}"
fi
