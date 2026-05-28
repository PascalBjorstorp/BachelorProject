#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Test 1: baseline CPU MPC vs the chosen FPGA implementation (UDP transport).
# One-shot driver. Edit the variables in the EDIT-ME section below, then run.
#
# What it does:
#   1. Extracts iter_count + solve_us + pose from the baseline bag.
#   2. Detects laps in the baseline run and keeps only race laps 2..MAX_LAP
#      (same convention as Test 2/3) so timing matches the cleaned 10-lap
#      window used in the FPGA thesis section.
#   3. Reads the FPGA UDP per-solve stats CSV that already exists alongside
#      the FPGA bag (under ActualOutput/*.stats.csv).
#   4. Renders boxplots, histograms, and an affine per-iteration fit
#      (matching the methodology of the FPGA timing section).
#
# Outputs land in: tools/output/test1_baseline_vs_fpga_<timestamp>/
# =============================================================================

# ============================ EDIT-ME ========================================
# Bag (CPU baseline). Empty = first .mcap inside tools/input/MPC10LapBaseline.
BASELINE_BAG=""
# FPGA UDP bag dir. Must contain ActualOutput/*.stats.csv.
FPGA_UDP_DIR="tools/input/FPGA_UDP"
# Optional explicit stats CSV path (overrides auto-discovery in FPGA_UDP_DIR/ActualOutput/).
FPGA_UDP_STATS=""
# Race-lap filter for the CPU baseline.
MAX_LAP="11"
MIN_ITER_GROUP="20"     # affine-fit per-iteration-count group size threshold
CPU_FREQ_MHZ="1700"     # Jetson CPU clock frequency [MHz]  (for cycle-count normalization)
FPGA_FREQ_MHZ="200"     # FPGA kernel clock frequency [MHz]
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
  --baseline-bag PATH          Override BASELINE_BAG
  --fpga-udp-dir PATH          Override FPGA_UDP_DIR
  --fpga-udp-stats PATH        Override FPGA_UDP_STATS
  --max-lap N                  Override MAX_LAP
  --min-iter-group N           Override MIN_ITER_GROUP
  --cpu-freq-mhz N             Override CPU_FREQ_MHZ
  --fpga-freq-mhz N            Override FPGA_FREQ_MHZ
  -h, --help                   Show this help

You can also edit the EDIT-ME block at the top of this file and re-run.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline-bag) BASELINE_BAG="$2"; shift 2 ;;
    --fpga-udp-dir) FPGA_UDP_DIR="$2"; shift 2 ;;
    --fpga-udp-stats) FPGA_UDP_STATS="$2"; shift 2 ;;
    --max-lap) MAX_LAP="$2"; shift 2 ;;
    --min-iter-group) MIN_ITER_GROUP="$2"; shift 2 ;;
    --cpu-freq-mhz) CPU_FREQ_MHZ="$2"; shift 2 ;;
    --fpga-freq-mhz) FPGA_FREQ_MHZ="$2"; shift 2 ;;
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
if [[ ! -f "${BASELINE_BAG}" ]]; then
  echo "ERROR: baseline bag not found (got '${BASELINE_BAG}')" >&2; exit 2
fi

FPGA_UDP_DIR=$(resolve_path "${FPGA_UDP_DIR}")
if [[ -z "${FPGA_UDP_STATS}" ]]; then
  FPGA_UDP_STATS=$(ls -1 "${FPGA_UDP_DIR}"/ActualOutput/*.stats.csv 2>/dev/null | head -1 || true)
fi
if [[ -z "${FPGA_UDP_STATS}" || ! -f "${FPGA_UDP_STATS}" ]]; then
  echo "ERROR: FPGA UDP stats not found (got '${FPGA_UDP_STATS}')" >&2; exit 2
fi

OUT_DIR="${DEFAULT_OUTPUT_PARENT}/test1_baseline_vs_fpga_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT_DIR}"
echo "Output dir: ${OUT_DIR}"

echo "[1/3] Sourcing ROS"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u; source "${REPO_ROOT}/install/setup.bash"; set -u
fi

BASELINE_CSV="${OUT_DIR}/baseline_timing.csv"

echo "[2/3] Extracting baseline timing from ${BASELINE_BAG}"
python3 "${HELPER_DIR}/export_baseline_timing_csv.py" \
  --bag "${BASELINE_BAG}" --out "${BASELINE_CSV}"

echo "[3/3] Plotting"
python3 "${HELPER_DIR}/plot_test1_baseline_vs_fpga.py" \
  --baseline-csv "${BASELINE_CSV}" \
  --fpga-udp-stats "${FPGA_UDP_STATS}" \
  --out-dir "${OUT_DIR}/plots" \
  --max-lap "${MAX_LAP}" \
  --min-iter-group "${MIN_ITER_GROUP}" \
  --cpu-freq-mhz "${CPU_FREQ_MHZ}" \
  --fpga-freq-mhz "${FPGA_FREQ_MHZ}"

echo
echo "=== Done ==="
echo "  baseline CSV:  ${BASELINE_CSV}"
echo "  FPGA UDP:      ${FPGA_UDP_STATS}"
echo "  plots:         ${OUT_DIR}/plots/"
