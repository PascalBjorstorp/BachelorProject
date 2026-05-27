#!/usr/bin/env bash
set -euo pipefail

# Test 2 harness: low-velocity-weight cost-dominance replay.
#
# Runs the CPU MPC twice on the same bag:
#   1. Baseline weights.
#   2. Reduced MPC_W_VELOCITY (default 0.01 of baseline; override with --low-vel-weight).
#
# Output: two CSVs with per-snapshot cost-term breakdown and planned velocity
# at horizon checkpoints. Plot scripts consume these CSVs to produce the
# spatial map and the cost-landscape visualization.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG_PATH=""
HORIZON=20
STEERING_SOURCE="auto"
LOW_VEL_WEIGHT=""           # absolute value; empty -> compute as 0.01 * baseline
OUT_DIR=""
SKIP_COMPILE=0
DEFAULT_INPUT_DIR="${REPO_ROOT}/tools/input/MPC_10Laps"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--bag <path.mcap>] [options]

Options:
  --bag PATH                Bag (.mcap). Default: first .mcap in ${DEFAULT_INPUT_DIR}
  --horizon N               Horizon length (default: ${HORIZON})
  --steering-source SRC     For CPU-bag synthesis: auto|servo|drive|ackermann|zero
  --low-vel-weight VALUE    Absolute weight_velocity for the low-weight run.
                            If omitted, uses 1% of the baseline (=2.0).
  --out-dir PATH            Output folder
  --skip-compile            Skip building the cost-replay binary
  -h, --help                Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --horizon) HORIZON="$2"; shift 2 ;;
    --steering-source) STEERING_SOURCE="$2"; shift 2 ;;
    --low-vel-weight) LOW_VEL_WEIGHT="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --skip-compile) SKIP_COMPILE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  if [[ -d "${DEFAULT_INPUT_DIR}" ]]; then
    BAG_PATH=$(ls -1 "${DEFAULT_INPUT_DIR}"/*.mcap 2>/dev/null | head -n1 || true)
  fi
fi
if [[ -z "${BAG_PATH}" || ! -f "${BAG_PATH}" ]]; then
  echo "ERROR: bag file not found. Pass --bag or place an .mcap in ${DEFAULT_INPUT_DIR}" >&2
  exit 2
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${DEFAULT_OUTPUT_PARENT}/test2_low_vel_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUT_DIR}"

if [[ -z "${LOW_VEL_WEIGHT}" ]]; then
  # 1% of WEIGHT_VELOCITY = 200.0 from MPC/include/mpc_types.h
  LOW_VEL_WEIGHT="2.0"
fi

echo "[1/5] Sourcing ROS"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u; source "${REPO_ROOT}/install/setup.bash"; set -u
fi

REPLAY_BIN="${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc_cost"
if [[ ${SKIP_COMPILE} -eq 0 ]]; then
  echo "[2/5] Building cost-replay binary"
  gcc -O3 -I"${REPO_ROOT}/MPC/include" \
      -I"${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include" \
      "${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc_cost.c" \
      "${REPO_ROOT}/MPC/src/util_math.c" \
      "${REPO_ROOT}/MPC/src/vehicle_model.c" \
      "${REPO_ROOT}/MPC/src/riccati_solver.c" \
      "${REPO_ROOT}/MPC/src/mpc.c" \
      -lm -o "${REPLAY_BIN}"
else
  echo "[2/5] Skipping build (--skip-compile)"
fi

STATE_CSV="${OUT_DIR}/state_replay.csv"

echo "[3/5] Exporting state CSV from bag: ${BAG_PATH}"
BAG_INFO_FILE="${OUT_DIR}/bag_info.txt"
ros2 bag info "${BAG_PATH}" > "${BAG_INFO_FILE}" || true
if grep -q "Topic: /mpc_state |" "${BAG_INFO_FILE}"; then
  echo "  /mpc_state present -> direct export"
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/export_mpc_state_csv.py" \
    --bag "${BAG_PATH}" --out "${STATE_CSV}" --horizon "${HORIZON}"
else
  echo "  /mpc_state absent -> synthesizing from CPU topics"
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/export_mpc_state_csv_from_cpu_bag.py" \
    --bag "${BAG_PATH}" --out "${STATE_CSV}" --horizon "${HORIZON}" \
    --steering-source "${STEERING_SOURCE}"
fi

BASELINE_OUT="${OUT_DIR}/cost_baseline.csv"
LOWVEL_OUT="${OUT_DIR}/cost_low_velocity_weight.csv"

echo "[4/5] Running baseline replay -> ${BASELINE_OUT}"
( unset MPC_W_VELOCITY
  "${REPLAY_BIN}" "${STATE_CSV}" "${BASELINE_OUT}" \
    >"${OUT_DIR}/baseline_raw.log" 2>&1 || true )

echo "[5/5] Running low-velocity-weight replay (MPC_W_VELOCITY=${LOW_VEL_WEIGHT}) -> ${LOWVEL_OUT}"
( export MPC_W_VELOCITY="${LOW_VEL_WEIGHT}"
  "${REPLAY_BIN}" "${STATE_CSV}" "${LOWVEL_OUT}" \
    >"${OUT_DIR}/low_vel_raw.log" 2>&1 || true )

echo
echo "Done."
echo "  out_dir:       ${OUT_DIR}"
echo "  baseline csv:  ${BASELINE_OUT}"
echo "  low-vel csv:   ${LOWVEL_OUT}"
echo "  state csv:     ${STATE_CSV}"
