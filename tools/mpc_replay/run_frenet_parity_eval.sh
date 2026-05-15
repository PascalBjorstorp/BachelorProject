#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG_PATH=""
STEERING_SOURCE="auto"
OUT_DIR=""
DEFAULT_INPUT_DIR="${REPO_ROOT}/tools/input"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"
HORIZON=20
SKIP_COMPILE=0

usage() {
  cat <<USAGE
Usage: $(basename "$0") --bag <path.mcap> [options]

Options:
  --bag PATH
  --steering-source SRC     auto|servo|drive|ackermann|zero (default: auto)
  --horizon N               Horizon (default: 20)
  --out-dir PATH            Output directory
  --skip-compile            Skip compiling replay and dump tools
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --steering-source) STEERING_SOURCE="$2"; shift 2 ;;
    --horizon) HORIZON="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --skip-compile) SKIP_COMPILE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  if [[ -d "${DEFAULT_INPUT_DIR}" ]]; then
    BAG_PATH=$(ls -1 "${DEFAULT_INPUT_DIR}"/*.mcap 2>/dev/null | head -n1 || true)
    if [[ -n "${BAG_PATH}" ]]; then
      echo "Using default bag: ${BAG_PATH}"
    fi
  fi
fi
if [[ -z "${BAG_PATH}" ]]; then
  echo "ERROR: --bag is required (or place a .mcap in ${DEFAULT_INPUT_DIR})" >&2
  exit 2
fi
if [[ ! -f "${BAG_PATH}" ]]; then
  echo "ERROR: bag not found: ${BAG_PATH}" >&2
  exit 2
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${DEFAULT_OUTPUT_PARENT}/frenet_parity_eval_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUT_DIR}"
mkdir -p "${DEFAULT_OUTPUT_PARENT}"

if [[ ${SKIP_COMPILE} -eq 0 ]]; then
  echo "[1/6] Building replay binaries"
  gcc -O3 -I"${REPO_ROOT}/MPC/include" \
    "${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc.c" \
    "${REPO_ROOT}/MPC/src/util_math.c" \
    "${REPO_ROOT}/MPC/src/vehicle_model.c" \
    "${REPO_ROOT}/MPC/src/riccati_solver.c" \
    "${REPO_ROOT}/MPC/src/mpc.c" \
    -lm -o "${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc"

  VITIS_INCLUDE="${VITIS_INCLUDE:-/home/akselmo/Vivado_program/2025.2/Vitis/include}"

  g++ -O2 -Wno-unknown-pragmas \
    -I"${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include" \
    -I"${VITIS_INCLUDE}" \
    "${REPO_ROOT}/tools/mpc_replay/helper/replay_fpga_scalar.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/fp_math_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/vehicle_model_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/riccati_solver_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/mpc_riccati_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/mpc_fpga_top.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/mpc_runtime_tune.cpp" \
    -lm -o "${REPO_ROOT}/tools/mpc_replay/helper/replay_fpga_scalar"

  echo "[2/6] Building Frenet dump tools"
  gcc -O3 -I"${REPO_ROOT}/MPC/include" \
    "${REPO_ROOT}/tools/mpc_replay/helper/dump_cpu_frenet.c" \
    "${REPO_ROOT}/MPC/src/util_math.c" \
    "${REPO_ROOT}/MPC/src/vehicle_model.c" \
    -lm -o "${REPO_ROOT}/tools/mpc_replay/helper/dump_cpu_frenet"

  g++ -O2 -Wno-unknown-pragmas \
    -I"${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include" \
    -I"${VITIS_INCLUDE}" \
    "${REPO_ROOT}/tools/mpc_replay/helper/dump_fpga_frenet.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/fp_math_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/vehicle_model_hls.cpp" \
    "${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/src/mpc_runtime_tune.cpp" \
    -lm -o "${REPO_ROOT}/tools/mpc_replay/helper/dump_fpga_frenet"
else
  echo "[1/6] Skipping compile"
fi

echo "[3/6] Running replay eval to generate state replay + CPU control history"
bash "${REPO_ROOT}/tools/mpc_replay/run_replay_eval.sh" \
  --bag "${BAG_PATH}" \
  --horizon "${HORIZON}" \
  --steering-source "${STEERING_SOURCE}" \
  --out-dir "${OUT_DIR}" \
  --skip-compile

STATE_CSV="${OUT_DIR}/state_replay.csv"
CPU_REPLAY="${OUT_DIR}/replay_cpu_out.csv"
CPU_FRENET="${OUT_DIR}/cpu_frenet.csv"
FPGA_FRENET="${OUT_DIR}/fpga_frenet.csv"
FRENET_DIFFS="${OUT_DIR}/frenet_diffs.csv"
FRENET_COMPARE="${OUT_DIR}/frenet_compare.txt"
CPU_RANGE="${OUT_DIR}/cpu_frenet_ranges.txt"
CPU_RANGE_CSV="${OUT_DIR}/cpu_frenet_ranges.csv"

echo "[4/6] Dumping CPU Frenet linearization (raw log -> ${OUT_DIR}/dump_cpu_frenet_raw.log)"
"${REPO_ROOT}/tools/mpc_replay/helper/dump_cpu_frenet" \
  "${STATE_CSV}" \
  "${CPU_FRENET}" \
  --accel-source cpu-prev \
  --cpu-replay-csv "${CPU_REPLAY}" >"${OUT_DIR}/dump_cpu_frenet_raw.log" 2>&1 || true
{ grep -v -E '^(CPU_BACK|CPU_BACK6|CPU_BACK6_vy)' "${OUT_DIR}/dump_cpu_frenet_raw.log" || true; } | sed -E 's/\r$//' | tee "${OUT_DIR}/dump_cpu_frenet_filtered.log"

echo "[5/6] Dumping FPGA Frenet linearization (raw log -> ${OUT_DIR}/dump_fpga_frenet_raw.log)"
"${REPO_ROOT}/tools/mpc_replay/helper/dump_fpga_frenet" \
  "${STATE_CSV}" \
  "${FPGA_FRENET}" \
  --accel-source cpu-prev \
  --cpu-replay-csv "${CPU_REPLAY}" >"${OUT_DIR}/dump_fpga_frenet_raw.log" 2>&1 || true
{ grep -v -E '^(CPU_BACK|CPU_BACK6|CPU_BACK6_vy)' "${OUT_DIR}/dump_fpga_frenet_raw.log" || true; } | sed -E 's/\r$//' | tee "${OUT_DIR}/dump_fpga_frenet_filtered.log"

echo "[6/6] Comparing and analyzing ranges"
python3 "${REPO_ROOT}/tools/mpc_replay/helper/compare_frenet_dumps.py" \
  --cpu "${CPU_FRENET}" \
  --fpga "${FPGA_FRENET}" \
  --out "${FRENET_DIFFS}" | tee "${FRENET_COMPARE}"

python3 "${REPO_ROOT}/tools/mpc_replay/helper/analyze_frenet_ranges.py" \
  --in "${CPU_FRENET}" \
  --out "${CPU_RANGE_CSV}" | tee "${CPU_RANGE}"

echo
echo "Done: ${OUT_DIR}"
echo "- cpu_frenet.csv"
echo "- fpga_frenet.csv"
echo "- frenet_compare.txt"
echo "- frenet_diffs.csv"
echo "- cpu_frenet_ranges.txt"
echo "- cpu_frenet_ranges.csv"
