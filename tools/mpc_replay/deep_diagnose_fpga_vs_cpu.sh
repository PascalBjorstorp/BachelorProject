#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG_PATH=""
STEERING_SOURCE="auto"
OUT_DIR=""
PARITY_MODE="all"
DEFAULT_INPUT_DIR="${REPO_ROOT}/tools/input"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag <path.mcap> [options]

Options:
  --bag PATH
  --steering-source SRC   auto|servo|drive|ackermann|zero (default: auto)
  --out-dir PATH          Output directory (default: ${DEFAULT_OUTPUT_PARENT}/mpc_deep_diag_<ts>)
  --parity MODE           Include parity checks: all|frenet|none (default: all)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --steering-source) STEERING_SOURCE="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --parity) PARITY_MODE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  # pick first bag from default input dir
  if [[ -d "${DEFAULT_INPUT_DIR}" ]]; then
    BAG_PATH=$(ls -1 "${DEFAULT_INPUT_DIR}"/*.mcap 2>/dev/null | head -n1 || true)
    if [[ -n "${BAG_PATH}" ]]; then
      echo "Using default bag: ${BAG_PATH}"
    fi
  fi
fi
if [[ -z "${BAG_PATH}" ]]; then
  echo "ERROR: --bag required (or place a .mcap in ${DEFAULT_INPUT_DIR})" >&2
  exit 2
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${DEFAULT_OUTPUT_PARENT}/mpc_deep_diag_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUT_DIR}"
mkdir -p "${DEFAULT_OUTPUT_PARENT}"

echo "[1/4] Running baseline replay eval"
bash "${REPO_ROOT}/tools/mpc_replay/run_replay_eval.sh" \
  --bag "${BAG_PATH}" \
  --steering-source "${STEERING_SOURCE}" \
  --out-dir "${OUT_DIR}"

echo "[2/4] Finding first status divergence index"
TRACE_IDX="$(python3 - << 'PY' "${OUT_DIR}/replay_cpu_out.csv" "${OUT_DIR}/replay_fpga_out.csv"
import csv,sys
cpu=list(csv.DictReader(open(sys.argv[1])))
fpga=list(csv.DictReader(open(sys.argv[2])))
by_cpu={int(r['idx']):r for r in cpu}
by_fpga={int(r['idx']):r for r in fpga}
common=sorted(set(by_cpu)&set(by_fpga))
idx=0
for i in common:
    if by_cpu[i]['status']!=by_fpga[i]['status']:
        idx=i
        break
if idx==0 and common:
    idx=common[0]
print(idx)
PY
)"

echo "  trace_idx=${TRACE_IDX}"

echo "[3/4] Dumping per-iteration ADMM traces at idx=${TRACE_IDX}"
"${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc" \
  "${OUT_DIR}/state_replay.csv" \
  "${OUT_DIR}/replay_cpu_out.csv" \
  --trace-idx "${TRACE_IDX}" \
  --trace-out "${OUT_DIR}/cpu_trace.csv" >/dev/null

"${REPO_ROOT}/tools/mpc_replay/helper/replay_fpga_scalar" \
  "${OUT_DIR}/state_replay.csv" \
  "${OUT_DIR}/replay_fpga_out.csv" \
  --trace-idx "${TRACE_IDX}" \
  --trace-out "${OUT_DIR}/fpga_trace.csv" >/dev/null

echo "[4/4] Comparing ADMM traces"
python3 "${REPO_ROOT}/tools/mpc_replay/helper/compare_admm_traces.py" \
  --cpu "${OUT_DIR}/cpu_trace.csv" \
  --fpga "${OUT_DIR}/fpga_trace.csv" | tee "${OUT_DIR}/trace_compare.txt"

# Optionally include frenet parity checks in deep diagnostics
if [[ "${PARITY_MODE}" != "none" ]]; then
  echo "Including parity checks (mode=${PARITY_MODE}) -> running frenet parity eval"
  PARITY_OUT="${OUT_DIR}/frenet_parity"
  mkdir -p "${PARITY_OUT}"
  bash "${REPO_ROOT}/tools/mpc_replay/run_frenet_parity_eval.sh" \
    --bag "${BAG_PATH}" \
    --horizon 20 \
    --out-dir "${PARITY_OUT}"
  echo "Frenet parity artifacts: ${PARITY_OUT}"
fi

echo
echo "Deep diagnostics complete: ${OUT_DIR}"
echo "Artifacts:"
echo "  - state_replay.csv"
echo "  - replay_cpu_out.csv"
echo "  - replay_fpga_out.csv"
echo "  - cpu_trace.csv"
echo "  - fpga_trace.csv"
echo "  - trace_compare.txt"
