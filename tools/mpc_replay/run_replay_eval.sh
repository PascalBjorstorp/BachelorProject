#!/usr/bin/env bash
set -euo pipefail

# One-command replay evaluation:
# - Builds CPU and FPGA scalar replay binaries
# - Exports replay CSV from bag (/mpc_state if present, else synthesize from CPU topics)
# - Runs CPU and FPGA scalar replays on identical input
# - Compares and prints compact convergence metrics

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

BAG_PATH=""
HORIZON=20
STEERING_SOURCE="auto"
OPENCL_CSV=""
OUT_DIR=""
DEFAULT_INPUT_DIR="${REPO_ROOT}/tools/input"
DEFAULT_OUTPUT_PARENT="${REPO_ROOT}/tools/output"
SKIP_COMPILE=0

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag <path.mcap> [options]

Required:
  --bag PATH                      Input bag (.mcap)

Options:
  --horizon N                     Horizon length (default: 20)
  --steering-source SRC           For CPU-bag synthesis: auto|servo|drive|ackermann|zero (default: auto)
  --opencl-csv PATH               Optional OpenCL raw CSV to include in compare
  --out-dir PATH                  Output folder (default: ${OUT_DIR})
  --skip-compile                  Skip building replay binaries
  -h, --help                      Show this help

Environment overrides:
  VITIS_INCLUDE                   Vitis include path for FPGA scalar build
                                  default: /home/akselmo/Vivado_program/2025.2/Vitis/include
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --horizon) HORIZON="$2"; shift 2 ;;
    --steering-source) STEERING_SOURCE="$2"; shift 2 ;;
    --opencl-csv) OPENCL_CSV="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --skip-compile) SKIP_COMPILE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  # pick first .mcap from default input dir
  if [[ -d "${DEFAULT_INPUT_DIR}" ]]; then
    first_bag=$(ls -1 "${DEFAULT_INPUT_DIR}"/*.mcap 2>/dev/null | head -n1 || true)
    if [[ -n "${first_bag}" ]]; then
      BAG_PATH="${first_bag}"
      echo "Using default bag: ${BAG_PATH}"
    fi
  fi
fi
if [[ -z "${BAG_PATH}" ]]; then
  echo "ERROR: --bag is required (or place a .mcap in ${DEFAULT_INPUT_DIR})" >&2
  usage
  exit 2
fi
if [[ ! -f "${BAG_PATH}" ]]; then
  echo "ERROR: bag file not found: ${BAG_PATH}" >&2
  exit 2
fi

if [[ -z "${OUT_DIR}" ]]; then
  OUT_DIR="${DEFAULT_OUTPUT_PARENT}/mpc_replay_eval_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUT_DIR}"
mkdir -p "${DEFAULT_OUTPUT_PARENT}"

echo "[1/6] Sourcing ROS/workspace"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # ROS setup scripts may reference unset vars; source with nounset disabled.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${REPO_ROOT}/install/setup.bash"
  set -u
fi

if [[ ${SKIP_COMPILE} -eq 0 ]]; then
  echo "[2/6] Building replay binaries"
  gcc -O3 -I"${REPO_ROOT}/MPC/include" -I"${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include" \
    "${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc.c" \
    "${REPO_ROOT}/MPC/src/util_math.c" \
    "${REPO_ROOT}/MPC/src/vehicle_model.c" \
    "${REPO_ROOT}/MPC/src/riccati_solver.c" \
    "${REPO_ROOT}/MPC/src/mpc.c" \
    -lm -o "${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc"

  VITIS_INCLUDE="${VITIS_INCLUDE:-/home/akselmo/Vivado_program/2025.2/Vitis/include}"
  if [[ ! -d "${VITIS_INCLUDE}" ]]; then
    echo "ERROR: VITIS include path not found: ${VITIS_INCLUDE}" >&2
    echo "Set VITIS_INCLUDE=/path/to/Vitis/include" >&2
    exit 2
  fi

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
else
  echo "[2/6] Skipping build (--skip-compile)"
fi

STATE_CSV="${OUT_DIR}/state_replay.csv"
CPU_OUT="${OUT_DIR}/replay_cpu_out.csv"
FPGA_OUT="${OUT_DIR}/replay_fpga_out.csv"
COMPARE_TXT="${OUT_DIR}/compare.txt"
SUMMARY_TXT="${OUT_DIR}/summary.txt"


echo "[3/6] Detecting bag type"
BAG_INFO_FILE="${OUT_DIR}/bag_info.txt"
ros2 bag info "${BAG_PATH}" > "${BAG_INFO_FILE}" || true
if grep -q "Topic: /mpc_state |" "${BAG_INFO_FILE}"; then
  echo "  Found /mpc_state topic -> exporting directly"
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/export_mpc_state_csv.py" \
    --bag "${BAG_PATH}" \
    --out "${STATE_CSV}" \
    --horizon "${HORIZON}"
else
  echo "  No /mpc_state topic -> synthesizing from CPU topics (steering-source=${STEERING_SOURCE})"
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/export_mpc_state_csv_from_cpu_bag.py" \
    --bag "${BAG_PATH}" \
    --out "${STATE_CSV}" \
    --horizon "${HORIZON}" \
    --steering-source "${STEERING_SOURCE}"
fi

echo "[4/6] Running CPU replay (raw log -> ${OUT_DIR}/replay_cpu_raw.log)"
"${REPO_ROOT}/tools/mpc_replay/helper/replay_cpu_mpc" "${STATE_CSV}" "${CPU_OUT}" >"${OUT_DIR}/replay_cpu_raw.log" 2>&1 || true
# filter noisy CPU_BACK lines for terminal display, keep full raw log
{ grep -v -E '^(CPU_BACK|CPU_BACK6|CPU_BACK6_vy)' "${OUT_DIR}/replay_cpu_raw.log" || true; } | sed -E 's/\r$//' | tee "${OUT_DIR}/replay_cpu_filtered.log"

echo "[5/6] Running FPGA scalar replay (raw log -> ${OUT_DIR}/replay_fpga_raw.log)"
"${REPO_ROOT}/tools/mpc_replay/helper/replay_fpga_scalar" "${STATE_CSV}" "${FPGA_OUT}" >"${OUT_DIR}/replay_fpga_raw.log" 2>&1 || true
{ grep -v -E '^(CPU_BACK|CPU_BACK6|CPU_BACK6_vy)' "${OUT_DIR}/replay_fpga_raw.log" || true; } | sed -E 's/\r$//' | tee "${OUT_DIR}/replay_fpga_filtered.log"

echo "[6/6] Comparing and summarizing"
if [[ -n "${OPENCL_CSV}" ]]; then
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/compare_replays_top.py" \
    --cpu "${CPU_OUT}" \
    --fpga "${FPGA_OUT}" \
    --opencl "${OPENCL_CSV}" \
    --top 10 | tee "${COMPARE_TXT}"
else
  python3 "${REPO_ROOT}/tools/mpc_replay/helper/compare_replays_top.py" \
    --cpu "${CPU_OUT}" \
    --fpga "${FPGA_OUT}" \
    --top 10 | tee "${COMPARE_TXT}"
fi

python3 - "${CPU_OUT}" "${FPGA_OUT}" "${SUMMARY_TXT}" << 'PY'
import csv
import sys

cpu_path, fpga_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]

def load(path):
    rows=[]
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows

def stats(rows):
    n=len(rows)
    sc={}
    it=[]
    first_nonzero=None
    max_streak=0
    cur=0
    for i,r in enumerate(rows, start=1):
        s=int(r['status'])
        sc[s]=sc.get(s,0)+1
        it.append(int(r['iters']))
        if s!=0 and first_nonzero is None:
            first_nonzero=i
        if s==1:
            cur+=1
            if cur>max_streak:max_streak=cur
        else:
            cur=0
    avg_it=(sum(it)/n) if n else 0.0
    return {
      'rows':n,
      'status':sc,
      'iter_min':min(it) if it else 0,
      'iter_avg':avg_it,
      'iter_max':max(it) if it else 0,
      'first_nonzero':first_nonzero,
      'max_iter_streak':max_streak,
      'max_iter_rate': (sc.get(1,0)/n*100.0) if n else 0.0,
      'ok_rate': (sc.get(0,0)/n*100.0) if n else 0.0,
    }

cpu=load(cpu_path)
fpga=load(fpga_path)
cs=stats(cpu)
fs=stats(fpga)

common=min(len(cpu),len(fpga))
status_match=0
for i in range(common):
    if cpu[i]['status']==fpga[i]['status']:
        status_match+=1
status_match_rate = (status_match/common*100.0) if common else 0.0

lines=[]
lines.append("=== Replay Summary ===")
lines.append(f"cpu:  rows={cs['rows']} status={cs['status']} iter(min/avg/max)={cs['iter_min']}/{cs['iter_avg']:.2f}/{cs['iter_max']} first_nonzero={cs['first_nonzero']} max_iter_rate={cs['max_iter_rate']:.2f}% max_iter_streak={cs['max_iter_streak']}")
lines.append(f"fpga: rows={fs['rows']} status={fs['status']} iter(min/avg/max)={fs['iter_min']}/{fs['iter_avg']:.2f}/{fs['iter_max']} first_nonzero={fs['first_nonzero']} max_iter_rate={fs['max_iter_rate']:.2f}% max_iter_streak={fs['max_iter_streak']}")
lines.append(f"cpu_vs_fpga status match over common rows ({common}): {status_match_rate:.2f}%")

with open(out_path,'w') as f:
    f.write("\n".join(lines)+"\n")
print("\n".join(lines))
PY

cat <<EOF

Done.
Artifacts:
  ${OUT_DIR}
  - state_replay.csv
  - replay_cpu_out.csv
  - replay_fpga_out.csv
  - compare.txt
  - summary.txt
EOF
