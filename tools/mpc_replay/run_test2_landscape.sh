#!/usr/bin/env bash
set -euo pipefail

# Test 2 cost landscape: at a critical snapshot, sweep MPC_W_VELOCITY and plot
# how the optimal plan + cost components shift.
#
# Defaults: re-uses the state CSV from a prior run of
# run_test2_low_velocity_weight.sh, and auto-picks the snapshot with the
# lowest planned v_h_end in the low-velocity-weight run (i.e. the most
# "stopped" pose).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PRIOR_OUT_DIR=""
SNAPSHOT_IDX=""
W_MIN="0.001"
W_MAX="500.0"
N_STEPS="40"
SKIP_COMPILE=0

usage() {
  cat <<EOF
Usage: $(basename "$0") --prior-out-dir <test2 out_dir> [options]

Required:
  --prior-out-dir PATH   Output dir from run_test2_low_velocity_weight.sh
                         (must contain state_replay.csv and
                          cost_low_velocity_weight.csv)

Options:
  --snapshot-idx N       Snapshot idx to use. Default: auto-pick lowest v_h_end
                         from cost_low_velocity_weight.csv (skipping warmup)
  --w-min VALUE          Min weight_velocity to sweep (default ${W_MIN})
  --w-max VALUE          Max weight_velocity to sweep (default ${W_MAX})
  --n-steps N            Number of log-spaced sweep points (default ${N_STEPS})
  --skip-compile         Skip building cost_landscape_at_pose
  -h, --help             Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prior-out-dir) PRIOR_OUT_DIR="$2"; shift 2 ;;
    --snapshot-idx) SNAPSHOT_IDX="$2"; shift 2 ;;
    --w-min) W_MIN="$2"; shift 2 ;;
    --w-max) W_MAX="$2"; shift 2 ;;
    --n-steps) N_STEPS="$2"; shift 2 ;;
    --skip-compile) SKIP_COMPILE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "${PRIOR_OUT_DIR}" || ! -d "${PRIOR_OUT_DIR}" ]]; then
  echo "ERROR: --prior-out-dir required and must exist" >&2
  exit 2
fi

STATE_CSV="${PRIOR_OUT_DIR}/state_replay.csv"
LOW_VEL_CSV="${PRIOR_OUT_DIR}/cost_low_velocity_weight.csv"
for f in "${STATE_CSV}" "${LOW_VEL_CSV}"; do
  if [[ ! -f "${f}" ]]; then
    echo "ERROR: required file missing: ${f}" >&2
    exit 2
  fi
done

if [[ -z "${SNAPSHOT_IDX}" ]]; then
  echo "[auto-pick] Finding snapshot with lowest v_h_end in low-vel run..."
  # Columns of cost_low_velocity_weight.csv: idx,stamp_ns,pos_x,pos_y,theta,v_now,v_h5,v_h10,v_h_end,...
  # Skip rows where pos_x==0 and pos_y==0 (pre-odom warmup).
  SNAPSHOT_IDX=$(awk -F, 'NR>1 && ($3*$3+$4*$4>0.01) {if (n==0 || $9<min){min=$9; idx=$1; px=$3; py=$4} n++}
                          END {printf "%s\n", idx; print "  (idx="idx", v_h_end="min", pos=("px","py"))" > "/dev/stderr"}' \
                  "${LOW_VEL_CSV}")
  echo "  picked idx=${SNAPSHOT_IDX}"
fi

OUT_DIR="${PRIOR_OUT_DIR}/landscape_idx${SNAPSHOT_IDX}"
mkdir -p "${OUT_DIR}"
LANDSCAPE_CSV="${OUT_DIR}/cost_landscape.csv"

BIN="${REPO_ROOT}/tools/mpc_replay/helper/cost_landscape_at_pose"
if [[ ${SKIP_COMPILE} -eq 0 ]]; then
  echo "[build] cost_landscape_at_pose"
  gcc -O3 -Wno-misleading-indentation \
    -I"${REPO_ROOT}/MPC/include" \
    -I"${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include" \
    "${REPO_ROOT}/tools/mpc_replay/helper/cost_landscape_at_pose.c" \
    "${REPO_ROOT}/MPC/src/util_math.c" \
    "${REPO_ROOT}/MPC/src/vehicle_model.c" \
    "${REPO_ROOT}/MPC/src/riccati_solver.c" \
    "${REPO_ROOT}/MPC/src/mpc.c" \
    -lm -o "${BIN}"
fi

echo "[sweep] snapshot=${SNAPSHOT_IDX} w_vel in [${W_MIN}, ${W_MAX}] across ${N_STEPS} log-steps"
"${BIN}" "${STATE_CSV}" "${SNAPSHOT_IDX}" "${LANDSCAPE_CSV}" "${W_MIN}" "${W_MAX}" "${N_STEPS}"

echo "[plot] landscape"
python3 "${REPO_ROOT}/tools/mpc_replay/helper/plot_test2_landscape.py" \
  --landscape-csv "${LANDSCAPE_CSV}" \
  --out-dir "${OUT_DIR}/plots"

echo
echo "Done."
echo "  landscape csv: ${LANDSCAPE_CSV}"
echo "  plots:         ${OUT_DIR}/plots/"
