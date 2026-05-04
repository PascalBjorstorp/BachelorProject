#!/usr/bin/env bash
#===========================================================================
# MPCC/run_sim.sh — Launch MPCC (Contouring Control) with F1/10th simulator
#
# Usage:
#   ./MPCC/run_sim.sh 120 f1tenth_planning/trajectories/my_track_centerline_smooth.csv
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = f1tenth_planning/trajectories/my_track_centerline_smooth.csv
#   MPCC_PROFILE     = convergence_debug
#
# This script:
#   1. Rebuilds the mpcc_f1_10th package via colcon
#   2. Launches gym_bridge + MPCC node
#   3. Monitors for collision, logs everything, produces summary
#
# Logs are saved to logs/run_<timestamp>/.
#===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION_SECONDS="${1:-120}"
TRAJECTORY_FILE="${2:-${ROOT_DIR}/f1tenth_planning/trajectories/my_track_centerline_smooth.csv}"

# Resolve to absolute path so ROS2 nodes can find it regardless of cwd
if [[ "${TRAJECTORY_FILE}" != /* ]]; then
    TRAJECTORY_FILE="$(cd "$(dirname "${TRAJECTORY_FILE}")" && pwd)/$(basename "${TRAJECTORY_FILE}")"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/logs/run_${RUN_ID}"
mkdir -p "${LOG_DIR}"

GYM_LOG="${LOG_DIR}/gym_bridge.log"
MPCC_LOG="${LOG_DIR}/mpcc.log"
SUMMARY_LOG="${LOG_DIR}/summary.txt"
CONFIG_LOG="${LOG_DIR}/run_config.env"

SIM_PID=""
MPC_PID=""

cleanup() {
    set +e
    if [[ -n "${MPC_PID}" ]] && kill -0 "${MPC_PID}" 2>/dev/null; then
        kill "${MPC_PID}" 2>/dev/null || true
    fi
    if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
        kill "${SIM_PID}" 2>/dev/null || true
    fi

    pkill -f 'ros2 launch mpcc_f1_10th mpcc_launch.py' 2>/dev/null || true
    pkill -f 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py' 2>/dev/null || true
    pkill -f '/mpcc_f1_10th/mpcc_node' 2>/dev/null || true
    pkill -f '/f1tenth_gym_ros/gym_bridge' 2>/dev/null || true
    pkill -f 'rviz2.*gym_bridge.rviz' 2>/dev/null || true
    pkill -f 'lifecycle_manager_localization' 2>/dev/null || true
    pkill -f 'nav2_map_server' 2>/dev/null || true
    pkill -f 'ego_robot_state_publisher' 2>/dev/null || true
}
trap cleanup EXIT

echo "=== MPCC (Contouring Control) Simulation Run ==="
echo "Logs: ${LOG_DIR}"
echo "Duration: ${DURATION_SECONDS}s"

#===========================================================================
# Step 1: Build the ROS2 package
#===========================================================================
echo ""
echo "--- Building f1tenth_gym_ros + mpcc_f1_10th ROS2 packages ---"

cd "${ROOT_DIR}"

# Source ROS2 base
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Build both gym and MPCC so run-time config in install/ is current.
colcon build --packages-select f1tenth_gym_ros mpcc_f1_10th 2>&1 | tail -8

# Re-source the workspace after build
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "  Build complete."

#===========================================================================
# Step 2: Launch simulation
#===========================================================================
echo ""
echo "--- Launching simulation ---"

export PYTHONPATH="${ROOT_DIR}/f1tenth_sim:${ROOT_DIR}/f1tenth_sim/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

SIM_CFG_SOURCE="${ROOT_DIR}/f1tenth_sim/config/sim.yaml"
yaml_number() {
    local key="$1"
    awk -v key="$key" '
        $1 == key ":" {
            v = $2
            sub(/#.*/, "", v)
            gsub(/[[:space:]]/, "", v)
            gsub(/["\047]/, "", v)
            print v
            exit
        }
    ' "${SIM_CFG_SOURCE}" 2>/dev/null
}

SIM_MU="$(yaml_number vehicle_mu)"
SIM_C_SF="$(yaml_number vehicle_C_Sf)"
SIM_C_SR="$(yaml_number vehicle_C_Sr)"
SIM_A_MAX="$(yaml_number vehicle_a_max)"
SIM_MAP_PATH="$(yaml_number map_path)"
SIM_MAP_IMG_EXT="$(yaml_number map_img_ext)"
SIM_A_MIN=""
if [[ -n "${SIM_A_MAX}" ]]; then
    SIM_A_MIN="$(awk -v a="${SIM_A_MAX}" 'BEGIN{printf "%.6f", -a}')"
fi

TRAJ_SX="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $2); print $2; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"
TRAJ_SY="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $3); print $3; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"
TRAJ_STHETA="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $4); print $4; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"

DEFAULT_GYM_MAP_PATH="${SIM_MAP_PATH:-my_track_map}"
DEFAULT_GYM_MAP_IMG_EXT="${SIM_MAP_IMG_EXT:-.pgm}"
TRAJ_BASENAME="$(basename "${TRAJECTORY_FILE}")"
if [[ "${TRAJ_BASENAME}" == *"my_track"* ]]; then
    DEFAULT_GYM_MAP_PATH="my_track_map"
    DEFAULT_GYM_MAP_IMG_EXT=".pgm"
elif [[ "${TRAJ_BASENAME}" == *"Spielberg"* ]]; then
    DEFAULT_GYM_MAP_PATH="Spielberg_map"
    DEFAULT_GYM_MAP_IMG_EXT=".png"
elif [[ "${TRAJ_BASENAME}" == *"hardware"* ]]; then
    DEFAULT_GYM_MAP_PATH="hardware_map"
    DEFAULT_GYM_MAP_IMG_EXT=".pgm"
fi

MPCC_PROFILE="${MPCC_PROFILE:-convergence_debug}"

set_default() {
    local var_name="$1"
    local default_value="$2"
    if [[ -z "${!var_name+x}" ]]; then
        export "${var_name}=${default_value}"
    fi
}

case "${MPCC_PROFILE}" in
    convergence_debug)
        set_default HORIZON 40
        set_default DT 0.03
        set_default Q_CONTOURING 80.0
        set_default Q_LAG 120.0
        set_default Q_PROGRESS 10.0
        set_default Q_VX 10.0
        set_default VX_REF 4.0
        set_default Q_VY 0.5
        set_default Q_OMEGA 1.5
        set_default R_DELTA 150.0
        set_default R_AX 0.05225
        set_default R_VTHETA 0.1
        set_default W_DELTA_RATE 8.0
        set_default W_AX_RATE 0.488
        set_default W_VTHETA_RATE 0.1105
        set_default Q_CONTOURING_TERM 600.0
        set_default Q_LAG_TERM 200.0
        set_default Q_PROGRESS_TERM 30.0
        set_default ADMM_RHO 5.0
        set_default ADMM_RHO_U 0.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default V_THETA_MAX 6.0
        set_default CROSS_CALL_SCALE 0.166667
        ;;
    tuned_fast)
        set_default HORIZON 20
        set_default DT 0.03
        set_default Q_CONTOURING 960.0
        set_default Q_LAG 200.0
        set_default Q_PROGRESS 15.6
        set_default Q_VX 30.0
        set_default VX_REF 4.0
        set_default Q_VY 0.5
        set_default Q_OMEGA 1.5
        set_default R_DELTA 100.0
        set_default R_AX 0.05225
        set_default R_VTHETA 0.1
        set_default W_DELTA_RATE 5.0
        set_default W_AX_RATE 0.488
        set_default W_VTHETA_RATE 0.1105
        set_default Q_CONTOURING_TERM 4800.0
        set_default Q_LAG_TERM 800.0
        set_default Q_PROGRESS_TERM 41.4
        set_default ADMM_RHO 5.0
        set_default ADMM_RHO_U 0.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default V_THETA_MAX 10.0
        set_default CROSS_CALL_SCALE 0.166667
        ;;
    manual)
        :
        ;;
    *)
        echo "ERROR: Unknown MPCC_PROFILE=${MPCC_PROFILE}" >&2
        echo "Valid profiles: convergence_debug, tuned_fast, manual" >&2
        exit 1
        ;;
esac

export MPCC_PROFILE
export MPCC_USE_RACELINE_VX_REF="${MPCC_USE_RACELINE_VX_REF:-0}"
export MPCC_USE_RACELINE_VX_LIMIT="${MPCC_USE_RACELINE_VX_LIMIT:-0}"
export MPCC_RACELINE_VX_LIMIT_SCALE="${MPCC_RACELINE_VX_LIMIT_SCALE:-1.0}"
export MPCC_CROSS_CALL_SCALE="${MPCC_CROSS_CALL_SCALE:-${CROSS_CALL_SCALE}}"
export VERBOSE="${VERBOSE:-1}"
export MU="${MU:-${SIM_MU:-0.745}}"
export C_SF="${C_SF:-${SIM_C_SF:-4.297}}"
export C_SR="${C_SR:-${SIM_C_SR:-3.473}}"
export AX_MAX="${AX_MAX:-${SIM_A_MAX:-7.0}}"
export AX_MIN="${AX_MIN:-${SIM_A_MIN:--10.0}}"
export GYM_MAP_PATH="${GYM_MAP_PATH:-${DEFAULT_GYM_MAP_PATH}}"
export GYM_MAP_IMG_EXT="${GYM_MAP_IMG_EXT:-${DEFAULT_GYM_MAP_IMG_EXT}}"
export GYM_SX="${GYM_SX:-${TRAJ_SX:-4.317150}}"
export GYM_SY="${GYM_SY:-${TRAJ_SY:--4.834061}}"
export GYM_STHETA="${GYM_STHETA:-${TRAJ_STHETA:--3.034858}}"

cat > "${CONFIG_LOG}" <<EOF
RUN_ID=${RUN_ID}
MPCC_PROFILE=${MPCC_PROFILE}
MPCC_SOLVER=ADMM+Riccati
TRAJECTORY_FILE=${TRAJECTORY_FILE}
GYM_MAP_PATH=${GYM_MAP_PATH}
GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}
GYM_SX=${GYM_SX}
GYM_SY=${GYM_SY}
GYM_STHETA=${GYM_STHETA}
HORIZON=${HORIZON}
DT=${DT}
Q_CONTOURING=${Q_CONTOURING}
Q_LAG=${Q_LAG}
Q_PROGRESS=${Q_PROGRESS}
Q_WALL_CLEARANCE=${Q_WALL_CLEARANCE:-}
WALL_CLEARANCE_MARGIN=${WALL_CLEARANCE_MARGIN:-}
Q_VX=${Q_VX}
VX_REF=${VX_REF}
MPCC_USE_RACELINE_VX_REF=${MPCC_USE_RACELINE_VX_REF}
MPCC_USE_RACELINE_VX_LIMIT=${MPCC_USE_RACELINE_VX_LIMIT}
MPCC_RACELINE_VX_LIMIT_SCALE=${MPCC_RACELINE_VX_LIMIT_SCALE}
Q_VY=${Q_VY}
Q_OMEGA=${Q_OMEGA}
R_DELTA=${R_DELTA}
R_AX=${R_AX}
R_VTHETA=${R_VTHETA}
W_DELTA_RATE=${W_DELTA_RATE}
W_AX_RATE=${W_AX_RATE}
W_VTHETA_RATE=${W_VTHETA_RATE}
Q_CONTOURING_TERM=${Q_CONTOURING_TERM}
Q_LAG_TERM=${Q_LAG_TERM}
Q_PROGRESS_TERM=${Q_PROGRESS_TERM}
ADMM_RHO=${ADMM_RHO}
ADMM_RHO_U=${ADMM_RHO_U}
ADMM_MAX_ITER=${ADMM_MAX_ITER}
ADMM_TOL=${ADMM_TOL}
ADMM_ADAPTIVE_RHO=${ADMM_ADAPTIVE_RHO}
ADMM_ALPHA_RELAX=${ADMM_ALPHA_RELAX}
V_THETA_MAX=${V_THETA_MAX}
CROSS_CALL_SCALE=${CROSS_CALL_SCALE}
MPCC_CROSS_CALL_SCALE=${MPCC_CROSS_CALL_SCALE}
EOF

echo "Profile: ${MPCC_PROFILE}"
echo "Solver: ADMM+Riccati"
echo "Model sync: MU=${MU} C_SF=${C_SF} C_SR=${C_SR} AX_MAX=${AX_MAX} AX_MIN=${AX_MIN}"
echo "Map override: GYM_MAP_PATH=${GYM_MAP_PATH} GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}"
echo "Spawn override: GYM_SX=${GYM_SX} GYM_SY=${GYM_SY} GYM_STHETA=${GYM_STHETA}"
echo "Run config: ${CONFIG_LOG}"

echo "Launching gym_bridge..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py >"${GYM_LOG}" 2>&1 &
SIM_PID=$!

sleep 4

echo "Launching MPCC with trajectory: ${TRAJECTORY_FILE}"
ros2 launch mpcc_f1_10th mpcc_launch.py trajectory_file:="${TRAJECTORY_FILE}" >"${MPCC_LOG}" 2>&1 &
MPC_PID=$!

#===========================================================================
# Step 3: Monitor for collision / timeout
#===========================================================================
echo "Running for up to ${DURATION_SECONDS}s (stops early on collision)..."
END_TIME=$((SECONDS + DURATION_SECONDS))
COLLISION_SEEN=0
while (( SECONDS < END_TIME )); do
    if grep -q 'Ego vehicle collision detected!' "${GYM_LOG}" 2>/dev/null; then
        COLLISION_SEEN=1
        break
    fi

    if [[ -n "${MPC_PID}" ]] && ! kill -0 "${MPC_PID}" 2>/dev/null; then
        break
    fi

    if [[ -n "${SIM_PID}" ]] && ! kill -0 "${SIM_PID}" 2>/dev/null; then
        break
    fi

    sleep 0.5
done

#===========================================================================
# Step 4: Produce summary
#===========================================================================
LOG_DIR_ENV="${LOG_DIR}" python3 - <<'PY' > "${SUMMARY_LOG}"
import os
import re
from pathlib import Path

log_dir = Path(os.environ["LOG_DIR_ENV"])
gym = (log_dir / "gym_bridge.log").read_text(errors="ignore").splitlines()
mpcc = (log_dir / "mpcc.log").read_text(errors="ignore").splitlines()

def first_line(lines, token):
    for i, line in enumerate(lines, start=1):
        if token in line:
            return i, line
    return None, None

first_collision_line, first_collision_text = first_line(gym, "Ego vehicle collision detected!")
first_status2_line, first_status2_text = first_line(mpcc, "status=2")
if first_status2_line is None:
    first_status2_line, first_status2_text = first_line(mpcc, "status=3")

status2_count = sum(("status=2" in line) or ("status=3" in line) for line in mpcc)
status1_count = sum("status=1" in line for line in mpcc)
status0_count = sum("status=0" in line for line in mpcc)

clip_vals = []
rho_update_vals = []
iter_vals = []
prim_vals = []
dual_vals = []
rho_vals = []
rho_u_vals = []
saturated_delta = 0
saturated_ax = 0
saturated_vtheta0 = 0
saturated_vthetamax = 0
for line in mpcc:
    if "[MPCC] status=" not in line:
        continue
    m_main = re.search(
        r"status=(\d+)\s+iter=(\d+)\s+prim=([0-9.]+)\s+dual=([0-9.]+)\s+rho=([0-9.]+)\s+rho_u=([0-9.]+).*delta=([-.0-9]+)\s+a_x=([-.0-9]+)\s+v_theta=([-.0-9]+)",
        line,
    )
    if m_main:
        iter_vals.append(int(m_main.group(2)))
        prim_vals.append(float(m_main.group(3)))
        dual_vals.append(float(m_main.group(4)))
        rho_vals.append(float(m_main.group(5)))
        rho_u_vals.append(float(m_main.group(6)))
        delta = float(m_main.group(7))
        a_x = float(m_main.group(8))
        v_theta = float(m_main.group(9))
        if abs(abs(delta) - 0.4189) < 0.01:
            saturated_delta += 1
        if abs(abs(a_x) - 7.308) < 0.1:
            saturated_ax += 1
        if abs(v_theta) < 1e-3:
            saturated_vtheta0 += 1
        if abs(v_theta - 10.0) < 0.1:
            saturated_vthetamax += 1
    m_clip = re.search(r"clip=(\d+)", line)
    if m_clip:
        clip_vals.append(int(m_clip.group(1)))
    m_rupd = re.search(r"rho_upd=(\d+)", line)
    if m_rupd:
        rho_update_vals.append(int(m_rupd.group(1)))

avg_clip = (sum(clip_vals) / len(clip_vals)) if clip_vals else 0.0
max_clip = max(clip_vals) if clip_vals else 0
avg_rho_updates = (sum(rho_update_vals) / len(rho_update_vals)) if rho_update_vals else 0.0
max_rho_updates = max(rho_update_vals) if rho_update_vals else 0
avg_iter = (sum(iter_vals) / len(iter_vals)) if iter_vals else 0.0
max_iter = max(iter_vals) if iter_vals else 0
hit_max_iter = sum(1 for v in iter_vals if v >= 300)
avg_prim = (sum(prim_vals) / len(prim_vals)) if prim_vals else 0.0
max_prim = max(prim_vals) if prim_vals else 0.0
avg_dual = (sum(dual_vals) / len(dual_vals)) if dual_vals else 0.0
max_dual = max(dual_vals) if dual_vals else 0.0
avg_rho = (sum(rho_vals) / len(rho_vals)) if rho_vals else 0.0
max_rho = max(rho_vals) if rho_vals else 0.0
avg_rho_u = (sum(rho_u_vals) / len(rho_u_vals)) if rho_u_vals else 0.0
max_rho_u = max(rho_u_vals) if rho_u_vals else 0.0

print(f"variant: MPCC (Contouring Control)")
print(f"first_collision_line: {first_collision_line}")
print(f"first_status2_line: {first_status2_line}")
print(f"status2_count: {status2_count}")
print(f"status1_count: {status1_count}")
print(f"status0_count: {status0_count}")
print(f"avg_iter: {avg_iter:.2f}")
print(f"max_iter: {max_iter}")
print(f"hit_max_iter: {hit_max_iter}")
print(f"avg_prim: {avg_prim:.4f}")
print(f"max_prim: {max_prim:.4f}")
print(f"avg_dual: {avg_dual:.4f}")
print(f"max_dual: {max_dual:.4f}")
print(f"avg_rho: {avg_rho:.3f}")
print(f"max_rho: {max_rho:.3f}")
print(f"avg_rho_u: {avg_rho_u:.3f}")
print(f"max_rho_u: {max_rho_u:.3f}")
print(f"avg_clip: {avg_clip:.2f}")
print(f"max_clip: {max_clip}")
print(f"avg_rho_updates: {avg_rho_updates:.2f}")
print(f"max_rho_updates: {max_rho_updates}")
print(f"saturated_delta_count: {saturated_delta}")
print(f"saturated_ax_count: {saturated_ax}")
print(f"vtheta_zero_count: {saturated_vtheta0}")
print(f"vtheta_max_count: {saturated_vthetamax}")

if first_collision_text:
    print(f"first_collision_text: {first_collision_text}")
if first_status2_text:
    print(f"first_status2_text: {first_status2_text}")
PY

echo "=== Run complete ==="
echo "collision_seen=${COLLISION_SEEN}"
echo "gym_log=${GYM_LOG}"
echo "mpcc_log=${MPCC_LOG}"
echo "summary=${SUMMARY_LOG}"

#===========================================================================
# Step 5: Append to persistent run history
#===========================================================================
HISTORY_FILE="${SCRIPT_DIR}/run_history.md"

# Create header if file doesn't exist
if [[ ! -f "${HISTORY_FILE}" ]]; then
    cat > "${HISTORY_FILE}" <<'HEADER'
# MPCC Simulation Run History

| Date | Duration | Collision | Solver OK | Solver Fail | Notes |
|------|----------|-----------|-----------|-------------|-------|
HEADER
fi

# Extract stats from summary
COLLISION_LINE=$(grep "^first_collision_line:" "${SUMMARY_LOG}" 2>/dev/null | cut -d' ' -f2)
STATUS_OK=$(grep "^status0_count:" "${SUMMARY_LOG}" 2>/dev/null | cut -d' ' -f2)
STATUS_MAX=$(grep "^status1_count:" "${SUMMARY_LOG}" 2>/dev/null | cut -d' ' -f2)
STATUS_FAIL=$(grep "^status2_count:" "${SUMMARY_LOG}" 2>/dev/null | cut -d' ' -f2)

if [[ "${COLLISION_SEEN}" -eq 1 ]]; then
    COLLISION_NOTE="Yes (log line ${COLLISION_LINE:-?})"
else
    COLLISION_NOTE="No"
fi

RUN_ERROR=0
ERROR_NOTE=""
if grep -Eq "Caught exception in launch|Failed to load map" "${GYM_LOG}" 2>/dev/null; then
    RUN_ERROR=1
    ERROR_NOTE="launch_or_map_error"
fi
if [[ "${STATUS_OK:-0}" -eq 0 && "${STATUS_MAX:-0}" -eq 0 && "${STATUS_FAIL:-0}" -eq 0 ]]; then
    RUN_ERROR=1
    if [[ -n "${ERROR_NOTE}" ]]; then
        ERROR_NOTE="${ERROR_NOTE},no_solver_samples"
    else
        ERROR_NOTE="no_solver_samples"
    fi
fi

NOTES="[log](logs/run_${RUN_ID}/) st1=${STATUS_MAX:-0}"
if [[ "${RUN_ERROR}" -eq 1 ]]; then
    NOTES="${NOTES} err=${ERROR_NOTE}"
fi

echo "| ${RUN_ID} | ${DURATION_SECONDS}s | ${COLLISION_NOTE} | ${STATUS_OK:-0} | ${STATUS_FAIL:-0} | ${NOTES} |" >> "${HISTORY_FILE}"

echo "History updated: ${HISTORY_FILE}"

if [[ "${RUN_ERROR}" -eq 1 ]]; then
    echo "Run error detected: ${ERROR_NOTE}" >&2
    exit 1
fi
