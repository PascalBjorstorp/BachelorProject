#!/usr/bin/env bash
#===========================================================================
# MPCC/run_sim.sh — Launch MPCC (Contouring Control) with F1/10th simulator
#
# Usage:
#   ./MPCC/run_sim.sh 120 f1tenth_planning/trajectories/my_track_raceline.csv
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = f1tenth_planning/trajectories/my_track_raceline.csv
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
TRAJECTORY_FILE="${2:-${ROOT_DIR}/f1tenth_planning/trajectories/my_track_raceline.csv}"

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

# Match tune_mpcc.py BASE_CONFIG so ROS run is comparable.
export HORIZON="${HORIZON:-5}"
export DT="${DT:-0.02205}"
export Q_CONTOURING="${Q_CONTOURING:-3070.625}"
export Q_LAG="${Q_LAG:-549.932291}"
export Q_PROGRESS="${Q_PROGRESS:-12.31296}"
export Q_VX="${Q_VX:-0.0}"
export VX_REF="${VX_REF:-4.08}"
export Q_VY="${Q_VY:-4.41}"
export Q_OMEGA="${Q_OMEGA:-0.194481}"
export R_DELTA="${R_DELTA:-13.5}"
export R_AX="${R_AX:-0.013716}"
export R_VTHETA="${R_VTHETA:-1.1232}"
export W_DELTA_RATE="${W_DELTA_RATE:-0.9405}"
export W_AX_RATE="${W_AX_RATE:-0.101821}"
export W_VTHETA_RATE="${W_VTHETA_RATE:-0.126}"
export Q_CONTOURING_TERM="${Q_CONTOURING_TERM:-493.7625}"
export Q_LAG_TERM="${Q_LAG_TERM:-1140.0}"
export Q_PROGRESS_TERM="${Q_PROGRESS_TERM:-5.564503}"
export ADMM_RHO="${ADMM_RHO:-53.0}"
export ADMM_MAX_ITER="${ADMM_MAX_ITER:-30}"
export ADMM_TOL="${ADMM_TOL:-0.0605}"
export V_THETA_MAX="${V_THETA_MAX:-7.176}"
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

echo "Model sync: MU=${MU} C_SF=${C_SF} C_SR=${C_SR} AX_MAX=${AX_MAX} AX_MIN=${AX_MIN}"
echo "Map override: GYM_MAP_PATH=${GYM_MAP_PATH} GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}"
echo "Spawn override: GYM_SX=${GYM_SX} GYM_SY=${GYM_SY} GYM_STHETA=${GYM_STHETA}"

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
for line in mpcc:
    if "[MPCC] status=" not in line:
        continue
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

print(f"variant: MPCC (Contouring Control)")
print(f"first_collision_line: {first_collision_line}")
print(f"first_status2_line: {first_status2_line}")
print(f"status2_count: {status2_count}")
print(f"status1_count: {status1_count}")
print(f"status0_count: {status0_count}")
print(f"avg_clip: {avg_clip:.2f}")
print(f"max_clip: {max_clip}")
print(f"avg_rho_updates: {avg_rho_updates:.2f}")
print(f"max_rho_updates: {max_rho_updates}")

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
