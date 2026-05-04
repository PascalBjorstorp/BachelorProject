#!/usr/bin/env bash
#===========================================================================
# MPCC/run_sim_2.sh — Launch MPCC (Contouring Control) with F1/10th simulator
#
# Usage:
#   ./MPCC/run_sim_2.sh [DURATION_SECONDS] [TRAJECTORY_FILE]
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = f1tenth_planning/trajectories/my_track_centerline_smooth.csv
#   MPCC_PROFILE     = track_racer
#
# This script:
#   1. Rebuilds f1tenth_gym_ros and mpcc_f1_10th via colcon
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

# Resolve to an absolute path so ROS2 nodes can find the trajectory regardless of cwd.
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
MPCC_PID=""

cleanup() {
    local had_errexit=0
    [[ $- == *e* ]] && had_errexit=1
    set +e
    if [[ -n "${MPCC_PID}" ]] && kill -0 "${MPCC_PID}" 2>/dev/null; then
        kill "${MPCC_PID}" 2>/dev/null || true
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
    sleep 0.5
    if [[ "${had_errexit}" -eq 1 ]]; then
        set -e
    fi
}
trap cleanup EXIT

if [[ ! -f "${TRAJECTORY_FILE}" ]]; then
    echo "ERROR: Trajectory file not found: ${TRAJECTORY_FILE}" >&2
    exit 1
fi

echo "=== MPCC (Contouring Control) Simulation Run ==="
echo "Logs: ${LOG_DIR}"
echo "Duration: ${DURATION_SECONDS}s"
echo "Trajectory: ${TRAJECTORY_FILE}"

#===========================================================================
# Step 1: Build the ROS2 packages
#===========================================================================
echo ""
echo "--- Building f1tenth_gym_ros + mpcc_f1_10th ROS2 packages ---"

echo "Cleaning up stale MPCC/gym/RViz processes from previous runs..."
cleanup

cd "${ROOT_DIR}"

# Source ROS2 base
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Build both packages that this script launches. The gym package owns the
# installed launch/RViz files, so it must be rebuilt when visualization changes.
colcon build --packages-select f1tenth_gym_ros mpcc_f1_10th --cmake-force-configure 2>&1 | tail -8

# Re-source the workspace after build
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

if ! ros2 pkg prefix mpcc_f1_10th >/dev/null 2>&1; then
    echo "ERROR: 'mpcc_f1_10th' is not discoverable after build/source." >&2
    echo "This usually means CMake configured without ROS dependencies in cache." >&2
    echo "Try removing the package build cache and rebuilding:" >&2
    echo "  rm -rf ${ROOT_DIR}/build/mpcc_f1_10th ${ROOT_DIR}/install/mpcc_f1_10th ${ROOT_DIR}/log/latest_build/mpcc_f1_10th" >&2
    echo "  source /opt/ros/jazzy/setup.bash" >&2
    echo "  colcon build --packages-select mpcc_f1_10th --cmake-force-configure" >&2
    exit 1
fi

echo "  Build complete."

#===========================================================================
# Step 3: Launch simulation
#===========================================================================
echo ""
echo "--- Launching simulation ---"

PYTHONPATH_EXTRA="${ROOT_DIR}/f1tenth_sim"
for venv_bin in "${ROOT_DIR}/.venv/bin" "${ROOT_DIR}/f1tenth_sim/.venv/bin"; do
    if [[ -d "${venv_bin}" ]]; then
        export PATH="${venv_bin}:${PATH}"
    fi
done
for site_dir in "${ROOT_DIR}"/.venv/lib/python*/site-packages "${ROOT_DIR}"/f1tenth_sim/.venv/lib/python*/site-packages; do
    if [[ -d "${site_dir}" ]]; then
        PYTHONPATH_EXTRA="${PYTHONPATH_EXTRA}:${site_dir}"
    fi
done
export PYTHONPATH="${PYTHONPATH_EXTRA}:${PYTHONPATH:-}"

# Keep the gym spawn and map aligned with the selected trajectory. This prevents
# the simulator from starting the car at a pose that is far away from MPCC's path.
TRAJ_SX="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $2); print $2; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"
TRAJ_SY="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $3); print $3; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"
TRAJ_STHETA="$(awk -F, 'NR==2 {gsub(/[[:space:]]/, "", $4); print $4; exit}' "${TRAJECTORY_FILE}" 2>/dev/null || true)"

DEFAULT_GYM_MAP_PATH="my_track_map"
DEFAULT_GYM_MAP_IMG_EXT=".pgm"
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

export GYM_MAP_PATH="${GYM_MAP_PATH:-${DEFAULT_GYM_MAP_PATH}}"
export GYM_MAP_IMG_EXT="${GYM_MAP_IMG_EXT:-${DEFAULT_GYM_MAP_IMG_EXT}}"
export GYM_SX="${GYM_SX:-${TRAJ_SX:--0.79}}"
export GYM_SY="${GYM_SY:-${TRAJ_SY:--4.88}}"
export GYM_STHETA="${GYM_STHETA:-${TRAJ_STHETA:-0.67}}"
export GYM_USE_RVIZ="${GYM_USE_RVIZ:-true}"

MPCC_PROFILE="${MPCC_PROFILE:-track_racer}"

set_default() {
    local var_name="$1"
    local default_value="$2"
    export "${var_name}=${default_value}"
}

set_manual_default() {
    local var_name="$1"
    local default_value="$2"
    if [[ -z "${!var_name+x}" ]]; then
        export "${var_name}=${default_value}"
    fi
}

case "${MPCC_PROFILE}" in
    track_racer)
        # Low-reference MPCC setup:
        # The reference line is used mostly as the path coordinate frame and
        # corridor center, while progress and wall clearance dominate.
        set_default HORIZON 40
        set_default DT 0.03
        set_default Q_CONTOURING 60.0
        set_default Q_LAG 120.0
        set_default Q_WALL_CLEARANCE 3200.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default MPCC_TRACK_BUFFER 0.05
        set_default Q_PROGRESS 8.0
        set_default Q_VX 0.0
        set_default VX_REF 0.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 1.0
        set_default Q_VY 1.0
        set_default Q_OMEGA 3.0
        set_default R_DELTA 8.0
        set_default R_AX 1.0
        set_default R_VTHETA 0.2
        set_default W_DELTA_RATE 2.0
        set_default W_AX_RATE 3.0
        set_default W_VTHETA_RATE 0.5
        set_default Q_CONTOURING_TERM 75.0
        set_default Q_LAG_TERM 60.0
        set_default Q_PROGRESS_TERM 10.0
        set_default ADMM_RHO 15.0
        set_default ADMM_RHO_U 8.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default MPCC_ACCEPT_MAX_ITER 0
        set_default MPCC_MAX_ITER_PRIMAL_TOL 0.01
        set_default MPCC_MAX_ITER_DUAL_TOL 0.01
        set_default MPCC_MAX_ITER_TRACK_TOL 0.005
        set_default V_THETA_MAX 8.0
        set_default CROSS_CALL_SCALE 0.166667
        ;;
    convergence_debug)
        set_default HORIZON 20
        set_default DT 0.03
        set_default Q_CONTOURING 60.0
        set_default Q_LAG 120.0
        set_default Q_WALL_CLEARANCE 6000.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default Q_PROGRESS 15.0
        set_default Q_VX 0.0
        set_default VX_REF 0.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 0.0
        set_default Q_VY 0.5
        set_default Q_OMEGA 1.5
        set_default R_DELTA 150.0
        set_default R_AX 0.05225
        set_default R_VTHETA 0.1
        set_default W_DELTA_RATE 8.0
        set_default W_AX_RATE 0.488
        set_default W_VTHETA_RATE 0.1105
        set_default Q_CONTOURING_TERM 800.0
        set_default Q_LAG_TERM 400.0
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
        set_default Q_WALL_CLEARANCE 3200.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default Q_PROGRESS 15.6
        set_default Q_VX 30.0
        set_default VX_REF 4.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 1.0
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
        set_manual_default HORIZON 20
        set_manual_default DT 0.03
        set_manual_default Q_CONTOURING 80.0
        set_manual_default Q_LAG 120.0
        set_manual_default Q_WALL_CLEARANCE 3200.0
        set_manual_default WALL_CLEARANCE_MARGIN 0.02
        set_manual_default Q_PROGRESS 10.0
        set_manual_default Q_VX 10.0
        set_manual_default VX_REF 4.0
        set_manual_default MPCC_USE_RACELINE_VX_REF 0
        set_manual_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_manual_default MPCC_RACELINE_VX_LIMIT_SCALE 1.0
        set_manual_default Q_VY 0.5
        set_manual_default Q_OMEGA 1.5
        set_manual_default R_DELTA 150.0
        set_manual_default R_AX 0.05225
        set_manual_default R_VTHETA 0.1
        set_manual_default W_DELTA_RATE 8.0
        set_manual_default W_AX_RATE 0.488
        set_manual_default W_VTHETA_RATE 0.1105
        set_manual_default Q_CONTOURING_TERM 600.0
        set_manual_default Q_LAG_TERM 200.0
        set_manual_default Q_PROGRESS_TERM 30.0
        set_manual_default ADMM_RHO 5.0
        set_manual_default ADMM_RHO_U 0.0
        set_manual_default ADMM_MAX_ITER 300
        set_manual_default ADMM_TOL 0.02
        set_manual_default ADMM_ADAPTIVE_RHO 1
        set_manual_default ADMM_ALPHA_RELAX 1.6
        set_manual_default V_THETA_MAX 6.0
        set_manual_default CROSS_CALL_SCALE 0.166667
        ;;
    *)
        echo "ERROR: Unknown MPCC_PROFILE=${MPCC_PROFILE}" >&2
        echo "Valid profiles: track_racer, convergence_debug, tuned_fast, manual" >&2
        exit 1
        ;;
esac

export MPCC_PROFILE
export MPCC_CROSS_CALL_SCALE="${MPCC_CROSS_CALL_SCALE:-${CROSS_CALL_SCALE}}"
export MPCC_TRACK_BUFFER="${MPCC_TRACK_BUFFER:-0.0}"
export MPCC_ACCEPT_MAX_ITER="${MPCC_ACCEPT_MAX_ITER:-0}"
export MPCC_MAX_ITER_PRIMAL_TOL="${MPCC_MAX_ITER_PRIMAL_TOL:-0.01}"
export MPCC_MAX_ITER_DUAL_TOL="${MPCC_MAX_ITER_DUAL_TOL:-0.01}"
export MPCC_MAX_ITER_TRACK_TOL="${MPCC_MAX_ITER_TRACK_TOL:-0.005}"

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
GYM_USE_RVIZ=${GYM_USE_RVIZ}
HORIZON=${HORIZON}
DT=${DT}
Q_CONTOURING=${Q_CONTOURING}
Q_LAG=${Q_LAG}
Q_WALL_CLEARANCE=${Q_WALL_CLEARANCE}
WALL_CLEARANCE_MARGIN=${WALL_CLEARANCE_MARGIN}
MPCC_TRACK_BUFFER=${MPCC_TRACK_BUFFER}
Q_PROGRESS=${Q_PROGRESS}
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
MPCC_ACCEPT_MAX_ITER=${MPCC_ACCEPT_MAX_ITER}
MPCC_MAX_ITER_PRIMAL_TOL=${MPCC_MAX_ITER_PRIMAL_TOL}
MPCC_MAX_ITER_DUAL_TOL=${MPCC_MAX_ITER_DUAL_TOL}
MPCC_MAX_ITER_TRACK_TOL=${MPCC_MAX_ITER_TRACK_TOL}
V_THETA_MAX=${V_THETA_MAX}
CROSS_CALL_SCALE=${CROSS_CALL_SCALE}
MPCC_CROSS_CALL_SCALE=${MPCC_CROSS_CALL_SCALE}
EOF

echo "Profile: ${MPCC_PROFILE}"
echo "Solver: ADMM+Riccati"
echo "Map override: GYM_MAP_PATH=${GYM_MAP_PATH} GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}"
echo "Spawn override: GYM_SX=${GYM_SX} GYM_SY=${GYM_SY} GYM_STHETA=${GYM_STHETA}"
echo "RViz: GYM_USE_RVIZ=${GYM_USE_RVIZ}"
echo "Run config: ${CONFIG_LOG}"

echo "Launching gym_bridge..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py use_rviz:="${GYM_USE_RVIZ}" >"${GYM_LOG}" 2>&1 &
SIM_PID=$!

sleep 10

if [[ -n "${SIM_PID}" ]] && ! kill -0 "${SIM_PID}" 2>/dev/null; then
    echo "ERROR: gym_bridge launch exited during startup. See ${GYM_LOG}" >&2
    tail -40 "${GYM_LOG}" >&2 || true
    exit 1
fi

echo "Launching MPCC with trajectory: ${TRAJECTORY_FILE}"
ros2 launch mpcc_f1_10th mpcc_launch.py "trajectory_file:=${TRAJECTORY_FILE}" >"${MPCC_LOG}" 2>&1 &
MPCC_PID=$!

#===========================================================================
# Step 4: Monitor for collision / timeout
#===========================================================================
echo "Running for up to ${DURATION_SECONDS}s (stops early on collision)..."
END_TIME=$((SECONDS + DURATION_SECONDS))
COLLISION_SEEN=0
while [ "${SECONDS}" -lt "${END_TIME}" ]; do
    if grep -q 'Ego vehicle collision detected!' "${GYM_LOG}" 2>/dev/null; then
        COLLISION_SEEN=1
        break
    fi

    if [[ -n "${MPCC_PID}" ]] && ! kill -0 "${MPCC_PID}" 2>/dev/null; then
        break
    fi

    if [[ -n "${SIM_PID}" ]] && ! kill -0 "${SIM_PID}" 2>/dev/null; then
        break
    fi

    sleep 0.5
done

#===========================================================================
# Step 5: Produce summary
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
rviz_lines = gym
first_rviz_crash_line, first_rviz_crash_text = first_line(rviz_lines, "cannot store a negative time point in rclcpp::Time")
if first_rviz_crash_line is None:
    first_rviz_crash_line, first_rviz_crash_text = first_line(rviz_lines, "process has died")
if first_rviz_crash_line is None:
    first_rviz_crash_line, first_rviz_crash_text = first_line(rviz_lines, "GLSL link result")
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
print(f"first_rviz_crash_line: {first_rviz_crash_line}")
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
if first_rviz_crash_text:
    print(f"first_rviz_crash_text: {first_rviz_crash_text}")
if first_status2_text:
    print(f"first_status2_text: {first_status2_text}")
PY

echo "=== Run complete ==="
echo "collision_seen=${COLLISION_SEEN}"
echo "gym_log=${GYM_LOG}"
echo "mpcc_log=${MPCC_LOG}"
echo "summary=${SUMMARY_LOG}"
