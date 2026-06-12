#!/usr/bin/env bash
#===========================================================================
# MPCC/run_sim_2.sh — Launch MPCC (Contouring Control) with F1/10th simulator
#
# Usage:
#   ./MPCC/run_sim_2.sh [DURATION_SECONDS] [TRAJECTORY_FILE]
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = f1tenth_planning/trajectories/my_track_centerline.csv
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
DEFAULT_TRAJECTORY_FILE="${ROOT_DIR}/f1tenth_planning/trajectories/my_track_centerline.csv"
TRAJECTORY_FILE="${2:-${DEFAULT_TRAJECTORY_FILE}}"

# Resolve to an absolute path so ROS2 nodes can find the trajectory regardless of cwd.
if [[ "${TRAJECTORY_FILE}" != /* ]]; then
    TRAJECTORY_FILE="$(cd "$(dirname "${TRAJECTORY_FILE}")" && pwd)/$(basename "${TRAJECTORY_FILE}")"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/logs/run_${RUN_ID}"
mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_DIR}/ros}"
mkdir -p "${ROS_LOG_DIR}"

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

# Copy maps from f1tenth_planning to f1tenth_sim so they get installed with the package
echo "Syncing maps from f1tenth_planning/maps to f1tenth_sim/maps..."
if [[ -d "${ROOT_DIR}/f1tenth_planning/maps" ]]; then
    mkdir -p "${ROOT_DIR}/f1tenth_sim/maps"
    cp -v "${ROOT_DIR}/f1tenth_planning/maps"/*.pgm "${ROOT_DIR}/f1tenth_sim/maps/" 2>/dev/null || true
    cp -v "${ROOT_DIR}/f1tenth_planning/maps"/*.png "${ROOT_DIR}/f1tenth_sim/maps/" 2>/dev/null || true
    cp -v "${ROOT_DIR}/f1tenth_planning/maps"/*.yaml "${ROOT_DIR}/f1tenth_sim/maps/" 2>/dev/null || true
fi

# Clean build directory for gym package to force complete rebuild and reinstall of maps
echo "Cleaning f1tenth_gym_ros build/install to force fresh rebuild with latest maps..."
rm -rf "${ROOT_DIR}/build/f1tenth_gym_ros" "${ROOT_DIR}/install/f1tenth_gym_ros"

# Build both packages that this script launches. The gym package owns the
# installed launch/RViz files, so it must be rebuilt when visualization changes.
colcon build --packages-select f1tenth_gym_ros mpcc_f1_10th --cmake-force-configure 2>&1 | tail -15

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

# Keep the gym spawn and map aligned with the selected trajectory. For MPCC's
# 9-column CSVs, pick the waypoint with the highest curvature-based speed limit
# within the next 7m (so we don't spawn inside or just before a hairpin) with
# a minimum clearance of 0.5m. Falls back to max-clearance if Python fails.
_PICK_SPAWN_PY="${ROOT_DIR}/MPCC/pick_spawn.py"
TRAJ_SPAWN="$(python3 "${_PICK_SPAWN_PY}" "${TRAJECTORY_FILE}" 2>/dev/null || \
    awk -F, 'function t(v){gsub(/^[[:space:]]+|[[:space:]]+$/,"",v);return v}
    $1~/^[[:space:]]*#/||NF<4{next}
    {x=t($2);y=t($3);p=t($4);if(!h){fx=x;fy=y;fp=p;h=1}
     if(NF>=9){l=t($8)+0;r=t($9)+0;c=(l<r)?l:r
       if(!b||c>bc){bc=c;bx=x;by=y;bp=p;b=1}}}
    END{if(b)printf"%s %s %s\n",bx,by,bp;else if(h)printf"%s %s %s\n",fx,fy,fp}
    ' "${TRAJECTORY_FILE}" 2>/dev/null)"
TRAJ_SX=""
TRAJ_SY=""
TRAJ_STHETA=""
if [[ -n "${TRAJ_SPAWN}" ]]; then
    read -r TRAJ_SX TRAJ_SY TRAJ_STHETA <<< "${TRAJ_SPAWN}"
fi

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
    if [[ -z "${!var_name+x}" ]]; then
        export "${var_name}=${default_value}"
    fi
}

case "${MPCC_PROFILE}" in
    track_racer)
        # Stable faster no-ref-speed profile:
        # keep the soft wall guard, extend lookahead modestly, and tighten
        # virtual-to-physical progress consistency so higher progress reward
        # does not over-commit in the narrow s≈22.6 section.
        set_default HORIZON 60
        set_default DT 0.025
        set_default Q_CONTOURING 8.0
        set_default Q_LAG 13.4
        set_default Q_HEADING 5.8
        set_default Q_WALL_CLEARANCE 450.0
        set_default WALL_CLEARANCE_MARGIN 0.12
        set_default MPCC_TRACK_BUFFER 0.00
        set_default Q_PROGRESS 0.9
        set_default Q_PHYSICAL_PROGRESS 1.78
        set_default Q_VX 0.0
        set_default VX_REF 0.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 0.0
        set_default Q_VY 0.0
        set_default Q_OMEGA 1.0
        set_default R_DELTA 0.95
        set_default R_AX 0.42
        set_default AX_MAX 10.25
        set_default AX_MIN -10.25
        set_default MPCC_AX_MIN_HARDWARE -10.25
        set_default R_VTHETA 0.09
        set_default W_DELTA_RATE 54.0
        set_default W_AX_RATE 3.6
        set_default W_VTHETA_RATE 0.42
        set_default Q_CONTOURING_TERM 100.0
        set_default Q_LAG_TERM 50.0
        set_default Q_HEADING_TERM 18.0
        set_default Q_PROGRESS_TERM 32.0
        set_default ADMM_RHO 60.0
        set_default ADMM_RHO_U 4.0
        set_default ADMM_MAX_ITER 125
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.0
        set_default MPCC_ACCEPT_MAX_ITER 1
        set_default MPCC_MAX_ITER_PRIMAL_TOL 0.04
        set_default MPCC_MAX_ITER_DUAL_TOL 0.04
        set_default MPCC_MAX_ITER_TRACK_TOL 0.05
        set_default V_THETA_MAX 20.0
        set_default DELTA_RATE_MAX 2.849
        set_default CROSS_CALL_SCALE 1.0
        ;;
    convergence_debug)
        set_default HORIZON 20
        set_default DT 0.03
        set_default Q_CONTOURING 60.0
        set_default Q_LAG 120.0
        set_default Q_HEADING 8.0
        set_default Q_WALL_CLEARANCE 6000.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default Q_PROGRESS 15.0
        set_default Q_PHYSICAL_PROGRESS 1.0
        set_default Q_VX 0.0
        set_default VX_REF 0.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 0.0
        set_default Q_VY 0.5
        set_default Q_OMEGA 0.0
        set_default R_DELTA 150.0
        set_default R_AX 0.05225
        set_default R_VTHETA 0.1
        set_default W_DELTA_RATE 8.0
        set_default W_AX_RATE 0.488
        set_default W_VTHETA_RATE 0.1105
        set_default Q_CONTOURING_TERM 800.0
        set_default Q_LAG_TERM 400.0
        set_default Q_HEADING_TERM 20.0
        set_default Q_PROGRESS_TERM 30.0
        set_default ADMM_RHO 5.0
        set_default ADMM_RHO_U 0.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default V_THETA_MAX 6.0
        set_default DELTA_RATE_MAX 2.849
        set_default CROSS_CALL_SCALE 0.166667
        ;;
    tuned_fast)
        set_default HORIZON 20
        set_default DT 0.03
        set_default Q_CONTOURING 960.0
        set_default Q_LAG 200.0
        set_default Q_HEADING 8.0
        set_default Q_WALL_CLEARANCE 3200.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default Q_PROGRESS 15.6
        set_default Q_PHYSICAL_PROGRESS 1.0
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
        set_default Q_HEADING_TERM 20.0
        set_default Q_PROGRESS_TERM 41.4
        set_default ADMM_RHO 5.0
        set_default ADMM_RHO_U 0.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default V_THETA_MAX 10.0
        set_default DELTA_RATE_MAX 2.849
        set_default CROSS_CALL_SCALE 0.166667
        ;;
    manual)
        set_default HORIZON 20
        set_default DT 0.03
        set_default Q_CONTOURING 80.0
        set_default Q_LAG 120.0
        set_default Q_HEADING 8.0
        set_default Q_WALL_CLEARANCE 3200.0
        set_default WALL_CLEARANCE_MARGIN 0.02
        set_default Q_PROGRESS 10.0
        set_default Q_PHYSICAL_PROGRESS 1.0
        set_default Q_VX 10.0
        set_default VX_REF 4.0
        set_default MPCC_USE_RACELINE_VX_REF 0
        set_default MPCC_USE_RACELINE_VX_LIMIT 0
        set_default MPCC_RACELINE_VX_LIMIT_SCALE 1.0
        set_default Q_VY 0.5
        set_default Q_OMEGA 1.5
        set_default R_DELTA 10.0
        set_default R_AX 0.05225
        set_default R_VTHETA 0.1
        set_default W_DELTA_RATE 8.0
        set_default W_AX_RATE 0.488
        set_default W_VTHETA_RATE 0.1105
        set_default Q_CONTOURING_TERM 600.0
        set_default Q_LAG_TERM 200.0
        set_default Q_HEADING_TERM 20.0
        set_default Q_PROGRESS_TERM 30.0
        set_default ADMM_RHO 5.0
        set_default ADMM_RHO_U 0.0
        set_default ADMM_MAX_ITER 300
        set_default ADMM_TOL 0.02
        set_default ADMM_ADAPTIVE_RHO 1
        set_default ADMM_ALPHA_RELAX 1.6
        set_default V_THETA_MAX 6.0
        set_default DELTA_RATE_MAX 2.849
        set_default CROSS_CALL_SCALE 0.166
        ;;
    *)
        echo "ERROR: Unknown MPCC_PROFILE=${MPCC_PROFILE}" >&2
        echo "Valid profiles: track_racer, convergence_debug, tuned_fast, manual" >&2
        exit 1
        ;;
esac

export MPCC_PROFILE
export MPCC_CROSS_CALL_SCALE="${MPCC_CROSS_CALL_SCALE:-${CROSS_CALL_SCALE}}"
export MPCC_CONTROL_PERIOD_MS="${MPCC_CONTROL_PERIOD_MS:-25}"
export MPCC_TRACK_BUFFER="${MPCC_TRACK_BUFFER:-0.0}"
export MPCC_ACCEPT_MAX_ITER="${MPCC_ACCEPT_MAX_ITER:-1}"
export MPCC_MAX_ITER_PRIMAL_TOL="${MPCC_MAX_ITER_PRIMAL_TOL:-0.04}"
export MPCC_MAX_ITER_DUAL_TOL="${MPCC_MAX_ITER_DUAL_TOL:-0.04}"
export MPCC_MAX_ITER_TRACK_TOL="${MPCC_MAX_ITER_TRACK_TOL:-0.05}"
export MPCC_S_QP_WINDOW="${MPCC_S_QP_WINDOW:-1.0}"
export W_VTHETA_PHYSICAL="${W_VTHETA_PHYSICAL:-130.0}"
export MPCC_WARM_START_MAX_S_ERROR="${MPCC_WARM_START_MAX_S_ERROR:-1.5}"
export DELTA_RATE_MAX="${DELTA_RATE_MAX:-2.849}"

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
Q_HEADING=${Q_HEADING}
Q_WALL_CLEARANCE=${Q_WALL_CLEARANCE}
WALL_CLEARANCE_MARGIN=${WALL_CLEARANCE_MARGIN}
MPCC_TRACK_BUFFER=${MPCC_TRACK_BUFFER}
Q_PROGRESS=${Q_PROGRESS}
Q_PHYSICAL_PROGRESS=${Q_PHYSICAL_PROGRESS}
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
Q_HEADING_TERM=${Q_HEADING_TERM}
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
MPCC_S_QP_WINDOW=${MPCC_S_QP_WINDOW}
W_VTHETA_PHYSICAL=${W_VTHETA_PHYSICAL}
MPCC_WARM_START_MAX_S_ERROR=${MPCC_WARM_START_MAX_S_ERROR}
MPCC_CONTROL_PERIOD_MS=${MPCC_CONTROL_PERIOD_MS}
ROS_LOG_DIR=${ROS_LOG_DIR}
V_THETA_MAX=${V_THETA_MAX}
DELTA_RATE_MAX=${DELTA_RATE_MAX}
CROSS_CALL_SCALE=${CROSS_CALL_SCALE}
MPCC_CROSS_CALL_SCALE=${MPCC_CROSS_CALL_SCALE}
EOF

echo "Profile: ${MPCC_PROFILE}"
echo "Solver: ADMM+Riccati"
echo "Map override: GYM_MAP_PATH=${GYM_MAP_PATH} GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}"
echo "Spawn override: GYM_SX=${GYM_SX} GYM_SY=${GYM_SY} GYM_STHETA=${GYM_STHETA}"
echo "RViz: GYM_USE_RVIZ=${GYM_USE_RVIZ}"
echo "Control period: MPCC_CONTROL_PERIOD_MS=${MPCC_CONTROL_PERIOD_MS}"
echo "Run config: ${CONFIG_LOG}"

echo "Launching gym_bridge..."
echo "  Map: ${GYM_MAP_PATH}${GYM_MAP_IMG_EXT}"
echo "  Spawn: x=${GYM_SX} y=${GYM_SY} theta=${GYM_STHETA}"
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
LOG_DIR_ENV="${LOG_DIR}" ADMM_MAX_ITER_ENV="${ADMM_MAX_ITER}" python3 - <<'PY' > "${SUMMARY_LOG}"
import os
import re
from pathlib import Path

log_dir = Path(os.environ["LOG_DIR_ENV"])
gym = (log_dir / "gym_bridge.log").read_text(errors="ignore").splitlines()
mpcc = (log_dir / "mpcc.log").read_text(errors="ignore").splitlines()
iteration_limit = int(os.environ.get("ADMM_MAX_ITER_ENV", "300"))

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

clip_vals = []
rho_update_vals = []
rho_state_update_vals = []
rho_control_update_vals = []
iter_vals = []
prim_vals = []
dual_vals = []
rho_vals = []
rho_u_vals = []
solve_gap_ms_vals = []
solve_rate_hz_vals = []
target_ms_vals = []
compute_ms_vals = []
compute_hz_vals = []
solve_samples = 0
status0_count = 0
status1_count = 0
status2_count = 0
min_rho = None
min_rho_u = None
last_rho = 0.0
last_rho_u = 0.0
last_rho_updates = 0
saturated_delta = 0
saturated_ax = 0
saturated_vtheta0 = 0
saturated_vthetamax = 0
for line in mpcc:
    if "[MPCC] solve=" not in line:
        continue
    m_main = re.search(
        r"solve=(\d+)\s+status=(\d+)\s+iter=(\d+)\s+prim=([-.0-9]+)\s+dual=([-.0-9]+)\s+rho=([-.0-9]+)\s+rho_u=([-.0-9]+)\s+rho_upd=(\d+)\s+clip=(\d+).*delta=([-.0-9]+)\s+a_x=([-.0-9]+)\s+v_theta=([-.0-9]+)\s+solve_gap_ms=([-.0-9]+)\s+solve_rate_hz=([-.0-9]+)\s+target_ms=([-.0-9]+)",
        line,
    )
    if m_main:
        status = int(m_main.group(2))
        rho = float(m_main.group(6))
        rho_u = float(m_main.group(7))
        rho_upd = int(m_main.group(8))
        clip = int(m_main.group(9))
        delta = float(m_main.group(10))
        a_x = float(m_main.group(11))
        v_theta = float(m_main.group(12))
        solve_gap_ms = float(m_main.group(13))
        solve_rate_hz = float(m_main.group(14))
        target_ms = float(m_main.group(15))
        m_compute = re.search(r"compute_ms=([-.0-9]+)\s+compute_hz=([-.0-9]+)", line)

        solve_samples += 1
        if status in (2, 3):
            status2_count += 1
        elif status == 1:
            status1_count += 1
        elif status == 0:
            status0_count += 1

        iter_vals.append(int(m_main.group(3)))
        prim_vals.append(float(m_main.group(4)))
        dual_vals.append(float(m_main.group(5)))
        rho_vals.append(rho)
        rho_u_vals.append(rho_u)
        clip_vals.append(clip)
        rho_update_vals.append(rho_upd)
        m_split_rho = re.search(r"rho_x_upd=(\d+)\s+rho_u_upd=(\d+)", line)
        if m_split_rho:
            rho_state_update_vals.append(int(m_split_rho.group(1)))
            rho_control_update_vals.append(int(m_split_rho.group(2)))
        last_rho = rho
        last_rho_u = rho_u
        last_rho_updates = rho_upd
        if min_rho is None or rho < min_rho:
            min_rho = rho
        if min_rho_u is None or rho_u < min_rho_u:
            min_rho_u = rho_u
        if solve_gap_ms > 0.0:
            solve_gap_ms_vals.append(solve_gap_ms)
        if solve_rate_hz > 0.0:
            solve_rate_hz_vals.append(solve_rate_hz)
        if target_ms > 0.0:
            target_ms_vals.append(target_ms)
        if m_compute:
            compute_ms_vals.append(float(m_compute.group(1)))
            compute_hz_vals.append(float(m_compute.group(2)))

        if abs(abs(delta) - 0.4189) < 0.01:
            saturated_delta += 1
        if abs(abs(a_x) - 7.308) < 0.1:
            saturated_ax += 1
        if abs(v_theta) < 1e-3:
            saturated_vtheta0 += 1
        if abs(v_theta - 10.0) < 0.1:
            saturated_vthetamax += 1

avg_clip = (sum(clip_vals) / len(clip_vals)) if clip_vals else 0.0
max_clip = max(clip_vals) if clip_vals else 0
avg_rho_updates = (sum(rho_update_vals) / len(rho_update_vals)) if rho_update_vals else 0.0
max_rho_updates = max(rho_update_vals) if rho_update_vals else 0
avg_rho_state_updates = (sum(rho_state_update_vals) / len(rho_state_update_vals)) if rho_state_update_vals else 0.0
max_rho_state_updates = max(rho_state_update_vals) if rho_state_update_vals else 0
avg_rho_control_updates = (sum(rho_control_update_vals) / len(rho_control_update_vals)) if rho_control_update_vals else 0.0
max_rho_control_updates = max(rho_control_update_vals) if rho_control_update_vals else 0
avg_iter = (sum(iter_vals) / len(iter_vals)) if iter_vals else 0.0
max_iter = max(iter_vals) if iter_vals else 0
hit_max_iter = sum(1 for v in iter_vals if v >= iteration_limit)
avg_prim = (sum(prim_vals) / len(prim_vals)) if prim_vals else 0.0
max_prim = max(prim_vals) if prim_vals else 0.0
avg_dual = (sum(dual_vals) / len(dual_vals)) if dual_vals else 0.0
max_dual = max(dual_vals) if dual_vals else 0.0
avg_rho = (sum(rho_vals) / len(rho_vals)) if rho_vals else 0.0
max_rho = max(rho_vals) if rho_vals else 0.0
avg_rho_u = (sum(rho_u_vals) / len(rho_u_vals)) if rho_u_vals else 0.0
max_rho_u = max(rho_u_vals) if rho_u_vals else 0.0
avg_solve_gap_ms = (sum(solve_gap_ms_vals) / len(solve_gap_ms_vals)) if solve_gap_ms_vals else 0.0
avg_solve_rate_hz = (sum(solve_rate_hz_vals) / len(solve_rate_hz_vals)) if solve_rate_hz_vals else 0.0
avg_target_ms = (sum(target_ms_vals) / len(target_ms_vals)) if target_ms_vals else 0.0
avg_compute_ms = (sum(compute_ms_vals) / len(compute_ms_vals)) if compute_ms_vals else 0.0
max_compute_ms = max(compute_ms_vals) if compute_ms_vals else 0.0
avg_compute_hz = (sum(compute_hz_vals) / len(compute_hz_vals)) if compute_hz_vals else 0.0

print(f"variant: MPCC (Contouring Control)")
print(f"first_collision_line: {first_collision_line}")
print(f"first_rviz_crash_line: {first_rviz_crash_line}")
print(f"first_status2_line: {first_status2_line}")
print(f"solve_samples: {solve_samples}")
print(f"status2_count: {status2_count}")
print(f"status1_count: {status1_count}")
print(f"status0_count: {status0_count}")
print(f"avg_iter: {avg_iter:.2f}")
print(f"max_iter: {max_iter}")
print(f"iteration_limit: {iteration_limit}")
print(f"hit_max_iter: {hit_max_iter}")
print(f"avg_prim: {avg_prim:.4f}")
print(f"max_prim: {max_prim:.4f}")
print(f"avg_dual: {avg_dual:.4f}")
print(f"max_dual: {max_dual:.4f}")
print(f"avg_rho: {avg_rho:.3f}")
print(f"min_rho: {0.0 if min_rho is None else min_rho:.3f}")
print(f"max_rho: {max_rho:.3f}")
print(f"last_rho: {last_rho:.3f}")
print(f"avg_rho_u: {avg_rho_u:.3f}")
print(f"min_rho_u: {0.0 if min_rho_u is None else min_rho_u:.3f}")
print(f"max_rho_u: {max_rho_u:.3f}")
print(f"last_rho_u: {last_rho_u:.3f}")
print(f"avg_clip: {avg_clip:.2f}")
print(f"max_clip: {max_clip}")
print(f"avg_rho_updates: {avg_rho_updates:.2f}")
print(f"max_rho_updates: {max_rho_updates}")
print(f"last_rho_updates: {last_rho_updates}")
print(f"avg_rho_state_updates: {avg_rho_state_updates:.2f}")
print(f"max_rho_state_updates: {max_rho_state_updates}")
print(f"avg_rho_control_updates: {avg_rho_control_updates:.2f}")
print(f"max_rho_control_updates: {max_rho_control_updates}")
print(f"avg_solve_gap_ms: {avg_solve_gap_ms:.2f}")
print(f"avg_solve_rate_hz: {avg_solve_rate_hz:.2f}")
print(f"target_control_ms: {avg_target_ms:.2f}")
print(f"avg_compute_ms: {avg_compute_ms:.2f}")
print(f"max_compute_ms: {max_compute_ms:.2f}")
print(f"avg_compute_hz: {avg_compute_hz:.2f}")
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
