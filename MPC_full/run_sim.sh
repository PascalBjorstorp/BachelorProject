#!/usr/bin/env bash
#===========================================================================
# MPC/run_sim.sh — Launch CPU MPC (Riccati-ADMM) with F1/10th simulator
#
# Usage:
#   ./run_sim.sh [DURATION_SECONDS] [TRAJECTORY_FILE]
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = <workspace>/MPC/trajectories/my_track_raceline.csv
#
# This script:
#   1. Syncs latest MPC sources into MPC_experimental (the ROS2 package)
#   2. Rebuilds the mpc_full package via colcon
#   3. Launches gym_bridge + MPC node
#   4. Monitors for collision, logs everything, produces summary
#
# Logs are saved to logs/run_<timestamp>/.
#===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION_SECONDS="${1:-120}"
TRAJECTORY_FILE="${2:-${ROOT_DIR}/MPC/trajectories/my_track_raceline.csv}"

# Resolve to an absolute path so ROS2 nodes can find the trajectory regardless of cwd.
if [[ "${TRAJECTORY_FILE}" != /* ]]; then
    TRAJECTORY_FILE="$(cd "$(dirname "${TRAJECTORY_FILE}")" && pwd)/$(basename "${TRAJECTORY_FILE}")"
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/logs/run_${RUN_ID}"
mkdir -p "${LOG_DIR}"

GYM_LOG="${LOG_DIR}/gym_bridge.log"
MPC_LOG="${LOG_DIR}/mpc.log"
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

    pkill -f 'ros2 launch mpc_full mpc_launch.py' 2>/dev/null || true
    pkill -f 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py' 2>/dev/null || true
    pkill -f '/mpc_full/mpc_node' 2>/dev/null || true
    pkill -f '/f1tenth_gym_ros/gym_bridge' 2>/dev/null || true
    pkill -f 'rviz2.*gym_bridge.rviz' 2>/dev/null || true
    pkill -f 'lifecycle_manager_localization' 2>/dev/null || true
    pkill -f 'nav2_map_server' 2>/dev/null || true
    pkill -f 'ego_robot_state_publisher' 2>/dev/null || true
}
trap cleanup EXIT

if [[ ! -f "${TRAJECTORY_FILE}" ]]; then
    echo "ERROR: Trajectory file not found: ${TRAJECTORY_FILE}" >&2
    exit 1
fi

echo "=== CPU MPC (Riccati-ADMM) Simulation Run ==="
echo "Logs: ${LOG_DIR}"
echo "Duration: ${DURATION_SECONDS}s"
echo "Trajectory: ${TRAJECTORY_FILE}"

#===========================================================================
# Step 1: Build the ROS2 package (from MPC/ directly)
#===========================================================================
echo ""
echo "--- Building mpc_full ROS2 package ---"

cd "${ROOT_DIR}"

# Source ROS2 base
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Build just the mpc_full package
colcon build --packages-select mpc_full --cmake-force-configure 2>&1 | tail -5

# Re-source the workspace after build
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

if ! ros2 pkg prefix mpc_full >/dev/null 2>&1; then
    echo "ERROR: 'mpc_full' is not discoverable after build/source." >&2
    echo "This usually means CMake configured without ROS dependencies in cache." >&2
    echo "Try removing the package build cache and rebuilding:" >&2
    echo "  rm -rf ${ROOT_DIR}/build/mpc_full ${ROOT_DIR}/install/mpc_full ${ROOT_DIR}/log/latest_build/mpc_full" >&2
    echo "  source /opt/ros/jazzy/setup.bash" >&2
    echo "  colcon build --packages-select mpc_full --cmake-force-configure" >&2
    exit 1
fi

echo "  Build complete."

#===========================================================================
# Step 3: Launch simulation
#===========================================================================
echo ""
echo "--- Launching simulation ---"

export PYTHONPATH="${ROOT_DIR}/f1tenth_sim:${ROOT_DIR}/f1tenth_sim/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

# Keep the gym spawn and map aligned with the selected raceline, just like the
# successful sweep / ground-truth runs. This prevents the simulator from
# starting the car at a pose that is far away from the MPC trajectory.
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
export GYM_SX="${GYM_SX:-${TRAJ_SX:-0.0}}"
export GYM_SY="${GYM_SY:-${TRAJ_SY:-0.0}}"
export GYM_STHETA="${GYM_STHETA:-${TRAJ_STHETA:-0.0}}"

echo "Map override: GYM_MAP_PATH=${GYM_MAP_PATH} GYM_MAP_IMG_EXT=${GYM_MAP_IMG_EXT}"
echo "Spawn override: GYM_SX=${GYM_SX} GYM_SY=${GYM_SY} GYM_STHETA=${GYM_STHETA}"

echo "Launching gym_bridge..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py >"${GYM_LOG}" 2>&1 &
SIM_PID=$!

sleep 10

echo "Launching MPC Riccati-ADMM..."
ros2 launch mpc_full mpc_launch.py "trajectory_file:=${TRAJECTORY_FILE}" >"${MPC_LOG}" 2>&1 &
MPC_PID=$!

#===========================================================================
# Step 4: Monitor for collision / timeout
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

    sleep 0.5
done

#===========================================================================
# Step 5: Produce summary
#===========================================================================
LOG_DIR_ENV="${LOG_DIR}" python3 - <<'PY' > "${SUMMARY_LOG}"
import os
from pathlib import Path

log_dir = Path(os.environ["LOG_DIR_ENV"])
gym = (log_dir / "gym_bridge.log").read_text(errors="ignore").splitlines()
mpc = (log_dir / "mpc.log").read_text(errors="ignore").splitlines()

def first_line(lines, token):
    for i, line in enumerate(lines, start=1):
        if token in line:
            return i, line
    return None, None

first_collision_line, first_collision_text = first_line(gym, "Ego vehicle collision detected!")
first_status2_line, first_status2_text = first_line(mpc, "WARNING: Solver status=2")
status2_count = sum("WARNING: Solver status=2" in line for line in mpc)
status0_count = sum("[MPC] Control:" in line and "status=0" in line for line in mpc)

print(f"variant: CPU MPC (Riccati-ADMM)")
print(f"first_collision_line: {first_collision_line}")
print(f"first_status2_line: {first_status2_line}")
print(f"status2_count: {status2_count}")
print(f"status0_count: {status0_count}")

if first_collision_text:
    print(f"first_collision_text: {first_collision_text}")
if first_status2_text:
    print(f"first_status2_text: {first_status2_text}")
PY

echo "=== Run complete ==="
echo "collision_seen=${COLLISION_SEEN}"
echo "gym_log=${GYM_LOG}"
echo "mpc_log=${MPC_LOG}"
echo "summary=${SUMMARY_LOG}"
