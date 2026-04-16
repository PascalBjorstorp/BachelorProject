#!/usr/bin/env bash
#===========================================================================
# MPC/run_sim.sh — Launch CPU MPC (Riccati-ADMM) with F1/10th simulator
#
# Usage:
#   ./run_sim.sh [DURATION_SECONDS] [TRAJECTORY_FILE]
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = <workspace>/f1tenth_planning/trajectories/my_track_raceline.csv
#
# This script:
#   1. Syncs latest MPC sources into MPC_experimental (the ROS2 package)
#   2. Rebuilds the mpc_riccati package via colcon
#   3. Launches gym_bridge + MPC node
#   4. Monitors for collision, logs everything, produces summary
#
# Logs are saved to logs/run_<timestamp>/.
#===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION_SECONDS="${1:-120}"
TRAJECTORY_FILE="${2:-${ROOT_DIR}/f1tenth_planning/trajectories/my_track_raceline.csv}"

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

    pkill -f 'ros2 launch mpc_riccati mpc_launch.py' 2>/dev/null || true
    pkill -f 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py' 2>/dev/null || true
    pkill -f '/mpc_riccati/mpc_node' 2>/dev/null || true
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
echo "--- Building mpc_riccati ROS2 package ---"

cd "${ROOT_DIR}"

# Source ROS2 base
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Build just the mpc_riccati package
colcon build --packages-select mpc_riccati 2>&1 | tail -5

# Re-source the workspace after build
set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

echo "  Build complete."

#===========================================================================
# Step 3: Launch simulation
#===========================================================================
echo ""
echo "--- Launching simulation ---"

export PYTHONPATH="${ROOT_DIR}/f1tenth_sim:${ROOT_DIR}/f1tenth_sim/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

echo "Launching gym_bridge..."
ros2 launch f1tenth_gym_ros gym_bridge_launch.py >"${GYM_LOG}" 2>&1 &
SIM_PID=$!

sleep 4

echo "Launching MPC Riccati-ADMM..."
ros2 launch mpc_riccati mpc_launch.py "trajectory_file:=${TRAJECTORY_FILE}" >"${MPC_LOG}" 2>&1 &
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
