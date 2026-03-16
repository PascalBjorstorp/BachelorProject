#!/usr/bin/env bash
#===========================================================================
# MPCC/run_sim.sh — Launch MPCC (Contouring Control) with F1/10th simulator
#
# Usage:
#   ./run_sim.sh [DURATION_SECONDS] [TRAJECTORY_FILE]
#
# Defaults:
#   DURATION_SECONDS = 120
#   TRAJECTORY_FILE  = f1tenth_planning/trajectories/Spielberg_raceline.csv
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
TRAJECTORY_FILE="${2:-${ROOT_DIR}/f1tenth_planning/trajectories/Spielberg_raceline_optimized_wide.csv}"

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
echo "--- Building mpcc_f1_10th ROS2 package ---"

cd "${ROOT_DIR}"

# Source ROS2 base
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Build the mpcc_f1_10th package (pulls in MPC shared sources via CMake)
colcon build --packages-select mpcc_f1_10th 2>&1 | tail -5

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

    sleep 0.5
done

#===========================================================================
# Step 4: Produce summary
#===========================================================================
LOG_DIR_ENV="${LOG_DIR}" python3 - <<'PY' > "${SUMMARY_LOG}"
import os
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
first_status2_line, first_status2_text = first_line(mpcc, "WARNING: Solver status=2")
status2_count = sum("WARNING: Solver status=2" in line for line in mpcc)
status0_count = sum("[MPCC] Control:" in line and "status=0" in line for line in mpcc)

print(f"variant: MPCC (Contouring Control)")
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
STATUS_FAIL=$(grep "^status2_count:" "${SUMMARY_LOG}" 2>/dev/null | cut -d' ' -f2)

if [[ "${COLLISION_SEEN}" -eq 1 ]]; then
    COLLISION_NOTE="Yes (log line ${COLLISION_LINE:-?})"
else
    COLLISION_NOTE="No"
fi

echo "| ${RUN_ID} | ${DURATION_SECONDS}s | ${COLLISION_NOTE} | ${STATUS_OK:-0} | ${STATUS_FAIL:-0} | [log](logs/run_${RUN_ID}/) |" >> "${HISTORY_FILE}"

echo "History updated: ${HISTORY_FILE}"
