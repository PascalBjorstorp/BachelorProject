#!/usr/bin/env bash
set -euo pipefail

# Records the lateral-planner localization bag with fixed topic set + QoS overrides.
# Usage:
#   ./record_lateral_planner_bag.sh
#   ./record_lateral_planner_bag.sh /home/f1tenth/BachelorProject/bags/LateralPlanner2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_STACK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 command not found. Source your ROS 2 workspace first."
  exit 1
fi

if [[ -f "${SOURCE_STACK_DIR}/config/rosbag_tf_qos.yaml" ]]; then
  STACK_DIR="${SOURCE_STACK_DIR}"
  WORKSPACE_ROOT="$(cd "${STACK_DIR}/../.." && pwd)"
else
  STACK_PREFIX="$(ros2 pkg prefix f1tenth_stack)"
  STACK_DIR="${STACK_PREFIX}/share/f1tenth_stack"
  WORKSPACE_ROOT="$(cd "${STACK_PREFIX}/../.." && pwd)"
fi

QOS_FILE="${STACK_DIR}/config/rosbag_tf_qos.yaml"

DEFAULT_BAG_PATH="${WORKSPACE_ROOT}/bags/LateralPlanner_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${1:-${DEFAULT_BAG_PATH}}"
STORAGE_ID="${ROSBAG_STORAGE_ID:-mcap}"

TOPICS=(
  "/tf_static"
  "/scan"
  "/scan_full"
  "/splitter_timing"
  "/opponent_marker"
  "/local_raceline"
  "/local_raceline_viz"
  "/ekf_pose"
  "/tf"
  "/scan_walls"
  "/amcl_timing"
  "/amcl_gpu_timing"
  "/amcl_kld_diagnostics"
  "/amcl_particle_count"
  "/ackermann_cmd"
  "/amcl_pose"
  "/particlecloud_weighted_pre_resample"
  "/particlecloud"
  "/particle_cloud"
  "/scan_obstacles"
  "/ego_racecar/odom"
  "/odom_pose"
  "/map"
  "/map_metadata"
  "/map_server/transition_event"
  "/initialpose"
  "/sensors/core"
  "/sensors/imu"
  "/sensors/imu/raw"
  "/sensors/servo_position_command"
  "/imu/filtered_angular_velocity"
  "/commands/motor/current"
  "/commands/motor/brake"
  "/commands/motor/speed"
  "/commands/servo/position"
  "/drive"
  "/mpc/timing/solve_us"
  "/mpc/timing/iteration_count"
  "/mpc/timing/control_gap_ms"
  "/mpc/timing/ekf_to_control_ms"
  "/mpc/timing/output_gap_ms"
  "/mpc/timing/drive_age_ms"
  "/mpc/timing/pose_seq"
  "/mpc/timing/skipped_poses"
  "/mpc/timing/solver_enter_seq"
  "/mpcc/predicted_path"
  "/mpc_state"
  "/mpcc/track_bounds"
)

if [[ ! -f "${QOS_FILE}" ]]; then
  echo "[error] QoS file not found: ${QOS_FILE}"
  exit 1
fi

if [[ "${STORAGE_ID}" == "mcap" ]] && ! ros2 bag record -h 2>/dev/null | grep -q "mcap"; then
  echo "[warn] mcap storage plugin not detected; falling back to sqlite3."
  STORAGE_ID="sqlite3"
fi

mkdir -p "$(dirname "${BAG_PATH}")"

echo "[info] Recording to: ${BAG_PATH}"
echo "[info] Storage backend: ${STORAGE_ID}"
echo "[info] QoS overrides: ${QOS_FILE}"
echo "[info] Topic count: ${#TOPICS[@]}"

echo "[info] Press Ctrl+C to stop recording."
exec ros2 bag record \
  -o "${BAG_PATH}" \
  -s "${STORAGE_ID}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  "${TOPICS[@]}"
