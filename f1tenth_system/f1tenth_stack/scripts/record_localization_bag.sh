#!/usr/bin/env bash
set -euo pipefail

# Records one bag with all topics needed for localization comparison runs.
# It is intentionally redundant so the same command works for GPU AMCL,
# Nav2 AMCL, and odom-only tests.
#
# Usage:
#   ./record_localization_bag.sh
#   ./record_localization_bag.sh /home/pascal/Documents/BachelorProject/bags/localization_test_01
#
# Optional:
#   ROSBAG_STORAGE_ID=sqlite3 ./record_localization_bag.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${STACK_DIR}/../.." && pwd)"

QOS_FILE="${STACK_DIR}/config/localization_rosbag_qos.yaml"
DEFAULT_BAG_PATH="${WORKSPACE_ROOT}/bags/Localization_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${1:-${DEFAULT_BAG_PATH}}"
STORAGE_ID="${ROSBAG_STORAGE_ID:-mcap}"

TOPICS=(
  "/scan"
  "/scan_walls"
  "/scan_obstacles"
  "/ego_racecar/odom"
  "/odom_pose"
  "/amcl_pose"
  "/amcl_timing"
  "/amcl_particle_count"
  "/ekf_pose"
  "/tf"
  "/tf_static"
  "/vrpn_mocap/car_pos/pose"
  "/map"
  "/map_metadata"
  "/initialpose"
  "/particlecloud"
  "/particle_cloud"
  "/drive"
  "/ackermann_cmd"
  "/clock"
)

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 command not found. Source your ROS 2 workspace first."
  exit 1
fi

if [[ ! -f "${QOS_FILE}" ]]; then
  echo "[error] QoS file not found: ${QOS_FILE}"
  exit 1
fi

if [[ "${STORAGE_ID}" == "mcap" ]] && ! ros2 bag record -h 2>/dev/null | grep -q "mcap"; then
  echo "[warn] mcap storage plugin not detected; falling back to sqlite3."
  STORAGE_ID="sqlite3"
fi

mkdir -p "$(dirname "${BAG_PATH}")"

echo "[info] Recording localization bag to: ${BAG_PATH}"
echo "[info] Storage backend: ${STORAGE_ID}"
echo "[info] QoS overrides: ${QOS_FILE}"
echo "[info] Topic count: ${#TOPICS[@]}"
printf '[info] Topics:\n'
printf '  %s\n' "${TOPICS[@]}"
echo "[info] Press Ctrl+C to stop recording."

exec ros2 bag record \
  -o "${BAG_PATH}" \
  -s "${STORAGE_ID}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  "${TOPICS[@]}"
