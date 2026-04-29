#!/usr/bin/env bash
set -euo pipefail

# Records localization, planner, TF, map, and OptiTrack pose topics.
# Usage:
#   ./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh
#   ./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh /path/to/output_bag

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Records localization, planner, TF, map, and OptiTrack pose topics.

Usage:
  ./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh
  ./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh /path/to/output_bag
EOF
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QOS_FILE="${SCRIPT_DIR}/optitrack_rosbag_qos.yaml"

find_workspace_root() {
  local dir="${SCRIPT_DIR}"
  while [[ "${dir}" != "/" ]]; do
    if [[ -d "${dir}/f1tenth_localization" && -d "${dir}/f1tenth_system" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
    dir="$(dirname "${dir}")"
  done
  return 1
}

if ! WORKSPACE_ROOT="$(find_workspace_root)"; then
  WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

DEFAULT_BAG_PATH="${WORKSPACE_ROOT}/bags/OptitrackBenchmark_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${1:-${DEFAULT_BAG_PATH}}"
STORAGE_ID="${ROSBAG_STORAGE_ID:-mcap}"

TOPICS=(
  "/tf_static"
  "/tf"
  "/vrpn_mocap/Car2/pose"
  "/scan"
  "/opponent_marker"
  "/local_raceline_wall_distance_markers"
  "/local_raceline"
  "/local_raceline_viz"
  "/ekf_pose"
  "/scan_walls"
  "/amcl_timing"
  "/ackermann_cmd"
  "/amcl_pose"
  "/particlecloud"
  "/scan_obstacles"
  "/ego_racecar/odom"
  "/odom_pose"
  "/map"
  "/map_server/transition_event"
  "/drive"
  "/mpcc/predicted_path"
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

echo "[info] Recording to: ${BAG_PATH}"
echo "[info] Storage backend: ${STORAGE_ID}"
echo "[info] QoS overrides: ${QOS_FILE}"
echo "[info] Topic count: ${#TOPICS[@]}"
echo "[info] Includes /tf and /tf_static, so the map <-> world calibration transform is recorded if it is being published."
echo "[info] Includes OptiTrack pose topic: /vrpn_mocap/Car2/pose"
echo "[info] Press Ctrl+C to stop recording."

exec ros2 bag record \
  -o "${BAG_PATH}" \
  -s "${STORAGE_ID}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  "${TOPICS[@]}"
