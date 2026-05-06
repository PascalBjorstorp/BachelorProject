#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DEFAULT_BAG="${WORKSPACE_ROOT}/bags/LateralPlanner_20260505_173000"
RVIZ_CONFIG="${WORKSPACE_ROOT}/f1tenth_localization/Rviz2Configs/LateralPlanner.rviz"
AMCL_PARAMS="${WORKSPACE_ROOT}/f1tenth_localization/config/gpu_amcl_cpp_params.yaml"
ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_ROOT}/log/ros_replay_lateral_planner}"

BAG_PATH="${DEFAULT_BAG}"
LOOP="true"
RATE="1.0"
START_RVIZ="false"
START_AMCL="true"
AMCL_CLOUD_RATE="20.0"

usage() {
  cat <<EOF
Usage:
  $0 [bag_path] [--once] [--rate RATE] [--rviz] [--no-amcl] [--amcl-cloud-rate RATE]

Default bag:
  ${DEFAULT_BAG}

Replays only source topics:
  /scan /map /tf /tf_static /ego_racecar/odom

Starts:
  f1tenth_localization/odom_fused_node
  f1tenth_localization/gpu_amcl_cpp_node
  f1tenth_localization/ekf_localization_node
  f1tenth_lidar/scan_splitter_node
  f1tenth_lateral_planner/lateral_planner_node

AMCL particle cloud publish rate:
  ${AMCL_CLOUD_RATE} Hz
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --once)
      LOOP="false"
      shift
      ;;
    --rate)
      RATE="${2:?missing value for --rate}"
      shift 2
      ;;
    --rviz)
      START_RVIZ="true"
      shift
      ;;
    --no-amcl)
      START_AMCL="false"
      shift
      ;;
    --amcl-cloud-rate)
      AMCL_CLOUD_RATE="${2:?missing value for --amcl-cloud-rate}"
      shift 2
      ;;
    *)
      BAG_PATH="$1"
      shift
      ;;
  esac
done

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 not found. Source ROS 2 and workspace first."
  exit 1
fi

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "[error] bag path not found: ${BAG_PATH}"
  exit 1
fi

mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

PIDS=()
cleanup() {
  echo
  echo "[info] stopping replay nodes"
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "[info] workspace: ${WORKSPACE_ROOT}"
echo "[info] bag: ${BAG_PATH}"
echo "[info] ROS logs: ${ROS_LOG_DIR}"
echo "[info] Localization stack: ${START_AMCL} (AMCL particle cloud ${AMCL_CLOUD_RATE} Hz)"
echo "[info] RViz:"
echo "       ros2 run rviz2 rviz2 -d ${RVIZ_CONFIG} --ros-args -p use_sim_time:=true"
echo

if [[ "${START_AMCL}" == "true" ]]; then
  ros2 run f1tenth_localization odom_fused_node \
    --ros-args \
    --params-file "${AMCL_PARAMS}" \
    -p use_sim_time:=true &
  PIDS+=("$!")

  ros2 run f1tenth_localization gpu_amcl_cpp_node \
    --ros-args \
    --params-file "${AMCL_PARAMS}" \
    -p use_sim_time:=true \
    -p "cloud_publish_rate:=${AMCL_CLOUD_RATE}" &
  PIDS+=("$!")

  ros2 run f1tenth_localization ekf_localization_node \
    --ros-args \
    --params-file "${AMCL_PARAMS}" \
    -p use_sim_time:=true &
  PIDS+=("$!")
fi

ros2 run f1tenth_lidar scan_splitter_node \
  --ros-args -p use_sim_time:=true &
PIDS+=("$!")

ros2 run f1tenth_lateral_planner lateral_planner_node \
  --ros-args -p use_sim_time:=true -p avoidance_enabled:=true &
PIDS+=("$!")

if [[ "${START_RVIZ}" == "true" ]]; then
  ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" --ros-args -p use_sim_time:=true &
  PIDS+=("$!")
fi

sleep 1.0

BAG_ARGS=(
  "${BAG_PATH}"
  --clock
  --rate "${RATE}"
  --topics
  /scan
  /map
  /tf
  /tf_static
  /ego_racecar/odom
)

if [[ "${LOOP}" == "true" ]]; then
  BAG_ARGS+=(--loop)
fi

echo "[info] playing bag. Ctrl+C stops all."
ros2 bag play "${BAG_ARGS[@]}" &
PIDS+=("$!")

wait "${PIDS[-1]}"
