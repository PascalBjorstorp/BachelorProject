#!/usr/bin/env bash
set -euo pipefail

# Automated ground-truth test runner.
# Launches core stack once, then runs 3 x 3 speed tests with bag recording.

WORKSPACE_ROOT="/home/f1tenth/BachelorProject"
TRAJECTORY_FILE="${WORKSPACE_ROOT}/f1tenth_planning/trajectories/my_track_raceline.csv"
TF_POSE_PUBLISHER="${WORKSPACE_ROOT}/f1tenth_localization/launch/tf_pose_publisher.py"
ROSBAG_QOS_FILE="${WORKSPACE_ROOT}/f1tenth_system/f1tenth_stack/config/rosbag_tf_qos.yaml"
ROSBAG_STORAGE_ID="${ROSBAG_STORAGE_ID:-auto}"

TOPICS_TO_RECORD=(
  "/tf_static"
  "/scan"
  "/opponent_marker"
  "/local_raceline_wall_distance_markers"
  "/local_raceline_viz"
  "/ekf_pose"
  "/tf"
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

LOOKAHEAD_MIN="0.37634354"
LOOKAHEAD_MAX="1.0562852"
LOOKAHEAD_GAIN="0.062011484"
CTE_LOOKAHEAD_WEIGHT="1.0"
CTE_LOOKAHEAD_GAIN="0.041540516"
CURVATURE_LOOKAHEAD_GAIN="1.9003721"
CURVATURE_SPEED_FACTOR="0.1015252"
CURVATURE_SPEED_FLOOR_RATIO="0.52401066"
CTE_SPEED_FACTOR="0.53756776"
CTE_SPEED_FLOOR_RATIO="0.7826799"
ODOM_TIMEOUT_S="0.20"
RUN_DURATION_SEC=38
SPEEDS=("1.0" "1.5" "2.0")
RUNS_PER_SPEED=3
EKF_WAIT_TIMEOUT_SEC=20
MANAGE_CORE="${MANAGE_CORE:-false}"
RESTART_CORE_EACH_RUN="${RESTART_CORE_EACH_RUN:-false}"

LOG_DIR="${WORKSPACE_ROOT}/Logs/ground_truth_test_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"
mkdir -p "${HOME}/BachelorProject/bags"

PIDS=()

bag_speed_label() {
  local speed="$1"
  echo "${speed//./dot}"
}

track_pid() {
  local pid="$1"
  PIDS+=("${pid}")
}

terminate_process() {
  local pid="$1"
  local name="$2"
  local target="-${pid}"

  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  kill -INT "${target}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  for _ in {1..10}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done

  echo "[warn] ${name} did not exit on SIGINT, sending SIGTERM"
  kill -TERM "${target}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..10}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done

  echo "[warn] ${name} still alive, sending SIGKILL"
  kill -KILL "${target}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

cleanup() {
  echo ""
  echo "[cleanup] Stopping active processes..."

  for pid in "${PIDS[@]:-}"; do
    terminate_process "${pid}" "tracked_process"
  done

  echo "[cleanup] Done. Logs: ${LOG_DIR}"
}

publish_stop() {
  # Publish several stop commands so the final command is reliably zero speed.
  for _ in {1..5}; do
    timeout 1 ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
      "{drive: {speed: 0.0, steering_angle: 0.0}}" >/dev/null 2>&1 || true
    sleep 0.2
  done
}

set_pp_enable() {
  local enabled="$1"
  timeout 1 ros2 topic pub --once /pp_enable std_msgs/msg/Bool "{data: ${enabled}}" >/dev/null 2>&1 || true
}

kill_all_pure_pursuit() {
  pkill -f "ros2 launch f1tenth_control pure_pursuit_launch.py" 2>/dev/null || true
  pkill -f "pure_pursuit_node_exe" 2>/dev/null || true
}

start_background() {
  local __pid_var="$1"
  local log_file="$2"
  shift 2
  if command -v setsid >/dev/null 2>&1; then
    setsid "$@" >"${log_file}" 2>&1 &
  else
    "$@" >"${log_file}" 2>&1 &
  fi
  local pid=$!
  track_pid "${pid}"
  printf -v "${__pid_var}" '%s' "${pid}"
}


wait_for_ekf_pose() {
  echo "[core] Waiting for /ekf_pose (timeout: ${EKF_WAIT_TIMEOUT_SEC}s)..."
  if timeout "${EKF_WAIT_TIMEOUT_SEC}" \
      ros2 topic echo /ekf_pose geometry_msgs/msg/PoseWithCovarianceStamped --once >/dev/null 2>&1; then
    echo "[core] /ekf_pose is available."
    return 0
  fi

  echo "[error] /ekf_pose not received within ${EKF_WAIT_TIMEOUT_SEC}s."
  echo "        Check localization startup and set initial pose in RViz if needed."
  return 1
}

trap cleanup EXIT INT TERM

echo "=========================================="
echo "Ground Truth Test Runner"
echo "Logs: ${LOG_DIR}"
echo "Manage core stack: ${MANAGE_CORE}"
echo "Restart core each run: ${RESTART_CORE_EACH_RUN}"
echo "=========================================="

if [[ "${MANAGE_CORE}" == "true" ]]; then
  if ! declare -F start_core_stack >/dev/null 2>&1 || ! declare -F stop_core_stack >/dev/null 2>&1; then
    echo "[error] MANAGE_CORE=true requires start_core_stack/stop_core_stack helpers, but they are not defined in this script."
    echo "        Define them or run with MANAGE_CORE=false."
    exit 1
  fi
fi

if [[ ! -f "${TRAJECTORY_FILE}" ]]; then
  echo "[error] Trajectory file missing: ${TRAJECTORY_FILE}"
  exit 1
fi

if [[ ! -f "${TF_POSE_PUBLISHER}" ]]; then
  echo "[error] tf_pose_publisher.py missing: ${TF_POSE_PUBLISHER}"
  exit 1
fi

if [[ ! -f "${ROSBAG_QOS_FILE}" ]]; then
  echo "[error] rosbag QoS file missing: ${ROSBAG_QOS_FILE}"
  exit 1
fi

if [[ "${ROSBAG_STORAGE_ID}" == "auto" ]]; then
  if ros2 bag record -h 2>/dev/null | grep -q "mcap"; then
    ROSBAG_STORAGE_ID="mcap"
  else
    ROSBAG_STORAGE_ID="sqlite3"
    echo "[warn] rosbag2 mcap storage plugin not detected; falling back to sqlite3."
  fi
fi

echo "[info] rosbag storage backend: ${ROSBAG_STORAGE_ID}"

if [[ "${MANAGE_CORE}" == "true" ]]; then
  if [[ "${RESTART_CORE_EACH_RUN}" != "true" ]]; then
    echo "Starting core stack once (stable mode)..."
    start_core_stack "session"
    if ! wait_for_ekf_pose; then
      echo "[fatal] Core stack did not provide /ekf_pose. Exiting."
      exit 1
    fi
  else
    echo "Beginning timed runs with full stack restart per run."
  fi
else
  echo "External core mode: you manage localization/bringup/tf_pose_publisher terminals."
fi

declare -i run_idx=0
for speed in "${SPEEDS[@]}"; do
  for test_num in $(seq 1 "${RUNS_PER_SPEED}"); do
    run_idx+=1
    speed_label="$(bag_speed_label "${speed}")"
    bag_name="GroundTruthTest_${speed_label}_${test_num}"
    bag_path="${HOME}/BachelorProject/bags/${bag_name}"

    if [[ -e "${bag_path}" ]]; then
      suffix="$(date +%Y%m%d_%H%M%S)"
      bag_name="${bag_name}_${suffix}"
      bag_path="${HOME}/BachelorProject/bags/${bag_name}"
      echo "[warn] Existing bag folder found. Using: ${bag_name}"
    fi

    echo ""
    echo "------------------------------------------"
    echo "Run ${run_idx}/9"
    echo "Speed: ${speed} m/s"
    echo "Bag:   ${bag_name}"
    echo "------------------------------------------"

    if [[ "${MANAGE_CORE}" == "true" ]]; then
      if [[ "${RESTART_CORE_EACH_RUN}" == "true" ]]; then
        run_tag="${speed_label}_${test_num}"
        start_core_stack "${run_tag}"

        if ! wait_for_ekf_pose; then
          stop_core_stack
          read -r -p "Fix localization/initial pose, then press Enter to retry this run..." _
          run_idx-=1
          continue
        fi
      fi
    else
      if ! wait_for_ekf_pose; then
        read -r -p "Start/restart localization stack, then press Enter to retry this run..." _
        run_idx-=1
        continue
      fi
    fi

    start_background bag_pid "${LOG_DIR}/bag_${bag_name}.log" \
      ros2 bag record \
        -o "${bag_path}" \
        -s "${ROSBAG_STORAGE_ID}" \
        --qos-profile-overrides-path "${ROSBAG_QOS_FILE}" \
        "${TOPICS_TO_RECORD[@]}"
    sleep 1

    start_background pp_pid "${LOG_DIR}/pure_pursuit_${bag_name}.log" \
      ros2 launch f1tenth_control pure_pursuit_launch.py \
        trajectory_file:="${TRAJECTORY_FILE}" \
        max_speed:="${speed}" \
        min_lookahead:="${LOOKAHEAD_MIN}" \
        max_lookahead:="${LOOKAHEAD_MAX}" \
        lookahead_gain:="${LOOKAHEAD_GAIN}" \
        cte_lookahead_weight:="${CTE_LOOKAHEAD_WEIGHT}" \
        cte_lookahead_gain:="${CTE_LOOKAHEAD_GAIN}" \
        curvature_lookahead_gain:="${CURVATURE_LOOKAHEAD_GAIN}" \
        curvature_speed_factor:="${CURVATURE_SPEED_FACTOR}" \
        curvature_speed_floor_ratio:="${CURVATURE_SPEED_FLOOR_RATIO}" \
        cte_speed_factor:="${CTE_SPEED_FACTOR}" \
        cte_speed_floor_ratio:="${CTE_SPEED_FLOOR_RATIO}" \
        odom_timeout_s:="${ODOM_TIMEOUT_S}"

    echo "Driving for ${RUN_DURATION_SEC}s..."
    set_pp_enable true
    sleep "${RUN_DURATION_SEC}"

    echo "Stopping recording first (timing priority)..."
    terminate_process "${bag_pid}" "rosbag"

    echo "Stopping controller..."
    set_pp_enable false
    publish_stop
	echo "Published stop command"
    terminate_process "${pp_pid}" "pure_pursuit"
	echo "Stopped pure pursuit process"
    kill_all_pure_pursuit
	echo "Ensured pure pursuit processes are killed"
    set_pp_enable false
    publish_stop
	

    if [[ "${MANAGE_CORE}" == "true" && "${RESTART_CORE_EACH_RUN}" == "true" ]]; then
      stop_core_stack
    fi

    echo "Run complete: ${bag_name}"
    read -r -p "Place the car back at start and press Enter for next run..." _
  done
done

echo ""
echo "All scheduled runs completed."
echo "Bags saved in: ${HOME}/BachelorProject/bags"
