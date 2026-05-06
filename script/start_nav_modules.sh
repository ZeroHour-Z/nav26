#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 可通过环境变量覆盖
MAP_YAML="${MAP_YAML:-${WS_ROOT}/src/rm_bringup/PCD/RMUL/newMap.yaml}"
ODIN_MODE="${ODIN_MODE:-slam}"  # odom|slam|relocalization
RELOCALIZATION_MAP="${RELOCALIZATION_MAP:-}"
START_DECISION="${START_DECISION:-1}"
START_COMM="${START_COMM:-1}"
PUBLISH_ODIN_BASE_TF="${PUBLISH_ODIN_BASE_TF:-1}"

if [[ "${ODIN_MODE}" == "relocalization" ]]; then
  PUBLISH_MAP_TO_ODOM_TF="false"
else
  PUBLISH_MAP_TO_ODOM_TF="true"
fi

if [[ ! -f "${MAP_YAML}" ]]; then
  echo "[ERROR] MAP_YAML not found: ${MAP_YAML}"
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble setup not found: /opt/ros/humble/setup.bash"
  exit 1
fi

if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "[ERROR] Workspace setup not found: ${WS_ROOT}/install/setup.bash"
  echo "        Please run colcon build first."
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"

if [[ "${ODIN_MODE}" == "relocalization" && -n "${RELOCALIZATION_MAP}" ]]; then
  if [[ -x "${WS_ROOT}/set_param.sh" ]]; then
    echo "[INFO] Set Odin relocalization map: ${RELOCALIZATION_MAP}"
    "${WS_ROOT}/set_param.sh" relocalization_map_abs_path "${RELOCALIZATION_MAP}" || true
  fi
fi

LOG_DIR="${WS_ROOT}/log/modular_nav/$(date +%Y%m%d_%H%M%S)"
PID_FILE="${WS_ROOT}/log/modular_nav/latest.pids"
mkdir -p "${LOG_DIR}"
mkdir -p "$(dirname "${PID_FILE}")"
: > "${PID_FILE}"

cleanup_on_exit() {
  local rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "[ERROR] Startup failed. Use script/stop_nav_modules.sh to clean up any partial processes."
  fi
}
trap cleanup_on_exit EXIT

start_bg() {
  local name="$1"
  shift
  local logfile="${LOG_DIR}/${name}.log"
  echo "[INFO] Starting ${name}"
  nohup "$@" >"${logfile}" 2>&1 &
  local pid=$!
  echo "${pid} ${name}" >> "${PID_FILE}"
  echo "[INFO] ${name} pid=${pid}, log=${logfile}"
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_ts
  start_ts=$(date +%s)

  echo "[INFO] Waiting for topic ${topic} (timeout=${timeout_sec}s)"
  while true; do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "[INFO] Topic ready: ${topic}"
      return 0
    fi

    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "[ERROR] Timeout waiting for topic: ${topic}"
      return 1
    fi
    sleep 1
  done
}

# 1) Odin 驱动
start_bg odin_driver ros2 launch odin_ros_driver odin1_ros2.launch.py

# 2) 基础静态 TF（可选）
if [[ "${PUBLISH_ODIN_BASE_TF}" == "1" ]]; then
  start_bg odin_base_to_base_link_tf ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odin1_base_link base_link
fi

# 3) 等待里程计上线，避免 Nav2 早起导致 TF 断树
wait_for_topic /odin1/odometry 45

# 4) 地形分析
start_bg terrain ros2 launch rm_terrain_analysis traversability_pointcloud.launch.py input_topic:=/odin1/cloud_slam rviz:=false
start_bg region_detector ros2 launch rm_terrain_analysis region_detector.launch.py

# 5) Nav2 栈
start_bg nav2 ros2 launch nav2_client_cpp nav2_stack_with_gvc.launch.py \
  map:="${MAP_YAML}" \
  gvc_odom_topic:=/odin1/odometry \
  gvc_base_frame:=base_link \
  publish_map_to_odom_tf:="${PUBLISH_MAP_TO_ODOM_TF}"

# 6) 决策与通信（按需）
if [[ "${START_DECISION}" == "1" ]]; then
  start_bg decision ros2 launch rm_decision bt.launch.py
fi

if [[ "${START_COMM}" == "1" ]]; then
  start_bg communication ros2 launch rm_communication communication_bringup.launch.py
fi

# 7) 基础健康检查
wait_for_topic /map 30
wait_for_topic /traversability/obstacles 30

echo "[INFO] Modular navigation startup complete."
echo "[INFO] PID file: ${PID_FILE}"
echo "[INFO] Logs dir : ${LOG_DIR}"
echo "[INFO] Stop all : ${WS_ROOT}/script/stop_nav_modules.sh"
