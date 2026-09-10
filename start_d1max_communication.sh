#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DIR="${ROOT_DIR}/d1max_ros2"
ENV_FILE="${ROS_DIR}/d1max_ros2_env.sh"
PID_FILE="${ROS_DIR}/rmw_zenohd.pid"
LOG_FILE="${ROS_DIR}/rmw_zenohd.log"
ROBOT_IP="${D1MAX_ROBOT_IP:-192.168.168.100}"
ROUTER_PORT="${D1MAX_ZENOH_PORT:-7447}"
WAIT_SECONDS="${D1MAX_TOPIC_WAIT_SECONDS:-20}"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "错误：找不到环境脚本 ${ENV_FILE}" >&2
  exit 1
fi

source "${ENV_FILE}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误：未找到 ros2。" >&2
  exit 1
fi

if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "错误：未安装 rmw_zenoh_cpp，请先运行 ${ROS_DIR}/setup.sh" >&2
  exit 1
fi

port_is_open() {
  (exec 3<>"/dev/tcp/127.0.0.1/${ROUTER_PORT}") 2>/dev/null
}

router_pid=""
if [[ -f "${PID_FILE}" ]]; then
  router_pid="$(<"${PID_FILE}")"
  if [[ ! "${router_pid}" =~ ^[0-9]+$ ]] || ! kill -0 "${router_pid}" 2>/dev/null; then
    router_pid=""
    rm -f "${PID_FILE}"
  fi
fi

if port_is_open; then
  if [[ -z "${router_pid}" ]]; then
    router_pid="$(pgrep -o -f '/rmw_zenohd$' 2>/dev/null || true)"
    if [[ -n "${router_pid}" ]]; then
      printf '%s\n' "${router_pid}" >"${PID_FILE}"
    fi
  fi
  echo "Zenoh router 已在 127.0.0.1:${ROUTER_PORT} 运行${router_pid:+，PID ${router_pid}}。"
else
  : >"${LOG_FILE}"
  nohup setsid ros2 run rmw_zenoh_cpp rmw_zenohd \
    >>"${LOG_FILE}" 2>&1 </dev/null &
  router_pid=$!
  printf '%s\n' "${router_pid}" >"${PID_FILE}"
  disown "${router_pid}" 2>/dev/null || true

  for _ in {1..40}; do
    if port_is_open; then
      break
    fi
    if ! kill -0 "${router_pid}" 2>/dev/null; then
      echo "错误：Zenoh router 启动失败。" >&2
      tail -n 40 "${LOG_FILE}" >&2 || true
      exit 1
    fi
    sleep 0.25
  done

  if ! port_is_open; then
    echo "错误：Zenoh router 未在 ${ROUTER_PORT} 端口监听。" >&2
    tail -n 40 "${LOG_FILE}" >&2 || true
    exit 1
  fi
  echo "Zenoh router 已启动，PID ${router_pid}。"
fi

if ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
  echo "机器狗 ${ROBOT_IP} 网络可达。"
else
  echo "警告：机器狗 ${ROBOT_IP} 暂未响应 ping，继续等待 ROS2 话题。" >&2
fi

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

required_topics=(
  /front_lidar
  /rear_lidar
  /front_lidar/imu
  /rear_lidar/imu
  /tf
  /tf_static
)

topic_list=""
for ((second = 1; second <= WAIT_SECONDS; second++)); do
  topic_list="$(timeout 4 ros2 topic list 2>/dev/null || true)"
  if grep -Fxq '/front_lidar' <<<"${topic_list}" && \
     grep -Fxq '/rear_lidar' <<<"${topic_list}"; then
    break
  fi
  sleep 1
done

echo
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "Zenoh endpoint=tcp://${ROBOT_IP}:${ROUTER_PORT}"
echo "Router log=${LOG_FILE}"
echo
echo "关键话题检查："

missing=0
for topic in "${required_topics[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    printf '  [OK]      %s\n' "${topic}"
  else
    printf '  [MISSING] %s\n' "${topic}"
    missing=$((missing + 1))
  fi
done

if ((missing > 0)); then
  echo
  echo "通信环境已启动，但有 ${missing} 个关键话题尚未发现。" >&2
  echo "请检查机器狗电源、网络和机器狗端 Zenoh 服务。" >&2
  exit 2
fi

echo
echo "机器狗 ROS2 通信已就绪。"
