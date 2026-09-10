#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${SCRIPT_DIR}/rmw_zenohd.pid"
LOG_FILE="${SCRIPT_DIR}/rmw_zenohd.log"
ROBOT_IP="192.168.168.100"

source "${SCRIPT_DIR}/d1max_ros2_env.sh"

if ! command -v ros2 >/dev/null 2>&1 || ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "错误：rmw_zenoh_cpp 尚未安装，请先运行 ${SCRIPT_DIR}/setup.sh"
  exit 1
fi

if ! ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
  echo "警告：机器狗 ${ROBOT_IP} 未响应 ping，仍将尝试启动 Zenoh。"
fi

if [[ -f "${PID_FILE}" ]] && kill -0 "$(<"${PID_FILE}")" 2>/dev/null; then
  echo "Zenoh 路由已运行，PID: $(<"${PID_FILE}")"
else
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd >"${LOG_FILE}" 2>&1 &
  ZENOH_PID=$!
  printf '%s\n' "${ZENOH_PID}" > "${PID_FILE}"

  for _ in {1..20}; do
    if (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
      exec 3>&-
      break
    fi
    if ! kill -0 "${ZENOH_PID}" 2>/dev/null; then
      echo "错误：Zenoh 路由启动失败，日志：${LOG_FILE}"
      tail -n 30 "${LOG_FILE}"
      exit 1
    fi
    sleep 0.25
  done
  echo "Zenoh 路由已启动，PID: ${ZENOH_PID}"
fi

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start

echo
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "机器狗 Zenoh 端点：tcp://${ROBOT_IP}:7447"
echo
echo "当前 ROS 2 话题："
ros2 topic list
