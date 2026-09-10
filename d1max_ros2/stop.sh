#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="${SCRIPT_DIR}/rmw_zenohd.pid"

source "${SCRIPT_DIR}/d1max_ros2_env.sh"

ros2 daemon stop >/dev/null 2>&1 || true

if [[ -f "${PID_FILE}" ]] && kill -0 "$(<"${PID_FILE}")" 2>/dev/null; then
  kill "$(<"${PID_FILE}")"
  echo "Zenoh 路由已停止。"
else
  echo "Zenoh 路由未运行。"
fi

rm -f "${PID_FILE}"
