#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 - <<'PY'
import socket
with socket.socket() as sock:
    if sock.connect_ex(('127.0.0.1',8769))==0:
        raise SystemExit('8769 已被占用；请先核对现有桥，不会自动清理未知进程。')
PY
cleanup() {
  for D1MAX_CHILD_PID in $(jobs -pr); do kill -TERM "${D1MAX_CHILD_PID}" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
bash "${D1MAX_MONITOR_DIR}/scripts/start_sdk_monitor.sh" &
if python3 "${D1MAX_MONITOR_DIR}/scripts/pcd_map_publisher.py" --is-enabled; then
  python3 "${D1MAX_MONITOR_DIR}/scripts/pcd_map_publisher.py" &
fi
bash "${D1MAX_MONITOR_DIR}/scripts/start_monitor_bridge.sh" &
echo 'D1 Max 监控：ws://127.0.0.1:8769 · 静态 PCD + 实时感知 + 单向软件急停；不提供运动控制。'
wait -n
