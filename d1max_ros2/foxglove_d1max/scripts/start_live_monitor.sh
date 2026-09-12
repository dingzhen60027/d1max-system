#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 - <<'PY'
import socket
for port in (8769,7448):
    with socket.socket() as sock:
        sock.settimeout(.5)
        if sock.connect_ex(('127.0.0.1',port))==0:
            raise SystemExit(f'{port} 已被占用；不会接管或清理未知进程。')
PY
cleanup() {
  for D1MAX_CHILD_PID in $(jobs -pr); do kill -TERM "${D1MAX_CHILD_PID}" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
export ZENOH_ROUTER_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-router-live.json5"
ros2 run rmw_zenoh_cpp rmw_zenohd &
D1MAX_LIVE_ROUTER_PID=$!
export D1MAX_LIVE_ROUTER_PID
python3 - <<'PY'
import os,socket,time
deadline=time.monotonic()+8
while time.monotonic()<deadline:
    os.kill(int(os.environ['D1MAX_LIVE_ROUTER_PID']),0)
    try:
        with socket.create_connection(('127.0.0.1',7448),timeout=.25):
            break
    except OSError:
        time.sleep(.1)
else:
    raise SystemExit('本机实机 Zenoh 中转未就绪；停止本进程组，不启动 SDK。')
PY
bash "${D1MAX_MONITOR_DIR}/scripts/start_sdk_monitor.sh" &
if python3 "${D1MAX_MONITOR_DIR}/scripts/pcd_map_publisher.py" --is-enabled; then
  python3 "${D1MAX_MONITOR_DIR}/scripts/pcd_map_publisher.py" &
fi
bash "${D1MAX_MONITOR_DIR}/scripts/start_monitor_bridge.sh" &
if [[ "${D1MAX_MANAGED_MONITOR:-0}" == "1" ]]; then
  python3 "${D1MAX_MONITOR_DIR}/scripts/monitor_health.py" &
fi
echo 'D1 Max 监控：ws://127.0.0.1:8769 · 静态 PCD + 实时感知 + 单向软件急停；不提供运动控制。'
wait -n
