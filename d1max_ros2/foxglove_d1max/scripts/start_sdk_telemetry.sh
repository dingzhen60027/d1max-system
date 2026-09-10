#!/usr/bin/env bash
set -euo pipefail
D1MAX_VIEW_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_VIEW_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_VIEW_DIR}/config/zenoh-live.json5"
set +u
source "${D1MAX_VIEW_DIR}/../sdk_bridge_ws/install/setup.bash"
set -u
# Refuse duplicate passive/control bridges, including another PC on this graph.
python3 - <<'PY'
import rclpy,time
from rclpy.node import Node
rclpy.init()
node=Node('d1max_sdk_observer_preflight',enable_rosout=False,start_parameter_services=False)
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
duplicate=bool(node.get_publishers_info_by_topic('/d1max_sdk_bridge/robot_state'))
node.destroy_node();rclpy.shutdown()
if duplicate:raise SystemExit('SDK 状态已有发布者，拒绝启动重复 SDK 连接。')
PY
exec "${D1MAX_VIEW_DIR}/../sdk_bridge_ws/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_telemetry_bridge" "$@"
