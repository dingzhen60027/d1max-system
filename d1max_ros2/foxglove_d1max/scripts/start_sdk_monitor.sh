#!/usr/bin/env bash
set -euo pipefail
D1MAX_MONITOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${D1MAX_MONITOR_DIR}/../d1max_ros2_env.sh"
export ZENOH_SESSION_CONFIG_URI="${D1MAX_MONITOR_DIR}/config/zenoh-live.json5"
set +u
source "${D1MAX_MONITOR_DIR}/../sdk_bridge_ws/install/setup.bash"
set -u
# The helper holds a process-lifetime lock, retains the ROS duplicate check and
# consumes motion opt-in only inside the existing managed service invocation.
exec python3 "${D1MAX_MONITOR_DIR}/scripts/sdk_motion_startup.py" _launch
