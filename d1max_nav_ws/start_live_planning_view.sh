#!/usr/bin/env bash
set -eo pipefail
D1MAX_LIVE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source '/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh'
source "$D1MAX_LIVE_WS/install/setup.bash"
export PYTHONPATH="$D1MAX_LIVE_WS/src/d1max_pct_scan:$D1MAX_LIVE_WS/src/d1max_pct_planner:$D1MAX_LIVE_WS:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=24
if [[ $# -eq 0 ]]; then set -- status; fi
exec /usr/bin/python3 -m d1max_pct_scan.live_session "$@"
