#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
D1MAX_LIVE_WS="$D1MAX_NAV_ROOT"
source "$D1MAX_ROS2_ENV"
source "$D1MAX_LIVE_WS/install/setup.bash"
export PYTHONPATH="$D1MAX_LIVE_WS/src/d1max_pct_scan:$D1MAX_LIVE_WS/src/d1max_pct_planner:$D1MAX_LIVE_WS:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=24
if [[ $# -eq 0 ]]; then set -- status; fi
exec /usr/bin/python3 -m d1max_pct_scan.live_session "$@"
