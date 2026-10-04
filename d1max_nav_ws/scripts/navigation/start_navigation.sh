#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
NAV_RUNTIME_WS="$D1MAX_NAV_ROOT"
source "$D1MAX_APP_ROOT/d1max_ros2_env.sh"
source "$NAV_RUNTIME_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=24
exec ros2 run d1max_navigation navigation_session "$@"
