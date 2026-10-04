#!/usr/bin/env bash
set -eo pipefail

source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
ROOT="$D1MAX_NAV_ROOT"
ROBOT_ENV="$D1MAX_ROS2_ENV"
source "$ROBOT_ENV"
source "$ROOT/install/setup.bash"
ros2 service call /d1max/slam/save std_srvs/srv/Trigger '{}'
