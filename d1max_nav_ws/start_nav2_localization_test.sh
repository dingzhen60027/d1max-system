#!/usr/bin/env bash
set -eo pipefail
NAV_TEST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export D1MAX_APP_ROOT="${D1MAX_APP_ROOT:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2}"
export D1MAX_NAV_ROOT="$NAV_TEST_WS"
source "$D1MAX_APP_ROOT/d1max_ros2_env.sh"
source "$NAV_TEST_WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=24
exec ros2 run d1max_navigation localization_test_session "$@"
