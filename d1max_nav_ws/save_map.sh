#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ENV="${D1MAX_ROS2_ENV:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh}"
source "$ROBOT_ENV"
source "$ROOT/install/setup.bash"
ros2 service call /d1max/slam/save std_srvs/srv/Trigger '{}'
