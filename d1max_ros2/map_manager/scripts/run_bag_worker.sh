#!/usr/bin/env bash
set -eo pipefail
source "$1/d1max_ros2/d1max_ros2_env.sh"
exec /usr/bin/python3 -m backend.bags.worker "$2"
