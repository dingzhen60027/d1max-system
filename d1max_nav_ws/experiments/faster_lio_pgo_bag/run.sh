#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset LD_LIBRARY_PATH PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER
source "/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
source /home/dndx/d1max_nav_ws/install/setup.bash
# User requirement: Zenoh only. No automatic middleware fallback.
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=217
unset ROS_LOCALHOST_ONLY
export ZENOH_SESSION_CONFIG_URI="$D1MAX_ROS2_DIR/foxglove_d1max/config/zenoh-local.json5"
export ZENOH_ROUTER_CONFIG_URI="$D1MAX_ROS2_DIR/foxglove_d1max/config/zenoh-router-local.json5"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
export QT_QPA_PLATFORM=xcb
exec /usr/bin/python3 "$task_root/run.py" "$@"
