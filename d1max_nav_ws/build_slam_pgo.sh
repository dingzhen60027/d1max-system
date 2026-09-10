#!/usr/bin/env bash
set -euo pipefail

WS=/home/dndx/d1max_nav_ws
ROBOT_ENV="/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/d1max_ros2_env.sh"
GTSAM_PREFIX=/home/dndx/.local/ros-humble-gtsam/opt/ros/humble

source "$ROBOT_ENV"
export CMAKE_PREFIX_PATH="$GTSAM_PREFIX:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$GTSAM_PREFIX/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

cd "$WS"
colcon build \
  --symlink-install \
  --packages-select faster_lio sc_pgo d1max_slam \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_DIR="$GTSAM_PREFIX/lib/cmake/GTSAM"
