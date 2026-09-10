#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select livox_ros_driver2 fastlio2 faster_lio d1max_slam
