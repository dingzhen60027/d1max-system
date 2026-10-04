#!/usr/bin/env bash
set -euo pipefail

source "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../d1max_env.sh"
WS="$D1MAX_NAV_ROOT"
ROBOT_ENV="$D1MAX_ROS2_ENV"
GTSAM_PREFIX="$D1MAX_GTSAM_PREFIX"

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
