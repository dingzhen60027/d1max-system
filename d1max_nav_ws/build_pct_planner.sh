#!/usr/bin/env bash
set -eo pipefail

WS=/home/dndx/d1max_nav_ws
VENDOR="$WS/src/pct_planner_vendor"
JOBS="${PCT_BUILD_JOBS:-4}"

source /opt/ros/humble/setup.bash

cmake -S "$VENDOR/planner/lib/3rdparty/gtsam-4.1.1" \
  -B "$VENDOR/planner/lib/3rdparty/gtsam-4.1.1/build" \
  -DCMAKE_INSTALL_PREFIX="$VENDOR/planner/lib/3rdparty/gtsam-4.1.1/install" \
  -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF
cmake --build "$VENDOR/planner/lib/3rdparty/gtsam-4.1.1/build" -j"$JOBS"
cmake --install "$VENDOR/planner/lib/3rdparty/gtsam-4.1.1/build"

cmake -S "$VENDOR/planner/lib/3rdparty/osqp" \
  -B "$VENDOR/planner/lib/3rdparty/osqp/build" \
  -DCMAKE_INSTALL_PREFIX="$VENDOR/planner/lib/3rdparty/osqp/install" \
  -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF
cmake --build "$VENDOR/planner/lib/3rdparty/osqp/build" -j"$JOBS"
cmake --install "$VENDOR/planner/lib/3rdparty/osqp/build"

cmake -S "$VENDOR/planner/lib" -B "$VENDOR/planner/lib/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$VENDOR/planner/lib/build" -j"$JOBS"
find "$VENDOR/planner/lib/build/src" -type f -name '*.so' -exec cp -f {} "$VENDOR/planner/lib/" \;

cd "$WS"
colcon build --symlink-install --packages-select d1max_pct_planner
echo "PCT Planner build complete."
