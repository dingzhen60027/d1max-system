#!/usr/bin/env bash
# Isolated development stage; does not alter the sealed motion release.
set -eo pipefail
STAGE_WS=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)
export D1MAX_NAV_ROOT="$STAGE_WS"
export D1MAX_RELEASE=$(/usr/bin/python3 "$STAGE_WS/tools/release/single_floor_entry_preflight.py" select --nav-root "$STAGE_WS")
export PYTHONPATH=
source /opt/ros/humble/setup.bash
STAGE_RMW="${D1MAX_RMW_PREFIX:-$HOME/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/local/opt/ros/humble}"
export AMENT_PREFIX_PATH="$STAGE_RMW:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$STAGE_RMW:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$STAGE_RMW/lib:$STAGE_RMW/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH"
export PATH="$STAGE_RMW/bin:$STAGE_RMW/lib/rmw_zenoh_cpp:$PATH"
source "$STAGE_WS/install/local_setup.bash"
for component in interfaces native tracker bt rviz application; do
  source "$D1MAX_RELEASE/$component/install/local_setup.bash"
done
export D1MAX_GLOBAL_STAGE_ROOT="$STAGE_WS/experiments/crossfloor_global_20261002"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=219
export D1MAX_NAV_ISOLATED=1 D1MAX_OFFLINE_ZENOH_TEST=1 D1MAX_NAV_TRANSPORT=isolated_mock
export D1MAX_NAV_ISOLATION_TOKEN=10202190219021902190219021902190
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG ZENOH_ROUTER_CONFIG ZENOH_SESSION_CONFIG_URI
if [ "${1:-}" = build ]; then
  exec colcon --log-base "$D1MAX_GLOBAL_STAGE_ROOT/build_logs" build \
    --base-paths "$STAGE_WS/src/d1max_navigation_bt" "$STAGE_WS/src/d1max_pct_rviz_tools" \
    --packages-select d1max_navigation_bt d1max_pct_rviz_tools --parallel-workers 2 \
    --build-base "$D1MAX_GLOBAL_STAGE_ROOT/build" --install-base "$D1MAX_GLOBAL_STAGE_ROOT/install" \
    --allow-overriding d1max_navigation_bt d1max_pct_rviz_tools \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
fi
if [ -f "$D1MAX_GLOBAL_STAGE_ROOT/install/local_setup.bash" ]; then
  source "$D1MAX_GLOBAL_STAGE_ROOT/install/local_setup.bash"
fi
# Development modules are explicit, not mistaken for deployed release imports.
export PYTHONPATH="$STAGE_WS/src/d1max_pct_planner:$STAGE_WS/src/d1max_pct_scan:$STAGE_WS:$PYTHONPATH"
if [ "${1:-}" = check ]; then
  exec /usr/bin/python3 -m pytest "$STAGE_WS/src/d1max_pct_planner/test/test_tomogram_selection.py" \
    "$STAGE_WS/src/d1max_pct_planner/test/test_preview_markers.py" \
    "$STAGE_WS/src/d1max_pct_planner/test/test_crossfloor_preview.py" \
    "$STAGE_WS/src/d1max_pct_scan/test/test_global_stage_contract.py" \
    "$STAGE_WS/src/d1max_pct_scan/test/test_source_route.py" \
    "$STAGE_WS/src/d1max_pct_scan/test/test_native_global_worker.py" \
    "$STAGE_WS/src/d1max_pct_scan/test/test_bt_action_adapters.py" -q
fi
if [ "${1:-}" = test-native ]; then
  ctest --test-dir "$D1MAX_GLOBAL_STAGE_ROOT/build/d1max_navigation_bt" --output-on-failure
  exec ctest --test-dir "$D1MAX_GLOBAL_STAGE_ROOT/build/d1max_pct_rviz_tools" --output-on-failure
fi
exec /usr/bin/python3 -m d1max_pct_scan.crossfloor_global_session "$@"
