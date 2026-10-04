#!/usr/bin/env bash
# Development diagnostics only. Never starts a live SDK or deploys a release.
set -eo pipefail
task_ws=$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)
export D1MAX_NAV_ROOT="$task_ws"
export D1MAX_RELEASE=$(/usr/bin/python3 "$task_ws/tools/release/single_floor_entry_preflight.py" select --nav-root "$task_ws")
export PYTHONPATH=
source /opt/ros/humble/setup.bash
task_rmw="${D1MAX_RMW_PREFIX:-$HOME/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/local/opt/ros/humble}"
export AMENT_PREFIX_PATH="$task_rmw:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$task_rmw:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$task_rmw/lib:$task_rmw/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH"
export PATH="$task_rmw/bin:$task_rmw/lib/rmw_zenoh_cpp:$PATH"
source "$task_ws/install/local_setup.bash"
for task_component in interfaces native tracker bt sdk rviz application; do
  source "$D1MAX_RELEASE/$task_component/install/local_setup.bash"
done
task_bt="$task_ws/experiments/crossfloor_global_20261002/install/local_setup.bash"
if [ -f "$task_bt" ]; then source "$task_bt"; fi
export D1MAX_LOCAL_AUDIT_ROOT="$task_ws/experiments/local_planning_audit_20261002"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=219
export D1MAX_NAV_ISOLATED=1 D1MAX_OFFLINE_ZENOH_TEST=1 D1MAX_NAV_TRANSPORT=isolated_mock
export D1MAX_NAV_ISOLATION_TOKEN=10202190219021902190219021902190
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 MKL_NUM_THREADS=1
unset ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG ZENOH_ROUTER_CONFIG ZENOH_SESSION_CONFIG_URI ZENOH_ROUTER_CONFIG_URI
export PYTHONPATH="$task_ws/src/d1max_pct_planner:$task_ws/src/d1max_pct_scan:$task_ws:$PYTHONPATH"
if [ "${1:-}" = build ]; then
  exec colcon --log-base "$D1MAX_LOCAL_AUDIT_ROOT/build_logs" build \
    --base-paths "$task_ws/src/scan_planner_vendor/plan_manage" "$task_ws/src/d1max_trajectory_tracker" \
    --packages-select scan_planner d1max_trajectory_tracker --parallel-workers 2 \
    --build-base "$D1MAX_LOCAL_AUDIT_ROOT/build" --install-base "$D1MAX_LOCAL_AUDIT_ROOT/install" \
    --allow-overriding scan_planner d1max_trajectory_tracker \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
fi
if [ -f "$D1MAX_LOCAL_AUDIT_ROOT/install/local_setup.bash" ]; then
  source "$D1MAX_LOCAL_AUDIT_ROOT/install/local_setup.bash"
fi
case "${1:-}" in
  contracts)
    shift
    exec /usr/bin/python3 -m pytest -q "$task_ws/src/d1max_pct_scan/test/test_continuous_reference.py" \
      "$task_ws/src/d1max_pct_scan/test/test_continuous_reference_transport.py" \
      "$task_ws/src/d1max_pct_scan/test/test_continuous_reference_node.py" \
      "$task_ws/src/d1max_pct_scan/test/test_bt_follow_policy.py" \
      "$task_ws/src/d1max_pct_scan/test/test_bt_follow_audit_boundaries.py" \
      "$task_ws/src/d1max_pct_scan/test/test_reference_refresh_progress.py" \
      "$task_ws/src/d1max_pct_scan/test/test_local_debug_snapshot_continuity.py" "$@"
    ;;
  graph)
    shift
    task_default_map=$(/usr/bin/python3 -c 'from d1max_pct_planner.paths import default_map_directory; print(default_map_directory())')
    task_map_named=false
    for task_argument in "$@"; do
      if [ "$task_argument" = --map-directory ] || [[ "$task_argument" = --map-directory=* ]]; then task_map_named=true; fi
    done
    # Maps are immutable input artifacts with original absolute identity paths.
    # Use that verified input directly rather than pretending a copied package
    # has been regenerated with different internal paths.
    if [ "$task_map_named" = false ]; then set -- "$@" --map-directory "$task_default_map"; fi
    export D1MAX_RELEASE="$D1MAX_LOCAL_AUDIT_ROOT/runtime_release"
    for task_component in interfaces native tracker bt sdk rviz application; do
      source "$D1MAX_RELEASE/$task_component/install/local_setup.bash"
    done
    export PYTHONPATH="$task_ws/src/d1max_pct_planner:$task_ws/src/d1max_pct_scan:$task_ws:$PYTHONPATH"
    exec /usr/bin/python3 "$task_ws/tools/validation/run_single_floor_graph.py" "$@"
    ;;
  bundle)
    exec /usr/bin/python3 "$task_ws/tools/validation/audit_local_planning_flow.py" bundle \
      --output "$D1MAX_LOCAL_AUDIT_ROOT/runtime_release"
    ;;
  audit|native-tests|envelope-probe|summarize)
    exec /usr/bin/python3 "$task_ws/tools/validation/audit_local_planning_flow.py" "$@"
    ;;
  *) echo 'Usage: local_planning_audit_entry.sh build | bundle | contracts | graph [probe options] | audit --route-artifact FILE --output DIR | native-tests --output DIR | envelope-probe --output DIR' >&2; exit 2;;
esac
