#!/usr/bin/env bash
set -euo pipefail
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_output="${1:?usage: run_forward_closed_loop.sh OUTPUT_DIRECTORY [TRACKER_INCLUDE [TRACKING_YAML [RUNTIME_JSON]]]}"
task_include="${2:-$task_ws/src/d1max_trajectory_tracker/include}"
task_yaml="${3:-$task_ws/src/d1max_pct_scan/config/motion_tracking.yaml}"
task_profile="${4:-$task_ws/src/d1max_pct_scan/config/runtime_motion_profile.json}"
task_sdk="${D1MAX_SDK_SOURCE_DIR:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/src}"
mkdir -p "$task_output"
sha256sum "$task_include/d1max_trajectory_tracker/tracker_core.hpp" \
  "$task_include/d1max_trajectory_tracker/forward_steering_controller.hpp" \
  "$task_yaml" "$task_profile" "$task_sdk/sdk_input_mapping.hpp" \
  "$task_ws/src/d1max_trajectory_tracker/test/test_forward_closed_loop.cpp" \
  > "$task_output/input_sha256.txt"
for task_header in motion_command_conditioner response_command_limiter velocity_smoother fixed_bspline_sampler; do
  task_header_path="$task_include/d1max_trajectory_tracker/$task_header.hpp"
  if [[ -f "$task_header_path" ]]; then
    sha256sum "$task_header_path" >> "$task_output/input_sha256.txt"
  fi
done
sha256sum "$task_ws/src/scan_planner_vendor/bspline_opt/src/uniform_bspline.cpp" \
  "$task_ws/src/scan_planner_vendor/bspline_opt/include/bspline_opt/spline_heading_contract.hpp" \
  "$task_ws/src/scan_planner_vendor/bspline_opt/include/bspline_opt/near_curve_direction.hpp" \
  >> "$task_output/input_sha256.txt"
g++ -std=c++17 -O2 -pthread -I "$task_include" -I /usr/include/eigen3 \
  -I "$task_ws/src/scan_planner_vendor/bspline_opt/include" -I "$task_sdk" \
  "$task_ws/src/d1max_trajectory_tracker/test/test_forward_closed_loop.cpp" \
  "$task_ws/src/scan_planner_vendor/bspline_opt/src/uniform_bspline.cpp" \
  -lgtest_main -lgtest -lyaml-cpp -o "$task_output/test_forward_closed_loop"
export D1MAX_TRACKING_CONFIG="$task_yaml"
export D1MAX_RUNTIME_MOTION_PROFILE="$task_profile"
export D1MAX_FORWARD_OUTPUT="$task_output"
# Same production-core entry now runs the original 40 contracts/scenes plus
# long-route response/feedback-delay sensitivity; no ROS or SDK connection.
task_args=(--gtest_output="xml:$task_output/results.xml")
if [[ -n "${D1MAX_FORWARD_GTEST_FILTER:-}" ]]; then
  task_args+=(--gtest_filter="$D1MAX_FORWARD_GTEST_FILTER")
fi
"$task_output/test_forward_closed_loop" "${task_args[@]}"
