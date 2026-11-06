#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_output="${1:?usage: run_rolling_follower_native.sh OUTPUT_DIRECTORY [TRACKER_INCLUDE]}"
task_tracker="${2:-$task_root/src/d1max_trajectory_tracker/include}"
task_native="${D1MAX_ROLLING_NATIVE_INCLUDE:-$task_root/src/scan_planner_vendor/plan_manage/include}"
task_tracker="$(cd -- "$task_tracker" && pwd)"
task_native="$(cd -- "$task_native" && pwd)"
task_build="$task_root/experiments/navigation_runtime/native/build/scan_planner"
task_sdk="${D1MAX_SDK_SOURCE_DIR:-$task_root/experiments/navigation_runtime/source_snapshot/sdk_bridge_ws/src/d1max_sdk_bridge/src}"
mkdir -p "$task_output"
task_output="$(cd -- "$task_output" && pwd)"
task_defs=$(sed -n 's/^CXX_DEFINES = //p' "$task_build/CMakeFiles/test_local_follower_native.dir/flags.make")
task_includes=$(sed -n 's/^CXX_INCLUDES = //p' "$task_build/CMakeFiles/test_local_follower_native.dir/flags.make")
# Reuse the recorded production dependency link, but write only our isolated
# object/executable. No make/install, ROS context, SDK connection or process.
cd "$task_build"
/usr/bin/c++ $task_defs -std=c++17 -O2 -g0 -fPIC -I"$task_tracker" -I"$task_sdk" -I"$task_native" \
  $task_includes -c "$task_root/src/scan_planner_vendor/plan_manage/test/test_rolling_follower_native.cpp" \
  -o "$task_output/rolling.o"
task_link=$(<"$task_build/CMakeFiles/test_local_follower_native.dir/link.txt")
task_link=${task_link/CMakeFiles\/test_local_follower_native.dir\/test\/test_local_follower_native.cpp.o/$task_output\/rolling.o}
task_link=${task_link/ -o test_local_follower_native / -o $task_output\/test_rolling_follower_native }
eval "$task_link -lyaml-cpp"
sha256sum "$task_tracker/d1max_trajectory_tracker/tracker_core.hpp" \
  "$task_tracker/d1max_trajectory_tracker/local_follower.hpp" \
  "$task_native/plan_manage/execution_validator.hpp" \
  "$task_root/src/scan_planner_vendor/plan_manage/test/test_rolling_follower_native.cpp" \
  "$task_root/src/d1max_pct_scan/config/motion_tracking.yaml" \
  "$task_root/src/d1max_pct_scan/config/runtime_motion_profile.json" \
  "$task_sdk/sdk_input_mapping.hpp" > "$task_output/inputs.sha256"
export D1MAX_ROLLING_SOURCE="$task_root"
"$task_output/test_rolling_follower_native" --gtest_output="xml:$task_output/results.xml"
