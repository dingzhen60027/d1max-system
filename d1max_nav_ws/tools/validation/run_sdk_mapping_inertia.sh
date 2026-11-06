#!/usr/bin/env bash
# Pure C++ only. No ROS initialization, shared vendor SDK, service or deployment.
set -euo pipefail
task_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
task_sdk_source="${D1MAX_SDK_SOURCE:-/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws/src/d1max_sdk_bridge/src}"
if [[ $# -gt 1 ]]; then
  printf 'Usage: bash %s [output_directory]\n' "${BASH_SOURCE[0]}" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then
  mkdir -p -- "$1"
  task_output="$(cd "$1" && pwd)"
else
  task_output="$(mktemp -d /tmp/d1max-sdk-inertia.XXXXXX)"
fi
task_cxx="${CXX:-g++}"
"$task_cxx" -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread \
  -I/usr/include/eigen3 \
  -I"$task_repo/src/d1max_trajectory_tracker/include" \
  -I"$task_repo/src/scan_planner_vendor/bspline_opt/include" \
  -I"$task_sdk_source" \
  "$task_repo/src/d1max_trajectory_tracker/test/test_sdk_mapping_inertia.cpp" \
  "$task_repo/src/scan_planner_vendor/bspline_opt/src/uniform_bspline.cpp" \
  -lyaml-cpp -lgtest -lgtest_main -o "$task_output/test_sdk_mapping_inertia"
sha256sum \
  "$task_repo/src/d1max_trajectory_tracker/test/test_sdk_mapping_inertia.cpp" \
  "$task_repo/src/d1max_trajectory_tracker/include/d1max_trajectory_tracker/tracker_core.hpp" \
  "$task_repo/src/d1max_pct_scan/config/motion_tracking.yaml" \
  "$task_repo/src/d1max_pct_scan/config/runtime_motion_profile.json" \
  "$task_sdk_source/sdk_input_mapping.hpp" \
  "$task_sdk_source/execution_kinematics.hpp" | tee "$task_output/source_hashes.txt"
# Current default: actual source YAML + JSON, real controller + real static
# SDK mapper. Historical V1/V2/V3 and invented-inertia experiments remain
# explicit opt-in: D1MAX_SDK_MAPPING_TEST_FILTER='SdkMappingInertia.*'. They
# are not current deployment acceptance, even when their math tests pass.
task_filter="${D1MAX_SDK_MAPPING_TEST_FILTER:-SdkV4ContinuousControl.*:SdkCurrentProfile.*}"
printf 'SDK mapping test filter: %s\n' "$task_filter"
D1MAX_INERTIA_OUTPUT="$task_output" "$task_output/test_sdk_mapping_inertia" \
  --gtest_filter="$task_filter" \
  --gtest_output="xml:$task_output/gtest.xml" | tee "$task_output/test.log"
printf 'Sensitivity output: %s\n' "$task_output"
