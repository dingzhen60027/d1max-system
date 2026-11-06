cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(d1max_planning_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs d1max_planning_interfaces)

add_executable(scan_planner_node
  src/motion_sweep.cpp
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen OpenSSL::Crypto)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS scripts/keypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_role_executors test/test_role_executors.cpp TIMEOUT 30)
  target_include_directories(test_role_executors PRIVATE include)
  target_compile_definitions(test_role_executors PRIVATE
    D1MAX_EXECUTOR_TEST_SESSION="${CMAKE_CURRENT_SOURCE_DIR}/test/role_executors_session.json5")
  ament_target_dependencies(test_role_executors rclcpp plan_env)
  add_test(NAME test_committed_display_wiring_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_committed_display_wiring_no_ros.py)
  add_test(NAME test_selected_candidate_delivery_wiring_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_selected_candidate_delivery_wiring_no_ros.py)
  add_test(NAME test_owner_map_dispatch_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_owner_map_dispatch_no_ros.py
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/owner_map_dispatch_no_ros)
  add_test(NAME test_final_endpoint_reseed_wiring
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_final_endpoint_reseed_wiring.py)
  add_test(NAME test_preview_connection_reseed_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_preview_connection_reseed_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp)
  add_test(NAME test_subscription_purpose_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_subscription_purpose_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp)
  add_test(NAME test_heading_startup_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_heading_startup_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_manager.cpp
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/heading_startup_no_ros)
  add_test(NAME test_async_worker_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_async_worker_no_ros.py)
  add_test(NAME test_snapshot_reader_scope_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_snapshot_reader_scope_no_ros.py)
  add_test(NAME test_admission_reader_scope_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_admission_reader_scope_no_ros.py)
  add_test(NAME test_snapshot_writer_handoff_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_snapshot_writer_handoff_no_ros.py)
  add_test(NAME test_candidate_publication_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_candidate_publication_no_ros.py)
  add_test(NAME test_replan_handoff_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_replan_handoff_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp)
  add_test(NAME test_replan_recovery_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_replan_recovery_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp)
  add_test(NAME test_reference_callback_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_reference_callback_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/reference_callback_no_ros)
  add_test(NAME test_task_endpoint_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_task_endpoint_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp)
  add_test(NAME test_visual_diagnostics_no_ros
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_visual_diagnostics_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp
      --native-build ${CMAKE_CURRENT_BINARY_DIR}
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/visual_diagnostics_no_ros)
  add_test(NAME test_blocked_entry_target_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_blocked_entry_target_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp
      --native-build ${CMAKE_CURRENT_BINARY_DIR}
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/blocked_entry_target_no_ros)
  add_test(NAME test_preview_revalidation_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_preview_revalidation_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_replan_fsm.cpp
      --build-dir ${CMAKE_CURRENT_BINARY_DIR}/preview_revalidation_no_ros)
  add_test(NAME test_manager_curve_evidence_no_ros
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_manager_curve_evidence_no_ros.py
      --source ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_manager.cpp
      --native-build ${CMAKE_CURRENT_BINARY_DIR})
  ament_add_gtest(test_input_contract test/test_input_contract.cpp)
  target_include_directories(test_input_contract PRIVATE include)
  target_link_libraries(test_input_contract Eigen3::Eigen)
  ament_add_gtest(test_local_plan_debug test/test_local_plan_debug.cpp)
  target_include_directories(test_local_plan_debug PRIVATE include)
  ament_target_dependencies(test_local_plan_debug
    bspline_opt d1max_planning_interfaces geometry_msgs nav_msgs std_msgs)
  target_link_libraries(test_local_plan_debug Eigen3::Eigen)
  ament_add_gtest(test_collision_reference test/test_collision_reference.cpp)
  target_include_directories(test_collision_reference PRIVATE include)
  ament_target_dependencies(test_collision_reference bspline_opt)
  target_link_libraries(test_collision_reference Eigen3::Eigen)
  ament_add_gtest(test_trajectory_collision test/test_trajectory_collision.cpp)
  target_include_directories(test_trajectory_collision PRIVATE include)
  ament_target_dependencies(test_trajectory_collision bspline_opt)
  target_link_libraries(test_trajectory_collision Eigen3::Eigen)
  ament_add_gtest(test_initial_yaw_native test/test_initial_yaw_native.cpp)
  target_include_directories(test_initial_yaw_native PRIVATE include)
  ament_target_dependencies(test_initial_yaw_native bspline_opt plan_env)
  target_link_libraries(test_initial_yaw_native Eigen3::Eigen)
  ament_add_gtest(test_reference_failure_diagnostics test/test_reference_failure_diagnostics.cpp)
  target_include_directories(test_reference_failure_diagnostics PRIVATE include)
  ament_target_dependencies(test_reference_failure_diagnostics bspline_opt plan_env)
  target_link_libraries(test_reference_failure_diagnostics Eigen3::Eigen)
  ament_add_gtest(test_candidate_adoption test/test_candidate_adoption.cpp)
  target_include_directories(test_candidate_adoption PRIVATE include)
  ament_target_dependencies(test_candidate_adoption bspline_opt plan_env)
  target_link_libraries(test_candidate_adoption Eigen3::Eigen)
  ament_add_gtest(test_execution_ledger test/test_execution_ledger.cpp)
  target_include_directories(test_execution_ledger PRIVATE include)
  ament_target_dependencies(test_execution_ledger bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_execution_ledger Eigen3::Eigen)
  ament_add_gtest(test_splice_heading_continuity test/test_splice_heading_continuity.cpp)
  target_include_directories(test_splice_heading_continuity PRIVATE include)
  ament_target_dependencies(test_splice_heading_continuity bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_splice_heading_continuity Eigen3::Eigen)
  ament_add_gtest(test_motion_check_diagnostics test/test_motion_check_diagnostics.cpp)
  target_include_directories(test_motion_check_diagnostics PRIVATE include)
  ament_target_dependencies(test_motion_check_diagnostics d1max_planning_interfaces)
  ament_add_gtest(test_motion_sweep test/test_motion_sweep.cpp src/motion_sweep.cpp)
  target_include_directories(test_motion_sweep PRIVATE include)
  ament_target_dependencies(test_motion_sweep bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_motion_sweep Eigen3::Eigen OpenSSL::Crypto)
  # Production LocalFollower + production Native GridMap/validator, with only
  # clock/output and deterministic input buffers isolated from ROS/SDK.
  ament_add_gtest(test_local_follower_native test/test_local_follower_native.cpp src/motion_sweep.cpp)
  target_include_directories(test_local_follower_native PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_local_follower_native bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_local_follower_native Eigen3::Eigen OpenSSL::Crypto)
  ament_add_gtest(test_local_command_delivery test/test_local_command_delivery.cpp src/motion_sweep.cpp TIMEOUT 30)
  target_include_directories(test_local_command_delivery PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_local_command_delivery bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_local_command_delivery Eigen3::Eigen OpenSSL::Crypto)
  ament_add_gtest(test_snapshot_proof_delivery test/test_snapshot_proof_delivery.cpp src/motion_sweep.cpp)
  target_include_directories(test_snapshot_proof_delivery PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_snapshot_proof_delivery bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_snapshot_proof_delivery Eigen3::Eigen OpenSSL::Crypto)
  ament_add_gtest(test_measured_projection_recovery test/test_measured_projection_recovery.cpp src/motion_sweep.cpp)
  target_include_directories(test_measured_projection_recovery PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_measured_projection_recovery bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_measured_projection_recovery Eigen3::Eigen OpenSSL::Crypto)
  ament_add_gtest(test_native_local_body_input test/test_native_local_body_input.cpp src/motion_sweep.cpp)
  target_include_directories(test_native_local_body_input PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_native_local_body_input bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_native_local_body_input Eigen3::Eigen OpenSSL::Crypto)
  ament_add_gtest(test_candidate_admission_timing test/test_candidate_admission_timing.cpp src/motion_sweep.cpp)
  target_include_directories(test_candidate_admission_timing PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_candidate_admission_timing bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
  target_link_libraries(test_candidate_admission_timing Eigen3::Eigen OpenSSL::Crypto)
  # Opt-in pure integration test: use the actual SDK transport core, never the
  # vendor SDK client. This path is not a runtime dependency or launch entry.
  set(D1MAX_SDK_CORE_INCLUDE_DIR "" CACHE PATH "SDK bridge src directory for native-to-writer timing regression")
  if(D1MAX_SDK_CORE_INCLUDE_DIR)
    if(NOT EXISTS "${D1MAX_SDK_CORE_INCLUDE_DIR}/execution_transport_core.hpp" OR
       NOT EXISTS "${D1MAX_SDK_CORE_INCLUDE_DIR}/test_execution_fixture.hpp")
      message(FATAL_ERROR "D1MAX_SDK_CORE_INCLUDE_DIR must contain the actual SDK transport core and fixture")
    endif()
    ament_add_gtest(test_native_sdk_timing test/test_native_sdk_timing.cpp src/motion_sweep.cpp)
    target_include_directories(test_native_sdk_timing PRIVATE include
      ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include
      "${D1MAX_SDK_CORE_INCLUDE_DIR}")
    ament_target_dependencies(test_native_sdk_timing bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
    target_link_libraries(test_native_sdk_timing Eigen3::Eigen OpenSSL::Crypto)
    # Real Native/GridMap/LocalFollower exchange with command-driven motion;
    # no ROS graph, vendor SDK client or replacement collision map.
    ament_add_gtest(test_rolling_follower_native test/test_rolling_follower_native.cpp src/motion_sweep.cpp)
    target_include_directories(test_rolling_follower_native PRIVATE include
      ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include
      "${D1MAX_SDK_CORE_INCLUDE_DIR}")
    ament_target_dependencies(test_rolling_follower_native bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
    target_link_libraries(test_rolling_follower_native Eigen3::Eigen OpenSSL::Crypto yaml-cpp)
    set_tests_properties(test_rolling_follower_native PROPERTIES
      ENVIRONMENT "D1MAX_ROLLING_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/../../..")
    ament_add_gtest(test_measured_start_rolling test/test_measured_start_rolling.cpp src/motion_sweep.cpp)
    target_include_directories(test_measured_start_rolling PRIVATE include
      ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include
      "${D1MAX_SDK_CORE_INCLUDE_DIR}")
    ament_target_dependencies(test_measured_start_rolling bspline_opt plan_env scan_planner_msgs d1max_planning_interfaces)
    target_link_libraries(test_measured_start_rolling Eigen3::Eigen OpenSSL::Crypto yaml-cpp)
    set_tests_properties(test_measured_start_rolling PROPERTIES
      ENVIRONMENT "D1MAX_ROLLING_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/../../.." TIMEOUT 180)
  endif()
  ament_add_gtest(test_reference_target test/test_reference_target.cpp test/test_final_endpoint_short_connection.cpp)
  target_include_directories(test_reference_target PRIVATE include)
  ament_target_dependencies(test_reference_target bspline_opt)
  target_link_libraries(test_reference_target Eigen3::Eigen)
  ament_add_gtest(test_rolling_reference test/test_rolling_reference.cpp)
  target_include_directories(test_rolling_reference PRIVATE include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../d1max_trajectory_tracker/include)
  ament_target_dependencies(test_rolling_reference bspline_opt d1max_planning_interfaces)
  target_link_libraries(test_rolling_reference Eigen3::Eigen)
  ament_add_gtest(test_visible_reference_target test/test_visible_reference_target.cpp)
  target_include_directories(test_visible_reference_target PRIVATE include)
  ament_target_dependencies(test_visible_reference_target bspline_opt plan_env path_searching d1max_planning_interfaces)
  target_link_libraries(test_visible_reference_target Eigen3::Eigen)
  ament_add_gtest(test_stair_attitude_scope test/test_stair_attitude_scope.cpp)
  ament_add_gtest(test_stair_model test/test_stair_model.cpp)
  target_include_directories(test_stair_model PRIVATE include)
  ament_target_dependencies(test_stair_model bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_model Eigen3::Eigen)
  ament_add_gtest(test_stair_contact_query test/test_stair_contact_query.cpp)
  target_include_directories(test_stair_contact_query PRIVATE include)
  ament_target_dependencies(test_stair_contact_query bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_contact_query Eigen3::Eigen)
  ament_add_gtest(test_stair_contact_map test/test_stair_contact_map.cpp)
  target_include_directories(test_stair_contact_map PRIVATE include)
  ament_target_dependencies(test_stair_contact_map bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_contact_map Eigen3::Eigen)
  ament_add_gtest(test_stair_riser_contact test/test_stair_riser_contact.cpp)
  target_include_directories(test_stair_riser_contact PRIVATE include)
  ament_target_dependencies(test_stair_riser_contact bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_riser_contact Eigen3::Eigen)
  target_include_directories(test_stair_attitude_scope PRIVATE include)
  ament_target_dependencies(test_stair_attitude_scope bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_attitude_scope Eigen3::Eigen)
  ament_add_gtest(test_stair_forward_approach test/test_stair_forward_approach.cpp)
  target_include_directories(test_stair_forward_approach PRIVATE include)
  ament_target_dependencies(test_stair_forward_approach bspline_opt plan_env d1max_planning_interfaces)
  target_link_libraries(test_stair_forward_approach Eigen3::Eigen)
  ament_add_gtest(test_stair_search_support test/test_stair_search_support.cpp)
  target_include_directories(test_stair_search_support PRIVATE include)
  ament_target_dependencies(test_stair_search_support bspline_opt plan_env path_searching d1max_planning_interfaces)
  target_link_libraries(test_stair_search_support Eigen3::Eigen)
  ament_add_gtest(test_stair_clearance test/test_stair_clearance.cpp)
  target_include_directories(test_stair_clearance PRIVATE include)
  ament_target_dependencies(test_stair_clearance bspline_opt plan_env path_searching d1max_planning_interfaces)
  target_link_libraries(test_stair_clearance Eigen3::Eigen)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
