cmake_minimum_required(VERSION 3.8)
project(d1max_planning_interfaces)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReferencePath.msg" "msg/TaggedBspline.msg" "msg/LocalPlanDebug.msg"
  "msg/ProjectedRays.msg" "msg/TrackingProgress.msg" "msg/NavigationState.msg"
  "msg/LocalNavigationState.msg" "msg/RouteProgress.msg"
  "msg/ExecutionVersion.msg" "msg/ExecutionPermit.msg" "msg/MotionDemand.msg"
  "msg/TrajectoryValidation.msg" "msg/StopReport.msg" "msg/SDKExecutionState.msg"
  "msg/MotionValidation.msg" "msg/SDKStationaryEvidence.msg"
  "msg/ExecutionHandoffGrant.msg" "msg/PreparedMotionDemand.msg" "msg/ExecutionCommitAck.msg"
  "msg/SupportReference.msg"
  "msg/TrajectoryAdmission.msg" "msg/TrackerGeometryReceipt.msg"
  "msg/ReferenceProposal.msg" "msg/ReferenceReceipt.msg"
  "srv/ExecutionGrant.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs scan_planner_msgs std_msgs sensor_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(nlohmann_json OpenSSL)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_isolated_reference_model test/test_isolated_reference_model.cpp)
  target_include_directories(test_isolated_reference_model PRIVATE include)
  target_link_libraries(test_isolated_reference_model nlohmann_json::nlohmann_json OpenSSL::Crypto)
endif()
ament_package()
