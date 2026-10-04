cmake_minimum_required(VERSION 3.8)
project(d1max_planning_interfaces)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
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
ament_package()
