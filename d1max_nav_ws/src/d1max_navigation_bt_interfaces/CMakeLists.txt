cmake_minimum_required(VERSION 3.8)
project(d1max_navigation_bt_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputeRoute.action" "action/FollowRoute.action" "action/Navigate.action"
  "msg/NavigationHealth.msg" "msg/RouteSegment.msg" "msg/RouteSnapshot.msg" "msg/RouteReference.msg"
  "srv/ConfirmExecution.srv" "srv/PrepareTransition.srv" "srv/PrepareInitialPose.srv"
  "msg/InitialPoseOutcome.msg"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
