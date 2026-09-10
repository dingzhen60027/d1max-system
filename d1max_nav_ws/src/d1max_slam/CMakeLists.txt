cmake_minimum_required(VERSION 3.8)
project(d1max_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(dual_lidar_adapter src/dual_lidar_adapter.cpp)
ament_target_dependencies(dual_lidar_adapter
  rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs livox_ros_driver2)

add_executable(map_capture_node src/map_capture_node.cpp)
ament_target_dependencies(map_capture_node
  rclcpp sensor_msgs std_srvs pcl_conversions)
target_include_directories(map_capture_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_capture_node ${PCL_LIBRARIES})

install(TARGETS
  dual_lidar_adapter
  map_capture_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
