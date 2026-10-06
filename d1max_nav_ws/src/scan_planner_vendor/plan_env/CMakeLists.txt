cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(d1max_planning_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(plan_env src/grid_map.cpp src/raycast.cpp src/static_occupancy_prior.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  cv_bridge d1max_planning_interfaces geometry_msgs message_filters nav_msgs pcl_conversions rclcpp
  sensor_msgs std_msgs tf2 tf2_ros visualization_msgs
)
target_link_libraries(plan_env Eigen3::Eigen nlohmann_json::nlohmann_json OpenSSL::Crypto ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge d1max_planning_interfaces Eigen3 geometry_msgs message_filters nav_msgs pcl_conversions
  rclcpp sensor_msgs std_msgs nlohmann_json OpenSSL tf2 tf2_ros visualization_msgs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_solve_budget test/test_solve_budget.cpp)
  target_include_directories(test_solve_budget PRIVATE include)
  ament_add_gtest(test_voxel_collision test/test_voxel_collision.cpp)
  target_include_directories(test_voxel_collision PRIVATE include)
  target_link_libraries(test_voxel_collision Eigen3::Eigen)
  ament_add_gtest(test_near_field_diagnostics test/test_near_field_diagnostics.cpp)
  target_include_directories(test_near_field_diagnostics PRIVATE include)
  target_link_libraries(test_near_field_diagnostics Eigen3::Eigen)
  ament_add_gtest(test_map_context test/test_map_context.cpp)
  target_link_libraries(test_map_context plan_env)
  ament_add_gtest(test_projected_rays test/test_projected_rays.cpp)
  target_link_libraries(test_projected_rays plan_env)
  ament_add_gtest(test_static_occupancy_prior test/test_static_occupancy_prior.cpp)
  target_link_libraries(test_static_occupancy_prior plan_env)
  ament_add_gtest(test_official_resolution test/test_official_resolution.cpp)
  target_link_libraries(test_official_resolution plan_env)
  # Explicit offline CLI only; not a CTest or ROS executable/node.
  add_executable(offline_projected_rays_probe test/offline_projected_rays_probe.cpp)
  target_link_libraries(offline_projected_rays_probe plan_env)
  add_executable(offline_collision_performance_probe test/offline_collision_performance_probe.cpp)
  target_link_libraries(offline_collision_performance_probe plan_env)
endif()
ament_package()
