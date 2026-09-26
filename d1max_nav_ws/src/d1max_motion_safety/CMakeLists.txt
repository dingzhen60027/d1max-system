cmake_minimum_required(VERSION 3.8)
project(d1max_motion_safety)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_collision_monitor REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(continuous_collision_monitor_core SHARED src/continuous_collision_monitor.cpp)
target_include_directories(continuous_collision_monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(continuous_collision_monitor_core
  geometry_msgs nav2_collision_monitor nav2_util rclcpp rclcpp_lifecycle)

add_executable(continuous_collision_monitor src/main.cpp)
target_link_libraries(continuous_collision_monitor continuous_collision_monitor_core)
ament_target_dependencies(continuous_collision_monitor rclcpp)
install(TARGETS continuous_collision_monitor_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS continuous_collision_monitor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest_executable(test_continuous_collision_monitor test/test_continuous_collision_monitor.cpp)
  if(TARGET test_continuous_collision_monitor)
    target_link_libraries(test_continuous_collision_monitor continuous_collision_monitor_core)
    ament_target_dependencies(test_continuous_collision_monitor
      geometry_msgs nav2_collision_monitor nav2_util rclcpp rclcpp_lifecycle)
    set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_continuous_collision_monitor.gtest.xml")
    ament_add_test(test_continuous_collision_monitor
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/run_offline_test.py"
        "$<TARGET_FILE:test_continuous_collision_monitor>" "--gtest_output=xml:${result_file}"
      RESULT_FILE "${result_file}"
      OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/test_continuous_collision_monitor.log"
      TIMEOUT 35)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(continuous_collision_monitor_core)
ament_export_dependencies(geometry_msgs nav2_collision_monitor nav2_util rclcpp rclcpp_lifecycle)
ament_package()
