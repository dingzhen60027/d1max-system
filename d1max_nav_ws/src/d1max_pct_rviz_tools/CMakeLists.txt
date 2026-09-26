cmake_minimum_required(VERSION 3.8)
project(d1max_pct_rviz_tools)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_library(${PROJECT_NAME} SHARED
  include/d1max_pct_rviz_tools/point_tools.hpp
  include/d1max_pct_rviz_tools/planning_panel.hpp
  include/d1max_pct_rviz_tools/navigation_diagnostics_panel.hpp
  include/d1max_pct_rviz_tools/motion_control_panel.hpp
  src/point_tools.cpp src/planning_panel.cpp src/navigation_diagnostics_panel.cpp src/motion_control_panel.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav_msgs pluginlib rclcpp rviz_common rviz_default_plugins std_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugins.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rviz_tools test/test_rviz_tools.cpp test/test_motion_control_panel.cpp)
  target_link_libraries(test_rviz_tools ${PROJECT_NAME} Qt5::Widgets)
  ament_target_dependencies(test_rviz_tools rviz_common rviz_default_plugins pluginlib)
  set_tests_properties(test_rviz_tools PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(geometry_msgs nav_msgs pluginlib rclcpp rviz_common rviz_default_plugins std_msgs)
ament_package()
