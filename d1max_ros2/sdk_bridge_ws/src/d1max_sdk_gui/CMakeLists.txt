cmake_minimum_required(VERSION 3.8)
project(d1max_sdk_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(d1max_sdk_gui
  include/d1max_sdk_gui/main_window.hpp
  include/d1max_sdk_gui/ros_bridge_client.hpp
  src/main.cpp
  src/main_window.cpp
  src/ros_bridge_client.cpp)

target_include_directories(d1max_sdk_gui PRIVATE include)
ament_target_dependencies(d1max_sdk_gui
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs)
target_link_libraries(d1max_sdk_gui Qt5::Widgets)

install(TARGETS d1max_sdk_gui DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
