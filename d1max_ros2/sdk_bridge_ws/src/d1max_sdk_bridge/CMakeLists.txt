cmake_minimum_required(VERSION 3.8)
project(d1max_sdk_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosgraph_msgs REQUIRED)

set(ROBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/robot_sdk")
set(ROBOT_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
set(ROBOT_SDK_LIBRARY
  "${ROBOT_SDK_ROOT}/lib/${ROBOT_SDK_ARCH}/librobot_sdk.so")

if(NOT EXISTS "${ROBOT_SDK_LIBRARY}")
  message(FATAL_ERROR
    "Robot SDK library not found for architecture ${ROBOT_SDK_ARCH}: "
    "${ROBOT_SDK_LIBRARY}")
endif()

add_library(robot_sdk_vendor SHARED IMPORTED GLOBAL)
set_target_properties(robot_sdk_vendor PROPERTIES
  IMPORTED_LOCATION "${ROBOT_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ROBOT_SDK_ROOT}/include")

add_executable(sdk_state_bridge src/sdk_state_bridge.cpp)
ament_target_dependencies(sdk_state_bridge
  geometry_msgs
  rclcpp
  std_msgs
  std_srvs)
target_link_libraries(sdk_state_bridge
  robot_sdk_vendor
  Threads::Threads)
set_target_properties(sdk_state_bridge PROPERTIES
  INSTALL_RPATH "\$ORIGIN/..")

add_executable(sdk_telemetry_bridge src/sdk_telemetry_bridge.cpp)
ament_target_dependencies(sdk_telemetry_bridge rclcpp std_msgs)
target_link_libraries(sdk_telemetry_bridge robot_sdk_vendor Threads::Threads nlohmann_json::nlohmann_json)
set_target_properties(sdk_telemetry_bridge PROPERTIES INSTALL_RPATH "\$ORIGIN/..")

add_executable(sdk_console_bridge src/sdk_console_bridge.cpp)
ament_target_dependencies(sdk_console_bridge rclcpp std_msgs std_srvs rosgraph_msgs)
target_link_libraries(sdk_console_bridge robot_sdk_vendor Threads::Threads nlohmann_json::nlohmann_json)
set_target_properties(sdk_console_bridge PROPERTIES INSTALL_RPATH "\$ORIGIN/..")
option(D1MAX_BUILD_CONSOLE_MOCK "Build non-installed SDK-free console test adapter" OFF)
add_executable(sdk_monitor_bridge src/sdk_monitor_bridge.cpp)
ament_target_dependencies(sdk_monitor_bridge rclcpp std_msgs std_srvs rosgraph_msgs)
target_link_libraries(sdk_monitor_bridge robot_sdk_vendor Threads::Threads nlohmann_json::nlohmann_json)
set_target_properties(sdk_monitor_bridge PROPERTIES INSTALL_RPATH "\$ORIGIN/..")
if(D1MAX_BUILD_CONSOLE_MOCK)
  add_executable(sdk_console_bridge_mock src/sdk_console_bridge.cpp src/mock_sdk_console.cpp)
  target_include_directories(sdk_console_bridge_mock SYSTEM PRIVATE "${ROBOT_SDK_ROOT}/include")
  ament_target_dependencies(sdk_console_bridge_mock rclcpp std_msgs std_srvs rosgraph_msgs)
  target_link_libraries(sdk_console_bridge_mock Threads::Threads nlohmann_json::nlohmann_json)
endif()

install(TARGETS sdk_state_bridge sdk_telemetry_bridge sdk_console_bridge sdk_monitor_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  "${ROBOT_SDK_ROOT}/lib/${ROBOT_SDK_ARCH}/"
  DESTINATION lib
  FILES_MATCHING PATTERN "librobot_sdk.so*")
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  foreach(test_name mc_report monitor_estop sdk_session sdk_ownership)
    add_executable(test_${test_name} src/test_${test_name}.cpp)
    # Keep assertions active in Release builds as well.
    target_compile_options(test_${test_name} PRIVATE -UNDEBUG)
    add_test(NAME test_${test_name} COMMAND test_${test_name})
  endforeach()
endif()

ament_package()
