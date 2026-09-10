cmake_minimum_required(VERSION 3.8)
project(fastlio2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SOPHUS_INSTALL OFF CACHE BOOL "" FORCE)
set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "" FORCE)
add_subdirectory(third_party/Sophus EXCLUDE_FROM_ALL)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

set(SRC_LIST
  src/map_builder/commons.cpp
  src/map_builder/ieskf.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
  src/utils.cpp
)

add_executable(lio_node src/lio_node.cpp ${SRC_LIST})
ament_target_dependencies(lio_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs livox_ros_driver2 pcl_conversions geometry_msgs)
target_link_libraries(lio_node
  yaml-cpp
  ${PCL_LIBRARIES}
  Sophus::Sophus
)

install(TARGETS lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fastlio2_core
    test/test_fastlio2_core.cpp
    src/map_builder/commons.cpp
    src/map_builder/ieskf.cpp
    src/map_builder/imu_processor.cpp
  )
  target_include_directories(test_fastlio2_core PRIVATE src/map_builder)
  target_link_libraries(test_fastlio2_core ${PCL_LIBRARIES} Sophus::Sophus)
endif()

ament_package()
