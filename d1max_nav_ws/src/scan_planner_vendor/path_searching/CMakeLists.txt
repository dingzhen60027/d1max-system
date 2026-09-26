cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)

add_library(path_searching src/dyn_a_star.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_searching plan_env rclcpp)
target_link_libraries(path_searching Eigen3::Eigen)

install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 plan_env rclcpp)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_search_lattice test/test_search_lattice.cpp)
  target_include_directories(test_search_lattice PRIVATE include)
  target_link_libraries(test_search_lattice Eigen3::Eigen)
  ament_add_gtest(test_search_endpoint test/test_search_endpoint.cpp)
  target_link_libraries(test_search_endpoint path_searching)
endif()
ament_package()
