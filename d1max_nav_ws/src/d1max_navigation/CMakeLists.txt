cmake_minimum_required(VERSION 3.8)
project(d1max_navigation)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/rviz_initial_pose_bridge scripts/localization_test_session scripts/navigation_test_status
  scripts/navigation_session scripts/navigation_simulator scripts/navigation_command_gate
  scripts/navigation_cloud_to_scan scripts/verify_navigation
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz behavior_trees DESTINATION share/${PROJECT_NAME})
install(FILES README.md LOCALIZATION_TEST.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_tests test TIMEOUT 45)
endif()
ament_package()
