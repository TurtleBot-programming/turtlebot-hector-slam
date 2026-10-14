cmake_minimum_required(VERSION 3.8)
project(tb4_hector_slam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(frontier_explorer src/frontier_explorer.cpp)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
ament_target_dependencies(frontier_explorer
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs std_msgs tf2 tf2_ros)
install(TARGETS frontier_explorer DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/floor_manager.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  rviz
  map
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
