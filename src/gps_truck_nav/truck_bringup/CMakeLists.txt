cmake_minimum_required(VERSION 3.5)
project(truck_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()

# Include headers
include_directories(include)

# Declare a C++ library
add_library(front_cabin_goal_checker SHARED
  src/front_cabin_goal_checker.cpp)

ament_target_dependencies(front_cabin_goal_checker
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  angles
  tf2
  tf2_geometry_msgs)

# Export the plugin
pluginlib_export_plugin_description_file(nav2_core front_cabin_plugin.xml)

# Install directives
install(TARGETS front_cabin_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(DIRECTORY launch params rviz worlds urdf models 
DESTINATION share/${PROJECT_NAME})

# Installation of plugin XML file
install(FILES
  front_cabin_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# One Python implementation tree. CMake preserves the .py executable suffix.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR scripts)
install(PROGRAMS
  scripts/logged_waypoint_follower.py
  scripts/gps_waypoint_logger.py
  scripts/follow_dynamic_gps_wps_lanes.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
