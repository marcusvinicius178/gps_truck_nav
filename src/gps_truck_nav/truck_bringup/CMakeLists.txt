cmake_minimum_required(VERSION 3.5)
project(truck_bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_cmake_python REQUIRED)  

nav2_package()

# Include headers
include_directories(include)

# Declare a C++ library
add_library(front_cabin_goal_checker SHARED
  src/front_cabin_goal_checker.cpp)

ament_target_dependencies(front_cabin_goal_checker
  rclcpp
  nav2_core
  geometry_msgs
  pluginlib)

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

# Install Python scripts to lib/${PROJECT_NAME}
file(GLOB PYTHON_SCRIPTS "scripts/*.py")
install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
