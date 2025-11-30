cmake_minimum_required(VERSION 3.8)
project(nav_virtual_lanes_planner)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependências do ament / ROS2 / Nav2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Diretórios de include
include_directories(
  include
)

# Biblioteca do planner (compila o seu src/virtual_lanes_planner.cpp)
add_library(${PROJECT_NAME} SHARED
  src/virtual_lanes_planner.cpp
)

# Liga a biblioteca com as dependências
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Exporta a descrição de plugin para o nav2_core (virtual_lanes_planner.xml)
pluginlib_export_plugin_description_file(nav2_core virtual_lanes_planner.xml)

# Exporta a biblioteca para outros pacotes, se precisarem linkar
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Instala a biblioteca compilada
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Instala os headers do include/
install(DIRECTORY include/
  DESTINATION include
)

# Finaliza o pacote ament
ament_package()
