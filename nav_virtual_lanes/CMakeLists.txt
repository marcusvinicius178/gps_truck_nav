cmake_minimum_required(VERSION 3.5)
project(nav_virtual_lanes)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  nav_virtual_lanes/lane_costmap_generator.py
  nav_virtual_lanes/visualize_lanes_from_gps.py
  nav_virtual_lanes/lane_occupancy_detector.py
  nav_virtual_lanes/visualize_lanes_from_gps_and_wps.py
  nav_virtual_lanes/wall_costmap_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
