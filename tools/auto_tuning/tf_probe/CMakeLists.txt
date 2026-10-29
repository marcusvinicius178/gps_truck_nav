cmake_minimum_required(VERSION 3.16)
project(nav2_phase_a_tf_probe LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(tf_authority_probe tf_authority_probe.cpp)
target_compile_features(tf_authority_probe PRIVATE cxx_std_17)
target_compile_options(tf_authority_probe PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tf_authority_probe rclcpp tf2_msgs)
install(TARGETS tf_authority_probe RUNTIME DESTINATION bin)
