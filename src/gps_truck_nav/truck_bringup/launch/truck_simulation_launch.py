"""Gazebo and robot state only; localization and Navigation2 are separate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('truck_bringup')
    with open(os.path.join(bringup_dir, 'urdf', 'arocs_truck.urdf'), encoding='utf-8') as stream:
        robot_description = stream.read()
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(bringup_dir, 'worlds', 'empty_ground.world')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gzclient', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', [
            os.path.join(bringup_dir, 'models'), os.pathsep,
            '/usr/share/gazebo-11/models', os.pathsep,
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', [
            os.path.join(bringup_dir, 'worlds'), os.pathsep,
            '/usr/share/gazebo-11', os.pathsep,
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')]),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        ExecuteProcess(
            cmd=['gzserver', '--verbose', LaunchConfiguration('world'), '-s', 'libgazebo_ros_init.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['gzclient', '--verbose'], output='screen',
            condition=IfCondition(LaunchConfiguration('use_gzclient'))),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(bringup_dir, 'rviz', 'truck.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz'))),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'robot_description': robot_description}]),
    ])
