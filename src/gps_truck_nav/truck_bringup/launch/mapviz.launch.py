"""Optional Mapviz display with a user-supplied local configuration."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('truck_bringup')
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mapviz_config', default_value=os.path.join(share, 'params', 'gps_wpf_demo.mvc')),
        Node(package='mapviz', executable='mapviz', name='mapviz',
             parameters=[{'config': LaunchConfiguration('mapviz_config')}, clock]),
        Node(package='swri_transform_util', executable='initialize_origin.py',
             name='initialize_origin', parameters=[clock], remappings=[('fix', 'gps/fix')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='swri_transform', arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin']),
    ])
