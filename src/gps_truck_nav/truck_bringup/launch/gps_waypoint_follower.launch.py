"""Full GPS/Nav2 simulation; includes Gazebo exactly once."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('truck_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    lc = LaunchConfiguration

    def include(filename, arguments, condition=None):
        options = {'condition': condition} if condition is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, filename)),
            launch_arguments=arguments.items(), **options)

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # CTVI matches the retained canonical datum and empty_world_gps_wps.yaml.
        # This public launch selection is not a historical benchmark run definition.
        DeclareLaunchArgument('world', default_value=os.path.join(bringup_dir, 'worlds', 'ctvi.world')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'params', 'truck_nav2_params.yaml')),
        DeclareLaunchArgument('localization_params_file', default_value=os.path.join(bringup_dir, 'params', 'dual_ekf_navsat_params.yaml')),
        DeclareLaunchArgument('use_gzclient', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_mapviz', default_value='false'),
        DeclareLaunchArgument('mapviz_config', default_value=os.path.join(bringup_dir, 'params', 'gps_wpf_demo.mvc')),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_gpu', default_value='false', description='Opt in to NVIDIA PRIME offload'),
        DeclareLaunchArgument('use_xcb', default_value='true'),
    ]
    return LaunchDescription(arguments + [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1', condition=IfCondition(lc('use_gpu'))),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia', condition=IfCondition(lc('use_gpu'))),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb', condition=IfCondition(lc('use_xcb'))),
        include('truck_simulation_launch.py', {
            'world': lc('world'), 'use_sim_time': lc('use_sim_time'),
            'use_gzclient': lc('use_gzclient'), 'use_rviz': 'false',
        }),
        Node(
            condition=IfCondition(lc('use_composition')),
            package='rclcpp_components', executable='component_container_isolated',
            name=lc('container_name'), output='screen',
            parameters=[{'use_sim_time': lc('use_sim_time')}]),
        include('dual_ekf_navsat.launch.py', {
            'use_sim_time': lc('use_sim_time'),
            'localization_params_file': lc('localization_params_file'),
        }),
        include('navigation_launch.py', {
            'namespace': '', 'use_sim_time': lc('use_sim_time'),
            'params_file': lc('params_file'), 'autostart': lc('autostart'),
            'use_composition': lc('use_composition'), 'container_name': lc('container_name'),
        }),
        include('rviz_launch.py', {'use_sim_time': lc('use_sim_time')}, IfCondition(lc('use_rviz'))),
        include('mapviz.launch.py', {
            'mapviz_config': lc('mapviz_config'), 'use_sim_time': lc('use_sim_time'),
        }, IfCondition(lc('use_mapviz'))),
        # Retained transform. The URDF also defines cloud; see docs/KNOWN_LIMITATIONS.md.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_transform_cloud', output='screen',
            arguments=['0', '0', '0', '0', '0', '0.70710678', '0.70710678', 'base_link', 'cloud']),
    ])
