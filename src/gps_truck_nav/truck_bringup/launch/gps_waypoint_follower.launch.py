"""Full GPS/Nav2 simulation; includes Gazebo exactly once."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('truck_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    lc = LaunchConfiguration

    # Composed planner/controller load requests only contain parameters selected
    # for those parent nodes. Their Costmap2DROS children use new NodeOptions and
    # need the complete file in the container process's global ROS arguments.
    # This full GPS launch currently uses the root namespace throughout.
    container_params = ParameterFile(
        RewrittenYaml(
            source_file=lc('params_file'), root_key='',
            param_rewrites={'autostart': lc('autostart')}, convert_types=True),
        allow_substs=True)

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
        DeclareLaunchArgument(
            'use_ackermann_bt', default_value='false',
            description='Opt in to prospective no-Spin/no-BackUp trees for both navigation actions'),
        DeclareLaunchArgument('use_gpu', default_value='false', description='Opt in to NVIDIA PRIME offload'),
        DeclareLaunchArgument('use_xcb', default_value='true'),
        DeclareLaunchArgument(
            'publish_legacy_cloud_tf', default_value='true',
            description='Retain legacy cloud TF; disable for the audited URDF-only sensor transform'),
    ]
    return LaunchDescription(arguments + [
        # Evaluate the original params_file before replacing its launch value.
        # Both the container and navigation include then receive this same
        # prospective source; no installed or canonical YAML is modified.
        SetLaunchConfiguration(
            'params_file',
            RewrittenYaml(
                source_file=lc('params_file'),
                param_rewrites={
                    'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                        '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_to_pose.xml',
                    'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                        '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_through_poses.xml',
                },
                convert_types=True),
            condition=IfCondition(lc('use_ackermann_bt'))),
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
            parameters=[container_params, {'use_sim_time': lc('use_sim_time')}]),
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
        # Compatibility default: the URDF also defines base_link -> cloud.
        # New audited trials disable this legacy publisher to avoid two authorities.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_transform_cloud', output='screen',
            condition=IfCondition(lc('publish_legacy_cloud_tf')),
            arguments=['0', '0', '0', '0', '0', '0.70710678', '0.70710678', 'base_link', 'cloud']),
    ])
