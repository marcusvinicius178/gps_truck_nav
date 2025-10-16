# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('truck_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    farm_dir = get_package_share_directory('offroad_sim')

    # Get the directory of 'laser_scan_integrator' Sensor Fusion package
    laser_scan_integrator_dir = get_package_share_directory('laser_scan_integrator')

# Path to the launch file 'integrate_2_scan.launch.py'
    integrate_2_scan_launch_file = os.path.join(laser_scan_integrator_dir, 'launch', 'integrate_2_scan.launch.py')
    # Path to the set_datum.py script
    set_datum_script = os.path.join(bringup_dir, 'scripts', 'set_datum.py')


    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    #Adding localization node
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    gps_ekf_params_file = LaunchConfiguration('gps_ekf_params_file')
    navsat_params_file = LaunchConfiguration('navsat_params_file')

    # Create the launch configuration variables
    kinect_params_file = LaunchConfiguration('kinect_params_file')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'infinite_map.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'truck_nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'truck.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        # worlds/turtlebot3_worlds/waffle.model')
        # default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
        default_value=os.path.join(bringup_dir, 'worlds', 'Lencois_Paulista_Farm.world'),
        description='Full path to world model file to load')
    
    declare_ekf_params_file_cmd = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'ekf.yaml'),
        description='Full path to the EKF parameters file to use')
    
    declare_gps_ekf_params_file_cmd = DeclareLaunchArgument(
        'gps_ekf_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'gps_ekf.yaml'),
        description='Full path to the GPS EKF parameters file to use')
    
    declare_navsat_params_file_cmd = DeclareLaunchArgument(
        'navsat_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navsat.yaml'),
        description='Full path to the GPS EKF parameters file to use')
    
    declare_kinect_params_file_cmd = DeclareLaunchArgument(
        'kinect_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'kinect_params.yaml'),
        description='Full path to the Kinect parameters file to use')


    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',  '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    #urdf = os.path.join(bringup_dir, 'urdf', 'ackerman.urdf')
    urdf = os.path.join(bringup_dir, 'urdf', 'arocs_truck.urdf')


    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart}.items())
    
        # Include 'integrate_2_scan.launch.py'
    integrate_2_scan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(integrate_2_scan_launch_file),
        # Add any launch_arguments if your launch file requires any
    )

    #Adding the truck improved (fused sensor) localization node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/fused_imu_odom')]
    )

    # navsat_transform_node configuration
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/fused_imu_odom'), 
            ('gps/fix', '/gps/data'),  
            ('imu/data', '/imu'),  
            ('odometry/gps', '/odometry/gps')  
        ]
    )

# Second robot_localization node for GPS fusion
    gps_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='gps_ekf_filter_node',
        output='screen',
        parameters=[gps_ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/fused_gps_imu_odom'),  # Final fused data topic
            ('odometry/gps', '/odometry/gps')  # Transformed GPS data from navsat_transform_node
        ]
    )

    # Execute the set_datum.py script
    set_datum_cmd = ExecuteProcess(
        cmd=['python3', set_datum_script],
        output='screen'
    )

    
    # Kinect depthimage to laserscan conversion node
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[kinect_params_file, {'use_sim_time': use_sim_time, 'output_frame': 'kinect_depth_frame'}],
        remappings=[('depth', '/kinect_sensor/depth/image_raw'),
                    ('scan', '/kinect_scan'),
                    ('depth_camera_info', '/kinect_sensor/depth/camera_info')],
        output='screen')


    #Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(integrate_2_scan_cmd)
    ld.add_action(declare_navsat_params_file_cmd)

    #Localization
    ld.add_action(declare_ekf_params_file_cmd)
    ld.add_action(robot_localization_node)
    ld.add_action(declare_gps_ekf_params_file_cmd)
    ld.add_action(gps_robot_localization_node)
    ld.add_action(navsat_transform_node)
    ld.add_action(set_datum_cmd)


    #Kinect depth to scan
    ld.add_action(declare_kinect_params_file_cmd)
    ld.add_action(depthimage_to_laserscan_node)

    return ld