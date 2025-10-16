# truck_bringup/launch/truck_simulation_launch.py
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('truck_bringup')

    # === Mundo ===
    world_file = os.path.join(bringup_dir, 'worlds', 'empty_ground.world')

    # RViz opcional: mantido aqui como no seu arquivo original
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'truck.rviz')

    # URDF (publicado para TF/visualização)
    truck_urdf_file = os.path.join(bringup_dir, 'urdf', 'arocs_truck.urdf')
    with open(truck_urdf_file, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # === Paths Gazebo/OGRE ===
    truck_models_dir    = os.path.join(bringup_dir, 'models')
    gazebo_models_dir   = '/usr/share/gazebo-11/models'   # <- inclui ground_plane e sun
    gazebo_share_dir    = '/usr/share/gazebo-11'

    # MODEL PATH: inclui seus modelos + modelos nativos
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            truck_models_dir,
            os.pathsep,
            gazebo_models_dir,
            os.pathsep,
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')
        ]
    )

    # RESOURCE PATH: inclui diretório share do Gazebo (worlds, media, shaders, etc.)
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            gazebo_share_dir,
            os.pathsep,
            os.path.join(bringup_dir, 'worlds'),
            os.pathsep,
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')
        ]
    )

    # BLOQUEIA consultas à model database online (evita travas)
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # Qt/Wayland/GL: integração estável
    set_qt_gl_integration = SetEnvironmentVariable(
        name='QT_XCB_GL_INTEGRATION',
        value='none'
    )

    # --- Processos Gazebo ---
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # RViz opcional (mantido)
    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    # Publica o URDF para TF/RViz (não faz spawn; o spawn vem do <include> no .world)
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        set_gazebo_resource_path,
        disable_model_db,
        set_qt_gl_integration,

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        start_rviz_cmd,
        robot_state_publisher_cmd,
    ])
