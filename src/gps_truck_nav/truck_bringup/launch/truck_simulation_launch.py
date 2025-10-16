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

    # === Mundo (ajuste aqui se quiser outro .world) ===
    world_file = os.path.join(bringup_dir, 'worlds', 'empty_ground.world')

    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'truck.rviz')
    truck_urdf_file = os.path.join(bringup_dir, 'urdf', 'arocs_truck.urdf')

    with open(truck_urdf_file, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # === Evitar travas: env para Gazebo/Qt/OpenGL ===
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.path.join(bringup_dir, 'models'),
            os.pathsep,
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')
        ]
    )

    # Worlds, media, materials etc.
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(bringup_dir, 'worlds'),
            os.pathsep,
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='')
        ]
    )

    # BLOQUEIA consultas à model database online (fonte comum de travas)
    disable_model_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    # Qt/Wayland/GL: integração mais estável em muitos desktops
    set_qt_gl_integration = SetEnvironmentVariable(
        name='QT_XCB_GL_INTEGRATION',
        value='none'
    )

    # Se ainda travar, DESCOMENTE abaixo para render por software (mais lento, mas à prova de driver):
    # force_sw_render = SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1')

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # Sobe o cliente com logs verbosos para capturar erros OGRE/GL
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    start_rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

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
        # force_sw_render,  # <-- descomente só se necessário
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        start_rviz_cmd,
        robot_state_publisher_cmd,
    ])
