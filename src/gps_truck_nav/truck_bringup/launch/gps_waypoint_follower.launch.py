import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # --- Locais do pacote ---
    bringup_dir = get_package_share_directory('truck_bringup')
    launch_dir  = os.path.join(bringup_dir, 'launch')
    params_dir  = os.path.join(bringup_dir, 'params')

    # --- Params do Nav2 (RewrittenYaml permite substituições via launch) ---
    nav2_params = os.path.join(params_dir, 'truck_nav2_params.yaml')
    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key="",
        param_rewrites="",
        convert_types=True
    )

    # ====== ARGUMENTOS ======
    use_rviz         = LaunchConfiguration('use_rviz')
    use_mapviz       = LaunchConfiguration('use_mapviz')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    autostart        = LaunchConfiguration('autostart')
    use_composition  = LaunchConfiguration('use_composition')
    container_name   = LaunchConfiguration('container_name')

    # Toggles para os ajustes de ambiente (todos default=on)
    use_gpu          = LaunchConfiguration('use_gpu')           # Força NVIDIA
    use_xcb          = LaunchConfiguration('use_xcb')           # Força backend X11 (evita bug do Wayland)
    use_local_models = LaunchConfiguration('use_local_models')  # Desliga Fuel e usa modelos locais
    set_gz_paths     = LaunchConfiguration('set_gz_paths')      # Injeta GAZEBO_MODEL_PATH/RESOURCE_PATH/OGRE

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False', description='Start RViz?')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz', default_value='True', description='Start Mapviz?')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use Gazebo clock')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart Nav2 nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Use composable nodes')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container', description='Composable container name')

    # Toggles de ambiente
    declare_use_gpu_cmd = DeclareLaunchArgument(
        'use_gpu', default_value='True',
        description='Force gzclient/rviz2 to render on NVIDIA (prime offload)')

    declare_use_xcb_cmd = DeclareLaunchArgument(
        'use_xcb', default_value='True',
        description='Force QT_QPA_PLATFORM=xcb (avoid Wayland issues)')

    declare_use_local_models_cmd = DeclareLaunchArgument(
        'use_local_models', default_value='True',
        description='Disable Fuel and prefer local models')

    declare_set_gz_paths_cmd = DeclareLaunchArgument(
        'set_gz_paths', default_value='True',
        description='Set GAZEBO_MODEL_PATH/RESOURCE_PATH/OGRE_RESOURCE_PATH')

    # ====== AJUSTES DE AMBIENTE (aplicam-se a TODOS os processos do launch) ======

    # 1) Força NVIDIA (equivalente ao "prime-run")
    set_gpu_env = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1', condition=IfCondition(use_gpu)),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME',  'nvidia', condition=IfCondition(use_gpu)),
    ]

    # 2) Evita o backend Wayland (o Gazebo 11/OGRE+Qt trava com frequência no Wayland)
    set_xcb_env = [
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb', condition=IfCondition(use_xcb)),
    ]

    # 3) Usa modelos locais e NÃO tenta baixar do Fuel (evita "Waiting for model database...")
    set_models_env = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '', condition=IfCondition(use_local_models)),
    ]

    # 4) Paths do Gazebo e OGRE
    #    * GAZEBO_MODEL_PATH: onde está seu "models/" do pacote
    #    * GAZEBO_RESOURCE_PATH: shaders, materials etc do Gazebo
    #    * OGRE_RESOURCE_PATH: RTShaderLib e demais assets do OGRE (ajuste se sua distro usar outro caminho)
    models_dir = os.path.join(bringup_dir, 'models')
    # Caminhos típicos no Ubuntu 22.04 com Gazebo 11 e OGRE 1.9:
    gazebo_share_guess = '/usr/share/gazebo-11'
    ogre_share_guess   = '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0'

    set_paths_env = [
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            f'{models_dir}:{os.environ.get("GAZEBO_MODEL_PATH", "")}',
            condition=IfCondition(set_gz_paths)
        ),
        SetEnvironmentVariable(
            'GAZEBO_RESOURCE_PATH',
            f'{gazebo_share_guess}:{os.environ.get("GAZEBO_RESOURCE_PATH", "")}',
            condition=IfCondition(set_gz_paths)
        ),
        SetEnvironmentVariable(
            'OGRE_RESOURCE_PATH',
            f'{ogre_share_guess}:{os.environ.get("OGRE_RESOURCE_PATH", "")}',
            condition=IfCondition(set_gz_paths)
        ),
    ]

    # ====== LAUNCHES INCLUÍDOS ======

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'truck_simulation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')),
        launch_arguments={
            'namespace': "",
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': container_name
        }.items()
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': "",
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'container_name': container_name
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Container + Nav2 (composable)
    nav2_cmd_group = GroupAction([
        PushROSNamespace(
            condition=IfCondition(use_composition),
            namespace=""),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            output='screen'),

        robot_localization_cmd,
        navigation2_cmd,
    ])

    # TF estática base_link -> cloud (sua rotação)
    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_cloud',
        arguments=['0', '0', '0', '0', '0', '0.70710678', '0.70710678', 'base_link', 'cloud'],
        output='screen'
    )

    # ====== ORDEM IMPORTA: primeiro os envs, depois os includes/nodes ======
    return LaunchDescription([
        # Declarações
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_use_mapviz_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_gpu_cmd,
        declare_use_xcb_cmd,
        declare_use_local_models_cmd,
        declare_set_gz_paths_cmd,

        # Envs globais (aplicam-se a tudo abaixo)
        *set_gpu_env,
        *set_xcb_env,
        *set_models_env,
        *set_paths_env,

        # Seus launches e nós
        gazebo_cmd,
        rviz_cmd,
        mapviz_cmd,
        nav2_cmd_group,
        static_transform_cmd,
    ])
