import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='truck_bringup',
            executable='classificador',
            name='camera_segmentation'
        ),
        Node(
            package='truck_bringup',
            executable='camera_costmap_generator',
            name='costmap_generator'
        )
    ])
