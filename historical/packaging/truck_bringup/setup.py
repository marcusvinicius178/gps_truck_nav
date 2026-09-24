from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'truck_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/arocs_truck'),
         glob('models/rocs_truck/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = truck_bringup.logged_waypoint_follower:main',
            'interactive_waypoint_follower = truck_bringup.interactive_waypoint_follower:main',
            'gps_waypoint_logger = truck_bringup.gps_waypoint_logger:main',
            'follow_dynamic_gps_wps_lanes = truck_bringup.follow_dynamic_gps_wps_lanes:main',
            'broadcast_odom_base_footprint = truck_bringup.broadcast_odom_base_footprint:main',
            'camera_costmap_generator = truck_bringup.camera_costmap_generator:main',
            'classificador = truck_bringup.classificador:main',
            'gps_datum_setter = truck_bringup.gps_datum_setter:main'
        ],
    },
)
