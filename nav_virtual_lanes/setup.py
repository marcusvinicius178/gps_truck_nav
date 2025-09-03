from setuptools import setup, find_packages

package_name = 'nav_virtual_lanes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'backup']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rota_2024',
    maintainer_email='marcus.posgrad.ufsc@gmail.com',
    description='Package to visualize virtual lanes in RViz and generate wall costmaps',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualize_lanes_from_gps_and_wps = nav_virtual_lanes.visualize_lanes_from_gps_and_wps:main',
            'lane_occupancy_detector = nav_virtual_lanes.lane_occupancy_detector:main',
            'wall_costmap_generator = nav_virtual_lanes.wall_costmap_generator:main',
            'lane_costmap_generator = nav_virtual_lanes.lane_costmap_generator:main',
        ],
    },
)
