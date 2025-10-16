#!/usr/bin/env python3

from math import sin, cos
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_localization.srv import FromLL

def yaw_to_quaternion(yaw):
    """Convert yaw (in radians) to a quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=sin(yaw * 0.5),
        w=cos(yaw * 0.5)
    )

class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from a YAML file.
    """
    def __init__(self, wps_file_path: str):
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic coordinates and yaw from the YAML file.
        """
        return self.wps_dict["waypoints"]

class GpsWpCommander(Node):
    """
    Class to use Nav2 GPS waypoint follower to follow a set of waypoints logged in a YAML file.
    """
    def __init__(self, wps_file_path):
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting again...')

    def convert_geo_pose_to_pose_stamped(self, latitude, longitude, yaw):
        self.get_logger().info(f'Converting latitude: {latitude}, longitude: {longitude}, yaw: {yaw}')
        request = FromLL.Request()
        request.ll_point.latitude = float(latitude)
        request.ll_point.longitude = float(longitude)
        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.map_point:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            # Adding 4.8025 meters to the x-coordinate to account for the truck's base_link frame offset
            pose_stamped.pose.position.x = response.map_point.x + 4.8025
            pose_stamped.pose.position.y = response.map_point.y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = yaw_to_quaternion(yaw)
            self.get_logger().info(f'Converted to local coordinates: x={pose_stamped.pose.position.x}, y={response.map_point.y}')
            return pose_stamped
        else:
            self.get_logger().error('Failed to get valid response from fromLL service')
            return None

    def start_wpf(self):
        """Function to start the waypoint following using NavigateThroughPoses."""
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        gps_waypoints = self.wp_parser.get_wps()
        poses = [self.convert_geo_pose_to_pose_stamped(wp['latitude'], wp['longitude'], wp['yaw']) for wp in gps_waypoints if self.convert_geo_pose_to_pose_stamped(wp['latitude'], wp['longitude'], wp['yaw']) is not None]
        self.navigator.goThroughPoses(poses)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        print("Waypoints completed successfully")

def main():
    rclpy.init()
    default_yaml_file_path = os.path.join(get_package_share_directory("nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    yaml_file_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml_file_path
    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
