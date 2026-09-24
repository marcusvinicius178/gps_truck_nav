#!/usr/bin/env python3
"""Convert stored GPS waypoints once each and submit NavigateThroughPoses.

The historical +4.8025 m map-x offset is deliberately preserved. This sender is
not a benchmark evaluator; a Nav2 action result is not a paper success metric.
"""

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from robot_localization.srv import FromLL
import yaml


def yaw_to_quaternion(yaw):
    """Convert yaw in radians to a quaternion."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


class YamlWaypointParser:
    """Read nonempty latitude/longitude/yaw mappings without altering values."""

    def __init__(self, wps_file_path):
        with open(wps_file_path, encoding='utf-8') as stream:
            self.wps_dict = yaml.safe_load(stream)
        waypoints = self.wps_dict.get('waypoints') if isinstance(self.wps_dict, dict) else None
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError('Waypoint YAML must contain a nonempty waypoints list')
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError(f'Waypoint {index} must be a mapping')
            for key in ('latitude', 'longitude', 'yaw'):
                value = waypoint.get(key)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f'Waypoint {index}: {key} must be a finite number')
            if not -90 <= waypoint['latitude'] <= 90 or not -180 <= waypoint['longitude'] <= 180:
                raise ValueError(f'Waypoint {index}: latitude/longitude outside valid ranges')

    def get_wps(self):
        return self.wps_dict['waypoints']


class GpsWpCommander(Node):
    """Use robot_localization/fromLL and Nav2's NavigateThroughPoses action."""

    def __init__(self, wps_file_path):
        # Validate the file before allocating ROS nodes.
        self.wp_parser = YamlWaypointParser(wps_file_path)
        super().__init__('gps_wp_commander')
        self.navigator = BasicNavigator('basic_navigator')
        self.from_ll_client = self.create_client(FromLL, 'fromLL')

    def convert_geo_pose_to_pose_stamped(self, latitude, longitude, yaw):
        request = FromLL.Request()
        request.ll_point.latitude = float(latitude)
        request.ll_point.longitude = float(longitude)
        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            future.cancel()
            raise RuntimeError('fromLL conversion timed out; check navsat_transform and localization')
        response = future.result()
        if response is None:
            raise RuntimeError('fromLL returned no valid response; no route was sent')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        # Historical map-x translation, retained exactly; not a rotated lever arm.
        pose.pose.position.x = response.map_point.x + 4.8025
        pose.pose.position.y = response.map_point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def start_wpf(self):
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError('ROS shut down while waiting for fromLL')
            self.get_logger().info('Waiting for fromLL; check the full GPS simulation launch')
        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        poses = []
        for waypoint in self.wp_parser.get_wps():
            pose = self.convert_geo_pose_to_pose_stamped(
                waypoint['latitude'], waypoint['longitude'], waypoint['yaw'])
            if pose is None:
                raise RuntimeError('Waypoint conversion failed; no partial route was sent')
            poses.append(pose)
        if not self.navigator.goThroughPoses(poses):
            raise RuntimeError('NavigateThroughPoses rejected the route')
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Nav2 action succeeded (not a benchmark success assessment)')
            return 0
        self.get_logger().error(f'Nav2 action finished without success: {result.name}')
        return 1


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('waypoint_yaml', nargs='?', help='YAML with latitude, longitude and yaw in radians')
    options = parser.parse_args(remove_ros_args(args=sys.argv if args is None else args)[1:])
    path = options.waypoint_yaml
    if path is None:
        path = os.path.join(get_package_share_directory('truck_bringup'), 'params', 'empty_world_gps_wps.yaml')
    rclpy.init(args=args)
    node = None
    try:
        node = GpsWpCommander(path)
        return node.start_wpf()
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.navigator.cancelTask()
        return 130
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        print(f'Waypoint sender failed: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.navigator.destroy_node()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
