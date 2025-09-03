#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import yaml
from robot_localization.srv import FromLL
from math import sin, cos, radians
import sys
import numpy as np 

class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from um arquivo YAML.
    """
    def __init__(self, wps_file_path: str):
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Retorna um array de coordenadas geográficas e yaw do arquivo YAML.
        """
        return self.wps_dict["waypoints"]

class LaneVisualizer(Node):
    def __init__(self, wps_file_path):
        super().__init__('lane_visualizer')
        self.marker_publisher_ = self.create_publisher(Marker, 'wall_marker', 10)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, 'virtual_wall_points', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting again...')
        self.waypoints = self.convert_gps_waypoints()

    def convert_geo_pose_to_pose(self, latitude, longitude, yaw):
        request = FromLL.Request()
        request.ll_point.latitude = float(latitude)
        request.ll_point.longitude = float(longitude)
        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.map_point:
            return {'x': response.map_point.x, 'y': response.map_point.y, 'yaw': yaw}
        else:
            self.get_logger().error('Failed to get valid response from fromLL service')
            return None

    def convert_gps_waypoints(self):
        gps_waypoints = self.wp_parser.get_wps()
        waypoints = [self.convert_geo_pose_to_pose(wp['latitude'], wp['longitude'], radians(wp['yaw'])) for wp in gps_waypoints]
        waypoints = [wp for wp in waypoints if wp is not None]
        return waypoints

    def interpolate_points(self, start, end, num_points=200):
        """Interpolate num_points between start and end points."""
        points = []
        dx = (end['x'] - start['x']) / num_points
        dy = (end['y'] - start['y']) / num_points
        for i in range(num_points + 1):
            point = Point()
            point.x = start['x'] + i * dx
            point.y = start['y'] + i * dy
            point.z = 0.0
            points.append(point)
        return points

    def timer_callback(self):
        lanes = self.create_lanes()
        for lane in lanes:
            self.marker_publisher_.publish(lane)

        pointcloud = self.create_pointcloud()
        self.pointcloud_publisher_.publish(pointcloud)

    def create_lanes(self):
        lanes = []
        offsets = [-7.5, -2.5, 2.5, 7.5]  # Left, center, right offsets in meters
        colors = [(0.0, 0.0, 0.0)] * len(offsets)  # All black lines

        for i, offset in enumerate(offsets):
            lane = Marker()
            lane.header.frame_id = "map"
            lane.header.stamp = self.get_clock().now().to_msg()
            lane.ns = "lanes"
            lane.id = i
            lane.type = Marker.LINE_STRIP
            lane.action = Marker.ADD
            lane.scale.x = 0.25  # Width of the lines
            lane.color.r = float(colors[i][0])
            lane.color.g = float(colors[i][1])
            lane.color.b = float(colors[i][2])
            lane.color.a = 1.0

            all_points = []

            for j in range(len(self.waypoints) - 1):
                start_wp = self.waypoints[j]
                end_wp = self.waypoints[j + 1]
                start_point = {
                    'x': start_wp['x'] - offset * sin(start_wp['yaw']),
                    'y': start_wp['y'] + offset * cos(start_wp['yaw']),
                    'yaw': start_wp['yaw']
                }
                end_point = {
                    'x': end_wp['x'] - offset * sin(end_wp['yaw']),
                    'y': end_wp['y'] + offset * cos(end_wp['yaw']),
                    'yaw': end_wp['yaw']
                }
                all_points += self.interpolate_points(start_point, end_point)

            for point in all_points:
                p = Point()
                p.x = point.x
                p.y = point.y
                p.z = 0.0
                lane.points.append(p)

            lanes.append(lane)
        return lanes

    def create_pointcloud(self):
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points = []
        offsets = [-7.5, -2.5, 2.5, 7.5]  # Left, center, right offsets in meters
        z_min = 0.0  # Starting height
        z_max = 3.0  # Maximum height for the virtual wall
        z_step = 0.5  # Step in Z direction

        for offset in offsets:
            for j in range(len(self.waypoints) - 1):
                start_wp = self.waypoints[j]
                end_wp = self.waypoints[j + 1]
                start_point = {
                    'x': start_wp['x'] - offset * sin(start_wp['yaw']),
                    'y': start_wp['y'] + offset * cos(start_wp['yaw'])
                }
                end_point = {
                    'x': end_wp['x'] - offset * sin(end_wp['yaw']),
                    'y': end_wp['y'] + offset * cos(end_wp['yaw'])
                }
                interpolated_points = self.interpolate_points(start_point, end_point)
                for point in interpolated_points:
                    for z in np.arange(z_min, z_max, z_step):
                        points.append((point.x, point.y, z))

        pointcloud = pc2.create_cloud(header, fields, points)
        return pointcloud

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run nav_virtual_lanes lane_visualizer <path_to_yaml_file>")
        return
    wps_file_path = sys.argv[1]
    node = LaneVisualizer(wps_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
