#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import yaml
from robot_localization.srv import FromLL
from math import sin, cos, radians, pi
import sys

class YamlWaypointParser:
    def __init__(self, wps_file_path):
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        return self.wps_dict["waypoints"]

class LaneVisualizer(Node):
    def __init__(self, wps_file_path):
        super().__init__('lane_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.waypoint_pub_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.preferred_lane_sub_ = self.create_subscription(Bool, '/binary_state', self.preferred_lane_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting again...')
        self.waypoints = self.convert_gps_waypoints()
        self.preferred_lane = 1  # Inicialmente na lane central
        self.lane_state = True   # True for navigable (green), False for non-navigable (red)
        self.stable_lane_counter = 0
        self.required_stability_duration = 3
        self.last_preferred_lane = 1  # Para verificar mudanças de lane

    def preferred_lane_callback(self, msg):
        previous_lane = self.preferred_lane

        # Supondo que `msg.data` indica qual lane é preferida
        self.preferred_lane = msg.data

        # Atualiza o estado de navegabilidade com base na lane escolhida
        self.lane_state = True  # A lane preferida será navegável (verde)

        # Atualizar o contador de estabilidade apenas se a lane mudou
        if previous_lane != self.preferred_lane:
            self.stable_lane_counter = 0
            self.get_logger().info(f"Changed lane from {['Right', 'Center', 'Left'][previous_lane]} to {['Right', 'Center', 'Left'][self.preferred_lane]}")
        else:
            self.stable_lane_counter += 1

        self.last_preferred_lane = self.preferred_lane

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

    def interpolate_points(self, start, end, num_points=10):
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

    def adjust_waypoints_for_lane(self):
        offsets = [-5.0, 0.0, 5.0]  # -5 = Direita, 0 = Centro, 5 = Esquerda
        lane_offset = offsets[self.preferred_lane]

        adjusted_waypoints = []
        for wp in self.waypoints:
            adjusted_wp = wp.copy()
            adjusted_wp['x'] += lane_offset * cos(wp['yaw'] + pi / 2)
            adjusted_wp['y'] += lane_offset * sin(wp['yaw'] + pi / 2)
            adjusted_waypoints.append(adjusted_wp)

        return adjusted_waypoints

    def timer_callback(self):
        self.publish_waypoints()
        self.publish_lanes()

    def publish_lanes(self):
        lanes = []
        colors = [
            (0.0, 1.0, 0.0) if self.preferred_lane == 0 else (1.0, 0.0, 0.0),  # Direita
            (0.0, 1.0, 0.0) if self.preferred_lane == 1 else (1.0, 0.0, 0.0),  # Centro
            (0.0, 1.0, 0.0) if self.preferred_lane == 2 else (1.0, 0.0, 0.0)   # Esquerda
        ]
        offsets = [-5.0, 0.0, 5.0]

        # Define the current robot position (origin)
        robot_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        for i, offset in enumerate(offsets):
            lane = Marker()
            lane.header.frame_id = "map"
            lane.header.stamp = self.get_clock().now().to_msg()
            lane.ns = "lanes"
            lane.id = i
            lane.type = Marker.LINE_STRIP
            lane.action = Marker.ADD
            lane.scale.x = 5.0
            lane.color.r = float(colors[i][0])
            lane.color.g = float(colors[i][1])
            lane.color.b = float(colors[i][2])
            lane.color.a = 1.0

            all_points = []

            # Connect the robot position to the first waypoint
            first_wp = self.waypoints[0]
            first_point = {
                'x': first_wp['x'] - offset * sin(first_wp['yaw']),
                'y': first_wp['y'] + offset * cos(first_wp['yaw']),
                'yaw': first_wp['yaw']
            }
            all_points += self.interpolate_points(robot_position, first_point)

            # Add points between waypoints
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
                lane.points.append(point)

            lanes.append(lane)

        for lane in lanes:
            self.publisher_.publish(lane)

    def publish_waypoints(self):
        marker_array = MarkerArray()
        adjusted_waypoints = self.adjust_waypoints_for_lane()

        for i, wp in enumerate(adjusted_waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.waypoint_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run nav_virtual_lanes visualize_lanes_from_gps <path_to_yaml_file>")
        return
    wps_file_path = sys.argv[1]
    node = LaneVisualizer(wps_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
