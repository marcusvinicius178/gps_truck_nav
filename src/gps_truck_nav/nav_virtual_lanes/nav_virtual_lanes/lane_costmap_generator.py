#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import yaml
from math import ceil, sin, cos, radians, pi
import sys
from robot_localization.srv import FromLL


class LaneCostmapGenerator(Node):
    def __init__(self, wps_file_path):
        super().__init__('lane_costmap_generator')

        # Publishers for central, right, and left lanes
        self.central_publisher_ = self.create_publisher(OccupancyGrid, 'central_virtual_lane', 10)
        self.right_publisher_ = self.create_publisher(OccupancyGrid, 'right_virtual_lane', 10)
        self.left_publisher_ = self.create_publisher(OccupancyGrid, 'left_virtual_lane', 10)

        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting again...')

        # Load waypoints from YAML file
        self.waypoints = self.load_waypoints(wps_file_path)
        self.resolution = 0.5  # Define the resolution of the costmap

        # Generate the costmaps
        self.generate_costmap('central')
        self.generate_costmap('right')
        self.generate_costmap('left')

    def load_waypoints(self, wps_file_path):
        with open(wps_file_path, 'r') as wps_file:
            wps_dict = yaml.safe_load(wps_file)

        waypoints = wps_dict.get("waypoints", [])
        converted_waypoints = []

        for wp in waypoints:
            if 'latitude' in wp and 'longitude' in wp and 'yaw' in wp:
                converted_wp = self.convert_geo_pose_to_pose(wp['latitude'], wp['longitude'], wp['yaw'])
                if converted_wp is not None:
                    converted_waypoints.append(converted_wp)
            else:
                self.get_logger().warning(f"Waypoint missing necessary fields: {wp}")

        return converted_waypoints

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

    def interpolate_points(self, start, end, num_points=120):
        """Interpolate num_points between start and end points."""
        points = []
        dx = (end['x'] - start['x']) / num_points
        dy = (end['y'] - start['y']) / num_points
        for i in range(num_points + 1):
            point = {
                'x': start['x'] + i * dx,
                'y': start['y'] + i * dy,
                'yaw': start['yaw']  # Preserve the yaw value
            }
            points.append(point)
        return points

    def generate_costmap(self, lane_type='central'):
        """Generate costmap for the specified lane type (central, right, or left)."""
        if not self.waypoints:
            self.get_logger().error("No valid waypoints were loaded, cannot generate costmap.")
            return

        if lane_type == 'central':
            adjusted_waypoints, left_borders, right_borders = self.adjust_waypoints_with_borders(offset=0.0)
            lane_value = 20  # Example value for central lane
        elif lane_type == 'right':
            adjusted_waypoints, left_borders, right_borders = self.adjust_waypoints_with_borders(offset=-5.0)  # Right lane offset
            lane_value = 60  # Example value for right lane
        elif lane_type == 'left':
            adjusted_waypoints, left_borders, right_borders = self.adjust_waypoints_with_borders(offset=5.0)  # Left lane offset
            lane_value = 100  # Example value for left lane
        else:
            self.get_logger().error("Invalid lane type specified.")
            return

        # Add a connection from the robot's origin to the first waypoint
        robot_origin = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        first_waypoint = adjusted_waypoints[0]
        connection_points = self.interpolate_points(robot_origin, first_waypoint)

        # Extend the adjusted waypoints to include the connection
        adjusted_waypoints = connection_points + adjusted_waypoints

        # Determine the bounds of the costmap and add a margin to accommodate the entire lane
        margin = 5.0  # Safety margin of 5 meters around the lane
        min_x = min(wp['x'] for wp in adjusted_waypoints + left_borders + right_borders) - margin
        max_x = max(wp['x'] for wp in adjusted_waypoints + left_borders + right_borders) + margin
        min_y = min(wp['y'] for wp in adjusted_waypoints + left_borders + right_borders) - margin
        max_y = max(wp['y'] for wp in adjusted_waypoints + left_borders + right_borders) + margin

        width = ceil((max_x - min_x) / self.resolution)
        height = ceil((max_y - min_y) / self.resolution)

        origin_x = min_x
        origin_y = min_y

        self.get_logger().info(f"Costmap bounds calculated for {lane_type} lane: Origin=({origin_x}, {origin_y}), Width={width}, Height={height}")

        # Create the OccupancyGrid message
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'map'
        costmap.info = MapMetaData()
        costmap.info.resolution = self.resolution
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin = Pose()
        costmap.info.origin.position.x = origin_x
        costmap.info.origin.position.y = origin_y

        # Initialize the costmap data to zero (free space)
        costmap.data = [0] * (width * height)

        # Mark the lane areas with the specific lane value
        lane_width_in_cells = ceil(5.0 / self.resolution)  # Assuming a lane width of 5 meters

        # Mark central lane with specific value
        for wp in adjusted_waypoints:
            i = int((wp['x'] - origin_x) / self.resolution)
            j = int((wp['y'] - origin_y) / self.resolution)
            for di in range(-lane_width_in_cells // 2, lane_width_in_cells // 2 + 1):
                for dj in range(-lane_width_in_cells // 2, lane_width_in_cells // 2 + 1):
                    index = (j + dj) * width + (i + di)
                    if 0 <= index < len(costmap.data):
                        costmap.data[index] = lane_value  # Assign different value per lane

        # Mark borders as semi-occupied (50)
        for wp in left_borders + right_borders:
            i = int((wp['x'] - origin_x) / self.resolution)
            j = int((wp['y'] - origin_y) / self.resolution)
            index = j * width + i
            if 0 <= index < len(costmap.data):
                costmap.data[index] = 50  # Semi-occupied (border)

        # Publish the appropriate costmap
        if lane_type == 'central':
            self.central_publisher_.publish(costmap)
        elif lane_type == 'right':
            self.right_publisher_.publish(costmap)
        elif lane_type == 'left':
            self.left_publisher_.publish(costmap)

        self.get_logger().info(f"{lane_type.capitalize()} lane costmap generated and published.")

    def adjust_waypoints_with_borders(self, offset=0.0):
        border_offset = 2.5  # Offset for borders

        # Adjust waypoints with the specified lane offset
        adjusted_waypoints = []
        for wp in self.waypoints:
            adjusted_wp = wp.copy()
            adjusted_wp['x'] += offset * cos(wp['yaw'] + pi / 2)
            adjusted_wp['y'] += offset * sin(wp['yaw'] + pi / 2)
            adjusted_waypoints.append(adjusted_wp)

        # Interpolating points between each pair of waypoints for lane
        interpolated_waypoints = []
        for i in range(len(adjusted_waypoints) - 1):
            start_wp = adjusted_waypoints[i]
            end_wp = adjusted_waypoints[i + 1]
            interpolated_waypoints += self.interpolate_points(start_wp, end_wp)

        # Adjust waypoints for borders (left and right)
        left_borders = []
        right_borders = []
        for wp in interpolated_waypoints:
            # Left border
            left_wp = wp.copy()
            left_wp['x'] -= border_offset * sin(wp['yaw'])
            left_wp['y'] += border_offset * cos(wp['yaw'])
            left_borders.append(left_wp)

            # Right border
            right_wp = wp.copy()
            right_wp['x'] += border_offset * sin(wp['yaw'])
            right_wp['y'] -= border_offset * cos(wp['yaw'])
            right_borders.append(right_wp)

        return interpolated_waypoints, left_borders, right_borders


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run nav_virtual_lanes lane_costmap_generator <path_to_yaml_file>")
        return
    wps_file_path = sys.argv[1]
    node = LaneCostmapGenerator(wps_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()