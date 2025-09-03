#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from math import sin, cos, floor

class LaneOccupancyDetector(Node):
    def __init__(self):
        super().__init__('lane_occupancy_detector')
        self.lane_offsets = [-5.0, 0.0, 5.0]  # Left, center, right lane offsets in meters
        self.lane_width = 5.0  # Width of each lane in meters

        self.costmap_subscriber_ = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',  # Replace with your actual costmap topic
            self.costmap_callback,
            10)
        self.binary_state_publisher_ = self.create_publisher(Bool, '/binary_state', 10)
        self.costmap_data = None

    def costmap_callback(self, msg):
        self.costmap_data = msg
        if self.costmap_data:
            occupancy = self.calculate_lane_occupancy(self.costmap_data)
            self.evaluate_lane_occupancy(occupancy)

    def calculate_lane_occupancy(self, costmap):
        occupancy = [0.0, 0.0, 0.0]  # Left, center, right lane occupancy
        lane_width_cells = self.lane_width / costmap.info.resolution

        for i in range(len(costmap.data)):
            cell_value = costmap.data[i]
            x, y = self.cell_index_to_coordinates(i, costmap.info)
            if cell_value == 100:  # Assuming 100 is the value for occupied cells
                for j, offset in enumerate(self.lane_offsets):
                    if self.is_point_in_lane(x, y, offset, lane_width_cells):
                        occupancy[j] += 1

        # Normalize occupancy to percentage of lane width and cap at 100%
        for i in range(3):
            occupancy[i] = min((occupancy[i] / lane_width_cells) * 100.0, 100.0)

        return occupancy

    def cell_index_to_coordinates(self, index, info):
        """Convert a cell index to its x, y coordinates in the costmap."""
        resolution = info.resolution
        width = info.width
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        x = (index % width) * resolution + origin_x
        y = (index // width) * resolution + origin_y
        return x, y

    def is_point_in_lane(self, x, y, lane_center, lane_width_cells):
        """Check if a point (x, y) is within the lane centered at lane_center."""
        return lane_center - self.lane_width / 2 <= y <= lane_center + self.lane_width / 2

    def evaluate_lane_occupancy(self, occupancy):
        if occupancy[1] == 0.0:  # If the center lane is free
            preferred_lane = 1  # Center lane
        elif occupancy[0] < occupancy[2]:  # Right lane is less occupied
            preferred_lane = 0  # Right lane
        else:
            preferred_lane = 2  # Left lane

        binary_state = Bool()
        binary_state.data = preferred_lane == 1  # True if center lane is preferred, else False
        self.binary_state_publisher_.publish(binary_state)

        self.log_lane_status(occupancy, preferred_lane)

    def log_lane_status(self, occupancy, preferred_lane):
        lane_names = ["Right", "Center", "Left"]
        status = ["free" if occ == 0 else "occupied" for occ in occupancy]

        self.get_logger().info(f"Preferred lane: {lane_names[preferred_lane]}")
        for i, occ in enumerate(occupancy):
            self.get_logger().info(f"{lane_names[i]} lane: {occ:.2f}% occupied")

def main(args=None):
    rclpy.init(args=args)
    node = LaneOccupancyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()