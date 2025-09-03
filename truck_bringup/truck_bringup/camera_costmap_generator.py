#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import time

class CostmapGenerator(Node):

    def __init__(self):
        super().__init__('costmap_generator')

        qos_profile_image = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.subscription = self.create_subscription(
            Int8MultiArray,
            'segmentation_mask',
            self.listener_callback,
            qos_profile_image
        )

        qos_profile_costmap = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.costmap_publisher = self.create_publisher(OccupancyGrid, 'custom_costmap', qos_profile_costmap)

    def listener_callback(self, data):
        size = (data.layout.dim[0].size, data.layout.dim[1].size)
        mask = np.array(data.data, dtype=np.int8).reshape(size)

        # Rotate and flip the mask to correct the orientation
        rotated_mask = np.rot90(mask, k=3)  # Rotate 90 degrees clockwise
        corrected_mask = np.flip(rotated_mask, axis=1)  # Flip horizontally


        costmap = np.full(corrected_mask.shape, -1, dtype=np.int8)  # Initialize costmap as unknown space

        costmap[corrected_mask == 1] = 100  # "estradaAsfalto" as occupied (black)
        costmap[corrected_mask == 0] = 0    # Other regions as free (white)

        self.publish_costmap(costmap)

    def publish_costmap(self, costmap):
        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = 'map'
        occ_grid.info.resolution = 0.03  # Grid resolution
        occ_grid.info.width = costmap.shape[1]
        occ_grid.info.height = costmap.shape[0]
        occ_grid.info.origin.position.x = -25.0
        occ_grid.info.origin.position.y = -13.0
        occ_grid.info.origin.position.z = 0.0

        # Optionally, you can set the orientation of the map if needed
        occ_grid.info.origin.orientation.x = 0.0
        occ_grid.info.origin.orientation.y = 0.0
        occ_grid.info.origin.orientation.z = 0.0  # No rotation in quaternion
        occ_grid.info.origin.orientation.w = 1.0

        occ_grid.data = costmap.flatten().tolist()
        self.costmap_publisher.publish(occ_grid)
        self.get_logger().info('Published costmap with corrected orientation')

def main(args=None):
    rclpy.init(args=args)
    costmap_generator = CostmapGenerator()

    try:
        while rclpy.ok():
            rclpy.spin_once(costmap_generator)
            time.sleep(0.1)  # Add a delay to control the rate of publishing
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            costmap_generator.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()