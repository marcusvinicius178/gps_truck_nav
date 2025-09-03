#!/usr/bin/env python3

import os
import sys 
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose

class InteractiveGpsWpCommander(Node):
    def __init__(self):
        # super().__init__('interactive_waypoint_follower')
        
        Node.__init__(self, 'gps_waypoint_logger')
        self.navigator = BasicNavigator()

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

    def mapviz_wp_cb(self, msg: PointStamped):
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point from mapviz that is not in wgs84 frame. This is not a gps point and won't be followed")
            return

        self.navigator.waitUntilNav2Active()
        wp = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        self.navigator.followGpsWaypoints(wp)
        if self.navigator.isTaskComplete():
            self.get_logger().info("Wps completed successfully")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveGpsWpCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
