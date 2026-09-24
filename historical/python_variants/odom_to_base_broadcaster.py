#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from math import cos, sin, radians

class DynamicBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf2_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to control the update rate

    def timer_callback(self):
        # Broadcast transform from odom to base_footprint
        odom_to_base = self.create_transform(
            'odom', 'base_footprint', 0.0, 0.0, 0.0, 0.0)
        self.br.sendTransform(odom_to_base)

        # Broadcast transform from map to odom ---> This must be done by the ekf_node_map
        map_to_odom = self.create_transform(
            'map', 'odom', 0.0, 0.0, 0.0, 0.0)  # Modify these values as needed
        self.br.sendTransform(map_to_odom)

    def create_transform(self, parent_frame, child_frame, x, y, z, yaw):
        """ Helper function to create and return a TransformStamped. """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = self.create_quaternion_from_yaw(yaw)
        return t

    def create_quaternion_from_yaw(self, yaw):
        """ Helper function to create quaternion from yaw angle in radians. """
        q = Quaternion()
        q.w = cos(radians(yaw) / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = sin(radians(yaw) / 2)
        return q

def main():
    rclpy.init()
    node = DynamicBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
