#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry
#import tf_transformations
import transforms3d.euler
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DynamicTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('tf2_dynamic_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        #Timer not requeired when using message_fitlers synch time.
        #self.timer = self.create_timer(0.1, self.broadcast_transforms)
        
        self._imu_data = None
        self._odom_data = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )

        imu_sub = Subscriber(self, Imu, 'imu', qos_profile = qos_profile)
        odom_sub = Subscriber(self, Odometry, 'odom',  qos_profile = qos_profile)

        #self.ts = ApproximateTimeSynchronizer([imu_sub, odom_sub], queue_size=100, slop=0.2)
        self.ts = ApproximateTimeSynchronizer([imu_sub, odom_sub], queue_size=100, slop=1.0)
        self.ts.registerCallback(self.callback)
    
    def callback(self, imu_msg, odom_msg):
        self._imu_data = imu_msg.orientation
        self._odom_data = odom_msg
        self.broadcast_transforms()
    

    def get_yaw_from_quaternion(self, q):
        #euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        euler = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
        return euler[2]  # yaw

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        if self._imu_data is None or self._odom_data is None:
            self.get_logger().info("Waiting for IMU and Odometry data...")
            return

        # Transform from odom to base_footprint
        odom_to_base_footprint = TransformStamped()
        #odom_to_base_footprint.header.stamp = now
        odom_to_base_footprint.header.stamp = self._odom_data.header.stamp
        odom_to_base_footprint.header.frame_id = 'odom'
        odom_to_base_footprint.child_frame_id = 'base_footprint'
        odom_to_base_footprint.transform.translation.x = self._odom_data.pose.pose.position.x
        odom_to_base_footprint.transform.translation.y = self._odom_data.pose.pose.position.y
        odom_to_base_footprint.transform.translation.z = 0.0

        yaw =  self.get_yaw_from_quaternion(self._imu_data) # math.radians(0) 
        
        odom_to_base_footprint.transform.rotation.x = 0.0
        odom_to_base_footprint.transform.rotation.y = 0.0
        odom_to_base_footprint.transform.rotation.z = math.sin(yaw / 2)
        odom_to_base_footprint.transform.rotation.w = math.cos(yaw / 2)

        # Broadcasting the transforms
        self.broadcaster.sendTransform(odom_to_base_footprint)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTransformBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    