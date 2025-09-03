#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TopicRepublisher(Node):
    def __init__(self):
        super().__init__('topic_republisher')

        # Parâmetros de frequência (Hz)
        self.declare_parameter('imu_rate', 5.0)          # Hz
        self.declare_parameter('odom_rate', 10.0)       # Hz
        self.declare_parameter('real_gps_rate', 10.0)   # Hz

        imu_rate = self.get_parameter('imu_rate').get_parameter_value().double_value
        odom_rate = self.get_parameter('odom_rate').get_parameter_value().double_value
        real_gps_rate = self.get_parameter('real_gps_rate').get_parameter_value().double_value

        # Buffers para armazenar as últimas mensagens
        self.latest_imu_msg = None
        self.latest_odom_msg = None
        self.latest_real_gps_msg = None

        # Assinaturas com QoS apropriado
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Assinar tópicos remapeados
        self.subscription_imu_trash = self.create_subscription(
            Imu,
            '/imu_trash',
            self.imu_callback,
            qos_profile
        )

        self.subscription_odom_trash = self.create_subscription(
            Odometry,
            '/odom_trash',
            self.odom_callback,
            qos_profile
        )

        self.subscription_real_gps_trash = self.create_subscription(
            NavSatFix,
            '/real_gps/fix_trash',
            self.real_gps_callback,
            qos_profile
        )

        # Publicadores para os tópicos corretos
        self.publisher_imu = self.create_publisher(Imu, '/imu', 10)
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)
        self.publisher_real_gps = self.create_publisher(NavSatFix, '/real_gps/fix', 10)

        # Timers para publicar nas frequências desejadas
        self.timer_imu = self.create_timer(1.0 / imu_rate, self.publish_imu)
        self.timer_odom = self.create_timer(1.0 / odom_rate, self.publish_odom)
        self.timer_real_gps = self.create_timer(1.0 / real_gps_rate, self.publish_real_gps)

        self.get_logger().info('Nó TopicRepublisher iniciado com as seguintes frequências:')
        self.get_logger().info(f'/imu_trash -> /imu: {imu_rate} Hz')
        self.get_logger().info(f'/odom_trash -> /odom: {odom_rate} Hz')
        self.get_logger().info(f'/real_gps/fix_trash -> /real_gps/fix: {real_gps_rate} Hz')

    def imu_callback(self, msg):
        self.latest_imu_msg = msg

    def odom_callback(self, msg):
        self.latest_odom_msg = msg

    def real_gps_callback(self, msg):
        self.latest_real_gps_msg = msg

    def publish_imu(self):
        if self.latest_imu_msg:
            self.publisher_imu.publish(self.latest_imu_msg)
            self.get_logger().debug('Publicou /imu')

    def publish_odom(self):
        if self.latest_odom_msg:
            self.publisher_odom.publish(self.latest_odom_msg)
            self.get_logger().debug('Publicou /odom')

    def publish_real_gps(self):
        if self.latest_real_gps_msg:
            self.publisher_real_gps.publish(self.latest_real_gps_msg)
            self.get_logger().debug('Publicou /real_gps/fix')

def main(args=None):
    rclpy.init(args=args)
    node = TopicRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Finalizando nó TopicRepublisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
