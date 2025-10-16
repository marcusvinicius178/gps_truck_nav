import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class QoSTranslator(Node):
    def __init__(self):
        super().__init__('qos_translator_node')
        self.last_published_time_ns = self.get_clock().now().nanoseconds
        # Set the publish interval to 1 second, converted to nanoseconds
        self.publish_interval_ns = int(1e9)  # 1e9 nanoseconds in a second

        self.subscription = self.create_subscription(
            Range,
            '/ultrassonic/front_left',
            self.listener_callback,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE))
        
        self.publisher = self.create_publisher(
            Range,
            '/ultrassonic/front_left_best_effort',
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT))

    def listener_callback(self, msg):
        current_time_ns = self.get_clock().now().nanoseconds
        elapsed_time_ns = current_time_ns - self.last_published_time_ns
        
        # Check if the elapsed time since the last published message is greater than the interval
        if elapsed_time_ns >= self.publish_interval_ns:
            self.publisher.publish(msg)
            self.get_logger().info('Publishing message...')
            self.last_published_time_ns = current_time_ns  # Update the last published time
        else:
            self.get_logger().info('Not time to publish yet.')

def main(args=None):
    rclpy.init(args=args)
    qos_translator_node = QoSTranslator()
    rclpy.spin(qos_translator_node)
    # Cleanup
    qos_translator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
