#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import pynmea2

class NMEAToNavSatFixNode(Node):
    def __init__(self):
        super().__init__('nmea_to_navsatfix_converter')
        # Subscription to the NMEA sentence topic
        self.subscription = self.create_subscription(
            String,
            '/nmea_sentence_topic',
            self.nmea_listener_callback,
            10)
        # Publisher for NavSatFix
        self.publisher_navsat = self.create_publisher(NavSatFix, '/real_gps/fix', 10)

        # Subscription to relay the NavSatFix data
        self.subscription_relay = self.create_subscription(
            NavSatFix,
            '/real_gps/fix',
            self.relay_listener_callback,
            10)
        # Publisher for relaying the data to another topic
        self.publisher_relay = self.create_publisher(NavSatFix, '/gps/fix', 10)

    def nmea_listener_callback(self, msg):
        try:
            # Parse the NMEA sentence
            if msg.data.startswith('$GPRMC'):
                nmea_obj = pynmea2.parse(msg.data)
                navsatfix_msg = NavSatFix()
                navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
                navsatfix_msg.header.frame_id = 'base_footprint'
                navsatfix_msg.latitude = nmea_obj.latitude
                navsatfix_msg.longitude = nmea_obj.longitude
                # Optionally add altitude if available
                self.publisher_navsat.publish(navsatfix_msg)
        except pynmea2.ParseError as e:
            self.get_logger().error('Failed to parse NMEA sentence: %s' % msg.data)

    def relay_listener_callback(self, msg):
        # Simply relay the message from /real_gps/fix to /gps/fix
        self.publisher_relay.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NMEAToNavSatFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()