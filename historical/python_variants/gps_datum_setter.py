#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSDatumSetter(Node):
    def __init__(self):
        super().__init__('gps_datum_setter')
        self.subscription = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.datum_set = False
        self.datum = None

    def gps_callback(self, msg):
        if not self.datum_set:
            self.datum = [msg.latitude, msg.longitude, msg.altitude]
            self.datum_set = True
            self.get_logger().info(f'Datum set to: {self.datum}')
            # You can now set this datum in your parameters or use it as needed
            # For example, write it to a file or set a parameter

def main(args=None):
    rclpy.init(args=args)
    gps_datum_setter = GPSDatumSetter()

    rclpy.spin(gps_datum_setter)

    gps_datum_setter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
