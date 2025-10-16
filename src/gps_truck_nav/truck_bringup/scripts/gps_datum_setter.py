#!/usr/bin/env python3
"""ROS node that captures the first GPS fix and exposes it as a datum."""

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSDatumSetter(Node):
    """Subscribe to a GPS fix topic and remember the first received position."""

    def __init__(self, topic: str = '/gps/fix') -> None:
        super().__init__('gps_datum_setter')
        self.declare_parameter('input_topic', topic)
        resolved_topic = self.get_parameter('input_topic').value

        self._datum: Optional[Tuple[float, float, float]] = None
        self._subscription = self.create_subscription(
            NavSatFix,
            resolved_topic,
            self._gps_callback,
            10,
        )
        self.get_logger().info('Waiting for first GPS fix on %s', resolved_topic)

    @property
    def datum(self) -> Optional[Tuple[float, float, float]]:
        """Return the stored datum if available."""
        return self._datum

    def _gps_callback(self, msg: NavSatFix) -> None:
        if self._datum is not None:
            return

        self._datum = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(
            'Datum locked to lat=%.8f lon=%.8f alt=%.3fm',
            *self._datum,
        )

        # External systems can now read ``self.datum`` or persist it elsewhere.
        self.destroy_subscription(self._subscription)
        self._subscription = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GPSDatumSetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down due to keyboard interrupt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
