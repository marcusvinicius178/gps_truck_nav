import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LaneVisualizer(Node):
    def __init__(self):
        super().__init__('lane_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        lanes = self.create_lanes()
        for lane in lanes:
            self.publisher_.publish(lane)

    def create_lanes(self):
        lanes = []
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # Red, Green, Blue
        y_positions = [-4.0, 0.0, 4.0]  # Left, center, right

        for i, y in enumerate(y_positions):
            lane = Marker()
            lane.header.frame_id = "map"
            lane.header.stamp = self.get_clock().now().to_msg()
            lane.ns = "lanes"
            lane.id = i
            lane.type = Marker.LINE_STRIP
            lane.action = Marker.ADD
            lane.scale.x = 4.0  # Increase the width of the lines
            lane.color.r = float(colors[i][0])
            lane.color.g = float(colors[i][1])
            lane.color.b = float(colors[i][2])
            lane.color.a = 1.0

            for x in range(0, 101, 5):
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.0
                lane.points.append(point)

            lanes.append(lane)
        return lanes

def main(args=None):
    rclpy.init(args=args)
    node = LaneVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
