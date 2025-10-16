import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import yaml
from robot_localization.srv import FromLL
from math import sin, cos
import sys

class YamlWaypointParser:
    """
    Parse a set of GPS waypoints from a YAML file.
    """
    def __init__(self, wps_file_path: str):
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic coordinates and yaw from the YAML file.
        """
        return self.wps_dict["waypoints"]

class WaypointVisualizer(Node):
    def __init__(self, wps_file_path):
        super().__init__('waypoint_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.from_ll_client = self.create_client(FromLL, 'fromLL')
        while not self.from_ll_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('fromLL service not available, waiting again...')
        self.waypoints = self.convert_gps_waypoints()

    def convert_geo_pose_to_pose(self, latitude, longitude, yaw):
        request = FromLL.Request()
        request.ll_point.latitude = float(latitude)
        request.ll_point.longitude = float(longitude)
        future = self.from_ll_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response and response.map_point:
            return {'x': response.map_point.x, 'y': response.map_point.y, 'yaw': yaw}
        else:
            self.get_logger().error('Failed to get valid response from fromLL service')
            return None

    def convert_gps_waypoints(self):
        gps_waypoints = self.wp_parser.get_wps()
        waypoints = [self.convert_geo_pose_to_pose(wp['latitude'], wp['longitude'], wp['yaw']) for wp in gps_waypoints]
        waypoints = [wp for wp in waypoints if wp is not None]
        return waypoints

    def timer_callback(self):
        marker_array = MarkerArray()
        markers = self.create_markers()
        marker_array.markers.extend(markers)
        self.publisher_.publish(marker_array)

    def create_markers(self):
        markers = []

        # Get the starting point (truck position)
        truck_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        all_points = [truck_position] + self.waypoints

        for i, point in enumerate(all_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            markers.append(marker)
        return markers

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run nav_virtual_lanes visualize_waypoints <path_to_yaml_file>")
        return
    wps_file_path = sys.argv[1]
    node = WaypointVisualizer(wps_file_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
