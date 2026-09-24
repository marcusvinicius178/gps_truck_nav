"""ROS-free regressions for sender orchestration, not a simulated ROS runtime."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'src/gps_truck_nav/truck_bringup/scripts/logged_waypoint_follower.py'


def load_sender():
    names = ['ament_index_python', 'ament_index_python.packages', 'geometry_msgs',
             'geometry_msgs.msg', 'nav2_simple_commander',
             'nav2_simple_commander.robot_navigator', 'rclpy', 'rclpy.node',
             'rclpy.utilities', 'robot_localization', 'robot_localization.srv']
    modules = {name: MagicMock() for name in names}
    modules['rclpy.node'].Node = type('Node', (), {})
    modules['rclpy.utilities'].remove_ros_args = lambda args: args
    spec = importlib.util.spec_from_file_location('sender_under_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


sender = load_sender()


class WaypointSenderTests(unittest.TestCase):
    def parse(self, text):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
            stream.write(text)
            stream.flush()
            return sender.YamlWaypointParser(stream.name).get_wps()

    def commander(self):
        node = sender.GpsWpCommander.__new__(sender.GpsWpCommander)
        node.wp_parser = MagicMock()
        node.wp_parser.get_wps.return_value = [
            {'latitude': -22.61, 'longitude': -47.50, 'yaw': 0.1},
            {'latitude': -22.62, 'longitude': -47.51, 'yaw': 0.2}]
        node.from_ll_client = MagicMock()
        node.from_ll_client.wait_for_service.return_value = True
        node.navigator = MagicMock()
        node.navigator.isTaskComplete.return_value = True
        node.navigator.getResult.return_value = sender.TaskResult.SUCCEEDED
        node.convert_geo_pose_to_pose_stamped = MagicMock(side_effect=['pose1', 'pose2'])
        node.get_logger = MagicMock()
        return node

    def test_converts_each_waypoint_once_and_preserves_order(self):
        node = self.commander()
        self.assertEqual(node.start_wpf(), 0)
        self.assertEqual(node.convert_geo_pose_to_pose_stamped.call_count, 2)
        node.navigator.goThroughPoses.assert_called_once_with(['pose1', 'pose2'])
        node.navigator.waitUntilNav2Active.assert_called_once_with(localizer='robot_localization')

    def test_failed_conversion_never_sends_a_partial_route(self):
        node = self.commander()
        node.convert_geo_pose_to_pose_stamped.side_effect = ['pose1', None]
        with self.assertRaises(RuntimeError):
            node.start_wpf()
        node.navigator.goThroughPoses.assert_not_called()

    def test_rejected_goal_is_not_success(self):
        node = self.commander()
        node.navigator.goThroughPoses.return_value = False
        with self.assertRaises(RuntimeError):
            node.start_wpf()
        node.navigator.getResult.assert_not_called()

    def test_failed_or_canceled_action_is_not_success(self):
        for result in [sender.TaskResult.FAILED, sender.TaskResult.CANCELED]:
            with self.subTest(result=result):
                node = self.commander()
                node.navigator.getResult.return_value = result
                self.assertEqual(node.start_wpf(), 1)

    def test_historical_map_x_offset_is_preserved(self):
        node = self.commander()
        del node.convert_geo_pose_to_pose_stamped
        node.get_clock = MagicMock()
        future = node.from_ll_client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = types.SimpleNamespace(map_point=types.SimpleNamespace(x=10., y=20.))
        pose = node.convert_geo_pose_to_pose_stamped(-22.6, -47.5, 0.)
        self.assertEqual(pose.pose.position.x, 14.8025)
        self.assertEqual(pose.pose.position.y, 20.)
        self.assertEqual(pose.header.frame_id, 'map')

    def test_bad_yaml_fails_before_navigation(self):
        for text in ['', 'waypoints: []', 'waypoints: [null]',
                     'waypoints: [{latitude: .nan, longitude: 1, yaw: 0}]',
                     'waypoints: [{latitude: 91, longitude: 1, yaw: 0}]',
                     'waypoints: [{latitude: true, longitude: 1, yaw: 0}]',
                     'waypoints: [{latitude: 1, longitude: 2}]']:
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.parse(text)

    def test_valid_yaml_values_are_not_retuned(self):
        self.assertEqual(self.parse('waypoints: [{latitude: -22.6, longitude: -47.5, yaw: 0.01}]'),
                         [{'latitude': -22.6, 'longitude': -47.5, 'yaw': 0.01}])


if __name__ == '__main__':
    unittest.main()
