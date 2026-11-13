"""Static contracts for the prospective Iron trees; no ROS or goals are started.

Tree syntax and ports are grounded in Nav2 1.2.10's shipped XMLs. Actual plugin
loading and both navigator activations remain obligations of the local probe.
"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
DIRECTORY = ROOT / 'config/auto_tuning/behavior_trees'
CANONICAL = ROOT / 'src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml'
TREES = {
    'ackermann_navigate_to_pose.xml': ('ComputePathToPose', 'goal', '1.0'),
    'ackermann_navigate_through_poses.xml': ('ComputePathThroughPoses', 'goals', '0.333'),
}
BUILTINS = {'root', 'BehaviorTree', 'Sequence', 'ReactiveSequence', 'Fallback', 'ReactiveFallback'}
PLUGINS = {
    'RecoveryNode': 'nav2_recovery_node_bt_node',
    'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
    'RateController': 'nav2_rate_controller_bt_node',
    'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
    'ComputePathThroughPoses': 'nav2_compute_path_through_poses_action_bt_node',
    'FollowPath': 'nav2_follow_path_action_bt_node',
    'WouldAPlannerRecoveryHelp': 'nav2_would_a_planner_recovery_help_condition_bt_node',
    'WouldAControllerRecoveryHelp': 'nav2_would_a_controller_recovery_help_condition_bt_node',
    'ClearEntireCostmap': 'nav2_clear_costmap_service_bt_node',
    'GoalUpdated': 'nav2_goal_updated_condition_bt_node',
    'RoundRobin': 'nav2_round_robin_node_bt_node',
    'Wait': 'nav2_wait_action_bt_node',
    'RemovePassedGoals': 'nav2_remove_passed_goals_action_bt_node',
}


class AckermannBehaviorTreeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load(CANONICAL.read_text())
        cls.trees = {name: ET.parse(DIRECTORY / name).getroot() for name in TREES}

    def test_both_nav2_navigators_have_one_self_contained_main_tree(self):
        configured = self.params['bt_navigator']['ros__parameters']['navigators']
        self.assertEqual(set(configured), {'navigate_to_pose', 'navigate_through_poses'})
        for name, root in self.trees.items():
            with self.subTest(tree=name):
                self.assertEqual(root.tag, 'root')
                self.assertEqual(root.attrib, {'main_tree_to_execute': 'MainTree'})
                self.assertEqual(len(root), 1)
                self.assertEqual(root[0].tag, 'BehaviorTree')
                self.assertEqual(root[0].attrib, {'ID': 'MainTree'})
                self.assertEqual(len(root[0]), 1)
                self.assertFalse(root.findall('.//include'))
                self.assertFalse(root.findall('.//SubTree'))

    def test_every_nonbuiltin_node_has_a_configured_iron_plugin(self):
        configured = set(self.params['bt_navigator']['ros__parameters']['plugin_lib_names'])
        for name, root in self.trees.items():
            with self.subTest(tree=name):
                tags = {node.tag for node in root.iter()}
                self.assertEqual(tags - BUILTINS - PLUGINS.keys(), set())
                self.assertTrue({PLUGINS[tag] for tag in tags - BUILTINS}.issubset(configured))

    def test_no_in_place_rotation_or_direct_motion_recovery_is_reachable(self):
        forbidden = {'Spin', 'SpinCancel', 'BackUp', 'BackUpCancel', 'DriveOnHeading',
                     'DriveOnHeadingCancel', 'AssistedTeleop', 'AssistedTeleopCancel'}
        for name, root in self.trees.items():
            with self.subTest(tree=name):
                self.assertFalse({node.tag for node in root.iter()} & forbidden)
                recovery = root.find('.//RoundRobin')
                self.assertIsNotNone(recovery)
                self.assertEqual([child.tag for child in recovery], ['Sequence', 'Wait'])
                self.assertEqual([child.tag for child in recovery[0]],
                                 ['ClearEntireCostmap', 'ClearEntireCostmap'])

    def test_planner_controller_and_error_ports_match_project_configuration(self):
        planner_ids = self.params['planner_server']['ros__parameters']['planner_plugins']
        controller_ids = self.params['controller_server']['ros__parameters']['controller_plugins']
        errors = self.params['bt_navigator']['ros__parameters']['error_code_names']
        for name, root in self.trees.items():
            action, goals_port, _ = TREES[name]
            with self.subTest(tree=name):
                planners = root.findall('.//' + action)
                self.assertEqual(len(planners), 1)
                self.assertEqual(planners[0].attrib, {
                    goals_port: '{' + goals_port + '}', 'path': '{path}',
                    'planner_id': 'GridBased', 'error_code_id': '{compute_path_error_code}'})
                self.assertIn('GridBased', planner_ids)
                controllers = root.findall('.//FollowPath')
                self.assertEqual(len(controllers), 1)
                self.assertEqual(controllers[0].attrib, {
                    'path': '{path}', 'controller_id': 'FollowPath',
                    'error_code_id': '{follow_path_error_code}'})
                self.assertIn('FollowPath', controller_ids)
                self.assertIn('compute_path_error_code', errors)
                self.assertIn('follow_path_error_code', errors)

    def test_control_arities_and_recovery_budget_follow_iron_contract(self):
        for name, root in self.trees.items():
            with self.subTest(tree=name):
                recoveries = root.findall('.//RecoveryNode')
                self.assertEqual(len(recoveries), 3)
                self.assertEqual([int(node.get('number_of_retries')) for node in recoveries], [6, 1, 1])
                for node in recoveries:
                    self.assertEqual(len(node), 2)
                rates = root.findall('.//RateController')
                self.assertEqual(len(rates), 1)
                self.assertEqual(rates[0].get('hz'), TREES[name][2])
                self.assertEqual(len(rates[0]), 1)
                self.assertEqual(len(root.find('.//PipelineSequence')), 2)
                fallback = root.find('.//ReactiveFallback')
                self.assertEqual([node.tag for node in fallback], ['GoalUpdated', 'RoundRobin'])

    def test_wait_and_clear_services_are_supported_without_spin_server(self):
        behavior = self.params['behavior_server']['ros__parameters']
        self.assertIn('wait', behavior['behavior_plugins'])
        self.assertEqual(behavior['wait']['plugin'], 'nav2_behaviors/Wait')
        self.assertNotIn('spin', behavior['behavior_plugins'])
        expected_services = {'local_costmap/clear_entirely_local_costmap',
                             'global_costmap/clear_entirely_global_costmap'}
        for name, root in self.trees.items():
            with self.subTest(tree=name):
                self.assertEqual(root.find('.//Wait').attrib, {'wait_duration': '5'})
                services = {node.get('service_name') for node in root.findall('.//ClearEntireCostmap')}
                self.assertEqual(services, expected_services)

    def test_through_poses_retains_upstream_goal_pruning_contract(self):
        through = self.trees['ackermann_navigate_through_poses.xml']
        self.assertEqual(through.find('.//RemovePassedGoals').attrib,
                         {'input_goals': '{goals}', 'output_goals': '{goals}', 'radius': '0.7'})
        single = self.trees['ackermann_navigate_to_pose.xml']
        self.assertIsNone(single.find('.//RemovePassedGoals'))


if __name__ == '__main__':
    unittest.main()
