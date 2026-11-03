"""Synthetic observations only: no ROS publisher, simulator or movement."""

import copy
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
from readiness_contract import candidate_expected_parameters, equivalent
from variant_runtime_contract import assess_candidate_runtime


def observations():
    return {
        'plant_candidate': {'navigation_reference': {'parent': 'base_footprint', 'frame': 'rear_axle_nav',
                                                     'translation_m': [-3.502, 0, 0]}},
        'topic_publishers': {'/actuation/steering_command': [], '/cmd_vel': [], '/cmd_vel_nav': []},
        'topic_subscribers': {'/actuation/steering_command': [
            {'node_name': 'ackermann_drive', 'topic_type': 'geometry_msgs/msg/Twist'}],
            '/cmd_vel': [], '/cmd_vel_nav': []},
        'topic_discovery_errors': {},
        'transforms': {'base_footprint->rear_axle_nav': {'status': 'OK',
            'translation': {'x': -3.502, 'y': 0, 'z': 0}, 'rotation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}},
        'tf_authority': {'observation_complete': True, 'endpoint_discovery_errors': [],
            'edges': [{'parent': 'base_footprint', 'child': 'rear_axle_nav', 'topic': '/tf_static',
                       'publisher_gids': ['rsp']}],
            'publishers': [{'topic': '/tf_static', 'gid': 'rsp', 'node_name': 'robot_state_publisher'}]},
    }


class CandidateRuntimeTests(unittest.TestCase):
    def checks(self, record):
        return {check['name']: check for check in assess_candidate_runtime(record)}

    def test_canonical_diagnostics_gain_no_unrequested_candidate_assertions(self):
        self.assertEqual(assess_candidate_runtime({}), [])

    def test_isolation_and_fixed_frame_pass_only_observed_contracts(self):
        checks = self.checks(observations())
        self.assertEqual({value['status'] for value in checks.values()}, {'PASS'})
        self.assertIn('Not an actuation test', checks['candidate_actuation_topic_isolated']['scope'])

    def test_missing_or_failed_discovery_is_not_zero_publishers_proof(self):
        for kind in ('missing', 'error'):
            record = observations()
            if kind == 'missing':
                del record['topic_publishers']['/actuation/steering_command']
            else:
                record['topic_discovery_errors']['/actuation/steering_command'] = 'fixture timeout'
            self.assertEqual(self.checks(record)['candidate_actuation_topic_isolated']['status'], 'UNKNOWN')

    def test_actuator_publisher_or_legacy_bypass_fails(self):
        for kind in ('publisher', 'bypass', 'duplicate_subscriber'):
            record = observations()
            endpoint = {'node_name': 'ackermann_drive', 'topic_type': 'geometry_msgs/msg/Twist'}
            if kind == 'publisher':
                record['topic_publishers']['/actuation/steering_command'] = [{'node_name': 'unexpected'}]
            elif kind == 'bypass':
                record['topic_subscribers']['/cmd_vel'].append(endpoint)
            else:
                record['topic_subscribers']['/actuation/steering_command'].append(endpoint)
            self.assertEqual(self.checks(record)['candidate_actuation_topic_isolated']['status'], 'FAIL')

    def test_wrong_geometry_parent_or_authority_fails(self):
        for kind in ('offset', 'parent', 'publisher'):
            record = observations()
            if kind == 'offset':
                record['transforms']['base_footprint->rear_axle_nav']['translation']['x'] = 3.502
            elif kind == 'parent':
                record['tf_authority']['edges'][0]['parent'] = 'odom'
            else:
                record['tf_authority']['publishers'][0]['node_name'] = 'ackermann_drive'
            self.assertEqual(self.checks(record)['candidate_rear_axle_reference']['status'], 'FAIL')

    def test_absent_or_incomplete_tf_authority_is_unknown(self):
        for kind in ('edges', 'complete'):
            record = observations()
            if kind == 'edges': record['tf_authority']['edges'] = []
            else: record['tf_authority']['observation_complete'] = False
            self.assertEqual(self.checks(record)['candidate_rear_axle_reference']['status'], 'UNKNOWN')

    def test_selected_frame_footprint_and_goal_contract_are_queried(self):
        for name in ('candidate20kmh', 'candidate25kmh'):
            config = yaml.safe_load((ROOT / 'config/auto_tuning/plant_candidate/navigation' / (name + '.yaml')).read_text())
            wanted = candidate_expected_parameters(config)
            self.assertEqual(wanted['/bt_navigator']['robot_base_frame'], 'rear_axle_nav')
            self.assertEqual(wanted['/behavior_server']['robot_base_frame'], 'rear_axle_nav')
            for costmap in ('local_costmap', 'global_costmap'):
                self.assertEqual(wanted['/' + costmap + '/' + costmap]['robot_base_frame'], 'rear_axle_nav')
                self.assertTrue(equivalent('footprint', wanted['/' + costmap + '/' + costmap]['footprint'],
                    config[costmap][costmap]['ros__parameters']['footprint']))
            self.assertEqual(wanted['/controller_server']['goal_checker_plugins'], ['general_goal_checker'])
            self.assertEqual(wanted['/controller_server']['general_goal_checker.plugin'], 'nav2_controller::SimpleGoalChecker')
            self.assertEqual(wanted['/controller_server']['general_goal_checker.xy_goal_tolerance'], 1.0)


if __name__ == '__main__':
    unittest.main()
