"""Regression fixtures for preserved odometry/joints and prospective sensor TF."""

import copy
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('variant_runtime_contract', ROOT / 'tools/auto_tuning/variant_runtime_contract.py')
contract = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contract)
POLICY = {'minimum_samples': 2, 'maximum_future_sim_seconds': 0.25, 'maximum_age_sim_seconds': 1.0}
PROFILE = json.loads((ROOT / 'config/auto_tuning/tf_sensor_model_variant.json').read_text())


def fixture():
    authority = {'observation_complete': True, 'edges': [], 'publishers': [
        {'topic': '/tf', 'gid': 'rsp', 'node_name': 'robot_state_publisher'},
        {'topic': '/tf', 'gid': 'gazebo', 'node_name': 'ackermann_drive'}]}
    for expected in PROFILE['expected_wheel_edges']:
        authority['edges'].append({'parent': expected['parent'], 'child': expected['child'],
                                   'topic': '/tf', 'publisher_gids': ['rsp']})
    authority['edges'].append({'parent': 'odom', 'child': 'base_footprint', 'topic': '/tf', 'publisher_gids': ['gazebo']})
    transforms = {'base_footprint->' + frame: {'status': 'OK',
        'translation': dict(zip(('x', 'y', 'z'), expected['translation_m'])),
        'rotation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
        for frame, expected in PROFILE['sensor_lever_arms'].items()}
    return {'model_variant': copy.deepcopy(PROFILE), 'tf_authority': authority,
            'source_files': {'/opt/auto_tuning/config/' + path: sha for path, sha in PROFILE['variant_sha256'].items()},
            'transforms': transforms, 'alignment': {'status': 'PASS', 'geometric_validated': False},
            'topics': {'/joint_states': {'last': {'names': PROFILE['joint_names'], 'positions': [0.0] * 6}},
                       '/sick_scan_2D': {'last': {'range_count': 360, 'finite_range_count': 0,
                           'positive_infinite_range_count': 360, 'nan_range_count': 0, 'invalid_range_count': 0}}},
            'clock': {'last': 10.0}, 'obstacle_source_topics': ['/sick_scan_2D', '/external_area_pointcloud'],
            'ackermann_library': {'sha256': 'fixture'},
            'runtime_binary_identity': {'targets': {'ackermann_drive': {'status': 'MAPPED_IDENTIFIED',
                'records': [{'sha256': 'fixture'}]}}}}


class VariantRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.observed = fixture()

    def check(self, name):
        return next(item for item in contract.assess_variant_runtime(self.observed, True, POLICY) if item['name'] == name)

    def test_six_urdf_edges_and_ackermann_odometry_are_independent(self):
        self.assertEqual(self.check('wheel_tf_urdf_authority')['status'], 'PASS')
        self.assertEqual(self.check('ackermann_odometry_tf_preserved')['status'], 'PASS')
        self.assertEqual(self.check('joint_states_contract')['status'], 'PASS')

    def test_old_direct_parent_wheel_tf_is_rejected(self):
        edge = dict(self.observed['tf_authority']['edges'][0], parent='base_footprint', publisher_gids=['gazebo'])
        self.observed['tf_authority']['edges'].append(edge)
        self.assertEqual(self.check('wheel_tf_urdf_authority')['status'], 'FAIL')

    def test_missing_edge_unknown_gid_and_wrong_publisher_cannot_pass(self):
        self.observed['tf_authority']['edges'].pop(0)
        self.assertEqual(self.check('wheel_tf_urdf_authority')['status'], 'UNKNOWN')
        self.observed = fixture()
        self.observed['tf_authority']['edges'][0]['publisher_gids'] = ['unknown']
        self.assertEqual(self.check('wheel_tf_urdf_authority')['status'], 'UNKNOWN')
        self.observed['tf_authority']['edges'][0]['publisher_gids'] = ['gazebo']
        self.assertEqual(self.check('wheel_tf_urdf_authority')['status'], 'FAIL')

    def test_missing_joint_nonfinite_position_and_geometry_change_fail(self):
        self.observed['topics']['/joint_states']['last']['positions'] = [0.0] * 5
        self.assertEqual(self.check('joint_states_contract')['status'], 'FAIL')
        self.observed['topics']['/joint_states']['last']['positions'] = [float('nan')] * 6
        self.assertEqual(self.check('joint_states_contract')['status'], 'FAIL')
        self.observed['transforms']['base_footprint->gps_link']['translation']['x'] = 0
        self.assertEqual(self.check('sensor_lever_arm:gps_link')['status'], 'FAIL')

    def test_prospective_hash_drift_is_not_runtime_approval(self):
        self.assertEqual(self.check('prospective_variant_files')['status'], 'PASS')
        self.observed['source_files'].clear()
        self.assertEqual(self.check('prospective_variant_files')['status'], 'FAIL')

    def test_empty_scan_returns_are_not_fabricated_obstacle_validation(self):
        check = self.check('obstacle_scan_payload')
        self.assertEqual(check['status'], 'PASS')
        self.assertFalse(check['obstacle_detection_validated'])
        self.observed['topics']['/sick_scan_2D']['last']['nan_range_count'] = 1
        self.assertEqual(self.check('obstacle_scan_payload')['status'], 'FAIL')

    def test_publisher_presence_does_not_replace_missing_obstacle_samples(self):
        self.observed['topic_publishers'] = {'/external_area_pointcloud': [{'node_name': 'fixture'}]}
        self.assertEqual(self.check('obstacle_source_samples:/external_area_pointcloud')['status'], 'BLOCKED_REVIEW')

    def test_loaded_ackermann_hash_must_match_inspected_file(self):
        self.assertEqual(self.check('loaded_ackermann_matches_installed')['status'], 'PASS')
        self.observed['runtime_binary_identity']['targets']['ackermann_drive']['records'][0]['sha256'] = 'different'
        self.assertEqual(self.check('loaded_ackermann_matches_installed')['status'], 'FAIL')

    def test_unknown_alignment_and_no_loaded_binary_remain_unknown(self):
        self.observed['alignment'] = {}
        self.observed['runtime_binary_identity'] = {}
        self.assertEqual(self.check('localization_time_alignment')['status'], 'UNKNOWN')
        self.assertEqual(self.check('loaded_binary:ackermann_drive')['status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
