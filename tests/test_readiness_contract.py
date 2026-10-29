"""Readiness regressions: artificial ROS responses plus sanitized real costmap dumps."""

import copy
import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('readiness_contract', ROOT / 'tools/auto_tuning/readiness_contract.py')
contract = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contract)
PROBE_SPEC = importlib.util.spec_from_file_location('probe_readiness', ROOT / 'tools/auto_tuning/probe_readiness.py')
probe = importlib.util.module_from_spec(PROBE_SPEC)
with mock.patch.dict(sys.modules, {'readiness_contract': contract}):
    PROBE_SPEC.loader.exec_module(probe)
CANONICAL = yaml.safe_load((ROOT / 'src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml').read_text())
EXPECTED = contract.expected_parameters(CANONICAL)
Q = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
COV = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]


def fixture():
    observed = {
        'collection_status': 'COMPLETE',
        'parameters': {key: {'status': 'OK', 'values': copy.deepcopy(value)} for key, value in EXPECTED.items()},
        'lifecycle': {'/' + name: {'status': 'OK', 'id': 3, 'label': 'active'} for name in contract.LIFECYCLE_NODES},
        'observation_window_end_wall_offset': 30.5,
        'clock': {'count': 100, 'first': 1.0, 'last': 10.0, 'forward_steps': 99, 'backward_steps': 0,
                  'last_received_wall_offset': 30.49, 'last_advance_wall_offset': 30.49},
        'topics': {}, 'tf_authority': {'observation_complete': True, 'edges': [],
                                     'publishers': [{'topic': '/tf', 'gid': 'a' * 32, 'node_name': 'fixture_authority'}]},
        'transforms': {key: {'status': 'OK', 'stamp': 9.9, 'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'rotation': Q.copy()}
                       for key in ('map->base_footprint', 'odom->base_footprint', 'base_footprint->base_link',
                                   'base_footprint->gps_link', 'base_footprint->imu_link', 'map->gps_link')},
        'from_ll': {key: {'status': 'OK', 'map_point': {'x': 1.0, 'y': 2.0, 'z': 0.0}} for key in ('datum', 'live_gps')},
        'costmap_bounds': {key: {'inside': True} for key in ('global_costmap', 'local_costmap')},
        'obstacle_source_topics': ['/sick_scan_2D', '/external_area_pointcloud'],
        'topic_publishers': {'/sick_scan_2D': [{'node_name': 'fixture_sensor'}],
                             '/external_area_pointcloud': [{'node_name': 'fixture_source'}]},
        'ackermann_library': {'sha256': 'a' * 64},
    }
    for topic in contract.REQUIRED_TOPICS:
        data = {'frame_id': 'map' if topic in ('/odometry/global', '/odometry/gps') else 'odom',
                'child_frame_id': 'base_footprint', 'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
                'orientation': Q.copy(), 'pose_covariance': [0.1 if i % 7 == 0 else 0.0 for i in range(36)]}
        if topic == '/gps/fix':
            data.update(frame_id='gps_link', latitude=-22.6, longitude=-47.5, altitude=600.0,
                        status=0, position_covariance=COV[:], position_covariance_type=2)
        if topic == '/imu':
            data.update(frame_id='imu_link', **{key: COV[:] for key in (
                'orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance')})
        observed['topics'][topic] = {'count': 30, 'first_stamp': 1.0, 'last_stamp': 9.9,
                                     'forward_steps': 29, 'backward_steps': 0, 'last': data}
    for parent, child in (('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'base_link')):
        observed['tf_authority']['edges'].append({'parent': parent, 'child': child, 'topic': '/tf',
            'publisher_gids': ['a' * 32], 'publisher_gid_unknown_count': 0, 'count': 50,
            'first_stamp': {'sec': 1, 'nanosec': 0},
            'last_stamp': {'sec': 9, 'nanosec': 900000000},
            'last_transform': {'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'rotation': Q.copy()}})
    return observed


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.observed = fixture()

    def report(self):
        return contract.assess(self.observed, EXPECTED)

    def check(self, name):
        return next(item for item in self.report()['checks'] if item['name'] == name)

    def test_contract_uses_canonical_dimensions_frames_layers_and_sensor_sources(self):
        global_map = EXPECTED['/global_costmap/global_costmap']
        local_map = EXPECTED['/local_costmap/local_costmap']
        self.assertEqual((global_map['width'], global_map['height']), (1000, 300))
        self.assertEqual((local_map['width'], local_map['height']), (200, 120))
        self.assertEqual(global_map['global_frame'], 'map')
        self.assertEqual(local_map['global_frame'], 'odom')
        for params in (global_map, local_map):
            self.assertTrue(params['rolling_window'])
            self.assertEqual(params['robot_base_frame'], 'base_footprint')
            self.assertNotIn('static_layer', params['plugins'])
            self.assertNotIn('clear_entirely_local_costmap', params)
        self.assertEqual(global_map['obstacle_layer.observation_sources'], 'sick_scan_2D external_pointcloud')
        self.assertEqual(local_map['voxel_layer.observation_sources'], 'sick_scan_2D')

    def test_real_b8922ae_costmap_defaults_fail_with_specific_differences(self):
        old = json.loads((ROOT / 'tests/fixtures/auto_tuning/costmaps_ipt_sim_01_b8922ae.json').read_text())
        self.assertEqual(old['provenance']['archive_sha256'], '6a48d2c9e96d8ca9202763dcb5f16250591afbbc598937c40b0fe936f6972034')
        self.observed['parameters'].update(old['parameters'])
        for node in ('/global_costmap/global_costmap', '/local_costmap/local_costmap'):
            result = self.check('parameters:' + node)
            self.assertEqual(result['status'], 'FAIL')
            self.assertTrue({'width', 'height', 'rolling_window', 'robot_base_frame', 'footprint',
                             'plugins', 'obstacle_layer.observation_sources'}.issubset(result['differences']))
            self.assertEqual(result['differences']['width']['actual'], 5)
        self.assertEqual(self.report()['diagnostic_status'], 'BLOCKED')

    def test_matching_effective_values_pass_costmap_checks_but_never_authorize_mission(self):
        for node in ('/global_costmap/global_costmap', '/local_costmap/local_costmap'):
            self.assertEqual(self.check('parameters:' + node)['status'], 'PASS')
        result = self.report()
        self.assertEqual(result['diagnostic_status'], 'REVIEW_REQUIRED')
        self.assertFalse(result['mission_ready'])
        self.assertEqual(result['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(result['gate1'], 'NOT_YET_VERIFIED')

    def test_missing_parameter_and_wrong_observation_source_are_not_silently_accepted(self):
        params = self.observed['parameters']['/global_costmap/global_costmap']['values']
        params.pop('rolling_window')
        params['obstacle_layer.observation_sources'] = ''
        diff = self.check('parameters:/global_costmap/global_costmap')['differences']
        self.assertIn('rolling_window', diff)
        self.assertIn('obstacle_layer.observation_sources', diff)

    def test_equivalent_footprint_format_and_numeric_types_do_not_mask_geometry_changes(self):
        self.assertTrue(contract.equivalent('footprint', '[[-1, 2], [3, 4]]', '[[-1,2],[3,4]]'))
        self.assertFalse(contract.equivalent('footprint', '[[-1, 2], [3, 4]]', '[]'))
        self.assertFalse(contract.equivalent('rolling_window', True, 1))

    def test_inactive_navigator_blocks_even_if_collection_completed(self):
        self.observed['lifecycle']['/bt_navigator']['id'] = 2
        self.assertEqual(self.check('lifecycle:/bt_navigator')['status'], 'FAIL')
        self.assertEqual(self.report()['collection_status'], 'COMPLETE')
        self.assertEqual(self.report()['diagnostic_status'], 'BLOCKED')

    def test_missing_parameter_service_is_unknown_not_a_pass(self):
        self.observed['parameters']['/bt_navigator'] = {'status': 'TIMEOUT'}
        self.assertEqual(self.check('parameters:/bt_navigator')['status'], 'UNKNOWN')
        self.assertEqual(self.report()['diagnostic_status'], 'INCOMPLETE')

    def test_default_or_wrong_bt_path_blocks(self):
        self.observed['parameters']['/bt_navigator']['values']['default_nav_to_pose_bt_xml'] = '/opt/ros/iron/default.xml'
        self.assertEqual(self.check('parameters:/bt_navigator')['status'], 'FAIL')

    def test_both_navigators_and_plugins_must_be_effectively_configured(self):
        params = self.observed['parameters']['/bt_navigator']['values']
        self.assertEqual(params['navigators'], ['navigate_to_pose', 'navigate_through_poses'])
        params['navigators'] = ['navigate_to_pose']
        params.pop('navigate_through_poses.plugin')
        result = self.check('parameters:/bt_navigator')
        self.assertEqual(result['status'], 'FAIL')
        self.assertIn('navigators', result['differences'])
        self.assertIn('navigate_through_poses.plugin', result['differences'])

    def test_stale_future_or_single_sensor_samples_fail_freshness(self):
        for update in ({'last_stamp': 1.0}, {'last_stamp': 20.0}, {'count': 1, 'forward_steps': 0}):
            with self.subTest(update=update):
                self.observed = fixture()
                self.observed['topics']['/gps/fix'].update(update)
                self.assertEqual(self.check('freshness:/gps/fix')['status'], 'FAIL')

    def test_frozen_or_backward_clock_fails(self):
        for update in ({'forward_steps': 0}, {'backward_steps': 1}):
            self.observed['clock'].update(update)
            self.assertEqual(self.check('clock_advancing')['status'], 'FAIL')

    def test_clock_and_sensors_stopping_together_do_not_look_fresh(self):
        # Simulated timestamps remain mutually close, but reception stopped early
        # in a 30 s wall window; historical progress must not pass this check.
        self.observed['clock'].update(last_received_wall_offset=2.0, last_advance_wall_offset=2.0)
        self.assertEqual(self.check('clock_advancing')['status'], 'FAIL')
        for topic in contract.REQUIRED_TOPICS:
            self.assertEqual(self.check('freshness:' + topic)['status'], 'FAIL')
        self.assertEqual(self.check('required_tf_numeric_and_fresh')['status'], 'FAIL')

    def test_recent_reception_of_unchanged_clock_still_fails(self):
        self.observed['clock'].update(last_received_wall_offset=30.49, last_advance_wall_offset=2.0)
        result = self.check('clock_advancing')
        self.assertEqual(result['status'], 'FAIL')
        self.assertGreater(result['last_advance_age_wall_seconds'], contract.POLICY['maximum_clock_silence_wall_seconds'])

    def test_missing_clock_monotonic_evidence_is_not_assumed_live(self):
        self.observed['clock'].pop('last_advance_wall_offset')
        self.assertEqual(self.check('clock_advancing')['status'], 'FAIL')

    def test_multiple_topic_publishers_do_not_imply_duplicate_tf_authority(self):
        self.observed['topic_publishers']['/tf'] = [{'node_name': str(i)} for i in range(4)]
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'PASS')

    def test_duplicate_gid_per_tf_edge_fails(self):
        self.observed['tf_authority']['edges'][0]['publisher_gids'].append('b' * 32)
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'FAIL')

    def test_duplicate_edge_across_tf_and_tf_static_fails(self):
        edge = copy.deepcopy(self.observed['tf_authority']['edges'][0])
        edge.update(topic='/tf_static', publisher_gids=['b' * 32])
        self.observed['tf_authority']['edges'].append(edge)
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'FAIL')

    def test_tf_parent_conflict_is_detected(self):
        edge = copy.deepcopy(self.observed['tf_authority']['edges'][0])
        edge['parent'] = 'world'
        self.observed['tf_authority']['edges'].append(edge)
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'FAIL')

    def test_unknown_gid_missing_edge_and_incomplete_window_remain_unknown(self):
        for mutate in (lambda obs: obs['edges'].pop(),
                       lambda obs: obs['edges'][0].update(publisher_gids=[]),
                       lambda obs: obs.update(observation_complete=False)):
            self.observed = fixture()
            mutate(self.observed['tf_authority'])
            self.assertEqual(self.check('tf_authority_per_edge')['status'], 'UNKNOWN')

    def test_static_zero_stamp_is_not_mistaken_for_stale_dynamic_tf(self):
        self.observed['tf_authority']['edges'][2].update(topic='/tf_static', last_stamp={'sec': 0, 'nanosec': 0})
        self.observed['tf_authority']['publishers'].append({'topic': '/tf_static', 'gid': 'a' * 32})
        self.observed['transforms']['base_footprint->gps_link']['stamp'] = 0.0
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'PASS')
        self.assertEqual(self.check('tf_lookup:base_footprint->gps_link')['status'], 'PASS')

    def test_unknown_tf_endpoint_or_missing_sensor_transform_remains_unknown(self):
        self.observed['tf_authority']['publishers'] = []
        self.observed['transforms']['base_footprint->gps_link'] = {'status': 'UNAVAILABLE'}
        self.assertEqual(self.check('tf_authority_per_edge')['status'], 'UNKNOWN')
        self.assertEqual(self.check('tf_lookup:base_footprint->gps_link')['status'], 'UNKNOWN')

    def test_stale_dynamic_transform_is_not_accepted_as_live(self):
        self.observed['transforms']['map->base_footprint']['stamp'] = 0.0
        self.assertEqual(self.check('tf_lookup:map->base_footprint')['status'], 'UNKNOWN')

    def test_required_dynamic_edge_stamps_and_quaternions_are_validated(self):
        edge = self.observed['tf_authority']['edges'][0]
        edge['last_stamp'] = {'sec': 0, 'nanosec': 0}
        self.assertEqual(self.check('required_tf_numeric_and_fresh')['status'], 'FAIL')
        edge['last_stamp'] = {'sec': 9, 'nanosec': 900000000}
        edge['last_transform']['rotation']['w'] = 0.0
        self.assertEqual(self.check('required_tf_numeric_and_fresh')['status'], 'FAIL')

    def test_single_or_nonadvancing_dynamic_tf_sample_does_not_pass(self):
        for update in ({'count': 1}, {'first_stamp': {'sec': 9, 'nanosec': 900000000}}):
            with self.subTest(update=update):
                self.observed = fixture()
                self.observed['tf_authority']['edges'][0].update(update)
                self.assertEqual(self.check('required_tf_numeric_and_fresh')['status'], 'FAIL')

    def test_real_sensor_frame_and_zero_covariance_findings_require_review(self):
        gps = self.observed['topics']['/gps/fix']['last']
        gps.update(frame_id='base_footprint', position_covariance=[0.0] * 9)
        imu = self.observed['topics']['/imu']['last']
        imu.update(frame_id='base_footprint', orientation_covariance=[0.0] * 9)
        self.observed['topics']['/odometry/gps']['last']['pose_covariance'] = [0.0] * 36
        for name in ('sensor_frame:/gps/fix', 'sensor_frame:/imu', 'gps_covariance', 'imu:orientation_covariance', 'gps_odometry_position_covariance'):
            self.assertEqual(self.check(name)['status'], 'BLOCKED_REVIEW')
        self.assertFalse(self.report()['mission_ready'])

    def test_negative_asymmetric_and_nonfinite_covariances_fail(self):
        for values in ([-1.0] + COV[1:], [0.1, 1.0] + COV[2:], [float('nan')] + COV[1:]):
            with self.subTest(values=values):
                self.assertEqual(contract.covariance_status(values)[0], 'FAIL')
        self.assertEqual(contract.covariance_status(COV)[0], 'PASS')

    def test_invalid_quaternion_or_gnss_fix_fails(self):
        self.observed['topics']['/imu']['last']['orientation'] = {'x': 0, 'y': 0, 'z': 0, 'w': 0}
        self.observed['topics']['/gps/fix']['last']['status'] = -1
        self.assertEqual(self.check('quaternion:/imu')['status'], 'FAIL')
        self.assertEqual(self.check('gnss_fix_valid')['status'], 'FAIL')

    def test_from_ll_requires_actual_response_not_only_service_type(self):
        self.observed['from_ll']['datum'] = {'service_type': 'robot_localization/srv/FromLL'}
        self.assertEqual(self.check('from_ll:datum')['status'], 'UNKNOWN')

    def test_missing_obstacle_source_and_unknown_bounds_do_not_pass(self):
        self.observed['topic_publishers']['/external_area_pointcloud'] = []
        self.observed['costmap_bounds']['local_costmap']['inside'] = None
        self.assertEqual(self.check('obstacle_source_publishers:/external_area_pointcloud')['status'], 'BLOCKED_REVIEW')
        self.assertEqual(self.check('robot_center_in:local_costmap')['status'], 'UNKNOWN')

    def test_empty_capture_never_claims_health_or_gate_success(self):
        report = contract.assess({}, {})
        self.assertIn(report['diagnostic_status'], ('BLOCKED', 'INCOMPLETE'))
        self.assertFalse(report['mission_ready'])
        self.assertEqual(report['gate0'], 'NOT_YET_VERIFIED')

    def test_costmap_bounds_respect_origin_resolution_and_rotation(self):
        metadata = {'frame_id': 'map', 'width': 10, 'height': 5, 'resolution': 2.0,
                    'origin_position': {'x': 100.0, 'y': 20.0, 'z': 0.0},
                    'origin_orientation': {'x': 0.0, 'y': 0.0, 'z': math.sqrt(0.5), 'w': math.sqrt(0.5)}}
        transform = {'status': 'OK', 'translation': {'x': 96.0, 'y': 28.0, 'z': 0.0}}
        result = contract.costmap_bounds(metadata, transform, 'map')
        self.assertTrue(result['inside'])
        self.assertAlmostEqual(result['robot_in_grid_m']['x'], 8.0)
        self.assertAlmostEqual(result['robot_in_grid_m']['y'], 4.0)
        transform['translation']['y'] = 41.0
        self.assertFalse(contract.costmap_bounds(metadata, transform, 'map')['inside'])
        metadata['origin_orientation']['w'] = float('nan')
        self.assertIsNone(contract.costmap_bounds(metadata, transform, 'map')['inside'])

    def test_collector_failure_preserves_report_and_never_returns_success(self):
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(probe, 'collect', side_effect=RuntimeError('fixture failure')):
            with contextlib.redirect_stdout(io.StringIO()):
                code = probe.main([temporary, '--duration', '10'])
            result = json.loads((Path(temporary) / 'readiness.json').read_text())
            observed = json.loads((Path(temporary) / 'readiness_observations.json').read_text())
            self.assertEqual(code, 2)
            self.assertEqual(result['collection_status'], 'FAILED')
            self.assertFalse(result['mission_ready'])
            self.assertIn('fixture failure', observed['error'])

    def test_assessment_failure_still_preserves_raw_observations(self):
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(probe, 'collect'), mock.patch.object(probe, 'assess', side_effect=ValueError('bad fixture')):
            with contextlib.redirect_stdout(io.StringIO()):
                code = probe.main([temporary])
            self.assertEqual(code, 2)
            result = json.loads((Path(temporary) / 'readiness.json').read_text())
            self.assertEqual(result['diagnostic_status'], 'INCOMPLETE')
            self.assertFalse(result['mission_ready'])
            self.assertTrue((Path(temporary) / 'readiness_observations.json').is_file())

    def test_existing_evidence_is_not_replaced(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'readiness.json'
            path.write_text('previous evidence')
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                probe.main([temporary])
            self.assertEqual(path.read_text(), 'previous evidence')

    def test_nonfinite_json_values_are_preserved_as_explicit_tokens(self):
        converted = probe.json_safe({'nan': float('nan'), 'inf': float('inf')})
        self.assertEqual(converted, {'nan': 'nan', 'inf': 'inf'})
        json.dumps(converted, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
