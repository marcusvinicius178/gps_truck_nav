"""Exact-time observation contracts; fixtures do not validate localization."""

import copy
import importlib.util
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'localization_alignment', ROOT / 'tools/auto_tuning/localization_alignment.py')
alignment = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(alignment)
STAMP = 125_000_832_000
IDENTITY = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def fixture():
    fix = {'stamp_ns': STAMP, 'frame_id': 'gps_link', 'status': 0,
           'latitude': -22.6, 'longitude': -47.5, 'altitude': 491.0}
    conversion = {
        'status': 'OK', 'service': '/fromLL', 'input_sensor_stamp_ns': STAMP,
        'input_frame_id': 'gps_link', 'output_frame_id': 'map',
        'input': {key: fix[key] for key in ('latitude', 'longitude', 'altitude')},
        'map_point': {'x': 13.8205, 'y': 20.0, 'z': 0.0},
    }
    odometry = {'stamp_ns': STAMP, 'frame_id': 'map', 'child_frame_id': '',
                'position': {'x': 10.0, 'y': 20.0, 'z': 0.0}}
    positions = {
        'map->gps_link': {'x': 13.8205, 'y': 20.0, 'z': 3.062},
        'map->base_footprint': {'x': 10.0, 'y': 20.0, 'z': 0.0},
        'base_footprint->gps_link': {'x': 3.8205, 'y': 0.0, 'z': 3.062},
    }
    transforms = {}
    for name, (parent, child, static) in alignment.TRANSFORMS.items():
        transforms[name] = {
            'status': 'OK', 'parent': parent, 'child': child,
            'requested_stamp_ns': STAMP, 'stamp_ns': 0 if static else STAMP,
            'lookup_mode': 'EXACT', 'is_static': static,
            'translation': positions[name], 'rotation': IDENTITY.copy(),
        }
    return conversion, fix, odometry, transforms


class LocalizationAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.args = fixture()
        self.conversion, self.fix, self.odometry, self.transforms = self.args

    def analyze(self):
        return alignment.analyze_alignment(*self.args)

    def test_exact_observations_allow_comparison_without_geometric_approval(self):
        report = self.analyze()
        self.assertEqual(report['status'], 'PASS')
        self.assertTrue(report['synchronized_measurements_verified'])
        self.assertFalse(report['geometric_validated'])
        self.assertFalse(report['internal_navsat_time_compensation_verified'])
        self.assertFalse(report['mission_ready'])
        for residual in report['residuals'].values():
            self.assertAlmostEqual(residual['horizontal_distance_m'], 0.0)
        self.assertAlmostEqual(report['residuals']['from_ll_gps_vs_map_gps_link']
                               ['vertical_difference_m_not_evaluated'], -3.062)

    def test_rotated_lever_arm_uses_map_heading(self):
        rotation = {'x': 0.0, 'y': 0.0, 'z': math.sqrt(0.5), 'w': math.sqrt(0.5)}
        self.transforms['map->base_footprint']['rotation'] = rotation
        self.transforms['map->gps_link']['rotation'] = rotation.copy()
        self.transforms['map->gps_link']['translation'].update(x=10.0, y=23.8205)
        self.conversion['map_point'].update(x=10.0, y=23.8205)
        report = self.analyze()
        self.assertEqual(report['status'], 'PASS')
        self.assertAlmostEqual(report['sensor_lever_arm_map_m']['x'], 0.0)
        self.assertAlmostEqual(report['sensor_lever_arm_map_m']['y'], 3.8205)
        self.assertAlmostEqual(report['residuals']['from_ll_minus_gps_odometry_vs_rotated_lever_arm']
                               ['horizontal_distance_m'], 0.0)

    def test_pitch_projects_vertical_lever_arm_into_horizontal_plane(self):
        pitch = 0.02016
        rotation = {'x': 0.0, 'y': math.sin(pitch / 2), 'z': 0.0, 'w': math.cos(pitch / 2)}
        self.transforms['map->base_footprint']['rotation'] = rotation
        self.transforms['map->gps_link']['rotation'] = rotation.copy()
        x = 3.8205 * math.cos(pitch) + 3.062 * math.sin(pitch)
        z = -3.8205 * math.sin(pitch) + 3.062 * math.cos(pitch)
        self.transforms['map->gps_link']['translation'].update(x=10 + x, z=z)
        self.conversion['map_point']['x'] = 10 + x
        report = self.analyze()
        self.assertEqual(report['status'], 'PASS')
        self.assertGreater(report['sensor_lever_arm_map_m']['x'] - 3.8205, 0.06)
        self.assertAlmostEqual(report['sensor_lever_arm_map_m']['x'], x)
        self.assertAlmostEqual(report['residuals']['tf_chain_composition']['horizontal_distance_m'], 0.0)

    def test_roll_projects_vertical_lever_arm_into_lateral_axis(self):
        roll = 0.04
        self.transforms['map->base_footprint']['rotation'] = {
            'x': math.sin(roll / 2), 'y': 0.0, 'z': 0.0, 'w': math.cos(roll / 2)}
        report = self.analyze()
        self.assertEqual(report['status'], 'PASS')
        self.assertAlmostEqual(report['sensor_lever_arm_map_m']['y'], -3.062 * math.sin(roll))

    def test_one_nanosecond_mismatch_does_not_select_nearest_odometry(self):
        self.odometry['stamp_ns'] += 1
        report = self.analyze()
        self.assertEqual(report['status'], 'FAIL')
        self.assertFalse(report['synchronized_measurements_verified'])
        self.assertEqual(report['residuals'], {})

    def test_mismatched_request_or_response_timestamp_is_rejected(self):
        for key in ('requested_stamp_ns', 'stamp_ns'):
            with self.subTest(key=key):
                args = fixture()
                args[3]['map->gps_link'][key] += 1
                self.assertEqual(alignment.analyze_alignment(*args)['status'], 'FAIL')

    def test_float_timestamp_is_not_silently_converted_to_integer_nanoseconds(self):
        for value in (125.000832, float(STAMP), True, 0):
            with self.subTest(value=value):
                self.conversion['input_sensor_stamp_ns'] = value
                self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_latest_lookup_is_rejected_even_if_returned_stamp_matches(self):
        self.transforms['map->gps_link']['lookup_mode'] = 'LATEST'
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_zero_dynamic_timestamp_cannot_masquerade_as_static(self):
        self.transforms['map->gps_link'].update(stamp_ns=0, is_static=True)
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_static_zero_requires_explicit_fixed_chain_identity(self):
        self.transforms['base_footprint->gps_link'].pop('is_static')
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_explicit_time_for_static_chain_is_also_valid(self):
        self.transforms['base_footprint->gps_link']['stamp_ns'] = STAMP
        self.assertEqual(self.analyze()['status'], 'PASS')

    def test_missing_exact_tf_produces_unknown_without_residuals(self):
        self.transforms['map->gps_link'] = {'status': 'UNAVAILABLE', 'error': 'extrapolation'}
        report = self.analyze()
        self.assertEqual(report['status'], 'UNKNOWN')
        self.assertFalse(report['synchronized_measurements_verified'])
        self.assertEqual(report['residuals'], {})

    def test_missing_sample_is_not_substituted_with_another_time(self):
        args = list(self.args)
        args[2] = None
        self.assertEqual(alignment.analyze_alignment(*args)['status'], 'UNKNOWN')

    def test_from_ll_input_identity_is_checked_not_just_timestamp(self):
        self.conversion['input']['longitude'] += 0.0001
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_sensor_and_transform_frames_must_match_the_physical_reference(self):
        for mutation in ('sensor', 'odometry', 'parent', 'child', 'from_ll'):
            with self.subTest(mutation=mutation):
                args = fixture()
                if mutation == 'sensor':
                    args[1]['frame_id'] = 'base_footprint'
                elif mutation == 'odometry':
                    args[2]['frame_id'] = 'odom'
                elif mutation in ('parent', 'child'):
                    args[3]['map->gps_link'][mutation] = 'wrong_frame'
                else:
                    args[0].pop('output_frame_id')
                self.assertEqual(alignment.analyze_alignment(*args)['status'], 'FAIL')

    def test_nonfinite_positions_and_quaternions_fail(self):
        for value in (float('nan'), float('inf'), float('-inf')):
            with self.subTest(value=value):
                args = fixture()
                args[3]['map->base_footprint']['translation']['x'] = value
                self.assertEqual(alignment.analyze_alignment(*args)['status'], 'FAIL')
                args = fixture()
                args[3]['map->base_footprint']['rotation']['w'] = value
                self.assertEqual(alignment.analyze_alignment(*args)['status'], 'FAIL')

    def test_nonunit_quaternion_is_not_repaired_to_make_check_pass(self):
        self.transforms['map->base_footprint']['rotation']['w'] = 1.2
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_finite_arithmetic_overflow_does_not_produce_pass_with_infinite_residual(self):
        self.conversion['map_point']['x'] = 1e308
        self.transforms['map->gps_link']['translation']['x'] = -1e308
        report = self.analyze()
        self.assertEqual(report['status'], 'FAIL')
        self.assertFalse(report['synchronized_measurements_verified'])
        self.assertEqual(report['residuals'], {})

    def test_missing_conversion_response_is_unknown_without_invented_measurement(self):
        self.conversion['status'] = 'TIMEOUT'
        report = self.analyze()
        self.assertEqual(report['status'], 'UNKNOWN')
        self.assertEqual(report['residuals'], {})

    def test_large_residual_remains_a_measurement_without_invented_acceptance_limit(self):
        self.conversion['map_point']['x'] += 100.0
        report = self.analyze()
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['residuals']['from_ll_gps_vs_map_gps_link']['horizontal_distance_m'], 100.0)
        self.assertFalse(report['geometric_validated'])

    def test_invalid_fix_is_not_evidence_of_geometric_alignment(self):
        self.fix['status'] = -1
        self.assertEqual(self.analyze()['status'], 'FAIL')

    def test_inputs_are_not_mutated(self):
        before = copy.deepcopy(self.args)
        self.analyze()
        self.assertEqual(self.args, before)


if __name__ == '__main__':
    unittest.main()
