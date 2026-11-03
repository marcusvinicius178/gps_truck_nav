"""Synthetic timing regressions; no ROS process or simulated movement is run."""

import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('tf_timing_contract', ROOT / 'tools/auto_tuning/tf_timing_contract.py')
contract = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contract)
POLICY = {'maximum_age_sim_seconds': 1.0, 'maximum_future_sim_seconds': 0.25,
          'maximum_clock_silence_wall_seconds': 2.0}


def fixture():
    observation = {
        'schema_version': 2, 'observation_complete': True,
        'clock_reference': contract.REFERENCE, 'observation_window_end_wall_offset': 30.0,
        'clock': {'count': 300, 'forward_steps': 299, 'backward_steps': 0,
                  'first_stamp': {'sec': 100, 'nanosec': 0},
                  'last_stamp': {'sec': 130, 'nanosec': 600000000},
                  'last_received_wall_offset': 29.99, 'last_advance_wall_offset': 29.99},
        'edges': [],
    }
    for parent, child in contract.REQUIRED_EDGES:
        static = parent == 'base_footprint'
        timing = {'count': 1 if static else 500, 'forward_steps': 0 if static else 499,
                  'backward_steps': 0, 'first_stamp': {'sec': 0 if static else 100, 'nanosec': 0},
                  'last_stamp': {'sec': 0 if static else 130, 'nanosec': 0 if static else 500000000},
                  'last_received_wall_offset': 0.1 if static else 29.9}
        observation['edges'].append({
            'parent': parent, 'child': child, 'topic': '/tf_static' if static else '/tf',
            'publisher_gids': ['publisher_a'], 'publisher_gid_unknown_count': 0,
            'nonfinite_transform_count': 0,
            'last_transform': {'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                               'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
            'publisher_observations': [{'gid': 'publisher_a', 'timing': timing}],
        })
    return observation


class TfTimingContractTests(unittest.TestCase):
    def setUp(self):
        self.observed = fixture()

    def result(self):
        return contract.assess_tf_timing(self.observed, POLICY)

    def sequence(self):
        return self.observed['edges'][0]['publisher_observations'][0]['timing']

    def test_later_python_snapshot_is_not_substituted_for_observer_clock(self):
        # This synthetic example models the direction of the old window mismatch,
        # without inferring a missing C++ clock in the original host evidence.
        self.observed['python_final_clock'] = 131.610171
        result = self.result()
        self.assertEqual(result['status'], 'PASS')
        self.assertAlmostEqual(result['edges'][0]['publishers'][0]['age_sim_seconds'], 0.1)
        self.assertEqual(POLICY['maximum_age_sim_seconds'], 1.0)

    def test_old_evidence_without_co_observed_clock_cannot_be_reclassified_pass(self):
        self.observed['schema_version'] = 1
        self.observed.pop('clock')
        self.observed.pop('clock_reference')
        self.assertEqual(self.result()['status'], 'UNKNOWN')

    def test_genuinely_stale_or_future_transform_fails_in_the_shared_window(self):
        for sec, nanosec in ((129, 500000000), (131, 0)):
            with self.subTest(stamp=(sec, nanosec)):
                self.sequence()['last_stamp'] = {'sec': sec, 'nanosec': nanosec}
                self.assertEqual(self.result()['status'], 'FAIL')

    def test_static_zero_stamp_and_latched_receipt_are_not_dynamic_staleness(self):
        result = self.result()
        static = result['edges'][2]
        self.assertEqual(result['status'], 'PASS')
        self.assertTrue(static['static_timestamp_exempt'])
        self.assertEqual(static['status'], 'PASS')

    def test_clock_freeze_or_regression_cannot_look_like_fresh_tf(self):
        for update in ({'last_advance_wall_offset': 2.0}, {'last_received_wall_offset': 2.0},
                       {'backward_steps': 1}, {'forward_steps': 0}):
            with self.subTest(update=update):
                self.observed = fixture()
                self.observed['clock'].update(update)
                self.assertEqual(self.result()['status'], 'FAIL')

    def test_missing_clock_reception_timing_or_incomplete_window_is_unknown(self):
        self.observed['clock'].pop('last_advance_wall_offset')
        self.assertEqual(self.result()['status'], 'UNKNOWN')
        self.observed = fixture()
        self.observed['observation_complete'] = False
        self.assertEqual(self.result()['status'], 'UNKNOWN')

    def test_single_nonadvancing_or_regressing_sequence_fails(self):
        for update in ({'count': 1}, {'first_stamp': {'sec': 130, 'nanosec': 500000000}},
                       {'backward_steps': 1}, {'forward_steps': 0}):
            with self.subTest(update=update):
                self.observed = fixture()
                self.sequence().update(update)
                self.assertEqual(self.result()['status'], 'FAIL')

    def test_fresh_gid_cannot_mask_an_old_sequence_from_a_second_publisher(self):
        edge = self.observed['edges'][0]
        second = copy.deepcopy(edge['publisher_observations'][0])
        second['gid'] = 'publisher_b'
        second['timing']['last_stamp'] = {'sec': 120, 'nanosec': 0}
        edge['publisher_gids'].append('publisher_b')
        edge['publisher_observations'].append(second)
        self.assertEqual(self.result()['status'], 'FAIL')

    def test_missing_gid_sequence_is_unknown_not_an_aggregate_pass(self):
        self.observed['edges'][0]['publisher_observations'] = []
        self.assertEqual(self.result()['status'], 'UNKNOWN')
        self.observed = fixture()
        self.observed['edges'][0]['publisher_gid_unknown_count'] = 1
        self.assertEqual(self.result()['status'], 'UNKNOWN')

    def test_nonfinite_transform_and_impossible_reception_offset_fail(self):
        self.observed['edges'][0]['last_transform']['translation']['x'] = float('nan')
        self.assertEqual(self.result()['status'], 'FAIL')
        self.observed = fixture()
        self.sequence()['last_received_wall_offset'] = 31.0
        self.assertEqual(self.result()['status'], 'FAIL')

    def test_missing_required_edge_or_per_gid_reception_evidence_is_unknown(self):
        self.observed['edges'].pop(0)
        self.assertEqual(self.result()['status'], 'UNKNOWN')
        self.observed = fixture()
        self.sequence().pop('last_received_wall_offset')
        self.assertEqual(self.result()['status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
