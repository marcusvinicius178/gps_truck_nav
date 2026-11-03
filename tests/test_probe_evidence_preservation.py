"""Partial readback evidence survives errors in another concurrent observer."""

from collections import deque
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
SPEC = importlib.util.spec_from_file_location('probe_readiness', ROOT / 'tools/auto_tuning/probe_readiness.py')
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class ProbeEvidencePreservationTests(unittest.TestCase):
    def fixture(self):
        report = {'collection_status': 'PENDING', 'jobs': [{'service': '/planner_server/get_parameters',
                  'attempts': [{'number': 1, 'status': 'OK'}]}],
                  'nodes': {'/planner_server': {'values': {'GridBased.non_straight_penalty': 2.5}}}}
        reader = mock.Mock(report=report)
        reader.finish.return_value = report
        observed = {'collection_status': 'FAILED', 'plugin_parameters': report,
                    'error': 'JSONDecodeError: incomplete TF JSON'}
        history = {'/gps/fix': deque([{'stamp_ns': 1234567890}], maxlen=128)}
        return reader, observed, history

    def test_other_observer_failure_preserves_attempts_values_and_history(self):
        reader, observed, history = self.fixture()
        probe.finish_plugin_capture(observed, reader, history)
        self.assertEqual(observed['error'], 'JSONDecodeError: incomplete TF JSON')
        self.assertEqual(observed['plugin_parameters']['jobs'][0]['attempts'][0]['status'], 'OK')
        self.assertEqual(observed['plugin_parameters']['nodes']['/planner_server']['values']['GridBased.non_straight_penalty'], 2.5)
        self.assertEqual(observed['localization_sample_history']['/gps/fix'][0]['stamp_ns'], 1234567890)
        self.assertEqual(observed['collection_status'], 'FAILED')

    def test_finish_failure_records_error_without_erasing_original_evidence(self):
        reader, observed, history = self.fixture()
        reader.finish.side_effect = RuntimeError('ROS context ended')
        probe.finish_plugin_capture(observed, reader, history)
        self.assertEqual(observed['error'], 'JSONDecodeError: incomplete TF JSON')
        self.assertEqual(observed['plugin_parameters']['collection_status'], 'PARTIAL')
        self.assertIn('ROS context ended', observed['plugin_parameters']['finalization_error'])
        self.assertEqual(len(observed['plugin_parameters']['jobs'][0]['attempts']), 1)
        self.assertEqual(observed['collection_status'], 'FAILED')


if __name__ == '__main__':
    unittest.main()
