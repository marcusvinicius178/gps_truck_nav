import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools/auto_tuning'))
from speed_chain_evidence import speed_chain_evidence


def observed(cap=25.0, smoother_cap=8.33):
    values = {'FollowPath.' + key: {'type': 3, 'value': value} for key, value in
              {'vx_max': cap, 'vx_min': -2.25, 'wz_max': 0.4, 'model_dt': 0.05, 'time_steps': 56}.items()}
    return {'navigation_profile': {'name': 'fixture'}, 'plugin_parameters': {'nodes': {
        '/controller_server': {'values': values}}}, 'parameters': {'/velocity_smoother': {
        'status': 'OK', 'values': {'max_velocity': [smoother_cap, 0.0, 0.4],
                                 'min_velocity': [-2.25, 0.0, -0.4], 'feedback': 'OPEN_LOOP'}}},
        'topic_publishers': {'/cmd_vel': [{'node_name': 'velocity_smoother'}, {'node_name': 'behavior_server'}]}}


class SpeedEvidenceTests(unittest.TestCase):
    def test_historical_envelopes_are_not_reinterpreted_as_typo(self):
        data = observed(); before = copy.deepcopy(data)
        result = speed_chain_evidence(data)
        self.assertEqual(result['mppi_effective']['vx_max'], 25.0)
        self.assertEqual(result['nominal_forward_ceiling_on_smoother_path_mps'], 8.33)
        self.assertFalse(result['matching_declared_envelopes'])
        self.assertEqual(result['command_path_status'], 'BYPASS_PRESENT')
        self.assertFalse(result['physical_speed_verified'])
        self.assertEqual(data, before)

    def test_profiles_are_caps_not_constant_corner_speeds(self):
        for kmh in (20, 25):
            result = speed_chain_evidence(observed(kmh / 3.6, kmh / 3.6))
            self.assertTrue(result['matching_declared_envelopes'])
            self.assertAlmostEqual(result['ideal_radius_at_forward_ceiling_due_to_yaw_cap_m'], kmh / 3.6 / 0.4)
            self.assertFalse(result['motion_authorized'])

    def test_missing_or_invalid_values_never_invent_effective_caps(self):
        for data in ({}, observed()):
            if data:
                data['parameters']['/velocity_smoother']['values']['max_velocity'][0] = float('nan')
            result = speed_chain_evidence(data)
            self.assertEqual(result['status'], 'INCOMPLETE')
            self.assertNotIn('nominal_forward_ceiling_on_smoother_path_mps', result)

    def test_unknown_publisher_is_not_proven_safe(self):
        data = observed(); data['topic_publishers']['/cmd_vel'] = [{}]
        self.assertEqual(speed_chain_evidence(data)['command_path_status'], 'BYPASS_PRESENT')
        data['topic_publishers'] = {}
        self.assertEqual(speed_chain_evidence(data)['command_path_status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
