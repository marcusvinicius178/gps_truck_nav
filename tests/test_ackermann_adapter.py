"""Offline fixtures only: no ROS, publisher, simulator or physical validation."""

import ast
import copy
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
from ackermann_adapter import (ShadowObserver, attach_subscriptions, convert_classic,
                                gid_text, json_safe, load_policy, message_info_evidence,
                                rear_right_offset, run_shadow, inherit_evidence_owner,
                                private_evidence_file)
from actuation_contract import ContractError, Policy
from audit_ackermann_geometry import audit_model


class AckermannAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audit = audit_model(ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf')

    def setUp(self):
        # Synthetic branch-testing thresholds, not an operational vehicle policy.
        self.policy = Policy(20 / 3.6, 2.25, 0.4, .01, .02, .25, .5, 1., 'SCALE_PRESERVE_CURVATURE')
        self.records = []
        self.observer = ShadowObserver(self.audit, self.policy, self.records.append, started_at=0.)

    def components(self, v=1., w=.04, **other):
        values = {'linear_x': v, 'linear_y': 0., 'linear_z': 0.,
                  'angular_x': 0., 'angular_y': 0., 'angular_z': w}
        values.update(other)
        return values

    def health(self, now=.1):
        self.observer.observe_clock(100 + now, now=now)
        self.observer.observe_odom(0., 0., 'base_footprint', stamp_seconds=100 + now, now=now)
        self.observer.observe_graph([{'gid': '01', 'node': '/velocity_smoother'}], [], now=now)

    def test_signed_right_rear_target_and_virtual_steering_in_four_quadrants(self):
        for v in (-1., 1.):
            for w in (-.04, .04):
                with self.subTest(v=v, w=w):
                    result = convert_classic(v, w, self.audit, self.policy)
                    candidate = result['candidate']
                    y = rear_right_offset(self.audit)
                    self.assertAlmostEqual(y, -1.299)
                    self.assertAlmostEqual(candidate['plugin_linear_x_mps'], v - w * y)
                    self.assertNotEqual(candidate['plugin_linear_x_mps'], v)
                    delta = candidate['plugin_angular_z_rad'] * math.copysign(1, candidate['plugin_linear_x_mps'])
                    self.assertAlmostEqual(v * math.tan(delta) / self.audit['wheelbase_euclidean_m'], w)
                    self.assertFalse(result['runtime_authorized'])
                    self.assertFalse(result['rolling_and_joint_sign_validated'])

    def test_straight_line_target_equals_center_speed(self):
        result = convert_classic(1, 0, self.audit, self.policy)
        self.assertEqual(result['candidate']['plugin_linear_x_mps'], 1)

    def test_zero_and_signed_zero_normalize_all_targets(self):
        for v in (-0., 0.):
            result = convert_classic(v, 0, self.audit, self.policy)
            self.assertEqual(result['status'], 'STOP')
            self.assertTrue(all(value == 0 and math.copysign(1, value) == 1 for value in result['candidate'].values()))

    def test_nonzero_yaw_at_zero_is_rejected(self):
        result = convert_classic(0., .1, self.audit, self.policy)
        self.assertEqual(result['reason'], 'UNREALIZABLE_NEAR_ZERO_YAW')
        self.assertTrue(all(value == 0 for value in result['candidate'].values()))

    def test_forward_only_policy_rejects_reverse(self):
        result = convert_classic(-1, .02, self.audit, replace(self.policy, reverse_speed_mps=0))
        self.assertEqual(result['reason'], 'REVERSE_DISABLED')

    def test_curvature_is_not_repaired_by_speed_saturation(self):
        result = convert_classic(10, 10 / 11.6, self.audit, self.policy)
        self.assertEqual(result['reason'], 'CURVATURE_EXCEEDS_STATIC_JOINT_ENVELOPE')

    def test_center_cap_scales_center_yaw_and_right_target_consistently(self):
        result = convert_classic(10, .3, self.audit, self.policy)
        c = result['candidate']
        self.assertEqual(result['status'], 'SCALED')
        self.assertAlmostEqual(c['rear_axle_speed_mps'], 20 / 3.6)
        self.assertAlmostEqual(c['yaw_rate_rps'] / c['rear_axle_speed_mps'], .03)
        self.assertAlmostEqual(c['plugin_linear_x_mps'], c['rear_axle_speed_mps'] + c['yaw_rate_rps'] * 1.299)

    def test_right_wheel_cap_is_separate_from_center_cap(self):
        policy = replace(self.policy, forward_speed_mps=20)
        result = convert_classic(20, .4, self.audit, policy)
        c = result['candidate']
        self.assertEqual(result['status'], 'SCALED')
        self.assertAlmostEqual(c['plugin_linear_x_mps'], 20)
        self.assertLess(c['rear_axle_speed_mps'], 20)
        self.assertAlmostEqual(c['yaw_rate_rps'] / c['rear_axle_speed_mps'], .02)
        rejected = convert_classic(20, .4, self.audit, replace(policy, speed_policy='REJECT'))
        self.assertEqual(rejected['reason'], 'RIGHT_WHEEL_PLUGIN_SPEED_LIMIT')

    def test_nonfinite_input_is_rejected_and_evidence_json_is_strict(self):
        result = self.observer.observe_command(self.components(math.nan, .1), now=.1)
        self.assertEqual(result['reason'], 'NONFINITE_INPUT')
        json.dumps(self.records[-1], allow_nan=False)
        self.assertEqual(self.records[-1]['twist']['linear_x'], 'nan')

    def test_unsupported_twist_components_never_silently_disappear(self):
        for field in ('linear_y', 'linear_z', 'angular_x', 'angular_y'):
            result = self.observer.observe_command(self.components(**{field: .1}), now=.1)
            self.assertEqual(result['reason'], 'UNSUPPORTED_TWIST_COMPONENTS')
            self.assertTrue(all(value == 0 for value in result['candidate'].values()))

    def test_rear_geometry_malformed_is_rejected(self):
        audit = copy.deepcopy(self.audit)
        audit['wheels']['rear_right_joint']['collision_center_model_m'][1] = 2.
        with self.assertRaises(ContractError):
            convert_classic(1., .04, audit, self.policy)

    def test_right_target_direction_change_is_rejected(self):
        audit = copy.deepcopy(self.audit)
        audit['wheels']['rear_right_joint']['collision_center_model_m'][1] = -100.
        audit['wheels']['rear_left_joint']['collision_center_model_m'][1] = 100.
        result = convert_classic(1., -.04, audit, self.policy)
        self.assertEqual(result['reason'], 'RIGHT_WHEEL_TARGET_INVALID_OR_REVERSES_SIGN')

    def test_no_commands_is_expected_but_conversion_not_verified(self):
        self.health()
        result = self.observer.summary(.1)
        self.assertEqual(result['conversion_status'], 'NOT_EXERCISED')
        self.assertIn('NO_COMMANDS_OBSERVED_EXPECTED_WITHOUT_GOAL', result['observation_reasons'])
        self.assertEqual(result['diagnostic_status'], 'BLOCKED_ACTUATION_REVIEW')
        self.assertEqual(result['command_publishers_created'], 0)
        self.assertFalse(result['mission_ready'])

    def test_command_receipt_expiration_is_not_a_stop_claim(self):
        self.health()
        self.observer.observe_command(self.components(0, 0), now=.1, publisher_gid='01')
        summary = self.observer.summary(.36)
        self.assertIn('COMMAND_RECEIPT_EXPIRED', summary['observation_reasons'])
        self.assertFalse(summary['physical_stop_verified'])

    def test_clock_stall_regression_and_odom_stamp_are_distinguished(self):
        self.health()
        self.observer.observe_clock(100.1, now=1.2)
        self.observer.observe_odom(0, 0, 'base_footprint', stamp_seconds=99, now=1.2)
        reasons = self.observer.health(1.2)
        self.assertIn('SIM_CLOCK_STALLED', reasons)
        self.assertIn('ODOMETRY_SIM_STAMP_OUTSIDE_CLOCK_WINDOW', reasons)
        self.observer.observe_clock(90, now=1.3)
        self.assertIn('SIM_CLOCK_REGRESSED', self.observer.health(1.3))

    def test_multiple_publishers_and_gid_mismatch_are_preserved(self):
        self.health()
        self.observer.observe_graph([{'gid': '01'}, {'gid': '02'}], [], now=.1)
        self.observer.observe_command(self.components(0, 0), now=.1, publisher_gid='03')
        self.assertIn('INPUT_AUTHORITY_NOT_UNIQUE', self.records[-1]['observation_reasons'])
        self.assertIn('SOURCE_GID_NOT_IN_OBSERVED_GRAPH', self.records[-1]['observation_reasons'])

    def test_nonzero_observed_flags_owning_no_motion_probe(self):
        self.observer.observe_command(self.components(), now=.1)
        self.assertTrue(self.observer.summary(.1)['unexpected_nonzero_observed'])

    def test_policy_must_be_complete_and_shadow_only(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'policy.json'
            document = {'schema_version': 1, 'mode': 'SHADOW_ONLY', 'policy': asdict(self.policy)}
            path.write_text(json.dumps(document))
            loaded, provenance = load_policy(path)
            self.assertEqual(loaded, self.policy)
            self.assertEqual(len(provenance['sha256']), 64)
            document['mode'] = 'LIVE'
            path.write_text(json.dumps(document))
            with self.assertRaises(ContractError):
                load_policy(path)
            document['mode'] = 'SHADOW_ONLY'
            del document['policy']['command_timeout_s']
            path.write_text(json.dumps(document))
            with self.assertRaises(ContractError):
                load_policy(path)

    def test_fake_transport_only_receives_three_topics_and_records_all_components(self):
        class ReceiveOnlyNode:
            def __init__(self):
                self.callbacks = {}
            def create_subscription(self, message_type, topic, callback, qos):
                self.callbacks[topic] = callback
                return topic
        node = ReceiveOnlyNode()
        handles = attach_subscriptions(node, self.observer,
            dict.fromkeys(('Twist', 'Clock', 'Odometry'), object), object(),
            input_topic='/candidate_input', odom_topic='/odom', monotonic=lambda: .1)
        self.assertEqual(set(handles), {'/candidate_input', '/clock', '/odom'})
        vector = lambda x, y=0., z=0.: SimpleNamespace(x=x, y=y, z=z)
        message = SimpleNamespace(linear=vector(1.), angular=vector(0., z=.04))
        iron_info = {'source_timestamp': 1760000000123456789, 'received_timestamp': 1760000000123456799,
                     'publication_sequence_number': 34, 'reception_sequence_number': None}
        self.health()
        node.callbacks['/candidate_input'](message, iron_info)
        self.assertIsNone(self.records[0]['publisher_gid'])
        self.assertEqual(self.records[0]['middleware_info']['fields'], iron_info)
        self.assertIn('SOURCE_GID_UNAVAILABLE_IN_RCLPY_IRON', self.records[0]['observation_reasons'])
        self.assertNotIn('SOURCE_GID_NOT_IN_OBSERVED_GRAPH', self.records[0]['observation_reasons'])
        self.assertFalse(self.records[0]['command_source_authority_verified'])
        self.assertEqual(len(self.records[0]['twist']), 6)
        self.assertAlmostEqual(self.records[0]['conversion']['candidate']['plugin_linear_x_mps'], 1.05196)

    def test_future_transport_real_gid_is_retained_and_still_checked(self):
        gid, metadata = message_info_evidence(SimpleNamespace(publisher_gid=[3], source_timestamp=12))
        self.assertEqual(gid, '03')
        self.assertEqual(metadata['fields'], {'source_timestamp': 12})
        self.health()
        self.observer.observe_command(self.components(0, 0), now=.1, publisher_gid=gid, middleware_info=metadata)
        reasons = self.records[0]['observation_reasons']
        self.assertIn('SOURCE_GID_NOT_IN_OBSERVED_GRAPH', reasons)
        self.assertNotIn('SOURCE_GID_UNAVAILABLE_IN_RCLPY_IRON', reasons)

    def test_optional_dictionary_gid_is_not_inferred_from_sequence_fields(self):
        gid, metadata = message_info_evidence({'publisher_gid': [1, 2], 'publication_sequence_number': 7})
        self.assertEqual(gid, '0102')
        self.assertTrue(metadata['publisher_gid_available'])
        gid, metadata = message_info_evidence({'publication_sequence_number': 7})
        self.assertIsNone(gid)
        self.assertFalse(metadata['publisher_gid_available'])

    def test_module_has_no_publisher_or_publish_call_or_live_cli(self):
        tree = ast.parse((ROOT / 'tools/auto_tuning/ackermann_adapter.py').read_text())
        attrs = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertFalse({'create_publisher', 'publish', 'create_client', 'call_async'} & attrs)
        cli = [node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)]
        self.assertFalse({'--live', '--enable', '--arm', '--output-topic'} & set(cli))

    def test_duration_validation_happens_before_ros_import_or_output_creation(self):
        for duration in (0., math.inf, 1801.):
            with self.assertRaises(ContractError):
                run_shadow('missing.sdf', 'missing.json', 'missing-dir', duration=duration)

    def test_ros_import_failure_preserves_machine_readable_incomplete_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            policy_path = Path(temp) / 'policy.json'
            policy_path.write_text(json.dumps({'schema_version': 1, 'mode': 'SHADOW_ONLY',
                                               'policy': asdict(self.policy)}))
            output = Path(temp) / 'new-shadow'
            with patch.dict(sys.modules, {'rclpy': None}):
                with self.assertRaises(ModuleNotFoundError):
                    run_shadow(ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf',
                               policy_path, output)
            summary = json.loads((output / 'shadow_summary.json').read_text())
            self.assertEqual(summary['status'], 'SHADOW_INCOMPLETE')
            self.assertIn('ModuleNotFoundError', summary['error'])
            self.assertFalse(summary['mission_ready'])
            # The container must hand off both private files and their directory
            # to the bind parent's owner, so an unprivileged host can read them.
            parent = output.parent.stat()
            self.assertEqual(summary['evidence_access']['owner_uid'], parent.st_uid)
            self.assertEqual(summary['evidence_access']['owner_gid'], parent.st_gid)
            for path, mode in ((output, 0o700), (output / 'shadow_summary.json', 0o600),
                               (output / 'shadow_commands.jsonl', 0o600)):
                self.assertEqual((path.stat().st_uid, path.stat().st_gid), (parent.st_uid, parent.st_gid))
                self.assertEqual(path.stat().st_mode & 0o777, mode)

    def test_container_root_identity_is_handed_to_bind_parent_owner(self):
        # Simulate root-created descriptors without requiring root in the test
        # runner. A real daemon/bind mapping still needs the host regression.
        with patch('ackermann_adapter.os.fstat', return_value=SimpleNamespace(st_uid=0, st_gid=0)), \
             patch('ackermann_adapter.os.fchown') as chown:
            inherit_evidence_owner(42, (1001, 1002))
        chown.assert_called_once_with(42, 1001, 1002)

    def test_matching_mapped_identity_requires_no_chown(self):
        with patch('ackermann_adapter.os.fstat', return_value=SimpleNamespace(st_uid=0, st_gid=0)), \
             patch('ackermann_adapter.os.fchown') as chown:
            inherit_evidence_owner(42, (0, 0))
        chown.assert_not_called()

    def test_private_file_refuses_existing_file_and_symlink(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'original'
            path.write_text('evidence retained')
            link = Path(temp) / 'link'
            link.symlink_to(path)
            for existing in (path, link):
                with self.assertRaises(FileExistsError):
                    private_evidence_file(existing, (os.getuid(), os.getgid()))
            self.assertEqual(path.read_text(), 'evidence retained')

    def test_failed_ownership_transfer_is_not_silently_accepted(self):
        with patch('ackermann_adapter.os.fstat', return_value=SimpleNamespace(st_uid=0, st_gid=0)), \
             patch('ackermann_adapter.os.fchown', side_effect=PermissionError('handoff denied')):
            with self.assertRaises(PermissionError):
                inherit_evidence_owner(42, (1001, 1002))

    def test_existing_evidence_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            policy_path = Path(temp) / 'policy.json'
            policy_path.write_text(json.dumps({'schema_version': 1, 'mode': 'SHADOW_ONLY',
                                               'policy': asdict(self.policy)}))
            output = Path(temp) / 'old-shadow'
            output.mkdir()
            prior = output / 'shadow_summary.json'
            prior.write_text('preserved')
            with self.assertRaises(FileExistsError):
                run_shadow(ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf', policy_path, output)
            self.assertEqual(prior.read_text(), 'preserved')

    def test_helpers_preserve_nonfinite_and_unknown_gid(self):
        self.assertEqual(json_safe({'x': math.inf}), {'x': 'inf'})
        self.assertIsNone(gid_text(None))


if __name__ == '__main__':
    unittest.main()
