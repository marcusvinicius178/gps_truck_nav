"""Synthetic time, geometry and message fixtures; no runtime motion evidence."""

import json
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
from commissioning_observer import (ALL_JOINTS, Telemetry, TelemetryError, attach_subscriptions, closest,
    quaternion_yaw, rear_state, requested_pose, trajectory_errors)


class CommissioningObserverTests(unittest.TestCase):
    def setUp(self):
        self.raw, self.matches = [], []
        self.recorder = Telemetry('straight', 'a' * 32, self.raw.append, self.matches.append)

    def odom(self, stamp=10., x=3.502, yaw=0., vx=.3, vy=0., w=0.):
        self.recorder.observe_odom(stamp, 'odom', 'base_footprint', x, 0., yaw, vx, vy, w, .1)

    def joints(self, stamp=10.):
        self.recorder.observe_joints(list(ALL_JOINTS), [.02, .019, 0., 0., 0., 0.],
                                     [0., 0., .5, .5, .5, .5], stamp, .1)

    def test_rear_pose_and_lateral_twist_transform_are_rigid_body_consistent(self):
        state = rear_state(10, 20, math.pi / 2, .3, .03502, .01)
        self.assertAlmostEqual(state['x_m'], 10)
        self.assertAlmostEqual(state['y_m'], 20 - 3.502)
        self.assertAlmostEqual(state['lateral_speed_mps'], 0)
        self.assertEqual(state['longitudinal_speed_mps'], .3)
        self.assertAlmostEqual(state['curvature_inv_m'], .01 / .3)

    def test_curvature_undefined_near_zero_and_signed_in_reverse(self):
        for speed in (-.05, 0., .05):
            self.assertIsNone(rear_state(0, 0, 0, speed, 0, .01)['curvature_inv_m'])
        self.assertAlmostEqual(rear_state(0, 0, 0, -.3, 0, .01)['curvature_inv_m'], -1 / 30)
        with self.assertRaises(TelemetryError):
            rear_state(math.nan, 0, 0, 0, 0, 0)

    def test_invalid_quaternion_not_silently_normalized(self):
        self.assertAlmostEqual(quaternion_yaw((0., 0., math.sin(.3), math.cos(.3))), .6)
        for q in ((0., 0., 0., 0.), (0., 0., math.nan, 1.), (0., 0., 0., 2.)):
            with self.subTest(q=q), self.assertRaises(TelemetryError):
                quaternion_yaw(q)

    def test_matching_uses_simulation_stamp_not_receipt_time(self):
        self.recorder.observe_clock(10., .1)
        self.joints(10.03)
        self.recorder.observe_reference(9.98, 'rear_axle_nav', .3, .003, 500.)
        self.odom(w=.004)
        self.recorder.observe_clock(10.2, .2)
        record = self.matches[0]
        self.assertAlmostEqual(record['joint_dt_s'], .03)
        self.assertAlmostEqual(record['requested_dt_s'], -.02)
        self.assertAlmostEqual(record['yaw_rate_error_rps'], .001)
        self.assertEqual(record['reasons'], [])

    def test_missing_or_stale_joint_and_reference_are_explicit(self):
        self.joints(9.9)
        self.recorder.observe_reference(10.1, 'rear_axle_nav', .3, .003, .1)
        self.odom()
        self.recorder.flush(force=True)
        record = self.matches[0]
        self.assertIsNone(record['joints'])
        self.assertIsNone(record['requested_speed_mps'])
        self.assertIn('NO_JOINT_SAMPLE_WITHIN_50MS', record['reasons'])
        self.assertIn('NO_STAMPED_REQUEST_WITHIN_50MS', record['reasons'])

    def test_frozen_stamps_do_not_become_fresh_samples(self):
        self.joints()
        self.joints()
        self.odom()
        self.odom()
        self.assertEqual(self.recorder.counts['joints_nonadvancing_stamp'], 1)
        self.assertEqual(self.recorder.counts['odometry_nonadvancing_stamp'], 1)
        self.assertEqual(len(self.recorder.odom_history), 1)
        self.assertEqual(len(self.recorder.joints), 1)

    def test_joint_shape_missing_joint_and_nan_are_invalid(self):
        self.recorder.observe_joints(['front_left_steer_joint'], [0.], [0.], 10., .1)
        self.recorder.observe_joints(list(ALL_JOINTS), [0.] * 6, [math.nan] * 6, 11., .2)
        self.recorder.observe_joints(list(ALL_JOINTS) + [ALL_JOINTS[0]], [0.] * 7, [0.] * 7, 12., .3)
        self.assertEqual(self.recorder.counts['joints_invalid'], 3)
        self.assertEqual(len(self.recorder.joints), 0)
        for record in self.raw:
            json.dumps(record, allow_nan=False)

    def test_runtime_subscriptions_preserve_the_plugin_as_sole_actuator_receiver(self):
        class ReadOnlyNode:
            def __init__(self):
                self.callbacks = {}
            def create_subscription(self, _kind, topic, callback, _qos):
                self.callbacks[topic] = callback
                return topic
            def create_publisher(self, *_args):
                raise AssertionError('Observer must never create a publisher')

        node = ReadOnlyNode()
        handles = attach_subscriptions(node, self.recorder,
            dict.fromkeys(('Clock', 'Odometry', 'JointState', 'Twist', 'TwistStamped'), object),
            object(), monotonic=lambda: .1)
        self.assertEqual(set(handles), {'/clock', '/odom', '/joint_states',
                                       '/commissioning/reference', '/commissioning/cmd_vel'})
        self.assertNotIn('/actuation/steering_command', node.callbacks)
        vector = lambda x=0., y=0., z=0.: SimpleNamespace(x=x, y=y, z=z)
        node.callbacks['/commissioning/cmd_vel'](
            SimpleNamespace(linear=vector(x=.3), angular=vector(z=.003)), {})
        self.assertEqual(self.raw[-1]['topic'], '/commissioning/cmd_vel')
        self.assertIsNone(self.raw[-1]['source_sim_stamp'])

    def test_unstamped_twist_never_claims_source_simulation_time(self):
        self.recorder.observe_twist('/commissioning/cmd_vel', {'linear_x': .3}, .1)
        record = self.raw[-1]
        self.assertIsNone(record['source_sim_stamp'])
        self.assertEqual(record['time_scope'], 'MONOTONIC_RECEIPT_ONLY_NOT_SOURCE_SIMULATION_TIME')

    def test_frame_mismatch_is_not_reinterpreted(self):
        self.recorder.observe_odom(10., 'map', 'base_link', 0, 0, 0, 0, 0, 0, .1)
        self.recorder.observe_reference(10., 'base_link', .3, 0., .1)
        self.assertEqual(self.recorder.counts['odometry_invalid'], 1)
        self.assertEqual(self.recorder.counts['reference_invalid'], 1)
        self.assertFalse(self.recorder.pending)

    def test_reference_integrates_straight_and_symmetric_curves(self):
        x, y, yaw = requested_pose('straight', 17.)
        self.assertAlmostEqual(x, 3.3, delta=.002)
        self.assertEqual((y, yaw), (0., 0.))
        left, right = requested_pose('left', 17.), requested_pose('right', 17.)
        self.assertAlmostEqual(left[0], right[0])
        self.assertAlmostEqual(left[1], -right[1])
        self.assertAlmostEqual(left[2], -right[2])
        self.assertAlmostEqual(left[2], .021, places=6)
        self.assertGreater(left[1], 0.)
        with self.assertRaises(TelemetryError):
            requested_pose('left', 31.)

    def test_trajectory_is_anchored_at_start_without_map_accuracy_claim(self):
        self.odom(10.)
        self.recorder.set_trial_start(10.)
        self.joints(11.)
        self.recorder.observe_reference(11., 'rear_axle_nav', .1, 0., .2)
        self.odom(11., x=3.502 + requested_pose('straight', 1.)[0])
        self.recorder.flush(force=True)
        result = self.matches[-1]['trajectory']
        self.assertAlmostEqual(result['position_error_m'], 0.)
        self.assertEqual(result['anchor_stamp_s'], 10.)
        self.assertEqual(result['reference_scope'], 'REQUESTED_REAR_AXLE_PATH_IN_LOCAL_ODOM_ANCHORED_AT_TRIAL_START')
        self.assertFalse(self.recorder.summary(.2)['dynamic_validation_complete'])
        with self.assertRaises(TelemetryError):
            self.recorder.set_trial_start(10.1)

    def test_missing_anchor_blocks_path_metrics(self):
        self.odom(9.)
        self.recorder.set_trial_start(10.)
        self.odom(11.)
        self.recorder.flush(force=True)
        self.assertIsNone(self.matches[-1]['trajectory'])
        self.assertIn('NO_ODOMETRY_ANCHOR_WITHIN_50MS_OF_TRIAL_START', self.matches[-1]['reasons'])

    def test_heading_wrapping_and_cross_track_sign(self):
        anchor = {'x_m': 0., 'y_m': 0., 'yaw_rad': math.pi / 2}
        rear = {'x_m': -1., 'y_m': 0., 'yaw_rad': math.pi / 2}
        result = trajectory_errors(rear, anchor, 'straight', 0.)
        self.assertAlmostEqual(result['cross_track_error_m'], 1.)
        self.assertAlmostEqual(result['along_track_error_m'], 0.)

    def test_clock_and_wall_freshness_are_separate(self):
        self.recorder.observe_clock(10., .1)
        self.joints()
        self.odom()
        self.assertEqual(self.recorder.health(.2), [])
        self.recorder.observe_clock(10., 1.2)
        reasons = self.recorder.health(1.2)
        self.assertIn('CLOCK_STALLED', reasons)
        self.assertIn('ODOMETRY_STALE_WALL_RECEIPT', reasons)
        self.recorder.observe_clock(9., 1.3)
        self.assertIn('CLOCK_REGRESSED', self.recorder.health(1.3))

    def test_observation_without_test_is_not_exercised(self):
        self.joints()
        self.odom()
        self.recorder.flush(force=True)
        summary = self.recorder.summary(.2)
        self.assertEqual(summary['trajectory_status'], 'NOT_EXERCISED')
        self.assertFalse(summary['mission_ready'])
        self.assertFalse(summary['physical_stop_verified'])


if __name__ == '__main__':
    unittest.main()
