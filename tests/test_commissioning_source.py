"""Pure and fake-file fixtures only; no ROS context or commands are created."""

import contextlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
from commissioning_source import (SourceError, SourceTimeline, adapter_state, approval_record,
                                  atomic_json, command_at, lease_fresh, main,
                                  private_directory, protocol, execute)


class CommissioningSourceTests(unittest.TestCase):
    def advance(self, timeline, sim, now):
        previous_sim, previous_wall = timeline.sim, timeline.last_wall
        steps = max(1, math.ceil((sim - previous_sim) / .2))
        for i in range(1, steps + 1):
            timeline.observe_clock(previous_sim + (sim - previous_sim) * i / steps,
                                   previous_wall + (now - previous_wall) * i / steps)

    def test_prepare_never_imports_ros_or_creates_publisher(self):
        with patch.dict(sys.modules, {'rclpy': None}), contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(['--prepare', '--trial', 'straight']), 0)
        result = json.loads(stream.getvalue())
        self.assertEqual(result['status'], 'OFFLINE_PROTOCOL_PREPARED')
        self.assertEqual(result['command_publishers_created'], 0)
        self.assertFalse(result['motion_authorized'])
        self.assertFalse(result['mission_ready'])

    def test_execution_has_no_implicit_default_or_authorization(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                main(['--trial', 'straight'])
            with self.assertRaises(SystemExit):
                main(['--execute', '--trial', 'straight'])

    def test_straight_trapezoid_is_bounded_low_speed_and_no_reverse(self):
        for t in (0., .01, .1, 1., 2.9, 3., 7., 11., 12., 13.99, 14., 20., 30.):
            v, w = command_at('straight', t)
            self.assertGreaterEqual(v, 0.)
            self.assertLessEqual(v, .3)
            self.assertEqual(w, 0.)
        self.assertAlmostEqual(command_at('straight', 1)[0], .1)
        self.assertEqual(command_at('straight', 14), (0., 0.))
        self.assertEqual(protocol('straight')['nominal_command_distance_m'], 3.3)

    def test_curve_steering_ramps_only_at_settled_speed(self):
        for name, sign in (('left', 1), ('right', -1)):
            self.assertEqual(command_at(name, 2)[1], 0.)
            self.assertEqual(command_at(name, 3)[1], 0.)
            self.assertAlmostEqual(command_at(name, 3.5)[1], sign * .0015)
            self.assertAlmostEqual(command_at(name, 5)[1], sign * .003)
            self.assertAlmostEqual(command_at(name, 10.5)[1], sign * .0015)
            self.assertEqual(command_at(name, 11)[1], 0.)
            self.assertEqual(command_at(name, 13)[1], 0.)

    def test_invalid_trial_or_time_is_rejected(self):
        for trial, time in (('reverse', 1), ('straight', -1), ('left', math.nan), ('right', math.inf)):
            with self.subTest(trial=trial, time=time), self.assertRaises(SourceError):
                command_at(trial, time)

    def test_clock_missing_stalled_regressed_and_monotonic_failure(self):
        timeline = SourceTimeline('straight', 0.)
        self.assertEqual(timeline.tick(.1, False), (0., 0.))
        with self.assertRaises(SourceError):
            timeline.tick(1., False)
        timeline = SourceTimeline('straight', 0.)
        timeline.observe_clock(10., .1)
        with self.assertRaises(SourceError):
            timeline.observe_clock(9., .2)
        with self.assertRaises(SourceError):
            timeline.tick(1., False)
        with self.assertRaises(SourceError):
            timeline.observe_clock(11., -.1)

    def test_large_forward_clock_step_aborts_before_profile_jump(self):
        timeline = SourceTimeline('straight', 0.)
        timeline.observe_clock(10., .1)
        timeline.tick(.1, True)
        with self.assertRaisesRegex(SourceError, 'jumped forward'):
            timeline.observe_clock(13., .2)
        self.assertEqual(timeline.sim, 10.)

    def test_no_motion_until_armed_and_no_automatic_rearm(self):
        timeline = SourceTimeline('left', 0.)
        timeline.observe_clock(10., .1)
        self.assertEqual(timeline.tick(.1, False), (0., 0.))
        self.advance(timeline, 11., .2)
        self.assertEqual(timeline.tick(.2, True), (0., 0.))
        self.assertEqual(timeline.run_start_sim, 11.)
        self.advance(timeline, 12., .3)
        self.assertAlmostEqual(timeline.tick(.3, True)[0], .1)
        with self.assertRaises(SourceError):
            timeline.tick(.31, False)

    def test_completed_source_keeps_zero_and_does_not_claim_stop(self):
        timeline = SourceTimeline('right', 0.)
        timeline.observe_clock(10., .1)
        timeline.tick(.1, True)
        self.advance(timeline, 24., .2)
        self.assertEqual(timeline.tick(.2, True), (0., 0.))
        self.assertEqual(timeline.phase, 'STOPPING')
        self.advance(timeline, 27., .3)
        self.assertEqual(timeline.tick(.3, True), (0., 0.))
        self.assertEqual(timeline.phase, 'COMPLETE')
        self.assertFalse(timeline.plan['physical_stop_verified'])
        self.advance(timeline, 40., .4)
        with self.assertRaises(SourceError):
            timeline.tick(.4, True)

    def test_independent_wall_bound_remains_when_sim_slow(self):
        timeline = SourceTimeline('straight', 0.)
        timeline.observe_clock(.1, 44.9)
        with self.assertRaises(SourceError):
            timeline.tick(45., False)

    def test_explicit_approval_is_bound_to_trial_and_session(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'approval.json'
            good = {'schema_version': 1, 'run_id': 'a' * 32, 'trial': 'straight',
                    'scope': 'SIMULATION_COMMISSIONING', 'motion_authorized': True}
            path.write_text(json.dumps(good))
            self.assertEqual(approval_record(path, 'a' * 32, 'straight')['document'], good)
            for change in ({'motion_authorized': False}, {'run_id': 'b' * 32}, {'trial': 'left'},
                           {'scope': 'MISSION'}, {'schema_version': 2}):
                path.write_text(json.dumps({**good, **change}))
                with self.subTest(change=change), self.assertRaises(SourceError):
                    approval_record(path, 'a' * 32, 'straight')

    def test_supervisor_lease_is_fresh_plain_monotonic_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'lease'
            path.write_text('100.0\n')
            self.assertEqual(lease_fresh(path, 100.2), 100.)
            for now in (99., 100.75, math.nan):
                with self.subTest(now=now), self.assertRaises(SourceError):
                    lease_fresh(path, now)
            path.write_text('NaN')
            with self.assertRaises(SourceError):
                lease_fresh(path, 100.)

    def test_runtime_freshness_samples_time_after_atomic_writer_payload(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'lease'
            path.write_text('placeholder')
            order = []

            def read_lease(*_args, **_kwargs):
                order.append('read')
                return '10.001'

            def sampled_after_read():
                order.append('clock')
                return 10.002

            with patch.object(Path, 'read_text', side_effect=read_lease), \
                    patch('commissioning_source.time.monotonic', side_effect=sampled_after_read):
                self.assertEqual(lease_fresh(path), 10.001)
            self.assertEqual(order, ['read', 'clock'])
            # Explicit test clocks still reject genuine future timestamps.
            path.write_text('10.001')
            with self.assertRaises(SourceError):
                lease_fresh(path, now=10.)

            order.clear()

            def read_adapter(*_args, **_kwargs):
                order.append('read')
                return {'run_id': 'a' * 32, 'monotonic_s': 10.001,
                        'state': 'ARMED', 'fault': None}

            with patch('commissioning_source.read_small_json', side_effect=read_adapter), \
                    patch('commissioning_source.time.monotonic', side_effect=sampled_after_read):
                self.assertEqual(adapter_state(path, 'a' * 32), 'ARMED')
            self.assertEqual(order, ['read', 'clock'])
            with patch('commissioning_source.read_small_json', side_effect=read_adapter):
                with self.assertRaises(SourceError):
                    adapter_state(path, 'a' * 32, now=10.)

    def test_adapter_heartbeat_checks_identity_state_and_freshness(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'adapter.json'
            good = {'run_id': 'a' * 32, 'monotonic_s': 1., 'state': 'ARMED', 'fault': None}
            path.write_text(json.dumps(good))
            self.assertEqual(adapter_state(path, 'a' * 32, 1.1), 'ARMED')
            for change in ({'run_id': 'b' * 32}, {'state': 'FAULT'}, {'fault': 'stale'},
                           {'monotonic_s': 0.5}, {'monotonic_s': 2}):
                path.write_text(json.dumps({**good, **change}))
                with self.subTest(change=change), self.assertRaises(SourceError):
                    adapter_state(path, 'a' * 32, 1.1)

    def test_fake_ros_execution_waits_for_arm_and_keeps_zero_after_complete(self):
        # All message objects and ROS functions below are local fakes. This
        # tests CLI/runtime orchestration without loading ROS or a simulator.
        class Twist:
            def __init__(self):
                self.linear = SimpleNamespace(x=0., y=0., z=0.)
                self.angular = SimpleNamespace(x=0., y=0., z=0.)

        class TwistStamped:
            def __init__(self):
                self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
                self.twist = Twist()

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            args = SimpleNamespace(run_id='a' * 32, trial='left', evidence_dir=root / 'source',
                approval_file=root / 'approval.json', lease_file=root / 'lease',
                adapter_heartbeat=root / 'adapter.json')
            args.approval_file.write_text(json.dumps({'schema_version': 1, 'run_id': args.run_id,
                'trial': args.trial, 'scope': 'SIMULATION_COMMISSIONING', 'motion_authorized': True}))
            wall = [100.]
            args.lease_file.write_text(str(wall[0]))
            callbacks, sent, handlers = {}, [], {}
            runtime = {'ok': True, 'destroyed': False, 'stopped': False}

            class Node:
                def create_publisher(self, _kind, topic, _qos):
                    def publish(message):
                        if topic == '/commissioning/cmd_vel':
                            sent.append((wall[0], message.linear.x, message.angular.z))
                    return SimpleNamespace(publish=publish)
                def create_subscription(self, _kind, topic, callback, _qos):
                    callbacks[topic] = callback
                    return topic
                def destroy_node(self):
                    runtime['destroyed'] = True

            rclpy = ModuleType('rclpy')
            rclpy.init = lambda **kwargs: None
            rclpy.create_node = lambda name: Node()
            rclpy.ok = lambda: runtime['ok']
            rclpy.shutdown = lambda: runtime.update(ok=False)

            def spin(_node, timeout_sec):
                wall[0] += .05
                seconds = 10. + wall[0] - 100.
                ns = round(seconds * 1e9)
                sec, nanosec = divmod(ns, 1_000_000_000)
                callbacks['/clock'](SimpleNamespace(clock=SimpleNamespace(sec=sec, nanosec=nanosec)))
                args.lease_file.write_text(str(wall[0]))
                args.adapter_heartbeat.write_text(json.dumps({'run_id': args.run_id,
                    'monotonic_s': wall[0], 'state': 'ARMED' if wall[0] >= 101. else 'ARMING', 'fault': None}))
                heartbeat = args.evidence_dir / 'source_heartbeat.json'
                if heartbeat.exists() and json.loads(heartbeat.read_text())['state'] == 'COMPLETE':
                    handlers[15](15, None)
                    runtime['stopped'] = True

            rclpy.spin_once = spin
            qos = ModuleType('rclpy.qos')
            qos.QoSProfile = lambda **kwargs: kwargs
            qos.ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
            qos.DurabilityPolicy = SimpleNamespace(VOLATILE=1)
            qos.qos_profile_sensor_data = object()
            geometry = ModuleType('geometry_msgs.msg')
            geometry.Twist, geometry.TwistStamped = Twist, TwistStamped
            rosgraph = ModuleType('rosgraph_msgs.msg')
            rosgraph.Clock = object
            modules = {'rclpy': rclpy, 'rclpy.qos': qos,
                       'geometry_msgs': ModuleType('geometry_msgs'), 'geometry_msgs.msg': geometry,
                       'rosgraph_msgs': ModuleType('rosgraph_msgs'), 'rosgraph_msgs.msg': rosgraph}
            with patch.dict(sys.modules, modules), patch('commissioning_source.time.monotonic', side_effect=lambda: wall[0]), \
                    patch('commissioning_source.signal.signal', side_effect=lambda sig, fn: handlers.update({sig: fn})):
                self.assertEqual(execute(args), 0)
            self.assertTrue(runtime['destroyed'])
            self.assertTrue(runtime['stopped'])
            self.assertTrue(any(v > 0 for _, v, _ in sent))
            self.assertTrue(all(v == w == 0 for stamp, v, w in sent if stamp < 101.))
            self.assertEqual(sent[-1][1:], (0., 0.))
            summary = json.loads((args.evidence_dir / 'source_summary.json').read_text())
            self.assertEqual(summary['last_phase'], 'COMPLETE')
            self.assertFalse(summary['physical_stop_verified'])
            self.assertFalse(summary['dynamic_validation_complete'])
            commands = [json.loads(line) for line in (args.evidence_dir / 'source_commands.jsonl').read_text().splitlines()]
            self.assertTrue(all(item['requested_speed_mps'] == item['requested_yaw_rate_rps'] == 0
                                for item in commands if item['state'] in ('STOPPING', 'COMPLETE')))

    def test_private_new_evidence_preserves_old_and_refuses_symlink_temp(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'source'
            owner = private_directory(output)
            self.assertEqual(output.stat().st_mode & 0o777, 0o700)
            path = output / 'heartbeat.json'
            atomic_json(path, {'seq': 1}, owner)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual((path.stat().st_uid, path.stat().st_gid), owner)
            atomic_json(path, {'seq': 2}, owner)
            self.assertEqual(json.loads(path.read_text())['seq'], 2)
            with self.assertRaises(FileExistsError):
                private_directory(output)
            path.with_name(path.name + '.new').symlink_to(path)
            with self.assertRaises(FileExistsError):
                atomic_json(path, {'seq': 3}, owner)
            self.assertEqual(json.loads(path.read_text())['seq'], 2)


if __name__ == '__main__':
    unittest.main()
