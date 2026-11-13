"""Actual observer/finalizer with fake ROS/processes; never starts a simulator."""

import argparse
import contextlib
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import commissioning_observer as observer
import commissioning_session as session
import commissioning_supervision as safety


def summary_fixture(run_id, *, dynamic=False):
    record = {'schema_version': 1, 'run_id': run_id, 'trial': 'straight',
        'status': 'TELEMETRY_COLLECTED', 'error': None, 'health_reasons': [],
        'counts': dict.fromkeys(('clock_received', 'odometry', 'joint_states',
                                'synchronized_records', 'joint_matched'), 10)}
    if dynamic:
        record.update(trial_start_sim_s=10., anchor_stamp_s=10.,
                      trajectory_status='MEASURED_RELATIVE_TO_REQUEST',
                      maxima=dict.fromkeys(('max_abs_cross_track_error_m', 'max_abs_position_error_m',
                                            'max_abs_heading_error_rad'), 0.))
        record['counts'].update(trajectory_metrics=10, trial_synchronized_records=10,
                               trial_joint_request_trajectory_matched=10)
    return record


class CompletionContractTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root / 'observer').mkdir()
        self.path = self.root / 'observer/telemetry_summary.json'
        self.manifest = {'run_id': 'a' * 32, 'trial': 'straight', 'mode': 'prepare', 'fault_drill': 'none'}
        self.record = summary_fixture(self.manifest['run_id'])
        self.write()

    def write(self, record=None):
        self.path.write_text(json.dumps(self.record if record is None else record))

    def confirmation(self, **changes):
        return safety.telemetry_confirmation(self.root, self.manifest,
            changes.get('returncode', 0), changes.get('timed_out', False))

    def test_normal_prepare_and_dynamic_summary_are_verified(self):
        confirmation = self.confirmation()
        self.assertEqual(confirmation['status'], 'VERIFIED')
        safety.verify_telemetry_confirmation({'telemetry_finalization': confirmation}, self.manifest, self.root)
        self.manifest['mode'] = 'execute'
        self.write(summary_fixture(self.manifest['run_id'], dynamic=True))
        self.confirmation()

    def test_nonzero_missing_boolean_exit_and_timeout_fail(self):
        for changes in ({'returncode': 3}, {'returncode': -2}, {'returncode': None},
                        {'returncode': False}, {'timed_out': True}, {'timed_out': None}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.confirmation(**changes)

    def test_missing_corrupt_nonfinite_or_nonobject_summary_fails(self):
        self.path.unlink()
        with self.assertRaises(ValueError):
            self.confirmation()
        for text in ('{', 'null', '[]', '{"counts": NaN}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.path.write_text(text)
                self.confirmation()

    def test_wrong_identity_status_error_and_counts_fail(self):
        for change in ({'run_id': 'b' * 32}, {'trial': 'left'}, {'status': 'TELEMETRY_INCOMPLETE'},
                       {'error': 'ExternalShutdownException'}, {'counts': {}},
                       {'counts': {**self.record['counts'], 'joints_invalid': 1}},
                       {'health_reasons': ['ODOMETRY_INVALID']}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.write({**self.record, **change})
                self.confirmation()
        record = copy.deepcopy(self.record)
        record.pop('error')
        self.write(record)
        with self.assertRaises(ValueError):
            self.confirmation()

    def test_final_receipt_age_after_plant_kill_is_not_a_sample_error(self):
        self.write({**self.record, 'health_reasons': ['ODOMETRY_STALE_WALL_RECEIPT', 'CLOCK_STALLED']})
        self.confirmation()

    def test_dynamic_metrics_must_belong_to_the_trial_not_warmup(self):
        self.manifest['mode'] = 'execute'
        dynamic = summary_fixture(self.manifest['run_id'], dynamic=True)
        for key in ('trajectory_metrics', 'trial_synchronized_records', 'trial_joint_request_trajectory_matched'):
            bad = copy.deepcopy(dynamic)
            bad['counts'][key] = 0
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.write(bad)
                self.confirmation()
        for change in ({'trial_start_sim_s': None}, {'anchor_stamp_s': 11.},
                       {'maxima': {}}, {'trajectory_status': 'NOT_EXERCISED'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.write({**dynamic, **change})
                self.confirmation()

    def test_host_rechecks_summary_hash_and_confirmation_identity(self):
        confirmation = self.confirmation()
        self.write({**self.record, 'extra': 'changed after session confirmation'})
        with self.assertRaises(ValueError):
            safety.verify_telemetry_confirmation({'telemetry_finalization': confirmation}, self.manifest, self.root)
        with self.assertRaises(ValueError):
            safety.verify_telemetry_confirmation({}, self.manifest, self.root)

    def test_only_normal_completions_require_finalization(self):
        self.assertTrue(safety.normal_telemetry_required(self.manifest, 'DURATION_LIMIT_REACHED'))
        for drill in ('adapter', 'session'):
            self.assertFalse(safety.normal_telemetry_required({**self.manifest, 'fault_drill': drill},
                                                           'DURATION_LIMIT_REACHED'))
        self.assertFalse(safety.normal_telemetry_required(self.manifest, 'OPERATOR_OR_HOST_STOP'))
        self.assertTrue(safety.normal_telemetry_required({**self.manifest, 'mode': 'execute'},
            'TRIAL_COMMANDS_COMPLETE_STANDSTILL_OBSERVED'))


class ObserverRunTests(unittest.TestCase):
    def run_observer(self, scenario, *, requested_signal=signal.SIGINT):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / 'observer'
        callbacks = {}
        node = NS(create_subscription=lambda _type, topic, callback, _qos:
                  callbacks.setdefault(topic, callback), destroy_node=lambda: None)
        modules = {}
        for package, names in (('geometry_msgs', ('Twist', 'TwistStamped')), ('nav_msgs', ('Odometry',)),
                               ('sensor_msgs', ('JointState',)), ('rosgraph_msgs', ('Clock',))):
            parent, msg = ModuleType(package), ModuleType(package + '.msg')
            for name in names:
                setattr(msg, name, object)
            parent.msg = msg
            modules[package], modules[package + '.msg'] = parent, msg
        ros, qos, signals = ModuleType('rclpy'), ModuleType('rclpy.qos'), ModuleType('rclpy.signals')
        signals.SignalHandlerOptions = NS(NO=object())
        qos.qos_profile_sensor_data = object()
        live = [True]
        def init(**kwargs):
            self.assertIs(kwargs.get('signal_handler_options'), signals.SignalHandlerOptions.NO)
        ros.init, ros.ok, ros.shutdown = init, lambda: live[0], lambda: live.__setitem__(0, False)
        ros.create_node = lambda _name: node
        def spin(*_args, **_kwargs):
            if scenario == 'exception':
                class ExternalShutdownException(Exception):
                    pass
                raise ExternalShutdownException()
            if scenario == 'callback_error':
                raise RuntimeError('Synthetic callback failure')
            stamp = NS(sec=10, nanosec=0)
            callbacks['/clock'](NS(clock=stamp))
            callbacks['/joint_states'](NS(header=NS(stamp=stamp), name=list(observer.ALL_JOINTS),
                position=[0.] * 6, velocity=[float('nan') if scenario == 'invalid_data' else 0.] * 6))
            callbacks['/odom'](NS(header=NS(stamp=stamp, frame_id='odom'), child_frame_id='base_footprint',
                pose=NS(pose=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
                twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))))
            if scenario == 'context_shutdown':
                live[0] = False
            else:
                signal.getsignal(requested_signal)(requested_signal, None)
        ros.spin_once = spin
        modules.update({'rclpy': ros, 'rclpy.qos': qos, 'rclpy.signals': signals})
        old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            with patch.dict(sys.modules, modules):
                return observer.run(argparse.Namespace(evidence_dir=self.root, trial='straight',
                    run_id='a' * 32, duration=10., source_heartbeat=None))
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)

    def test_requested_sigint_and_sigterm_flush_jsonl_and_complete_summary(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(sig=sig):
                self.assertEqual(self.run_observer('normal', requested_signal=sig), 0)
                record = json.loads((self.root / 'telemetry_summary.json').read_text())
                safety.validate_telemetry_summary(record, {'run_id': 'a' * 32, 'trial': 'straight', 'mode': 'prepare'})
                self.assertTrue((self.root / 'telemetry_raw.jsonl').read_text())
                self.assertTrue((self.root / 'telemetry_synchronized.jsonl').read_text())

    def test_unexpected_ros_shutdown_exception_and_callback_failure_remain_failures(self):
        for scenario in ('context_shutdown', 'exception', 'callback_error', 'invalid_data'):
            with self.subTest(scenario=scenario):
                self.assertEqual(self.run_observer(scenario), 3)
                record = json.loads((self.root / 'telemetry_summary.json').read_text())
                self.assertEqual(record['status'], 'TELEMETRY_INCOMPLETE')
                self.assertTrue(record['error'])

    def test_raw_write_error_is_not_reclassified_as_success(self):
        original = observer.private_evidence_file
        @contextlib.contextmanager
        def file_with_write_failure(path, owner):
            with original(path, owner) as stream:
                if path.name == 'telemetry_raw.jsonl':
                    def fail(_value):
                        raise OSError('Synthetic full disk')
                    yield NS(write=fail, flush=stream.flush)
                else:
                    yield stream
        with patch.object(observer, 'private_evidence_file', file_with_write_failure):
            self.assertEqual(self.run_observer('normal'), 3)
        record = json.loads((self.root / 'telemetry_summary.json').read_text())
        self.assertIn('OSError', record['error'])

    def test_summary_write_failure_propagates_nonzero_process_failure(self):
        original = observer.private_evidence_file
        def file_with_write_failure(path, owner):
            if path.name == 'telemetry_summary.json':
                raise OSError('Synthetic summary write failure')
            return original(path, owner)
        with patch.object(observer, 'private_evidence_file', file_with_write_failure):
            with self.assertRaises(OSError):
                self.run_observer('normal')


class ProcessFinalizerTests(unittest.TestCase):
    def test_early_session_startup_failure_still_runs_finalizer(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'manifest.json').write_text(json.dumps({'mode': 'prepare',
                'navigation_profile': 'candidate20kmh', 'duration_wall_s': 10.,
                'run_id': 'a' * 32, 'trial': 'straight', 'fault_drill': 'none'}))
            handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                with patch.object(session, 'EVIDENCE', root), patch.object(sys, 'argv', ['session']), \
                        patch.dict(os.environ), patch.dict(sys.modules, {'ament_index_python.packages': None}):
                    self.assertEqual(session.main(), 2)
            finally:
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
            record = json.loads((root / 'session_summary.json').read_text())
            self.assertIn('ModuleNotFoundError', record['reason'])
            self.assertEqual(record['telemetry_finalization']['status'], 'NOT_REQUIRED')

    def exercise(self, *, hangs=False, exit_code=0):
        with tempfile.TemporaryDirectory() as tmp:
            clock, events, beats = [100.], [], []
            class Process:
                def __init__(self, pid):
                    self.pid, self.code = pid, None
                def poll(self):
                    if self.pid == 2 and not hangs and clock[0] >= 100.1:
                        self.code = exit_code
                    return self.code
                def wait(self, timeout):
                    if self.poll() is None:
                        raise subprocess.TimeoutExpired('fake-process', timeout)
                    return self.code
            processes = {'bringup': Process(1), 'observer': Process(2), 'adapter': Process(3)}
            def kill(pid, sig):
                events.append((pid, sig, clock[0]))
                if sig == signal.SIGKILL:
                    next(proc for proc in processes.values() if proc.pid == pid).code = -9
            def sleep(seconds):
                clock[0] += seconds
            with patch.object(session.os, 'killpg', side_effect=kill), \
                    patch.object(session.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(session.time, 'sleep', side_effect=sleep), \
                    patch.object(session.safety, 'atomic_private', side_effect=lambda _p, value: beats.append(value)):
                sequence, result = session.finalize_processes(processes,
                    {'run_id': 'a' * 32, 'mode': 'prepare'}, 5, {}, evidence=Path(tmp))
            self.assertEqual(events[0][:2], (1, signal.SIGKILL))
            self.assertEqual(events[1][:2], (3, signal.SIGKILL))
            self.assertEqual(events[2][:2], (2, signal.SIGINT))
            self.assertTrue(beats)
            self.assertTrue(all(beat['state'] == 'FINALIZING' for beat in beats))
            self.assertGreater(sequence, 5)
            self.assertLessEqual(clock[0] - 100., 2.051)
            return result, events

    def test_plant_kill_precedes_observer_flush_and_heartbeats_continue(self):
        result, _ = self.exercise()
        self.assertEqual(result, {'observer_returncode': 0, 'timed_out': False})

    def test_nonzero_observer_exit_is_retained(self):
        result, _ = self.exercise(exit_code=3)
        self.assertEqual(result, {'observer_returncode': 3, 'timed_out': False})

    def test_stuck_observer_is_killed_at_bounded_deadline(self):
        result, events = self.exercise(hangs=True)
        self.assertTrue(result['timed_out'])
        self.assertEqual(events[-1][:2], (2, signal.SIGKILL))


if __name__ == '__main__':
    unittest.main()
