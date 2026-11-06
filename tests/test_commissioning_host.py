"""Host orchestration fixtures only: no Docker, ROS, publishers or movement.

The fake Docker runner models one explicitly owned container. Tests execute the
actual host run/cleanup functions and inspect their recorded requests/results.
"""

import argparse
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import commissioning as host
import commissioning_session as session


CID = 'b' * 64
IMAGE = 'sha256:' + 'c' * 64
SOURCE = {'commit': 'd' * 40, 'tree': 'e' * 40, 'input_sha256': {'fixture': 'f' * 64}}


class Clock:
    def __init__(self):
        self.now = 100.0
        self.after_sleep = None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.after_sleep:
            callback, self.after_sleep = self.after_sleep, None
            callback()


class FakeDocker:
    def __init__(self, output, clock, *, publisher=False, fault_component=None):
        self.output, self.clock, self.publisher = output, clock, publisher
        self.fault_component = fault_component
        self.calls = []
        self.info = None

    def run(self, name, argv, **kwargs):
        self.calls.append((name, list(argv), kwargs))
        if name == 'commissioning_create':
            state = json.loads((self.output / 'manifest.json').read_text())
            self.info = {
                'Id': CID, 'Image': IMAGE,
                'Config': {'Labels': {'project': 'nav2-phase-a',
                    'nav2.phase_a.run_id': state['run_id'], 'nav2.phase_a.role': 'commissioning'}},
                'HostConfig': {'NetworkMode': 'none'}, 'State': {'Running': False, 'Pid': 0}}
            (self.output / 'container.id').write_text(CID + '\n')
        elif name == 'commissioning_start':
            self.info['State'] = {'Running': True, 'Pid': 123}
            state = json.loads((self.output / 'manifest.json').read_text())
            (self.output / 'guard_heartbeat.json').write_text(json.dumps({
                'run_id': state['run_id'], 'monotonic_s': self.clock.now, 'seq': 1,
                'ready': True, 'mode': state['mode'],
                'adapter': {'publisher_created': self.publisher, 'state': 'READY_READ_ONLY'},
                'pid': 10, 'adapter_pid': 11}))
            self.clock.after_sleep = lambda: (self.output / 'session_summary.json').write_text(json.dumps({
                'reason': 'DURATION_LIMIT_REACHED', 'mode': state['mode'], 'run_id': state['run_id']}))
        elif name == 'kill_owned_simulator':
            self.info['State'] = {'Running': False, 'Pid': 0}
        elif name == 'inject_read_only_process_stall':
            state = json.loads((self.output / 'manifest.json').read_text())
            if state['fault_drill'] == 'session':
                # The frozen guard emits no more heartbeat or summary.
                self.clock.after_sleep = None
            elif self.fault_component:
                self.clock.after_sleep = lambda: (self.output / 'guard_fault.json').write_text(json.dumps({
                    'run_id': state['run_id'], 'component': self.fault_component,
                    'reason': 'ValueError: HEARTBEAT_TIMESTAMP_OR_SEQUENCE_INVALID'}))
        elif 'inspect' in argv:
            return json.dumps([self.info])
        return ''


class CommissioningHostTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repo = self.root / 'repo'
        model = self.repo / host.MODEL
        model.parent.mkdir(parents=True)
        model.write_text('synthetic model identity, never loaded by ROS\n')
        self.build, self.probe = self.root / 'build', self.root / 'probe'
        self.build.mkdir()
        self.probe.mkdir()
        (self.build / 'status.json').write_text(json.dumps({
            'stage': 'build', 'status': 'IMAGE_VALIDATED', 'source': SOURCE, 'image_id': IMAGE}))
        (self.build / 'image_id.txt').write_text(IMAGE + '\n')
        (self.probe / 'status.json').write_text(json.dumps({
            'stage': 'probe', 'status': 'OBSERVATIONS_COLLECTED', 'source': SOURCE,
            'image_id': IMAGE, 'navigation_profile': 'candidate20kmh'}))
        self.clock = Clock()
        self.runners = []

    def args(self, **changes):
        values = dict(operation='prepare', repo=self.repo, build_dir=self.build,
                      probe_dir=self.probe, output_dir=self.root / 'attempt',
                      duration=10., port=8765, fault_drill='none')
        values.update(changes)
        return argparse.Namespace(**values)

    def run_host(self, args, *, publisher=False, fault_component=None):
        def runner(output):
            result = FakeDocker(output, self.clock, publisher=publisher, fault_component=fault_component)
            self.runners.append(result)
            return result

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(host.phase, 'Runner', side_effect=runner))
            stack.enter_context(patch.object(host.phase, 'docker_prefix', return_value=['fake-docker']))
            stack.enter_context(patch.object(host.phase, 'source_snapshot', return_value=copy.deepcopy(SOURCE)))
            stack.enter_context(patch.object(host.phase, 'readiness_result', return_value=({}, '0' * 64)))
            stack.enter_context(patch.object(host.phase, 'image_inspection', return_value={'Id': IMAGE}))
            stack.enter_context(patch.object(host.phase, 'inspect_container',
                                            side_effect=lambda runner, *_: copy.deepcopy(runner.info)))
            stack.enter_context(patch.object(host.time, 'monotonic', side_effect=self.clock.monotonic))
            stack.enter_context(patch.object(host.time, 'sleep', side_effect=self.clock.sleep))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            result = host.run(args)
        report = json.loads((args.output_dir / 'commissioning_report.json').read_text())
        return result, report

    def assert_no_container_created(self):
        self.assertFalse(any(name == 'commissioning_create' for runner in self.runners
                             for name, _, _ in runner.calls))

    def preparation(self, **changes):
        result = {'mode': 'prepare', 'status': 'PREPARATION_COLLECTED',
                  'read_only_ready_observed': True, 'source': SOURCE, 'image_id': IMAGE,
                  'actuator_publishers_created': 0,
                  'containment': {'status': 'OWNED_SIMULATOR_TERMINATED', 'running': False, 'pid': 0}}
        result.update(changes)
        return result

    def test_prepare_requires_no_authorization_and_cannot_start_motion(self):
        # Namespace deliberately lacks execute-only flags and authorization.
        args = self.args()
        code, report = self.run_host(args)
        self.assertEqual(code, 0)
        self.assertEqual(report['mode'], 'prepare')
        self.assertEqual(report['actuator_publishers_created'], 0)
        self.assertTrue(report['read_only_ready_observed'])
        self.assertFalse(report['mission_ready'])
        self.assertFalse(report['dynamic_validation_complete'])
        self.assertEqual(report['containment']['status'], 'OWNED_SIMULATOR_TERMINATED')
        self.assertFalse((args.output_dir / 'authorization.json').exists())
        self.assertFalse((args.output_dir / 'operator_start.json').exists())
        template = json.loads((args.output_dir / 'authorization.template.json').read_text())
        self.assertIs(template['motion_authorized'], False)
        self.assertIs(template['area_confirmed_clear'], False)
        commands = [argv for runner in self.runners for _, argv, _ in runner.calls]
        self.assertFalse(any('--execute' in command for command in commands))
        self.assertFalse(any('commissioning_source.py' in ' '.join(command) for command in commands))

    def test_prepare_detects_actuator_publisher_and_contains_attempt(self):
        code, report = self.run_host(self.args(), publisher=True)
        self.assertEqual(code, 2)
        self.assertEqual(report['status'], 'FAILED')
        self.assertIn('PREPARATION_CREATED_ACTUATOR_PUBLISHER', report['error'])
        self.assertEqual(report['containment']['status'], 'OWNED_SIMULATOR_TERMINATED')
        self.assertFalse(report['fault_drill_passed'])

    def test_mismatched_source_or_profile_blocks_before_container_creation(self):
        original = json.loads((self.probe / 'status.json').read_text())
        for number, change in enumerate(({'source': {'commit': 'other'}},
                                        {'navigation_profile': 'canonical'},
                                        {'image_id': 'sha256:' + '0' * 64})):
            with self.subTest(change=change):
                (self.probe / 'status.json').write_text(json.dumps({**original, **change}))
                code, report = self.run_host(self.args(output_dir=self.root / f'mismatch{number}'))
                self.assertEqual(code, 2)
                self.assertEqual(report['status'], 'FAILED')
                self.assert_no_container_created()

    def test_existing_evidence_is_never_reused_or_overwritten(self):
        output = self.root / 'old-evidence'
        output.mkdir()
        marker = output / 'marker'
        marker.write_text('preserve this evidence')
        with self.assertRaises(host.phase.PhaseError):
            self.run_host(self.args(output_dir=output))
        self.assertEqual(marker.read_text(), 'preserve this evidence')
        self.assert_no_container_created()

    def test_preparation_requires_zero_publishers_and_exact_build_identity(self):
        host.verify_preparation(self.preparation(), SOURCE, IMAGE)
        for change in ({'mode': 'execute'}, {'read_only_ready_observed': False},
                       {'actuator_publishers_created': 1}, {'actuator_publishers_created': None},
                       {'source': {}}, {'image_id': 'wrong'}, {'containment': {}},
                       {'status': 'FAILED'}, {'error': 'fault'}):
            with self.subTest(change=change), self.assertRaises(host.phase.PhaseError):
                host.verify_preparation(self.preparation(**change), SOURCE, IMAGE)

    def test_adapter_fault_drill_requires_adapter_heartbeat_failure_not_observer(self):
        for component, expected in (('adapter', True), ('observer', False)):
            with self.subTest(component=component):
                code, report = self.run_host(self.args(fault_drill='adapter',
                    output_dir=self.root / component), fault_component=component)
                self.assertEqual(report['fault_drill_passed'], expected)
                self.assertEqual(code, 0 if expected else 2)
                self.assertEqual(report['actuator_publishers_created'], 0)
                self.assertFalse(report['physical_stop_verified'])
                self.assertEqual(report['containment']['status'], 'OWNED_SIMULATOR_TERMINATED')

    def test_session_fault_drill_expires_stalled_guard_and_contains_simulator(self):
        code, report = self.run_host(self.args(fault_drill='session'))
        self.assertEqual(code, 0)
        self.assertTrue(report['fault_drill_passed'])
        self.assertIn('SESSION_GUARD_HEARTBEAT_FAILED', report['reason'])
        self.assertLess(report['fault_to_host_containment_wall_s'], 3.)
        self.assertEqual(report['actuator_publishers_created'], 0)

    def test_natural_end_is_not_a_successful_watchdog_proof(self):
        code, report = self.run_host(self.args(fault_drill='adapter'))
        self.assertTrue(report['fault_drill_triggered'])
        self.assertFalse(report['fault_drill_passed'])

    def test_execute_requires_both_distinct_fault_drill_types_before_authorization(self):
        prepare, proof = self.root / 'prepared', self.root / 'proof'
        prepare.mkdir()
        proof.mkdir()
        (prepare / 'manifest.json').write_text('{}')
        record = self.preparation(fault_drill='adapter', fault_drill_passed=True)
        (prepare / 'commissioning_report.json').write_text(json.dumps(record))
        (proof / 'commissioning_report.json').write_text(json.dumps(record))
        args = self.args(operation='execute', prepare_dir=prepare, watchdog_proof_dir=proof,
                         authorization=self.root / 'does-not-exist.json')
        code, report = self.run_host(args)
        self.assertEqual(code, 2)
        self.assertIn('Both adapter-loss and session-loss', report['error'])
        self.assert_no_container_created()
        self.assertFalse((prepare / 'authorization.used').exists())

    def test_second_fault_proof_must_match_the_exact_source(self):
        prepare, proof = self.root / 'prepared', self.root / 'proof'
        prepare.mkdir()
        proof.mkdir()
        (prepare / 'manifest.json').write_text('{}')
        (prepare / 'commissioning_report.json').write_text(json.dumps(
            self.preparation(fault_drill='adapter', fault_drill_passed=True)))
        (proof / 'commissioning_report.json').write_text(json.dumps(
            self.preparation(fault_drill='session', fault_drill_passed=True, source={'commit': 'wrong'})))
        args = self.args(operation='execute', prepare_dir=prepare, watchdog_proof_dir=proof,
                         authorization=self.root / 'not-read.json')
        code, report = self.run_host(args)
        self.assertEqual(code, 2)
        self.assertIn('Preparation must demonstrate', report['error'])
        self.assert_no_container_created()

    def test_execution_container_name_does_not_collide_with_preserved_preparation(self):
        prepare, proof = self.root / 'prepared', self.root / 'proof'
        prepare.mkdir()
        proof.mkdir()
        run_id = 'a' * 32
        protocol_path = self.root / 'protocol-fixture.json'
        host.safety.atomic_private(protocol_path, host.protocol_record())
        prepared = {'run_id': run_id, 'model_sha256': host.safety.digest(self.repo / host.MODEL),
                    'protocol_sha256': host.safety.digest(protocol_path)}
        (prepare / 'manifest.json').write_text(json.dumps(prepared))
        (prepare / 'commissioning_report.json').write_text(json.dumps(
            self.preparation(**prepared, fault_drill='adapter', fault_drill_passed=True)))
        (proof / 'commissioning_report.json').write_text(json.dumps(
            self.preparation(**{**prepared, 'run_id': '9' * 32}, fault_drill='session', fault_drill_passed=True)))
        authorization = self.root / 'synthetic-authorization.json'
        authorization.write_text(json.dumps({**prepared,
            'schema_version': 1, 'scope': 'SIMULATION_COMMISSIONING', 'motion_authorized': True,
            'area_confirmed_clear': True, 'source_commit': SOURCE['commit'], 'image_id': IMAGE,
            'navigation_profile': 'candidate20kmh', 'trial': 'straight',
            'expires_at_unix': time.time() + 60, 'operator': 'OFFLINE TEST FIXTURE',
            'authorization_reference': 'Synthetic fixture; no actual runtime authorization'}))
        args = self.args(operation='execute', prepare_dir=prepare, watchdog_proof_dir=proof,
                         authorization=authorization)
        _, report = self.run_host(args)
        creates = [argv for runner in self.runners for name, argv, _ in runner.calls
                   if name == 'commissioning_create']
        self.assertEqual(len(creates), 1, report)
        execution_name = creates[0][creates[0].index('--name') + 1]
        preparation_options = host.phase.container_options(self.root / 'unused', run_id, 'commissioning-prepare')
        preparation_name = preparation_options[preparation_options.index('--name') + 1]
        self.assertNotEqual(execution_name, preparation_name)
        self.assertIn('commissioning-execute', execution_name)
        self.assertTrue((prepare / 'authorization.used').is_file())
        self.assertTrue((prepare / 'commissioning_report.json').is_file())
        args.output_dir = self.root / 'execution-retry'
        second_code, second_report = self.run_host(args)
        self.assertEqual(second_code, 2)
        self.assertIn('FileExistsError', second_report['error'])
        self.assertEqual(sum(name == 'commissioning_create' for runner in self.runners
                             for name, _, _ in runner.calls), 1)

    def test_foreign_container_is_never_signalled(self):
        path = self.root / 'stop'
        path.mkdir()
        state = {'container_id': CID, 'image_id': IMAGE, 'run_id': 'a' * 32}
        runner = FakeDocker(path, self.clock)
        runner.info = {'Id': CID, 'Image': IMAGE,
                       'Config': {'Labels': {'project': 'nav2-phase-a',
                           'nav2.phase_a.run_id': '9' * 32, 'nav2.phase_a.role': 'commissioning'}},
                       'HostConfig': {'NetworkMode': 'none'}, 'State': {'Running': True, 'Pid': 123}}
        with patch.object(host.phase, 'inspect_container', side_effect=lambda *_: copy.deepcopy(runner.info)):
            with self.assertRaises(ValueError):
                host.stop_owned(path, state, runner=runner, docker=['fake-docker'])
        self.assertFalse(any(name == 'kill_owned_simulator' for name, _, _ in runner.calls))
        self.assertFalse((path / 'containment.json').exists())

    def test_container_stop_is_not_reported_if_still_running(self):
        path = self.root / 'stop'
        path.mkdir()
        state = {'container_id': CID, 'image_id': IMAGE, 'run_id': 'a' * 32}
        info = {'Id': CID, 'Image': IMAGE,
                'Config': {'Labels': {'project': 'nav2-phase-a', 'nav2.phase_a.run_id': state['run_id'],
                    'nav2.phase_a.role': 'commissioning'}},
                'HostConfig': {'NetworkMode': 'none'}, 'State': {'Running': True, 'Pid': 123}}
        runner = FakeDocker(path, self.clock)
        runner.info = copy.deepcopy(info)
        with patch.object(host, 'inspect_bounded', return_value=info):
            with self.assertRaises(host.phase.PhaseError):
                host.stop_owned(path, state, runner=runner, docker=['fake-docker'])
        self.assertFalse((path / 'containment.json').exists())

    def test_cancellation_is_visible_before_a_blocked_docker_rpc(self):
        path = self.root / 'stop-before-rpc'
        path.mkdir()
        state = {'container_id': CID, 'image_id': IMAGE, 'run_id': 'a' * 32}
        runner = FakeDocker(path, self.clock)
        def blocked(*_):
            request = json.loads((path / 'cancel.request.json').read_text())
            self.assertEqual(request['run_id'], state['run_id'])
            raise TimeoutError('Synthetic Docker RPC timeout')
        with patch.object(host, 'inspect_bounded', side_effect=blocked):
            with self.assertRaises(TimeoutError):
                host.stop_owned(path, state, runner=runner, docker=['fake-docker'])
        self.assertFalse((path / 'containment.json').exists())

    def test_adapter_arguments_derive_unchanged_geometry_from_candidate(self):
        model = ROOT / host.MODEL
        manifest = {'mode': 'prepare', 'run_id': '1' * 32, 'duration_wall_s': 120.,
                    'model_sha256': host.safety.digest(model)}
        command, parameters = session.adapter_arguments(manifest, model, evidence=self.root)
        self.assertEqual(command[0], '/opt/auto_tuning/bin/commissioning_adapter')
        self.assertEqual(parameters['mode'], 'prepare')
        self.assertAlmostEqual(parameters['wheelbase_m'], 6.816500073351341)
        self.assertAlmostEqual(parameters['rear_right_y_m'], -1.299)
        self.assertEqual(parameters['plugin_max_speed_mps'], 20.)
        self.assertEqual(parameters['left_upper_rad'], .524)
        self.assertEqual(parameters['right_lower_rad'], -.524)
        self.assertEqual(parameters['base_to_rear_x_m'], -3.502)
        self.assertIn('run_id:="' + '1' * 32 + '"', command)
        with self.assertRaises(ValueError):
            session.adapter_arguments({**manifest, 'model_sha256': '0' * 64}, model, evidence=self.root)


if __name__ == '__main__':
    unittest.main()
