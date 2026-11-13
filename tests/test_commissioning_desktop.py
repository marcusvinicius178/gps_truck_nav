"""Offline desktop orchestration contracts; no display, Docker, ROS or movement.

The Docker fixture below models one owned simulator. Graphical authorization is
an in-memory fake, so these tests cannot create operational authorizations or
connect to a user's X server.
"""

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
import test_commissioning_host as fixtures

FakeDocker, IMAGE, SOURCE = fixtures.FakeDocker, fixtures.IMAGE, fixtures.SOURCE


class DesktopCliTests(unittest.TestCase):
    @staticmethod
    def prepare_arguments():
        return ['prepare', '--repo', '/synthetic/repo', '--build-dir', '/synthetic/build',
                '--probe-dir', '/synthetic/probe', '--output-dir', '/synthetic/output']

    def test_default_remains_xvfb(self):
        with patch.object(host, 'run', return_value=0) as run:
            self.assertEqual(host.main(self.prepare_arguments()), 0)
        self.assertEqual(run.call_args.args[0].gui_mode, 'xvfb')

    def test_desktop_is_an_explicit_prepare_option(self):
        with patch.object(host, 'run', return_value=0) as run:
            self.assertEqual(host.main(self.prepare_arguments() + ['--gui-mode', 'desktop']), 0)
        args = run.call_args.args[0]
        self.assertEqual((args.operation, args.gui_mode), ('prepare', 'desktop'))

    def test_unknown_gui_mode_is_rejected(self):
        with contextlib.redirect_stderr(io.StringIO()), patch.object(host, 'run') as run:
            with self.assertRaises(SystemExit):
                host.main(self.prepare_arguments() + ['--gui-mode', 'remote-web-input'])
        run.assert_not_called()

    def test_execute_cli_cannot_enable_desktop(self):
        arguments = self.prepare_arguments()
        arguments[0] = 'execute'
        arguments += ['--prepare-dir', '/synthetic/prepared', '--watchdog-proof-dir', '/synthetic/proof',
                      '--authorization', '/synthetic/not-created.json', '--gui-mode', 'desktop']
        with contextlib.redirect_stderr(io.StringIO()), patch.object(host, 'run') as run:
            with self.assertRaises(SystemExit):
                host.main(arguments)
        run.assert_not_called()


class FakeDesktop:
    """Generated X cookie is simulated; no actual authority data exists."""
    def __init__(self, evidence, run_id, duration, events, *, health_error=None, cleanup_result=None):
        self.events, self.health_error = events, health_error
        self.cleanup_result = ({'status': 'DESKTOP_ACCESS_REVOKED', 'authorization_revoked': True,
                                'credentials_removed': True} if cleanup_result is None else cleanup_result)
        self.metadata = {'display': ':0', 'machine': 'arocs-' + run_id[:12],
            'gui_mode': 'desktop', 'status': 'DESKTOP_AUTHENTICATED', 'run_id': run_id,
            'xdg_session_type': 'x11', 'wayland_display_present': False,
            'xauthority_source': '/synthetic/user/authority',
            'socket_path': '/tmp/.X11-unix/X0',
            'auth_cookie_path': '/synthetic/private/runtime/authority',
            'guardian_pid': 456, 'authorization_scope': 'SESSION_COOKIE_TRUSTED_X11',
            'glx_available': True}
        events.append('display_start')

    def docker_options(self):
        return ['--hostname', self.metadata['machine'],
                '--mount', 'type=bind,src=/tmp/.X11-unix/X0,dst=/tmp/.X11-unix/X0,readonly',
                '--mount', 'type=bind,src=/synthetic/private/runtime/authority,dst=/run/nav2-desktop.Xauthority,readonly',
                '--env', 'DISPLAY=:0', '--env', 'XAUTHORITY=/run/nav2-desktop.Xauthority']

    def check_alive(self):
        self.events.append('display_health')
        if self.health_error:
            raise RuntimeError(self.health_error)

    def close(self):
        self.events.append('display_close')
        return self.cleanup_result


class DesktopDocker(FakeDocker):
    def __init__(self, *args, events, **kwargs):
        super().__init__(*args, **kwargs)
        self.events = events

    def run(self, name, argv, **kwargs):
        self.events.append(name)
        result = super().run(name, argv, **kwargs)
        if name == 'commissioning_start':
            state = json.loads((self.output / 'manifest.json').read_text())
            machine = 'arocs-' + state['run_id'][:12]
            runtime = {'run_id': state['run_id'], 'gui_mode': 'desktop', 'status': 'OBSERVING',
                'commissioning_mode': 'prepare', 'gui_ready': True, 'updated_at_unix': time.time(),
                'machine': machine, 'desktop_machine': machine, 'window_machine': machine,
                'window_checked_monotonic_s': self.clock.now,
                'windows': {key: {'window': index, 'pid': index + 10, 'title': key,
                                  'mapped': True, 'machine': machine}
                            for index, key in enumerate(('gazebo', 'rviz'), 1)}}
            (self.output / 'runtime_status.json').write_text(json.dumps(runtime))
        return result


class DesktopHostTests(unittest.TestCase):
    setUp = fixtures.CommissioningHostTests.setUp
    args = fixtures.CommissioningHostTests.args

    def run_desktop(self, args, *, publisher=False, fault_component=None, auth_error=None,
                    health_error=None, cleanup_result=None):
        events, displays = [], []

        def runner(output):
            result = DesktopDocker(output, self.clock, publisher=publisher,
                                   fault_component=fault_component, events=events)
            self.runners.append(result)
            return result

        def display_start(evidence, run_id, duration):
            if auth_error:
                events.append('display_start_failed')
                raise ValueError(auth_error)
            result = FakeDesktop(evidence, run_id, duration, events, health_error=health_error,
                                 cleanup_result=cleanup_result)
            displays.append(result)
            return result

        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(host.phase, 'Runner', side_effect=runner))
            stack.enter_context(patch.object(host.phase, 'docker_prefix', return_value=['fake-docker']))
            stack.enter_context(patch.object(host.phase, 'source_snapshot', return_value=copy.deepcopy(SOURCE)))
            stack.enter_context(patch.object(host.phase, 'readiness_result', return_value=({}, '0' * 64)))
            stack.enter_context(patch.object(host.phase, 'image_inspection', return_value={'Id': IMAGE}))
            stack.enter_context(patch.object(host.time, 'monotonic', side_effect=self.clock.monotonic))
            stack.enter_context(patch.object(host.time, 'sleep', side_effect=self.clock.sleep))
            start = stack.enter_context(patch.object(host.desktop_display.DesktopSession, 'start',
                                                    side_effect=display_start))
            browser = stack.enter_context(patch.object(host, 'HTTPServer',
                side_effect=AssertionError('Desktop must not start HTTP')))
            pixels = stack.enter_context(patch.object(host.visual, 'read_frame',
                side_effect=AssertionError('Desktop must not depend on Xvfb pixels')))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            result = host.run(args)
        browser.assert_not_called()
        pixels.assert_not_called()
        report = json.loads((args.output_dir / 'commissioning_report.json').read_text())
        return result, report, events, displays, start

    def assert_containment_before_revoke(self, events):
        self.assertIn('kill_owned_simulator', events)
        self.assertLess(events.index('kill_owned_simulator'), events.index('display_close'))
        self.assertLess(events.index('stopped_identity'), events.index('display_close'))

    def test_desktop_prepare_uses_real_display_route_without_browser_or_motion(self):
        args = self.args(gui_mode='desktop')
        code, report, events, displays, start = self.run_desktop(args)
        self.assertEqual(code, 0, report)
        self.assertEqual(report['gui_mode'], 'desktop')
        self.assertEqual(report['status'], 'PREPARATION_COLLECTED')
        self.assertEqual(report['actuator_publishers_created'], 0)
        self.assertTrue(report['normal_telemetry_confirmed'])
        self.assertFalse(report['mission_ready'])
        start.assert_called_once()
        self.assert_containment_before_revoke(events)
        creates = [argv for runner in self.runners for name, argv, _ in runner.calls
                   if name == 'commissioning_create']
        self.assertEqual(len(creates), 1)
        command = creates[0]
        self.assertEqual(command[command.index('--network') + 1], 'none')
        self.assertEqual(command[command.index('--hostname') + 1], displays[0].metadata['machine'])
        self.assertIn('DISPLAY=:0', command)
        self.assertIn('XAUTHORITY=/run/nav2-desktop.Xauthority', command)
        for forbidden in ('--privileged', '--network=host', '/var/run/docker.sock', '--execute'):
            self.assertNotIn(forbidden, ' '.join(command))
        self.assertFalse((args.output_dir / 'framebuffer').exists())
        self.assertFalse((args.output_dir / 'source').exists())
        for name in ('authorization.json', 'authorization.template.json', 'authorization.used', 'operator_start.json'):
            self.assertFalse((args.output_dir / name).exists(), name)

    def test_denied_display_does_not_create_simulator(self):
        code, report, events, _, _ = self.run_desktop(self.args(gui_mode='desktop'),
            auth_error='DISPLAY_AUTHENTICATION_DENIED')
        self.assertEqual(code, 2)
        self.assertEqual(report['status'], 'FAILED')
        self.assertIn('DISPLAY_AUTHENTICATION_DENIED', report['error'])
        self.assertNotIn('commissioning_create', events)
        self.assertNotIn('commissioning_start', events)

    def test_display_guardian_failure_contains_before_cleanup(self):
        code, report, events, _, _ = self.run_desktop(self.args(gui_mode='desktop'),
            health_error='DISPLAY_GUARDIAN_EXITED')
        self.assertEqual(code, 2)
        self.assertIn('DISPLAY_GUARDIAN_EXITED', report['error'])
        self.assert_containment_before_revoke(events)

    def test_actuator_publisher_still_causes_containment(self):
        code, report, events, _, _ = self.run_desktop(self.args(gui_mode='desktop'), publisher=True)
        self.assertEqual(code, 2)
        self.assertIn('PREPARATION_CREATED_ACTUATOR_PUBLISHER', report['error'])
        self.assert_containment_before_revoke(events)

    def test_unconfirmed_display_revocation_fails_only_after_containment(self):
        successful = {'status': 'DESKTOP_ACCESS_REVOKED', 'authorization_revoked': True,
                      'credentials_removed': True}
        for number, bad in enumerate(({}, {**successful, 'status': 'UNKNOWN'},
                                      {**successful, 'authorization_revoked': False},
                                      {**successful, 'credentials_removed': False},
                                      {**successful, 'authorization_revoked': 1})):
            with self.subTest(cleanup=bad):
                args = self.args(gui_mode='desktop', output_dir=self.root / f'cleanup-{number}')
                code, report, events, _, _ = self.run_desktop(args, cleanup_result=bad)
                self.assertEqual(code, 2)
                self.assertEqual(report['status'], 'FAILED')
                self.assertIn('DESKTOP_AUTH_CLEANUP_FAILED', report['error'])
                self.assertEqual(report['containment']['status'], 'OWNED_SIMULATOR_TERMINATED')
                self.assertIs(report['containment']['running'], False)
                self.assertEqual(report['containment']['pid'], 0)
                self.assert_containment_before_revoke(events)
                self.assertEqual(events.count('commissioning_create'), 1)
                self.assertEqual(events.count('commissioning_start'), 1)
                self.assertEqual(events.count('kill_owned_simulator'), 1)
                self.assertEqual(events.count('display_close'), 1)

    def test_both_read_only_fault_drills_keep_containment_and_revoke_display(self):
        for drill in ('adapter', 'session'):
            with self.subTest(drill=drill):
                args = self.args(gui_mode='desktop', fault_drill=drill, output_dir=self.root / drill)
                code, report, events, _, _ = self.run_desktop(args,
                    fault_component='adapter' if drill == 'adapter' else None)
                self.assertEqual(code, 0, report)
                self.assertTrue(report['fault_drill_passed'])
                self.assertLess(report['fault_to_host_containment_wall_s'], 3.)
                self.assertEqual(report['actuator_publishers_created'], 0)
                self.assert_containment_before_revoke(events)

    def test_direct_execute_call_cannot_bypass_cli_desktop_restriction(self):
        args = self.args(operation='execute', gui_mode='desktop')
        with patch.object(host.desktop_display.DesktopSession, 'start') as start:
            with self.assertRaises(host.phase.PhaseError):
                host.run(args)
        start.assert_not_called()
        self.assertFalse(args.output_dir.exists())

    def test_default_xvfb_does_not_initialize_host_display(self):
        with patch.object(host.desktop_display.DesktopSession, 'start') as start:
            code, report = fixtures.CommissioningHostTests.run_host(self, self.args())
        self.assertEqual(code, 0, report)
        start.assert_not_called()


class DesktopSessionConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)

    def test_desktop_environment_preserves_ros_isolation_and_uses_dedicated_xauth(self):
        original = {'DISPLAY': ':5', 'XAUTHORITY': '/run/nav2-desktop.Xauthority',
            'WAYLAND_DISPLAY': 'wayland-0', 'ROS_DOMAIN_ID': '3', 'ROS_LOCALHOST_ONLY': '0',
            'GAZEBO_MASTER_URI': 'http://external:11345', 'QT_QPA_PLATFORM': 'wayland'}
        saved = dict(original)
        manifest = {'mode': 'prepare', 'gui_mode': 'desktop',
                    'desktop': {'display': ':5', 'machine': 'arocs-' + 'a' * 12}}
        result = session.gui_environment(manifest, original)
        self.assertEqual(original, saved)
        self.assertEqual(result['DISPLAY'], ':5')
        self.assertEqual(result['XAUTHORITY'], '/run/nav2-desktop.Xauthority')
        self.assertEqual(result['QT_QPA_PLATFORM'], 'xcb')
        self.assertEqual(result['LIBGL_ALWAYS_SOFTWARE'], '1')
        self.assertEqual(result['QT_X11_NO_MITSHM'], '1')
        self.assertEqual(result['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(result['ROS_DOMAIN_ID'], '71')
        self.assertEqual(result['GAZEBO_MASTER_URI'], 'http://127.0.0.1:11345')
        self.assertNotIn('WAYLAND_DISPLAY', result)

    def test_xvfb_environment_remains_private_default(self):
        result = session.gui_environment({'mode': 'prepare'}, {'DISPLAY': ':2', 'WAYLAND_DISPLAY': 'wayland-0'})
        self.assertEqual(result['DISPLAY'], ':91')
        self.assertEqual(result['ROS_LOCALHOST_ONLY'], '1')
        self.assertEqual(result['ROS_DOMAIN_ID'], '71')

    def test_desktop_does_not_accept_host_authority_path_or_remote_display(self):
        for display, auth in ((':5', '/home/user/.Xauthority'), ('host.example:0', '/run/nav2-desktop.Xauthority')):
            with self.subTest(display=display, auth=auth), self.assertRaises(ValueError):
                session.gui_environment({'mode': 'prepare', 'gui_mode': 'desktop',
                    'desktop': {'display': display, 'machine': 'arocs-' + 'a' * 12}},
                    {'DISPLAY': display, 'XAUTHORITY': auth})

    def test_container_also_rejects_desktop_execution(self):
        with self.assertRaises(ValueError):
            session.gui_environment({'mode': 'execute', 'gui_mode': 'desktop'}, {})

    def test_desktop_rviz_changes_only_camera_tool_and_preserves_source(self):
        import yaml
        source = ROOT / 'config/auto_tuning/observation.rviz'
        before = source.read_bytes()
        destination = self.directory / 'desktop.rviz'
        selected = session.prepare_rviz_config('desktop', source, destination)
        self.assertEqual(selected, destination)
        self.assertEqual(source.read_bytes(), before)
        original, shown = yaml.safe_load(before), yaml.safe_load(destination.read_text())
        tools = shown['Visualization Manager'].pop('Tools')
        original['Visualization Manager'].pop('Tools')
        self.assertEqual(shown, original)
        self.assertEqual([tool['Class'] for tool in tools], ['rviz_default_plugins/MoveCamera'])

    def test_xvfb_rviz_config_is_returned_without_copy_or_changes(self):
        source = ROOT / 'config/auto_tuning/observation.rviz'
        destination = self.directory / 'unused.rviz'
        self.assertEqual(session.prepare_rviz_config('xvfb', source, destination), source)
        self.assertFalse(destination.exists())

    def test_runtime_rviz_copy_never_overwrites_prior_evidence(self):
        source = ROOT / 'config/auto_tuning/observation.rviz'
        destination = self.directory / 'preserve.rviz'
        destination.write_text('previous evidence\n')
        with self.assertRaises(FileExistsError):
            session.prepare_rviz_config('desktop', source, destination)
        self.assertEqual(destination.read_text(), 'previous evidence\n')

    def test_unreviewed_rviz_command_tools_are_not_silently_preserved(self):
        import yaml
        record = yaml.safe_load((ROOT / 'config/auto_tuning/observation.rviz').read_text())
        record['Visualization Manager']['Tools'] = [{'Class': 'rviz_default_plugins/SetGoal'}]
        source, destination = self.directory / 'unreviewed.rviz', self.directory / 'desktop.rviz'
        source.write_text(yaml.safe_dump(record))
        with self.assertRaises(ValueError):
            session.prepare_rviz_config('desktop', source, destination)
        self.assertFalse(destination.exists())


class DesktopRuntimeEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root / 'runtime_status.json'
        self.state = {'run_id': 'a' * 32, 'desktop': {'machine': 'arocs-' + 'a' * 12}}
        self.record = {'run_id': self.state['run_id'], 'gui_mode': 'desktop', 'status': 'OBSERVING',
            'window_machine': self.state['desktop']['machine'], 'gui_ready': True,
            'updated_at_unix': 200., 'window_checked_monotonic_s': 100.,
            'windows': {'gazebo': {'window': 1, 'pid': 10}, 'rviz': {'window': 2, 'pid': 11}}}

    def read(self, **change):
        self.path.write_text(json.dumps({**self.record, **change}))
        with patch.object(host.time, 'time', return_value=200.1), \
                patch.object(host.time, 'monotonic', return_value=100.1):
            return host.read_desktop_runtime(self.root, self.state)

    def test_live_owned_windows_do_not_require_framebuffer(self):
        with patch.object(host.visual, 'read_frame', side_effect=AssertionError('No screenshot dependency')):
            result = self.read()
        self.assertEqual(result['window_machine'], self.state['desktop']['machine'])
        self.assertFalse((self.root / 'framebuffer').exists())

    def test_foreign_unready_or_incomplete_windows_are_rejected(self):
        for change in ({'run_id': 'b' * 32}, {'gui_mode': 'xvfb'}, {'window_machine': 'other'},
                       {'status': 'STOPPED'}, {'gui_ready': False}, {'gui_ready': 1},
                       {'windows': {'gazebo': {'window': 1}}}, {'windows': {}}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.read(**change)

    def test_stale_or_future_display_observations_are_rejected(self):
        for change in ({'updated_at_unix': 190.}, {'updated_at_unix': 201.},
                       {'window_checked_monotonic_s': 90.}, {'window_checked_monotonic_s': 101.}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.read(**change)

    def test_missing_or_corrupt_runtime_evidence_is_not_treated_as_live(self):
        with self.assertRaises(FileNotFoundError):
            host.read_desktop_runtime(self.root, self.state)
        self.path.write_text('{')
        with self.assertRaises(ValueError):
            host.read_desktop_runtime(self.root, self.state)


if __name__ == '__main__':
    unittest.main()
