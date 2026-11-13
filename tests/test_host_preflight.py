"""Read-only host diagnostic regressions; no Docker or ROS runtime is mocked as PASS."""

import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / 'tools/auto_tuning/host_preflight.py'
SPEC = importlib.util.spec_from_file_location('host_preflight', SCRIPT)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


class HostPreflightTests(unittest.TestCase):
    def test_missing_docker_is_reported_without_daemon_probe(self):
        with patch.object(preflight.shutil, 'which', return_value=None), \
                patch.object(preflight, 'run_command') as command:
            result = preflight.docker_evidence(True, {})
        self.assertEqual(result['daemon_status'], 'CLI_UNAVAILABLE')
        self.assertFalse(result['runtime_verified'])
        command.assert_not_called()

    def test_remote_environment_override_never_reaches_daemon(self):
        with patch.object(preflight.shutil, 'which', return_value='/bin/docker'), \
                patch.object(preflight, 'run_command', return_value={'returncode': 0}) as command:
            result = preflight.docker_evidence(True, {'DOCKER_HOST': 'ssh://robot-host'})
        self.assertEqual(result['daemon_status'], 'REMOTE_OR_INVALID_HOST_REFUSED')
        self.assertEqual([call.args[0][1:] for call in command.call_args_list],
                         [['--version'], ['compose', 'version']])

    def test_remote_context_is_refused_even_when_default_env_is_empty(self):
        responses = [{'returncode': 0}, {'returncode': 0},
                     {'returncode': 0, 'stdout': 'remote\n'},
                     {'returncode': 0, 'stdout': '"tcp://10.0.0.2:2375"', 'stderr': ''}]
        with patch.object(preflight.shutil, 'which', return_value='/bin/docker'), \
                patch.object(preflight, 'run_command', side_effect=responses) as command:
            result = preflight.docker_evidence(True, {})
        self.assertEqual(result['daemon_status'], 'REMOTE_OR_INVALID_CONTEXT_REFUSED')
        self.assertNotIn('info', [arg for call in command.call_args_list for arg in call.args[0]])

    def test_daemon_probe_requires_explicit_option(self):
        with patch.object(preflight.shutil, 'which', return_value='/bin/docker'), \
                patch.object(preflight, 'run_command', return_value={'returncode': 0}) as command:
            result = preflight.docker_evidence(False, {'DOCKER_HOST': 'unix:///var/run/docker.sock'})
        self.assertEqual(result['daemon_status'], 'NOT_PROBED')
        self.assertEqual(command.call_count, 2)

    def test_local_daemon_query_forces_socket_and_removes_ambient_overrides(self):
        env = {'DOCKER_HOST': 'unix:///run/user/1000/docker.sock',
               'DOCKER_TLS_VERIFY': '1', 'DOCKER_CERT_PATH': '/unused/certs',
               'PATH': '/usr/bin'}
        with patch.object(preflight.shutil, 'which', return_value='/bin/docker'), \
                patch.object(preflight, 'run_command', return_value={'returncode': 0}) as command, \
                patch.object(preflight.Path, 'stat', return_value=SimpleNamespace(st_mode=preflight.stat.S_IFSOCK)):
            result = preflight.docker_evidence(True, env)
        self.assertEqual(result['daemon_status'], 'LOCAL_DAEMON_RESPONDED')
        daemon_call = command.call_args_list[-1]
        self.assertEqual(daemon_call.args[0][:4],
                         ['/bin/docker', '--host', env['DOCKER_HOST'], 'info'])
        self.assertEqual(daemon_call.kwargs['env'], {'PATH': '/usr/bin'})
        self.assertFalse(result['runtime_verified'])

    def test_non_socket_file_cannot_be_used_as_docker_endpoint(self):
        with patch.object(preflight.shutil, 'which', return_value='/bin/docker'), \
                patch.object(preflight, 'run_command', return_value={'returncode': 0}) as command, \
                patch.object(preflight.Path, 'stat', return_value=SimpleNamespace(st_mode=preflight.stat.S_IFREG)):
            result = preflight.docker_evidence(True, {'DOCKER_HOST': 'unix:///tmp/file'})
        self.assertEqual(result['daemon_status'], 'LOCAL_SOCKET_UNAVAILABLE')
        self.assertEqual(command.call_count, 2)

    def test_invalid_and_remote_endpoints_are_rejected(self):
        for endpoint in (None, '', 'unix://relative', 'tcp://localhost:2375',
                         'ssh://host', 'unix:///tmp/sock?host=remote', 'unix:///tmp/sock\n'):
            with self.subTest(endpoint=endpoint):
                self.assertFalse(preflight.validate_local_endpoint(endpoint))
        self.assertTrue(preflight.validate_local_endpoint('unix:///run/user/1000/docker.sock'))

    def test_cgroup_limits_handle_unlimited_invalid_and_fractional_cpu(self):
        self.assertEqual(preflight.parse_cpu_max('150000 100000'), 1.5)
        for text in ('max 100000', '-1 100000', '100 0', 'invalid', None):
            self.assertIsNone(preflight.parse_cpu_max(text))
        self.assertEqual(preflight.parse_memory_limit('1073741824'), 1073741824)
        for text in ('max', '9223372036854771712', '0', None):
            self.assertIsNone(preflight.parse_memory_limit(text))

    def test_lfs_asset_count_uses_unique_payloads_and_does_not_follow_external_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'repo'
            models = root / 'src/gps_truck_nav/truck_bringup/models'
            models.mkdir(parents=True)
            pointer = 'version https://git-lfs.github.com/spec/v1\noid sha256:' + 'a' * 64 + '\nsize 4096\n'
            (models / 'a.dae').write_text(pointer)
            (models / 'b.dae').write_text(pointer)
            (models / 'real.sdf').write_text('<sdf/>')
            outside = Path(directory) / 'outside.dae'
            outside.write_text(pointer)
            (models / 'escape.dae').symlink_to(outside)
            with patch.object(preflight.shutil, 'which', return_value=None):
                result = preflight.repository_evidence(root)
        self.assertEqual(len(result['unmaterialized_lfs_assets']), 2)
        self.assertEqual(result['unique_missing_lfs_payload_bytes'], 4096)
        self.assertEqual(result['assets_scanned'], 3)
        self.assertEqual(len(result['asset_read_errors']), 1)

    def test_timeout_is_classified_without_shell(self):
        with patch.object(preflight.subprocess, 'run', side_effect=preflight.subprocess.TimeoutExpired(['probe'], 1)) as run:
            result = preflight.run_command(['probe'], timeout=1)
        self.assertTrue(result['timed_out'])
        self.assertIsNone(result['returncode'])
        self.assertNotIn('shell', run.call_args.kwargs)

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'existing.json'
            path.write_text('prior evidence')
            with patch.object(preflight, 'collect_report') as collect, self.assertRaises(SystemExit) as error:
                preflight.main(['--output', str(path)])
            self.assertEqual(error.exception.code, 2)
            self.assertEqual(path.read_text(), 'prior evidence')
            collect.assert_not_called()

    def test_successful_report_never_marks_gate_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'new.json'
            stub = {'gate0': 'NOT YET VERIFIED', 'gate1': 'NOT YET VERIFIED'}
            with patch.object(preflight, 'collect_report', return_value=stub):
                self.assertEqual(preflight.main(['--output', str(path)]), 0)
            self.assertEqual(json.loads(path.read_text()), stub)


if __name__ == '__main__':
    unittest.main()
