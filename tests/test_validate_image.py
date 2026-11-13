"""Offline regressions for the real validator prelude; no Docker/ROS is launched."""

import ast
from pathlib import Path
import subprocess
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / 'tools/auto_tuning/validate_image.sh'
SOURCE = SCRIPT.read_text().split("<<'PY'\n", 1)[1].rsplit('\nPY', 1)[0]
BANNER = 'Gazebo multi-robot simulator, version 11.10.2\n'


def load_prelude():
    # Execute the actual embedded definitions, stopping before the image checks.
    # AST selection avoids importing ROS or accidentally running the main body.
    tree = ast.parse(SOURCE, filename=str(SCRIPT))
    prelude = []
    for node in tree.body:
        if isinstance(node, ast.With):
            break
        prelude.append(node)
    namespace = {}
    exec(compile(ast.Module(body=prelude, type_ignores=[]), str(SCRIPT), 'exec'), namespace)
    return namespace


class GazeboVersionTests(unittest.TestCase):
    def setUp(self):
        self.validator = load_prelude()

    def query(self, returncode=255, stdout=BANNER, stderr=''):
        result = subprocess.CompletedProcess(['gzserver', '--version'], returncode, stdout, stderr)
        with mock.patch.object(subprocess, 'run', return_value=result) as run:
            record = self.validator['gazebo_version_check']()
        run.assert_called_once_with(['gzserver', '--version'], capture_output=True,
                                    text=True, timeout=20, check=False)
        return record

    def all_passed(self):
        return all(check['passed'] for check in self.validator['checks'])

    def test_valid_banner_exit_255_is_identification_only_with_evidence_intact(self):
        record = self.query()
        self.assertTrue(self.all_passed())
        self.assertEqual(record['returncode'], 255)
        self.assertEqual(record['stdout'], BANNER)
        self.assertEqual(record['stderr'], '')
        self.assertFalse(record['process_succeeded'])
        self.assertTrue(record['accepted_nonzero_exit'])
        self.assertTrue(record['version_identified'])
        self.assertEqual(record['detected_version'], '11.10.2')
        self.assertEqual(record['validation_scope'], 'VERSION_IDENTIFICATION_ONLY')
        self.assertFalse(record['simulator_runtime_verified'])

    def test_valid_banner_exit_zero_passes(self):
        record = self.query(returncode=0)
        self.assertTrue(self.all_passed())
        self.assertTrue(record['process_succeeded'])
        self.assertFalse(record['accepted_nonzero_exit'])
        self.assertFalse(record['simulator_runtime_verified'])

    def test_exact_banner_on_stderr_is_preserved(self):
        record = self.query(stdout='', stderr=BANNER)
        self.assertTrue(self.all_passed())
        self.assertEqual(record['stdout'], '')
        self.assertEqual(record['stderr'], BANNER)

    def test_banner_can_have_copyright_lines_without_weakening_version_match(self):
        record = self.query(stdout=BANNER + 'Copyright (C) 2012 Open Source Robotics Foundation.\n')
        self.assertTrue(self.all_passed())
        self.assertEqual(record['detected_version'], '11.10.2')

    def test_invalid_output_never_passes_even_with_exit_zero(self):
        invalid = ('Gazebo 11.10.2', 'Gazebo multi-robot simulator, version 11.',
                   'Gazebo multi-robot simulator, version 11.10.2broken',
                   'error: ' + BANNER, 'gzserver: error while loading shared libraries')
        for output in invalid:
            for code in (0, 255):
                with self.subTest(output=output, code=code):
                    record = self.query(code, output)
                    self.assertFalse(record['passed'])
                    self.assertFalse(record['accepted_nonzero_exit'])

    def test_absent_output_never_passes(self):
        for code in (0, 255):
            with self.subTest(code=code):
                self.assertFalse(self.query(code, '', '')['passed'])

    def test_conflicting_version_banners_are_rejected(self):
        record = self.query(stderr=BANNER.replace('11.10.2', '11.11.0'))
        self.assertFalse(record['passed'])
        self.assertIsNone(record['detected_version'])

    def test_wrong_major_does_not_validate_image(self):
        for code in (0, 255):
            with self.subTest(code=code):
                self.validator['checks'].clear()
                record = self.query(code, BANNER.replace('11.10.2', '10.10.2'))
                self.assertFalse(self.all_passed())
                self.assertFalse(record['accepted_nonzero_exit'])

    def test_unexpected_exit_or_signal_is_not_accepted_with_valid_banner(self):
        for code in (1, 2, 127, 139, -11, -15):
            with self.subTest(code=code):
                record = self.query(code)
                self.assertFalse(record['passed'])
                self.assertEqual(record['returncode'], code)
                self.assertFalse(record['accepted_nonzero_exit'])

    def test_timeout_rejects_partial_valid_banner_and_preserves_partial_output(self):
        for stdout in (None, BANNER, BANNER.encode()):
            with self.subTest(stdout=stdout):
                error = subprocess.TimeoutExpired(['gzserver', '--version'], 20,
                                                  output=stdout, stderr=b'partial diagnostic')
                with mock.patch.object(subprocess, 'run', side_effect=error):
                    record = self.validator['gazebo_version_check']()
                self.assertFalse(record['passed'])
                self.assertFalse(record['version_identified'])
                self.assertTrue(record['timed_out'])
                self.assertIsNone(record['returncode'])
                self.assertEqual(record['stdout'], BANNER if stdout else '')
                self.assertEqual(record['stderr'], 'partial diagnostic')
                self.assertEqual(record['error'], 'Command timed out')

    def test_missing_executable_is_rejected(self):
        with mock.patch.object(subprocess, 'run', side_effect=FileNotFoundError('gzserver unavailable')):
            record = self.validator['gazebo_version_check']()
        self.assertFalse(record['passed'])
        self.assertIsNone(record['returncode'])
        self.assertIn('unavailable', record['stderr'])

    def test_other_commands_remain_strict_and_cannot_use_version_exception(self):
        for code in (1, 255, -11):
            with self.subTest(code=code):
                result = subprocess.CompletedProcess(['ldd', 'fixture.so'], code, BANNER, 'failure')
                with mock.patch.object(subprocess, 'run', return_value=result):
                    record = self.validator['command']('linked_libraries', ['ldd', 'fixture.so'])
                self.assertFalse(record['passed'])
                self.assertFalse(record['process_succeeded'])
                self.assertEqual(record['returncode'], code)

    def test_gazebo_success_does_not_hide_another_failed_check(self):
        self.query()
        self.validator['add']('unrelated_image_failure', False)
        self.assertFalse(self.all_passed())


if __name__ == '__main__':
    unittest.main()
