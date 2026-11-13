"""Fake X11/process observations only: never opens a real display or simulator."""

import ctypes
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import desktop_windows as desktop


HOSTNAME = 'nav2-desktop-aabbcc'
GROUPS = {'gazebo': 101, 'rviz': 201}


def record(pid, title, **changes):
    return {'machine': HOSTNAME, 'pid': pid, 'title': title, 'classes': [title],
            'mapped': True, 'iconic': False, 'managed': True, 'transient': False, **changes}


class FakeBackend:
    root = 1

    def __init__(self, records=None, clients=None, tree=None):
        self.records = records if records is not None else {10: record(102, 'Gazebo'), 20: record(202, 'RViz2')}
        self.clients = list(self.records) if clients is None and tree is None else clients
        self.tree = tree or {}
        self.closed = False
        self.title_reads = []
        self.tree_reads = []

    def client_list(self):
        return self.clients

    def children(self, window):
        self.tree_reads.append(window)
        return self.tree.get(window, [])

    def machine(self, window):
        return self.records.get(window, {}).get('machine')

    def pid(self, window):
        return self.records.get(window, {}).get('pid')

    def details(self, window):
        self.title_reads.append(window)
        return self.records.get(window)

    def close(self):
        self.closed = True


class DesktopWindowTests(unittest.TestCase):
    def inspect(self, backend=None, groups=None):
        backend = backend or FakeBackend()
        def getpgid(pid):
            return (groups or {102: 101, 202: 201}).get(pid, 999)
        result = desktop.inspect_windows(':0', HOSTNAME, GROUPS,
                                         backend_factory=lambda display: backend, getpgid=getpgid)
        self.assertTrue(backend.closed)
        return result

    def test_owned_gui_descendant_processes_are_found(self):
        result = self.inspect()
        self.assertEqual(set(result), {'gazebo', 'rviz'})
        self.assertEqual(result['rviz']['pid'], 202)
        self.assertEqual(result['rviz']['process_group'], 201)

    def test_unrelated_host_titles_are_not_read(self):
        backend = FakeBackend({10: record(102, 'Gazebo'),
                               20: record(202, 'RViz2', machine='host'),
                               30: record(333, 'Gazebo')})
        self.assertEqual(set(self.inspect(backend)), {'gazebo'})
        self.assertEqual(backend.title_reads, [10])

    def test_matching_title_and_pid_without_machine_is_not_ownership(self):
        backend = FakeBackend({10: record(102, 'Gazebo', machine=None)})
        self.assertEqual(self.inspect(backend), {})
        self.assertEqual(backend.title_reads, [])

    def test_matching_machine_and_title_without_owned_pid_is_not_ownership(self):
        for pid in (None, True, 0, 1, -1, 999):
            with self.subTest(pid=pid):
                backend = FakeBackend({10: record(pid, 'Gazebo')})
                self.assertEqual(self.inspect(backend), {})
                self.assertEqual(backend.title_reads, [])

    def test_reparented_clients_found_without_ewmh_list(self):
        backend = FakeBackend(tree={1: [2, 3], 2: [4], 4: [10], 3: [20]})
        self.assertEqual(set(self.inspect(backend)), {'gazebo', 'rviz'})

    def test_ewmh_list_avoids_unnecessary_tree_inspection(self):
        backend = FakeBackend()
        self.inspect(backend)
        self.assertEqual(backend.tree_reads, [])

    def test_minimized_owned_window_remains_present(self):
        backend = FakeBackend({10: record(102, 'Gazebo', mapped=False, iconic=True),
                               20: record(202, 'RViz2')})
        result = self.inspect(backend)
        self.assertEqual(set(result), {'gazebo', 'rviz'})
        self.assertFalse(result['gazebo']['mapped'])
        self.assertTrue(result['gazebo']['iconic'])

    def test_unmapped_not_yet_shown_or_transient_window_not_ready(self):
        for change in ({'mapped': False, 'managed': False}, {'transient': True}):
            with self.subTest(change=change):
                backend = FakeBackend({10: record(102, 'Gazebo', **change)})
                self.assertEqual(self.inspect(backend), {})

    def test_managed_normal_window_hidden_on_another_workspace_remains_present(self):
        backend = FakeBackend({10: record(102, 'Gazebo', mapped=False, managed=True, iconic=False),
                               20: record(202, 'RViz2')})
        result = self.inspect(backend)
        self.assertEqual(set(result), {'gazebo', 'rviz'})
        self.assertFalse(result['gazebo']['mapped'])
        self.assertFalse(result['gazebo']['iconic'])
        self.assertTrue(result['gazebo']['managed'])

    def test_class_can_identify_main_window_with_custom_title(self):
        backend = FakeBackend({10: record(102, 'Simulation', classes=['gzclient']),
                               20: record(202, 'observation.rviz - RViz', classes=['rviz2'])})
        self.assertEqual(set(self.inspect(backend)), {'gazebo', 'rviz'})

    def test_owned_other_application_does_not_imply_required_gui(self):
        backend = FakeBackend({10: record(102, 'Terminal', classes=['xterm'])})
        self.assertEqual(self.inspect(backend), {})

    def test_process_exit_during_query_is_not_accepted(self):
        backend = FakeBackend()
        def gone(pid):
            raise ProcessLookupError(pid)
        self.assertEqual(desktop.inspect_windows(':0', HOSTNAME, GROUPS,
            backend_factory=lambda display: backend, getpgid=gone), {})
        self.assertTrue(backend.closed)

    def test_excessive_client_list_fails_closed_and_closes_connection(self):
        backend = FakeBackend(clients=list(range(desktop.MAX_WINDOWS + 1)))
        with self.assertRaisesRegex(RuntimeError, 'bound'):
            self.inspect(backend)
        self.assertTrue(backend.closed)

    def test_tree_is_bounded_and_cycles_are_not_followed_forever(self):
        backend = FakeBackend(tree={1: [2], 2: [1, 10, 20]})
        self.assertEqual(set(self.inspect(backend)), {'gazebo', 'rviz'})
        self.assertEqual(backend.tree_reads.count(1), 1)
        backend = FakeBackend(tree={1: list(range(2, desktop.MAX_WINDOWS + 2))})
        with self.assertRaisesRegex(RuntimeError, 'bound'):
            self.inspect(backend)

    def test_remote_display_bad_machine_or_unbound_process_groups_refused(self):
        cases = [('host:0', HOSTNAME, GROUPS), (':0\n', HOSTNAME, GROUPS),
                 (':0', '', GROUPS), (':0', 'name/elsewhere', GROUPS),
                 (':0', HOSTNAME, {'gazebo': 1, 'rviz': 201}),
                 (':0', HOSTNAME, {'gazebo': 101, 'rviz': 101}),
                 (':0', HOSTNAME, {'gazebo': True, 'rviz': 201}),
                 (':0', HOSTNAME, {'gazebo': 101})]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                desktop.validate_request(*args)

    def test_bounded_child_uses_two_second_deadline_and_explicit_identity(self):
        expected = self.inspect()
        calls = []
        def runner(argv, **kwargs):
            calls.append((argv, kwargs))
            return SimpleNamespace(stdout=json.dumps(expected))
        self.assertEqual(desktop.inspect_windows_bounded(':0', HOSTNAME, GROUPS, runner=runner), expected)
        argv, kwargs = calls[0]
        self.assertEqual(kwargs, {'timeout': 2, 'check': True, 'capture_output': True, 'text': True})
        self.assertEqual(argv[-8:], ['--display', ':0', '--machine', HOSTNAME,
                                     '--gazebo-pgid', '101', '--rviz-pgid', '201'])

    def test_wedged_x_server_timeout_remains_failure(self):
        def runner(argv, **kwargs):
            raise subprocess.TimeoutExpired(argv, kwargs['timeout'])
        with self.assertRaises(subprocess.TimeoutExpired):
            desktop.inspect_windows_bounded(':0', HOSTNAME, GROUPS, runner=runner)

    def test_child_result_requires_owned_identity_and_ready_window(self):
        valid = self.inspect()
        for payload in ([], {'other': {}}, {'gazebo': {}},
                        {'gazebo': {**valid['gazebo'], 'machine': 'host'}},
                        {'gazebo': {**valid['gazebo'], 'process_group': 201}},
                        {'gazebo': {**valid['gazebo'], 'mapped': False, 'iconic': False, 'managed': False}},
                        {'gazebo': {**valid['gazebo'], 'window': True}}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                desktop.inspect_windows_bounded(':0', HOSTNAME, GROUPS,
                    runner=lambda *a, **k: SimpleNamespace(stdout=json.dumps(payload)))


class NativeXPropertyTests(unittest.TestCase):
    def test_32_bit_property_uses_native_unsigned_long_slots(self):
        values = (ctypes.c_ulong * 2)(42, 123456)
        freed = []
        class FakeX:
            def XInternAtom(self, *args):
                return 99
            def XGetWindowProperty(self, display, window, atom, offset, length, delete, requested,
                                   actual, format_, count, remaining, data):
                ctypes.cast(actual, ctypes.POINTER(ctypes.c_ulong))[0] = requested
                ctypes.cast(format_, ctypes.POINTER(ctypes.c_int))[0] = 32
                ctypes.cast(count, ctypes.POINTER(ctypes.c_ulong))[0] = 2
                ctypes.cast(remaining, ctypes.POINTER(ctypes.c_ulong))[0] = 0
                ctypes.cast(data, ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)))[0] = ctypes.cast(
                    values, ctypes.POINTER(ctypes.c_ubyte))
                return 0
            def XFree(self, data):
                freed.append(data)
        backend = desktop.X11Backend.__new__(desktop.X11Backend)
        backend.x, backend.display, backend._errors = FakeX(), 123, []
        self.assertEqual(backend.property(1, '_NET_CLIENT_LIST', 32, 'WINDOW', 10), [42, 123456])
        self.assertEqual(len(freed), 1)

    def test_bad_window_query_race_is_missing_candidate_not_process_failure(self):
        backend = desktop.X11Backend.__new__(desktop.X11Backend)
        backend.display, backend._errors = 123, []
        event = desktop.XErrorEvent(error_code=3, resourceid=10)
        class FakeX:
            def XInternAtom(self, *args):
                return 99
            def XGetWindowProperty(self, *args):
                backend._capture_error(123, ctypes.pointer(event))
                return 3
        backend.x = FakeX()
        self.assertIsNone(backend.property(10, 'WM_CLIENT_MACHINE', 8, 'STRING', 100))
        self.assertEqual(backend._errors, [])
        # The callback itself must return normally across the C boundary.
        callback = desktop.XErrorHandler(backend._capture_error)
        self.assertEqual(callback(123, ctypes.pointer(event)), 0)
        self.assertTrue(backend._query_had_bad_window())
        self.assertFalse(backend._query_had_bad_window())

    def test_other_protocol_errors_are_not_hidden_by_bad_window(self):
        backend = desktop.X11Backend.__new__(desktop.X11Backend)
        for codes in ([2], [10], [3, 9], [3, 11]):
            with self.subTest(codes=codes):
                backend._errors = list(codes)
                with self.assertRaisesRegex(RuntimeError, 'X11 query protocol error'):
                    backend._query_had_bad_window()

    def test_closing_unrelated_client_does_not_hide_owned_windows(self):
        backend = FakeBackend()
        backend.clients = [999, 10, 20]
        # Missing machine is what a captured BadWindow property query returns.
        result = desktop.inspect_windows(':0', HOSTNAME, GROUPS,
            backend_factory=lambda display: backend,
            getpgid=lambda pid: {102: 101, 202: 201}[pid])
        self.assertEqual(set(result), {'gazebo', 'rviz'})
        self.assertEqual(backend.title_reads, [10, 20])

    def test_close_restores_previous_handler_even_when_protocol_error_occurs(self):
        for code in (None, 3, 10):
            with self.subTest(code=code):
                backend = desktop.X11Backend.__new__(desktop.X11Backend)
                backend.display, backend._errors = 123, []
                backend._handler, backend._previous_handler = object(), 456
                calls = []
                class FakeX:
                    def XSync(self, display, discard):
                        calls.append(('sync', display, discard))
                        if code is not None:
                            backend._errors.append(code)
                    def XCloseDisplay(self, display):
                        calls.append(('close', display))
                    def XSetErrorHandler(self, handler):
                        calls.append(('restore', handler))
                backend.x = FakeX()
                if code == 10:
                    with self.assertRaises(RuntimeError):
                        backend.close()
                else:
                    backend.close()
                self.assertEqual(calls, [('sync', 123, 0), ('close', 123), ('restore', 456)])
                self.assertIsNone(backend.display)
                self.assertIsNone(backend._handler)


if __name__ == '__main__':
    unittest.main()
