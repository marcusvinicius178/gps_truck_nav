"""Display policy and guardian regression tests; no real X/ROS/Docker connection."""

import ctypes as C
import io
import json
import os
from pathlib import Path
import stat
import struct
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import desktop_display as desktop


def record(number='0', cookie=b'0123456789abcdef', *, family=65535, address=b''):
    return struct.pack('>H', family) + b''.join(struct.pack('>H', len(part)) + part
        for part in (address, number.encode(), desktop.PROTOCOL, cookie))


class DisplayPolicyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.sockets = self.root / 'sockets'; self.sockets.mkdir()
        self.runtime = self.root / 'runtime'; self.runtime.mkdir(mode=0o700)
        self.auth = self.root / 'authority'
        self.auth.write_bytes(record()); self.auth.chmod(0o600)
        self.uid = os.getuid() or 1000
        if os.getuid() == 0:
            os.chown(self.auth, self.uid, -1)
            os.chown(self.runtime, self.uid, -1)
        self.env = {'XDG_SESSION_TYPE': 'x11', 'DISPLAY': ':0',
                    'XAUTHORITY': str(self.auth), 'XDG_RUNTIME_DIR': str(self.runtime)}

    def inspect(self, *, socket_uid=None, **changes):
        original = Path.lstat
        def lstat(path):
            if path == self.sockets / 'X0':
                return SimpleNamespace(st_mode=stat.S_IFSOCK | 0o777,
                                       st_uid=self.uid if socket_uid is None else socket_uid)
            return original(path)
        with patch.object(Path, 'lstat', lstat):
            return desktop.inspect_host({**self.env, **changes}, socket_directory=self.sockets,
                                        uid=self.uid, hostname='host')

    def test_x11_and_xwayland_metadata_preserve_only_selected_local_display(self):
        metadata, cookie = self.inspect()
        self.assertEqual(metadata['display'], ':0')
        self.assertEqual(cookie, b'0123456789abcdef')
        metadata, _ = self.inspect(DISPLAY='unix:0.0', XDG_SESSION_TYPE='wayland', WAYLAND_DISPLAY='wayland-0')
        self.assertEqual(metadata['backend'], 'XWAYLAND')
        self.assertTrue(metadata['wayland_display_present'])
        self.assertNotIn('0123456789abcdef', json.dumps(metadata))

    def test_remote_missing_or_nonzero_screen_displays_are_rejected(self):
        for display in ('', 'localhost:10.0', '192.0.2.1:0', ':0.1', '/tmp/x', ':0,ro'):
            with self.subTest(display=display), self.assertRaises(desktop.DesktopError):
                self.inspect(DISPLAY=display)

    def test_non_graphical_session_or_sudo_is_rejected(self):
        with self.assertRaises(desktop.DesktopError):
            self.inspect(XDG_SESSION_TYPE='tty')
        with self.assertRaises(desktop.DesktopError):
            desktop.inspect_host(self.env, socket_directory=self.sockets, uid=0)

    def test_missing_socket_and_symlink_socket_are_rejected(self):
        with self.assertRaises(OSError):
            self.inspect(DISPLAY=':1')
        (self.sockets / 'X1').symlink_to(self.sockets / 'X0')
        with self.assertRaises(desktop.DesktopError):
            self.inspect(DISPLAY=':1')

    def test_foreign_socket_and_replaceable_shared_socket_directory_fail_before_cookie_read(self):
        with patch.object(desktop, 'private_file') as cookie_read:
            with self.assertRaises(desktop.DesktopError):
                self.inspect(socket_uid=self.uid + 1)
            cookie_read.assert_not_called()
            self.sockets.chmod(0o777)
            with self.assertRaises(desktop.DesktopError):
                self.inspect()
            cookie_read.assert_not_called()
        self.sockets.chmod(0o1777)
        self.assertEqual(self.inspect(socket_uid=0)[0]['display'], ':0')

    def test_authority_permissions_symlink_and_wrong_owner_fail(self):
        self.auth.chmod(0o644)
        with self.assertRaises(desktop.DesktopError):
            self.inspect()
        self.auth.chmod(0o600)
        link = self.root / 'link'; link.symlink_to(self.auth)
        with self.assertRaises(OSError):
            self.inspect(XAUTHORITY=str(link))
        with self.assertRaises(desktop.DesktopError):
            desktop.private_file(self.auth, self.uid + 1)

    def test_runtime_must_be_private_and_owned(self):
        self.runtime.chmod(0o755)
        with self.assertRaises(desktop.DesktopError):
            self.inspect()
        with self.assertRaises(desktop.DesktopError):
            self.inspect(XDG_RUNTIME_DIR='relative')

    def test_auth_database_selects_only_matching_display_and_hostname(self):
        data = record('1', b'aaaaaaaaaaaaaaaa') + record('0', family=256, address=b'host')
        self.assertEqual(desktop.authority_cookie(data, '0', 'host'), b'0123456789abcdef')
        with self.assertRaises(desktop.DesktopError):
            desktop.authority_cookie(data, '0', 'other-host')
        with self.assertRaises(desktop.DesktopError):
            desktop.authority_cookie(record() + record(cookie=b'bbbbbbbbbbbbbbbb'), '0', 'host')

    def test_malformed_missing_or_non_mit_credentials_fail(self):
        for data in (b'', record()[:-1], b'\xff', record().replace(desktop.PROTOCOL, b'OTHER-PROTOCOL---'),
                     record(cookie=b'bad-length')):
            with self.subTest(data_length=len(data)), self.assertRaises(desktop.DesktopError):
                desktop.authority_cookie(data, '0', 'host')

    def test_generated_record_contains_one_wild_address_entry_and_exact_display(self):
        data = desktop.authority_record('23', b'fedcba9876543210')
        self.assertEqual(struct.unpack('>H', data[:2])[0], 65535)
        self.assertEqual(desktop.authority_cookie(data, '23', 'container'), b'fedcba9876543210')
        with self.assertRaises(desktop.DesktopError):
            desktop.authority_cookie(data, '0', 'container')

    def test_options_mount_only_one_socket_and_generated_cookie_read_only(self):
        metadata = {'display': ':0', 'machine': 'arocs-' + 'a' * 12,
                    'socket_path': '/tmp/.X11-unix/X0', 'auth_cookie_path': '/run/user/1000/new/Xauthority'}
        session = desktop.DesktopSession(None, self.root, io.BytesIO(), metadata)
        options = session.docker_options()
        mounts = [options[i + 1] for i, value in enumerate(options) if value == '--mount']
        self.assertEqual(len(mounts), 2)
        self.assertTrue(all(value.endswith(',readonly') for value in mounts))
        self.assertIn('dst=/run/nav2-desktop.Xauthority', mounts[1])
        for forbidden in ('--privileged', '--network', '--ipc', '/var/run/docker.sock', str(self.auth)):
            self.assertNotIn(forbidden, options)
        self.assertEqual(options[options.index('--hostname') + 1], metadata['machine'])

    def test_unsafe_mount_path_and_dead_guardian_fail_closed(self):
        session = desktop.DesktopSession(Mock(), self.root, io.BytesIO(),
            {'display': ':0', 'machine': 'arocs-a', 'socket_path': '/tmp/X0', 'auth_cookie_path': '/tmp/,evil'})
        with self.assertRaises(desktop.DesktopError):
            session.docker_options()
        session.proc.poll.return_value = 2
        with self.assertRaises(desktop.DesktopError):
            session.check_alive()


class GuardianTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.run_id = 'a' * 32
        self.runtime = self.root / ('arocs-desktop-' + self.run_id[:12] + '.test')
        self.runtime.mkdir(mode=0o700)
        self.metadata = {'display': ':0', 'display_number': '0', 'runtime_directory': str(self.root)}
        self.backend = Mock()
        self.connection = self.backend.return_value
        self.connection.generate.return_value = b'fedcba9876543210'
        self.connection.probe.return_value = {'glx_available': True}
        self.connection.close.return_value = True
        self.records = []

    def run_guardian(self, *, eof=True, duration=10):
        read, write = os.pipe()
        os.close(write)
        with os.fdopen(read, 'rb') as control, \
                patch.object(desktop, 'inspect_host', return_value=(self.metadata, b'0123456789abcdef')), \
                patch.object(desktop.signal, 'signal'):
            return desktop.guardian(self.runtime, self.run_id, duration,
                                    backend=self.backend, control=control, emit=self.records.append)

    def test_parent_pipe_eof_revokes_cookie_and_removes_private_runtime(self):
        self.assertEqual(self.run_guardian(), 0)
        self.connection.close.assert_called_once()
        self.assertEqual(self.records[0]['status'], 'DESKTOP_AUTHENTICATED')
        self.assertEqual(self.records[-1]['status'], 'DESKTOP_ACCESS_REVOKED')
        self.assertFalse(self.runtime.exists())
        text = json.dumps(self.records)
        self.assertNotIn('fedcba9876543210', text)
        self.assertNotIn('0123456789abcdef', text)

    def test_generated_cookie_failure_is_revoked_without_exposing_ready(self):
        self.connection.probe.side_effect = desktop.DesktopError('GLX unavailable')
        self.assertEqual(self.run_guardian(), 2)
        self.connection.close.assert_called_once()
        self.assertEqual(len(self.records), 1)
        self.assertTrue(self.records[-1]['authorization_revoked'])
        self.assertFalse(self.runtime.exists())

    def test_original_host_cookie_is_never_exported_as_session_credential(self):
        self.connection.generate.return_value = b'0123456789abcdef'
        self.assertEqual(self.run_guardian(), 2)
        self.connection.probe.assert_not_called()
        self.connection.close.assert_called_once()
        self.assertFalse(self.runtime.exists())
        self.assertNotIn('0123456789abcdef', json.dumps(self.records))

    def test_failed_revocation_never_reports_success(self):
        self.connection.close.return_value = False
        self.assertEqual(self.run_guardian(), 2)
        self.assertEqual(self.records[-1]['status'], 'DESKTOP_ACCESS_FAILED')
        self.assertFalse(self.records[-1]['authorization_revoked'])

    def test_hard_lifetime_ends_guardian_even_with_parent_pipe_open(self):
        with patch.object(desktop.time, 'monotonic', side_effect=[100, 141]), \
                patch.object(desktop.select, 'select') as select_mock:
            self.assertEqual(self.run_guardian(), 0)
        select_mock.assert_not_called()
        self.connection.close.assert_called_once()

    def test_invalid_runtime_does_not_delete_existing_credentials(self):
        path = self.runtime / 'Xauthority'; path.write_bytes(b'preserve this')
        self.assertEqual(self.run_guardian(), 2)
        self.assertEqual(path.read_bytes(), b'preserve this')
        self.backend.assert_not_called()

    def test_bounded_ready_reader_does_not_accept_failure_or_wrong_json(self):
        for value in ({'status': 'FAILED', 'error': 'no authorization'}, []):
            proc = subprocess.Popen([sys.executable, '-c', 'print(' + repr(json.dumps(value)) + ')'],
                                    stdout=subprocess.PIPE)
            try:
                with self.assertRaises((desktop.DesktopError, AttributeError)):
                    desktop.read_ready(proc, timeout=1)
            finally:
                proc.wait(timeout=1); proc.stdout.close()

    def test_close_timeout_reports_failure_and_removes_only_owned_runtime(self):
        proc = Mock()
        proc.communicate.side_effect = [subprocess.TimeoutExpired('guardian', 3), (b'', None)]
        other = self.root / 'other'; other.mkdir()
        log = io.BytesIO()
        session = desktop.DesktopSession(proc, self.runtime, log, {'run_id': self.run_id})
        result = session.close()
        proc.kill.assert_called_once()
        self.assertEqual(result['status'], 'DESKTOP_ACCESS_FAILED')
        self.assertFalse(result['authorization_revoked'])
        self.assertFalse(self.runtime.exists())
        self.assertTrue(other.exists())

    def test_nonzero_guardian_exit_cannot_pass_with_success_json(self):
        proc = Mock()
        proc.returncode = 3
        proc.communicate.return_value = (json.dumps({
            'status': 'DESKTOP_ACCESS_REVOKED', 'run_id': self.run_id,
            'authorization_revoked': True, 'credentials_removed': True}).encode() + b'\n', None)
        session = desktop.DesktopSession(proc, self.runtime, io.BytesIO(), {'run_id': self.run_id})
        result = session.close()
        self.assertEqual(result['status'], 'DESKTOP_ACCESS_FAILED')
        self.assertEqual(result['guardian_returncode'], 3)
        self.assertTrue(result['authorization_revoked'])
        self.assertTrue(result['credentials_removed'])
        self.assertFalse(self.runtime.exists())

    def test_failed_diagnostic_preserves_environment_evidence(self):
        output = self.root / 'diagnostic'
        env = {'XDG_SESSION_TYPE': 'wayland', 'DISPLAY': ':9', 'WAYLAND_DISPLAY': 'wayland-0',
               'XAUTHORITY': '/private/auth', 'XDG_RUNTIME_DIR': '/run/user/1000'}
        with patch.dict(os.environ, env, clear=True), \
                patch.object(desktop.DesktopSession, 'start', side_effect=desktop.DesktopError('no access')), \
                patch('builtins.print'):
            self.assertEqual(desktop.main(['diagnose', '--output-dir', str(output)]), 2)
        report = json.loads((output / 'display_diagnostic.json').read_text())
        self.assertEqual(report['display'], ':9')
        self.assertEqual(report['xdg_session_type'], 'wayland')
        self.assertEqual(report['status'], 'DISPLAY_DIAGNOSTIC_FAILED')
        self.assertFalse(report['motion_authorized'])

    def test_diagnostic_rejects_nonzero_guardian_exit_despite_valid_cleanup_booleans(self):
        output = self.root / 'nonzero-diagnostic'
        session = Mock()
        session.metadata = {'display': ':0', 'status': 'DESKTOP_AUTHENTICATED'}
        session.close.return_value = {'status': 'DESKTOP_ACCESS_FAILED', 'guardian_returncode': 3,
                                     'authorization_revoked': True, 'credentials_removed': True}
        with patch.object(desktop.DesktopSession, 'start', return_value=session), patch('builtins.print'):
            self.assertEqual(desktop.main(['diagnose', '--output-dir', str(output)]), 2)
        report = json.loads((output / 'display_diagnostic.json').read_text())
        self.assertEqual(report['status'], 'DISPLAY_DIAGNOSTIC_FAILED')
        self.assertEqual(report['cleanup']['guardian_returncode'], 3)
        session.close.assert_called_once()


class XSecurityABITests(unittest.TestCase):
    def construct(self, *, enabled=True, extension=True, connected=True):
        x, ext = Mock(), Mock()
        x.XOpenDisplay.return_value = 123 if connected else None
        def hosts(_display, _count, flag):
            C.cast(flag, C.POINTER(C.c_int)).contents.value = int(enabled)
            return None
        x.XListHosts.side_effect = hosts
        ext.XSecurityQueryExtension.return_value = int(extension)
        with patch.object(desktop.C.util, 'find_library', side_effect=['libX11', 'libXext']), \
                patch.object(desktop.C, 'CDLL', side_effect=[x, ext]):
            return desktop.XSecurity(':0', b'0123456789abcdef'), x, ext

    def test_host_access_requires_cookie_connection_access_control_and_security(self):
        for options, error in (({'connected': False}, 'AUTHENTICATION_FAILED'),
                               ({'enabled': False}, 'ACCESS_CONTROL_DISABLED'),
                               ({'extension': False}, 'SECURITY_EXTENSION_UNAVAILABLE')):
            with self.subTest(options=options), self.assertRaisesRegex(desktop.DesktopError, error):
                self.construct(**options)
        security, x, _ = self.construct()
        x.XSetAuthorization.assert_called_once_with(desktop.PROTOCOL, len(desktop.PROTOCOL),
                                                   b'0123456789abcdef', 16)
        self.assertTrue(security.close())

    def test_generated_cookie_must_connect_and_expose_glx(self):
        security, x, _ = self.construct()
        x.XQueryExtension.return_value = 0
        with self.assertRaisesRegex(desktop.DesktopError, 'GLX_UNAVAILABLE'):
            security.probe(b'fedcba9876543210')
        x.XCloseDisplay.assert_called_once_with(123)
        x.XOpenDisplay.return_value = None
        with self.assertRaisesRegex(desktop.DesktopError, 'GENERATED_X11_AUTHENTICATION_FAILED'):
            security.probe(b'fedcba9876543210')
        x.XOpenDisplay.return_value = 124
        x.XQueryExtension.return_value = 1
        self.assertTrue(security.probe(b'fedcba9876543210')['generated_cookie_connection_verified'])
        security.close()

    def test_revoke_is_server_side_and_synchronized_before_connection_close(self):
        security, x, ext = self.construct()
        security.auth_id = 99
        ext.XSecurityRevokeAuthorization.return_value = 1
        self.assertTrue(security.close())
        ext.XSecurityRevokeAuthorization.assert_called_once_with(123, 99)
        x.XSync.assert_called_once_with(123, 0)
        x.XCloseDisplay.assert_called_once_with(123)

    def test_generate_passes_trusted_glx_policy_and_frees_only_library_owned_data(self):
        security = object.__new__(desktop.XSecurity)
        security.connection, security.auth_id = 123, 0
        security.ext = Mock()
        request = desktop.Xauth()
        generated = desktop.Xauth()
        cookie = C.create_string_buffer(b'fedcba9876543210')
        generated.data = C.cast(cookie, C.c_void_p)
        generated.data_length = 16
        security.ext.XSecurityAllocXauth.return_value = C.pointer(request)
        def generate(connection, req, mask, attributes, auth_id):
            self.assertEqual(connection, 123)
            self.assertEqual(mask, 3)
            attrs = C.cast(attributes, C.POINTER(desktop.SecurityAttributes)).contents
            self.assertEqual(attrs.timeout, 60)
            self.assertEqual(attrs.trust_level, 0)
            self.assertEqual(C.string_at(req.contents.name, req.contents.name_length), desktop.PROTOCOL)
            C.cast(auth_id, C.POINTER(C.c_ulong)).contents.value = 99
            return C.pointer(generated)
        security.ext.XSecurityGenerateAuthorization.side_effect = generate
        self.assertEqual(security.generate(), b'fedcba9876543210')
        self.assertEqual(security.auth_id, 99)
        self.assertEqual(security.ext.XSecurityFreeXauth.call_count, 2)
        self.assertIsNone(request.name)
        self.assertEqual(request.name_length, 0)
        self.assertEqual(desktop.SecurityAttributes.timeout.offset, 0)
        self.assertEqual(desktop.SecurityAttributes.trust_level.offset, C.sizeof(C.c_uint))


if __name__ == '__main__':
    unittest.main()
