"""Offline GDM/libXau matching and authenticated-guardian regressions.

All credentials are synthetic. The optional libXau checks read a temporary
authority file only; no test contacts an X server, Docker or ROS.
"""

import base64
import ctypes as C
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools' / 'auto_tuning'))
import desktop_display as desktop


HOST = 'ipt-sim-01'
COOKIE = b'gdm-test-secret1'
OTHER = b'gdm-test-secret2'
GENERATED = b'session-secret3!'


def record(number=b'', cookie=COOKIE, *, family=256, address=HOST.encode(),
           protocol=desktop.PROTOCOL):
    return struct.pack('>H', family) + b''.join(
        struct.pack('>H', len(value)) + value
        for value in (address, number, protocol, cookie))


def gdm_pair(cookie=COOKIE):
    # GDM writes the same authorization twice, changing Local to Wild only.
    return record(cookie=cookie) + record(cookie=cookie, family=65535)


class GdmAuthorityTests(unittest.TestCase):
    def assert_no_secrets(self, value):
        text = json.dumps(value, sort_keys=True)
        for secret in (COOKIE, OTHER, GENERATED):
            for representation in (secret.decode(), secret.hex(),
                                   base64.b64encode(secret).decode(),
                                   hashlib.sha256(secret).hexdigest()):
                self.assertNotIn(representation, text)

    def test_real_gdm_shape_deduplicates_in_both_orders(self):
        local, wild = record(), record(family=65535)
        for data in (local + wild, wild + local):
            with self.subTest(data_order='local-first' if data == local + wild else 'wild-first'):
                metadata = {}
                self.assertEqual(desktop.authority_cookie(data, '1', HOST, metadata=metadata), COOKIE)
                self.assertEqual(metadata, {
                    'matching_records': 2, 'unique_credentials': 1, 'duplicate_records': 1,
                    'empty_number_records': 2, 'exact_number_records': 0,
                    'local_records': 1, 'wild_records': 1,
                    'selection_policy': 'REQUIRE_UNIQUE_MATCHING_CREDENTIAL'})
                self.assert_no_secrets(metadata)

    def test_distinct_local_and_wild_credentials_fail_in_both_orders(self):
        first, second = record(), record(cookie=OTHER, family=65535)
        for data in (first + second, second + first):
            metadata = {}
            with self.assertRaisesRegex(desktop.DesktopError, 'XAUTHORITY_AMBIGUOUS_CREDENTIALS') as error:
                desktop.authority_cookie(data, '1', HOST, metadata=metadata)
            self.assertEqual(metadata['matching_records'], 2)
            self.assertEqual(metadata['unique_credentials'], 2)
            self.assertEqual(metadata['duplicate_records'], 0)
            self.assert_no_secrets({'error': str(error.exception), 'metadata': metadata})

    def test_empty_numbers_match_local_and_wild_individually(self):
        for family in (256, 65535):
            with self.subTest(family=family):
                self.assertEqual(desktop.authority_cookie(record(family=family), '1', HOST), COOKIE)

    def test_local_requires_hostname_but_wild_ignores_address(self):
        wrong_local = record(cookie=OTHER, address=b'other-host')
        wild = record(family=65535, address=b'arbitrary-address')
        self.assertEqual(desktop.authority_cookie(wrong_local + wild, '1', HOST), COOKIE)
        with self.assertRaisesRegex(desktop.DesktopError, 'XAUTHORITY_NO_MATCHING_CREDENTIAL'):
            desktop.authority_cookie(wrong_local, '1', HOST)

    def test_wild_does_not_ignore_nonempty_display_number(self):
        with self.assertRaisesRegex(desktop.DesktopError, 'XAUTHORITY_NO_MATCHING_CREDENTIAL'):
            desktop.authority_cookie(record(b'2', family=65535), '1', HOST)
        data = record(b'2', cookie=OTHER, family=65535) + record(b'1')
        self.assertEqual(desktop.authority_cookie(data, '1', HOST), COOKIE)
        self.assertEqual(desktop.authority_cookie(data, '2', HOST), OTHER)

    def test_exact_and_empty_duplicate_records_are_valid(self):
        metadata = {}
        self.assertEqual(desktop.authority_cookie(record() + record(b'1'), '1', HOST,
                                                 metadata=metadata), COOKIE)
        self.assertEqual(metadata['empty_number_records'], 1)
        self.assertEqual(metadata['exact_number_records'], 1)
        self.assertEqual(metadata['duplicate_records'], 1)

    def test_exact_number_does_not_override_distinct_empty_number_secret(self):
        empty, exact = record(), record(b'1', cookie=OTHER)
        for data in (empty + exact, exact + empty):
            with self.assertRaisesRegex(desktop.DesktopError, 'XAUTHORITY_AMBIGUOUS_CREDENTIALS'):
                desktop.authority_cookie(data, '1', HOST)

    def test_strict_protocol_length_and_family_filters_remain(self):
        for data in (record(protocol=b'OTHER-PROTOCOL'), record(cookie=b''),
                     record(cookie=b'x' * 15), record(cookie=b'x' * 17),
                     record(family=0)):
            with self.subTest(record=data), self.assertRaisesRegex(
                    desktop.DesktopError, 'XAUTHORITY_NO_MATCHING_CREDENTIAL'):
                desktop.authority_cookie(data, '1', HOST)

    def test_query_must_identify_an_explicit_display_number(self):
        for number in ('', '*', ':1', '1.0', '1\n', None, 1):
            with self.subTest(number=number), self.assertRaises(desktop.DesktopError):
                desktop.authority_cookie(record(), number, HOST)

    def test_malformed_trailer_cannot_be_hidden_by_matching_record(self):
        for trailer in (b'\x00', struct.pack('>HH', 256, 20) + b'short'):
            metadata = {}
            with self.assertRaisesRegex(desktop.DesktopError, 'Malformed|Truncated'):
                desktop.authority_cookie(gdm_pair() + trailer, '1', HOST, metadata=metadata)
            self.assertEqual(metadata, {})


class LibXauNumberSemanticsTests(unittest.TestCase):
    """Differential check against the installed library, without any X server."""

    @classmethod
    def setUpClass(cls):
        library = C.util.find_library('Xau')
        if not library:
            raise unittest.SkipTest('Optional libXau unavailable')
        cls.lib = C.CDLL(library)
        cls.lib.XauDisposeAuth.argtypes = [C.POINTER(desktop.Xauth)]
        cls.lib.XauDisposeAuth.restype = None
        cls.lib.XauGetAuthByAddr.argtypes = [C.c_ushort, C.c_ushort, C.c_char_p,
                                           C.c_ushort, C.c_char_p, C.c_ushort, C.c_char_p]
        cls.lib.XauGetAuthByAddr.restype = C.POINTER(desktop.Xauth)
        cls.lib.XauGetBestAuthByAddr.argtypes = [C.c_ushort, C.c_ushort, C.c_char_p,
                                               C.c_ushort, C.c_char_p, C.c_int,
                                               C.POINTER(C.c_char_p), C.POINTER(C.c_int)]
        cls.lib.XauGetBestAuthByAddr.restype = C.POINTER(desktop.Xauth)

    def lookup(self, data, *, best):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'synthetic.Xauthority'
            path.write_bytes(data)
            path.chmod(0o600)
            with patch.dict(os.environ, {'XAUTHORITY': str(path)}):
                args = (256, len(HOST), HOST.encode(), 1, b'1')
                if best:
                    names = (C.c_char_p * 1)(desktop.PROTOCOL)
                    lengths = (C.c_int * 1)(len(desktop.PROTOCOL))
                    result = self.lib.XauGetBestAuthByAddr(*args, 1, names, lengths)
                else:
                    result = self.lib.XauGetAuthByAddr(*args, len(desktop.PROTOCOL), desktop.PROTOCOL)
                if not result:
                    return None
                try:
                    return C.string_at(result.contents.data, result.contents.data_length)
                finally:
                    self.lib.XauDisposeAuth(result)

    def test_empty_number_and_family_wild_semantics_match_libxau(self):
        cases = ((gdm_pair(), COOKIE), (record(), COOKIE),
                 (record(family=65535, address=b'ignored-host'), COOKIE),
                 (record(b'1'), COOKIE), (record(b'2', family=65535), None),
                 (record(address=b'wrong-host'), None))
        for data, expected in cases:
            for best in (False, True):
                with self.subTest(best=best, expected_match=expected is not None):
                    self.assertEqual(self.lookup(data, best=best), expected)
                    if expected is None:
                        with self.assertRaises(desktop.DesktopError):
                            desktop.authority_cookie(data, '1', HOST)
                    else:
                        self.assertEqual(desktop.authority_cookie(data, '1', HOST), expected)

    def test_application_rejects_libxau_first_match_ambiguity(self):
        local, wild = record(), record(cookie=OTHER, family=65535)
        for data, first_cookie in ((local + wild, COOKIE), (wild + local, OTHER)):
            for best in (False, True):
                self.assertEqual(self.lookup(data, best=best), first_cookie)
            with self.assertRaisesRegex(desktop.DesktopError, 'XAUTHORITY_AMBIGUOUS_CREDENTIALS'):
                desktop.authority_cookie(data, '1', HOST)


class GdmGuardianTests(unittest.TestCase):
    assert_no_secrets = GdmAuthorityTests.assert_no_secrets

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.socket_dir = self.root / 'sockets'
        self.socket_dir.mkdir()
        self.user_runtime = self.root / 'user-runtime'
        self.user_runtime.mkdir(mode=0o700)
        self.run_id = 'b' * 32
        self.runtime = self.user_runtime / ('arocs-desktop-' + self.run_id[:12] + '.test')
        self.runtime.mkdir(mode=0o700)
        self.auth = self.root / 'GDM.Xauthority'
        self.auth.write_bytes(gdm_pair())
        self.auth.chmod(0o600)
        self.uid = os.getuid() or 1001
        if os.getuid() == 0:
            for path in (self.auth, self.user_runtime, self.runtime):
                os.chown(path, self.uid, -1)
        self.env = {'XDG_SESSION_TYPE': 'x11', 'DISPLAY': ':1',
                    'XAUTHORITY': str(self.auth), 'XDG_RUNTIME_DIR': str(self.user_runtime)}
        self.backend = Mock()
        self.connection = self.backend.return_value
        self.connection.generate.return_value = GENERATED
        self.connection.probe.return_value = {'glx_available': True,
                                              'generated_cookie_connection_verified': True,
                                              'access_control_enabled': True}
        self.connection.close.return_value = True
        self.records = []
        self.generated_files = []

    def run_guardian(self):
        real_lstat, real_inspect = Path.lstat, desktop.inspect_host

        def lstat(path):
            if path == self.socket_dir / 'X1':
                return SimpleNamespace(st_mode=stat.S_IFSOCK | 0o777, st_uid=self.uid)
            return real_lstat(path)

        def inspect():
            return real_inspect(self.env, socket_directory=self.socket_dir, uid=self.uid, hostname=HOST)

        def emit(value):
            self.records.append(value)
            if value['status'] == 'DESKTOP_AUTHENTICATED':
                self.generated_files.append((self.runtime / 'Xauthority').read_bytes())

        reader, writer = os.pipe()
        os.close(writer)  # EOF requests immediate normal revocation, no sleep.
        with os.fdopen(reader, 'rb') as control, patch.object(Path, 'lstat', lstat), \
                patch.object(desktop, 'inspect_host', side_effect=inspect), \
                patch.object(desktop.os, 'getuid', return_value=self.uid), \
                patch.object(desktop.signal, 'signal'):
            return desktop.guardian(self.runtime, self.run_id, 10, backend=self.backend,
                                    control=control, emit=emit)

    def test_gdm_credential_reaches_selected_display_and_only_new_cookie_is_exported(self):
        self.assertEqual(self.run_guardian(), 0)
        self.backend.assert_called_once_with(':1', COOKIE)
        self.connection.probe.assert_called_once_with(GENERATED)
        self.assertEqual(self.generated_files, [desktop.authority_record('1', GENERATED)])
        self.assertEqual(self.records[0]['authority_match']['duplicate_records'], 1)
        self.assertTrue(self.records[0]['generated_cookie_connection_verified'])
        self.assertFalse(self.records[0]['host_authority_mounted'])
        self.assertFalse(self.records[0]['motion_authorized'])
        self.assertTrue(self.records[-1]['authorization_revoked'])
        self.assertFalse(self.runtime.exists())
        self.assertEqual(self.auth.read_bytes(), gdm_pair())
        self.assert_no_secrets(self.records)

    def test_ambiguous_gdm_authority_never_reaches_backend_or_ready(self):
        ambiguous = record() + record(cookie=OTHER, family=65535)
        self.auth.write_bytes(ambiguous)
        self.assertEqual(self.run_guardian(), 2)
        self.backend.assert_not_called()
        self.assertEqual(len(self.records), 1)
        self.assertIn('XAUTHORITY_AMBIGUOUS_CREDENTIALS', self.records[0]['error'])
        self.assertEqual(self.generated_files, [])
        self.assertFalse((self.runtime / 'Xauthority').exists())
        self.assertEqual(self.auth.read_bytes(), ambiguous)
        self.assert_no_secrets(self.records)

    def test_rejected_host_authentication_access_control_or_security_never_exports(self):
        for error in ('HOST_X11_AUTHENTICATION_FAILED', 'HOST_X11_ACCESS_CONTROL_DISABLED',
                      'HOST_X11_SECURITY_EXTENSION_UNAVAILABLE'):
            with self.subTest(error=error):
                self.backend.side_effect = desktop.DesktopError(error)
                self.assertEqual(self.run_guardian(), 2)
                self.assertEqual(self.records[-1]['status'], 'DESKTOP_ACCESS_FAILED')
                self.assertIn(error, self.records[-1]['error'])
                self.assertFalse(any(item['status'] == 'DESKTOP_AUTHENTICATED' for item in self.records))
                self.assertEqual(self.generated_files, [])
                self.assertFalse(self.runtime.exists())
                self.assertEqual(self.auth.read_bytes(), gdm_pair())
                self.assert_no_secrets(self.records)
                self.runtime.mkdir(mode=0o700)
                if os.getuid() == 0:
                    os.chown(self.runtime, self.uid, -1)

    def test_generated_authentication_or_glx_rejection_revokes_without_ready(self):
        for error in ('GENERATED_X11_AUTHENTICATION_FAILED', 'HOST_X11_GLX_UNAVAILABLE'):
            with self.subTest(error=error):
                self.connection.probe.side_effect = desktop.DesktopError(error)
                self.assertEqual(self.run_guardian(), 2)
                self.assertEqual(self.records[-1]['status'], 'DESKTOP_ACCESS_FAILED')
                self.assertTrue(self.records[-1]['authorization_revoked'])
                self.assertFalse(any(item['status'] == 'DESKTOP_AUTHENTICATED' for item in self.records))
                self.assertEqual(self.generated_files, [])
                self.assertFalse(self.runtime.exists())
                self.assertEqual(self.auth.read_bytes(), gdm_pair())
                self.assert_no_secrets(self.records)
                self.runtime.mkdir(mode=0o700)
                if os.getuid() == 0:
                    os.chown(self.runtime, self.uid, -1)


if __name__ == '__main__':
    unittest.main()
