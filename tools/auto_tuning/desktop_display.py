#!/usr/bin/env python3
"""Session-scoped authenticated X11 access; never starts ROS, Docker or motion.

The trusted XSECURITY cookie is required for ordinary GLX clients. It is a
revocable credential, not isolation from other X11 clients on the host display.
The guardian owns its lifetime independently of the commissioning supervisor.
"""

import argparse
import ctypes as C
import ctypes.util
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import time


PROTOCOL = b'MIT-MAGIC-COOKIE-1'
CONTAINER_AUTH = '/run/nav2-desktop.Xauthority'
IDLE_AUTH_TIMEOUT = 60  # Idle/no-client timeout, not the hard session lifetime.


class DesktopError(RuntimeError):
    pass


def private_file(path, uid):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != uid
                or info.st_mode & 0o077 or info.st_nlink != 1 or info.st_size > 1048576):
            raise DesktopError('XAUTHORITY must be a private, owned regular file')
        data = stream.read(1048577)
        if len(data) > 1048576:
            raise DesktopError('XAUTHORITY exceeds the bounded size')
        return data


def authority_cookie(data, number, hostname, *, metadata=None):
    """Match libXau's local/number wildcards, but require one distinct secret.

    GDM can write Local and Wild records with an empty number and the same
    cookie. Unlike libXau's first-match tie break, different eligible secrets
    are rejected regardless of order or apparent record specificity.
    """
    if not isinstance(number, str) or not re.fullmatch(r'[0-9]+', number):
        raise DesktopError('Explicit selected X11 display number required')
    offset, matches = 0, set()
    counts = dict.fromkeys(('matching_records', 'empty_number_records', 'exact_number_records',
                           'local_records', 'wild_records'), 0)
    while offset < len(data):
        if offset + 2 > len(data):
            raise DesktopError('Malformed XAUTHORITY record')
        family = struct.unpack_from('>H', data, offset)[0]
        offset += 2
        fields = []
        for _ in range(4):
            if offset + 2 > len(data):
                raise DesktopError('Malformed XAUTHORITY field')
            size = struct.unpack_from('>H', data, offset)[0]
            offset += 2
            if offset + size > len(data):
                raise DesktopError('Truncated XAUTHORITY field')
            fields.append(data[offset:offset + size])
            offset += size
        address, display_number, name, cookie = fields
        if (family == 65535 or (family == 256 and address == hostname.encode())) and \
                display_number in (b'', number.encode()) and name == PROTOCOL and len(cookie) == 16:
            matches.add(cookie)
            counts['matching_records'] += 1
            counts['empty_number_records' if not display_number else 'exact_number_records'] += 1
            counts['local_records' if family == 256 else 'wild_records'] += 1
    counts.update(unique_credentials=len(matches),
                  duplicate_records=counts['matching_records'] - len(matches),
                  selection_policy='REQUIRE_UNIQUE_MATCHING_CREDENTIAL')
    if metadata is not None:
        metadata.update(counts)  # Counts only: no bytes, encodings, hashes or fingerprints.
    if not matches:
        raise DesktopError('XAUTHORITY_NO_MATCHING_CREDENTIAL: matching_records=0, unique_credentials=0')
    if len(matches) > 1:
        raise DesktopError('XAUTHORITY_AMBIGUOUS_CREDENTIALS: '
                           f"matching_records={counts['matching_records']}, unique_credentials={len(matches)}")
    return next(iter(matches))


def authority_record(number, cookie):
    if not re.fullmatch(r'[0-9]+', number) or len(cookie) != 16:
        raise DesktopError('Invalid generated X11 authorization')
    # FamilyWild changes address matching only. The secret and display number
    # still apply, while the container deliberately has its own hostname.
    return struct.pack('>H', 65535) + b''.join(
        struct.pack('>H', len(value)) + value
        for value in (b'', number.encode(), PROTOCOL, cookie))


def inspect_host(environ=None, *, socket_directory=Path('/tmp/.X11-unix'), uid=None, hostname=None):
    env = os.environ if environ is None else environ
    uid = os.getuid() if uid is None else uid
    if uid == 0:
        raise DesktopError('Run from the logged-in desktop user, without sudo')
    session_type = env.get('XDG_SESSION_TYPE', '')
    if session_type not in ('x11', 'wayland'):
        raise DesktopError('A local X11 or Wayland desktop session is required')
    display = env.get('DISPLAY', '')
    match = re.fullmatch(r'(?:unix)?:([0-9]{1,4})(?:\.0)?', display)
    if not match:
        raise DesktopError('DISPLAY must name a local Unix X11 display (:N or :N.0), not TCP/SSH')
    number = str(int(match.group(1)))
    display = ':' + number
    socket_path = socket_directory / ('X' + number)
    directory_info, socket_info = socket_directory.lstat(), socket_path.lstat()
    if (not stat.S_ISDIR(directory_info.st_mode) or directory_info.st_uid not in (0, uid)
            or (directory_info.st_mode & 0o022 and not directory_info.st_mode & stat.S_ISVTX)
            or not stat.S_ISSOCK(socket_info.st_mode) or socket_info.st_uid not in (0, uid)):
        raise DesktopError('Local X11 socket/directory must be real and owned by root or the desktop user')
    authority = Path(env.get('XAUTHORITY') or str(Path(env.get('HOME', '')).expanduser() / '.Xauthority'))
    if not authority.is_absolute():
        raise DesktopError('XAUTHORITY must have an absolute path')
    authority_match = {}
    cookie = authority_cookie(private_file(authority, uid), number, hostname or socket.gethostname(),
                              metadata=authority_match)
    runtime = Path(env.get('XDG_RUNTIME_DIR', ''))
    if not runtime.is_absolute():
        raise DesktopError('XDG_RUNTIME_DIR must identify the private local user runtime directory')
    info = runtime.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != uid or info.st_mode & 0o077:
        raise DesktopError('XDG_RUNTIME_DIR must be an owned private directory')
    metadata = {'display': display, 'display_number': number, 'socket_path': str(socket_path),
                'xdg_session_type': session_type, 'wayland_display_present': bool(env.get('WAYLAND_DISPLAY')),
                'xauthority_source': str(authority), 'runtime_directory': str(runtime),
                'authority_match': authority_match,
                'backend': 'XWAYLAND' if session_type == 'wayland' else 'X11'}
    return metadata, cookie


class Xauth(C.Structure):
    _fields_ = [('family', C.c_ushort), ('address_length', C.c_ushort), ('address', C.c_void_p),
                ('number_length', C.c_ushort), ('number', C.c_void_p),
                ('name_length', C.c_ushort), ('name', C.c_void_p),
                ('data_length', C.c_ushort), ('data', C.c_void_p)]


class SecurityAttributes(C.Structure):
    # Header ABI order is timeout THEN trust_level (not the documentation table order).
    _fields_ = [('timeout', C.c_uint), ('trust_level', C.c_uint),
                ('group', C.c_ulong), ('event_mask', C.c_long)]


class XSecurity:
    """Used only in a killable child; a blocked Xlib call cannot trap supervision."""
    def __init__(self, display, cookie):
        xname, ename = C.util.find_library('X11'), C.util.find_library('Xext')
        if not xname or not ename:
            raise DesktopError('Host libX11/libXext unavailable')
        self.x, self.ext = C.CDLL(xname), C.CDLL(ename)
        self.display, self.connection, self.auth_id = display.encode(), None, 0
        self.x.XSetAuthorization.argtypes = [C.c_char_p, C.c_int, C.c_char_p, C.c_int]
        self.x.XSetAuthorization.restype = None
        self.x.XOpenDisplay.argtypes = [C.c_char_p]; self.x.XOpenDisplay.restype = C.c_void_p
        self.x.XCloseDisplay.argtypes = [C.c_void_p]
        self.x.XSync.argtypes = [C.c_void_p, C.c_int]
        self.x.XListHosts.argtypes = [C.c_void_p, C.POINTER(C.c_int), C.POINTER(C.c_int)]
        self.x.XListHosts.restype = C.c_void_p
        self.x.XFree.argtypes = [C.c_void_p]
        self.x.XQueryExtension.argtypes = [C.c_void_p, C.c_char_p,
                                           C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_int)]
        self.ext.XSecurityQueryExtension.argtypes = [C.c_void_p, C.POINTER(C.c_int), C.POINTER(C.c_int)]
        self.ext.XSecurityAllocXauth.restype = C.POINTER(Xauth)
        self.ext.XSecurityAllocXauth.argtypes = []
        self.ext.XSecurityFreeXauth.argtypes = [C.POINTER(Xauth)]
        self.ext.XSecurityFreeXauth.restype = None
        self.ext.XSecurityGenerateAuthorization.argtypes = [C.c_void_p, C.POINTER(Xauth), C.c_ulong,
            C.POINTER(SecurityAttributes), C.POINTER(C.c_ulong)]
        self.ext.XSecurityGenerateAuthorization.restype = C.POINTER(Xauth)
        self.ext.XSecurityRevokeAuthorization.argtypes = [C.c_void_p, C.c_ulong]
        self.x.XSetAuthorization(PROTOCOL, len(PROTOCOL), cookie, len(cookie))
        self.connection = self.x.XOpenDisplay(self.display)
        if not self.connection:
            raise DesktopError('HOST_X11_AUTHENTICATION_FAILED')
        count, enabled, major, minor = C.c_int(), C.c_int(), C.c_int(), C.c_int()
        hosts = self.x.XListHosts(self.connection, C.byref(count), C.byref(enabled))
        if hosts:
            self.x.XFree(hosts)
        if not enabled.value:
            self.close()
            raise DesktopError('HOST_X11_ACCESS_CONTROL_DISABLED')
        if not self.ext.XSecurityQueryExtension(self.connection, C.byref(major), C.byref(minor)):
            self.close()
            raise DesktopError('HOST_X11_SECURITY_EXTENSION_UNAVAILABLE')

    def generate(self):
        request = self.ext.XSecurityAllocXauth()
        if not request:
            raise DesktopError('Cannot allocate X11 authorization request')
        protocol = C.create_string_buffer(PROTOCOL)
        request.contents.name_length = len(PROTOCOL)
        request.contents.name = C.cast(protocol, C.c_void_p)
        attrs = SecurityAttributes(IDLE_AUTH_TIMEOUT, 0, 0, 0)  # Trusted for GLX.
        auth_id = C.c_ulong()
        generated = None
        try:
            generated = self.ext.XSecurityGenerateAuthorization(
                self.connection, request, 3, C.byref(attrs), C.byref(auth_id))
            self.auth_id = auth_id.value
            if not generated or not self.auth_id or generated.contents.data_length != 16:
                raise DesktopError('XSECURITY_GENERATION_FAILED')
            return C.string_at(generated.contents.data, generated.contents.data_length)
        finally:
            if generated:
                self.ext.XSecurityFreeXauth(generated)
            request.contents.name = None
            request.contents.name_length = 0
            self.ext.XSecurityFreeXauth(request)

    def probe(self, cookie):
        self.x.XSetAuthorization(PROTOCOL, len(PROTOCOL), cookie, len(cookie))
        probe = self.x.XOpenDisplay(self.display)
        if not probe:
            raise DesktopError('GENERATED_X11_AUTHENTICATION_FAILED')
        try:
            opcode, event, error = C.c_int(), C.c_int(), C.c_int()
            if not self.x.XQueryExtension(probe, b'GLX', C.byref(opcode), C.byref(event), C.byref(error)):
                raise DesktopError('HOST_X11_GLX_UNAVAILABLE')
            return {'glx_available': True, 'generated_cookie_connection_verified': True,
                    'access_control_enabled': True}
        finally:
            self.x.XCloseDisplay(probe)

    def close(self):
        revoked = self.auth_id == 0
        if self.connection:
            if self.auth_id:
                revoked = bool(self.ext.XSecurityRevokeAuthorization(self.connection, self.auth_id))
                self.x.XSync(self.connection, 0)
            self.x.XCloseDisplay(self.connection)
            self.connection = None
        return revoked


def guardian(runtime, run_id, duration, *, backend=XSecurity, control=None, emit=None):
    control = sys.stdin if control is None else control
    emit = (lambda value: print(json.dumps(value), flush=True)) if emit is None else emit
    connection = None
    stopped = False
    def stop(_signum, _frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    result = {'status': 'DESKTOP_ACCESS_FAILED', 'authorization_revoked': False}
    owns_runtime = False
    try:
        metadata, cookie = inspect_host()
        info = runtime.lstat()
        if (not re.fullmatch('[0-9a-f]{32}', run_id) or not 10 <= duration <= 1800
                or runtime.parent != Path(metadata['runtime_directory'])
                or not runtime.name.startswith('arocs-desktop-' + run_id[:12] + '.')
                or not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o077 or any(runtime.iterdir())):
            raise DesktopError('Invalid private display guardian directory or identity')
        owns_runtime = True
        connection = backend(metadata['display'], cookie)
        generated = connection.generate()
        if generated == cookie:
            raise DesktopError('XSECURITY_DID_NOT_GENERATE_A_DISTINCT_SESSION_COOKIE')
        probe = connection.probe(generated)
        auth = runtime / 'Xauthority'
        descriptor = os.open(auth, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(authority_record(metadata['display_number'], generated))
        metadata.update(probe, status='DESKTOP_AUTHENTICATED', gui_mode='desktop', run_id=run_id,
                        machine='arocs-' + run_id[:12], auth_cookie_path=str(auth), guardian_pid=os.getpid(),
                        authorization_scope='SESSION_COOKIE_TRUSTED_X11',
                        idle_authorization_timeout_s=IDLE_AUTH_TIMEOUT,
                        guardian_hard_lifetime_s=duration + 30,
                        host_authority_mounted=False, motion_authorized=False)
        emit(metadata)
        deadline = time.monotonic() + duration + 30
        while not stopped and time.monotonic() < deadline:
            readable, _, _ = select.select([control], [], [], 0.1)
            if readable and not os.read(control.fileno(), 1024):
                break
        revoked = connection.close()
        connection = None
        result = {'status': 'DESKTOP_ACCESS_REVOKED' if revoked else 'DESKTOP_ACCESS_FAILED',
                  'authorization_revoked': revoked, 'run_id': run_id}
    except BaseException as error:
        result.update(error=f'{type(error).__name__}: {error}', run_id=run_id)
    finally:
        if connection:
            result['authorization_revoked'] = connection.close()
        if owns_runtime:
            try:
                (runtime / 'Xauthority').unlink(missing_ok=True)
                runtime.rmdir()
                result['credentials_removed'] = True
            except OSError as error:
                result.update(status='DESKTOP_ACCESS_FAILED', credentials_removed=False, cleanup_error=str(error))
        emit(result)
    return 0 if result['status'] == 'DESKTOP_ACCESS_REVOKED' else 2


def read_ready(proc, timeout=8):
    deadline, data = time.monotonic() + timeout, b''
    while time.monotonic() < deadline:
        readable, _, _ = select.select([proc.stdout], [], [], max(0, deadline - time.monotonic()))
        if not readable:
            break
        part = os.read(proc.stdout.fileno(), 4096)
        if not part:
            break
        data += part
        if len(data) > 65536:
            raise DesktopError('Excessive display guardian response')
        if b'\n' in data:
            record = json.loads(data.split(b'\n', 1)[0])
            if not isinstance(record, dict):
                raise DesktopError('Invalid display guardian record')
            if record.get('status') != 'DESKTOP_AUTHENTICATED':
                raise DesktopError(record.get('error', 'Desktop authentication failed'))
            return record
    raise DesktopError('DISPLAY_DIAGNOSTIC_TIMEOUT_OR_EXIT')


class DesktopSession:
    def __init__(self, proc, runtime, log, metadata):
        self.proc, self.runtime, self.log, self.metadata = proc, runtime, log, metadata

    @classmethod
    def start(cls, evidence, run_id, duration):
        if not re.fullmatch('[0-9a-f]{32}', run_id) or not 10 <= duration <= 1800:
            raise DesktopError('Invalid desktop session identity or duration')
        metadata, _ = inspect_host()
        runtime = Path(tempfile.mkdtemp(prefix='arocs-desktop-' + run_id[:12] + '.',
                                        dir=metadata['runtime_directory']))
        try:
            log = (evidence / 'display_guardian.log').open('xb')
        except BaseException:
            runtime.rmdir()
            raise
        proc = None
        try:
            proc = subprocess.Popen([sys.executable, '-I', str(Path(__file__).resolve()), '_guardian',
                '--runtime', str(runtime), '--run-id', run_id, '--duration', str(duration)],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, start_new_session=True)
            record = read_ready(proc)
            if record.get('run_id') != run_id or record.get('machine') != 'arocs-' + run_id[:12]:
                raise DesktopError('Display guardian identity mismatch')
            return cls(proc, runtime, log, record)
        except BaseException:
            instance = cls(proc, runtime, log, {'run_id': run_id})
            instance.close()
            raise

    def check_alive(self):
        if self.proc.poll() is not None:
            raise DesktopError('DESKTOP_AUTHORIZATION_GUARDIAN_EXITED')

    def docker_options(self):
        info = self.metadata
        for path in (info['socket_path'], info['auth_cookie_path']):
            if any(c in path for c in (',', '\r', '\n')):
                raise DesktopError('Invalid desktop bind mount path')
        return ['--hostname', info['machine'],
                '--mount', f"type=bind,src={info['socket_path']},dst={info['socket_path']},readonly",
                '--mount', f"type=bind,src={info['auth_cookie_path']},dst={CONTAINER_AUTH},readonly",
                '--env', 'DISPLAY=' + info['display'], '--env', 'XAUTHORITY=' + CONTAINER_AUTH,
                '--env', 'QT_QPA_PLATFORM=xcb', '--env', 'QT_X11_NO_MITSHM=1']

    def close(self):
        result = {'status': 'DESKTOP_ACCESS_FAILED', 'authorization_revoked': False,
                  'run_id': self.metadata.get('run_id')}
        try:
            if self.proc:
                if self.proc.stdin:
                    self.proc.stdin.close()
                    self.proc.stdin = None
                try:
                    output, _ = self.proc.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    output, _ = self.proc.communicate(timeout=1)
                    result['error'] = 'DESKTOP_AUTHORIZATION_REVOCATION_TIMEOUT'
                else:
                    lines = output.splitlines()
                    if lines:
                        record = json.loads(lines[-1])
                        if record.get('run_id') == self.metadata.get('run_id'):
                            result.update(record)
                returncode = self.proc.returncode
                result['guardian_returncode'] = returncode if type(returncode) is int else None
                if type(returncode) is not int or returncode != 0:
                    result['status'] = 'DESKTOP_ACCESS_FAILED'
                    result.setdefault('error', 'DESKTOP_AUTHORIZATION_GUARDIAN_EXIT_FAILED')
        finally:
            # Remove only the exact private directory this instance created.
            (self.runtime / 'Xauthority').unlink(missing_ok=True)
            if self.runtime.exists():
                self.runtime.rmdir()
            result['credentials_removed'] = not self.runtime.exists()
            self.log.close()
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    p = sub.add_parser('diagnose')
    p.add_argument('--output-dir', type=Path, required=True)
    p = sub.add_parser('_guardian', help=argparse.SUPPRESS)
    p.add_argument('--runtime', type=Path, required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--duration', type=float, required=True)
    args = parser.parse_args(argv)
    if args.operation == '_guardian':
        return guardian(args.runtime, args.run_id, args.duration)
    output = args.output_dir.absolute()
    output.mkdir(mode=0o700)  # Exclusive fresh evidence; never overwrite a previous attempt.
    report = {'status': 'DISPLAY_DIAGNOSTIC_FAILED', 'motion_authorized': False,
              'xdg_session_type': os.environ.get('XDG_SESSION_TYPE', ''),
              'display': os.environ.get('DISPLAY', ''),
              'wayland_display_present': bool(os.environ.get('WAYLAND_DISPLAY')),
              'xauthority_source': os.environ.get('XAUTHORITY', ''),
              'runtime_directory': os.environ.get('XDG_RUNTIME_DIR', '')}
    session = None
    try:
        import uuid
        session = DesktopSession.start(output, uuid.uuid4().hex, 10)
        report.update(session.metadata)
        cleanup = session.close()
        session = None
        report['cleanup'] = cleanup
        report['status'] = ('DISPLAY_DIAGNOSTIC_PASS'
                            if cleanup.get('status') == 'DESKTOP_ACCESS_REVOKED'
                            and type(cleanup.get('guardian_returncode')) is int
                            and cleanup['guardian_returncode'] == 0
                            and cleanup.get('authorization_revoked') is True
                            and cleanup.get('credentials_removed') is True else 'DISPLAY_DIAGNOSTIC_FAILED')
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if session:
            report['cleanup'] = session.close()
        (output / 'display_diagnostic.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'DISPLAY_DIAGNOSTIC_PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
