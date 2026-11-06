"""Wall-clock supervision shared by the host and isolated commissioning session.

These are simulator containment mechanisms, not certified brakes. No ROS import
or publisher exists here. Docker's default time namespace shares CLOCK_MONOTONIC
with the host; incompatible timestamps fail closed instead of being adjusted.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time


HOST_LEASE_TIMEOUT = 0.75
PROCESS_HEARTBEAT_TIMEOUT = 0.75
STARTUP_TIMEOUT = 90.0
EXECUTION_WALL_LIMIT = 60.0
PREPARE_WALL_LIMIT = 180.0
ROLE = 'commissioning'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_private(path, value):
    """Replace only a session-owned artifact, retaining the bind parent's owner."""
    path = Path(path)
    parent = path.parent.stat()
    if path.is_symlink():
        raise ValueError('Refuse symbolic evidence target')
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        current = os.fstat(fd)
        if (current.st_uid, current.st_gid) != (parent.st_uid, parent.st_gid):
            os.fchown(fd, parent.st_uid, parent.st_gid)
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write(value if isinstance(value, str) else json.dumps(value, indent=2, allow_nan=False) + '\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_lease(path, now=None, timeout=HOST_LEASE_TIMEOUT):
    try:
        path = Path(path)
        if path.is_symlink() or not path.is_file() or path.stat().st_size > 128:
            return False
        stamp = float(path.read_text())
        now = time.monotonic() if now is None else now
        return math.isfinite(stamp) and 0 <= now - stamp < timeout
    except (OSError, ValueError):
        return False


class HeartbeatWatch:
    """Require a matching run, advancing sequence and recent monotonic stamp."""

    def __init__(self, run_id, timeout=PROCESS_HEARTBEAT_TIMEOUT):
        self.run_id, self.timeout = run_id, timeout
        self.sequence = self.stamp = self.changed_at = None

    def check(self, record, now):
        if not isinstance(record, dict) or record.get('run_id') != self.run_id:
            raise ValueError('HEARTBEAT_RUN_ID_MISMATCH')
        stamp, sequence = record.get('monotonic_s'), record.get('seq')
        if (not isinstance(stamp, (float, int)) or isinstance(stamp, bool)
                or not math.isfinite(stamp) or not 0 <= now - stamp < self.timeout
                or type(sequence) is not int or sequence < 0):
            raise ValueError('HEARTBEAT_TIMESTAMP_OR_SEQUENCE_INVALID')
        if self.sequence is not None:
            if sequence < self.sequence or stamp < self.stamp:
                raise ValueError('HEARTBEAT_REGRESSION')
            if sequence == self.sequence and stamp != self.stamp:
                raise ValueError('HEARTBEAT_STAMP_WITHOUT_PROGRESS')
            if sequence > self.sequence and stamp <= self.stamp:
                raise ValueError('HEARTBEAT_SEQUENCE_WITHOUT_TIME_PROGRESS')
        if self.sequence is None or sequence > self.sequence:
            self.changed_at = now
        if now - self.changed_at >= self.timeout:
            raise ValueError('HEARTBEAT_NOT_ADVANCING')
        self.sequence, self.stamp = sequence, stamp
        return record


def read_heartbeat(path, watch, now=None):
    path = Path(path)
    if path.is_symlink() or path.stat().st_size > 65536:
        raise ValueError('Invalid heartbeat file')
    record = json.loads(path.read_text())
    # Observe the clock after the interprocess snapshot; a concurrent writer may
    # legitimately have published after a caller's earlier loop timestamp.
    return watch.check(record, time.monotonic() if now is None else now)


def verify_container(info, state):
    labels = info.get('Config', {}).get('Labels', {}) or {}
    if (not re.fullmatch('[0-9a-f]{64}', state.get('container_id', ''))
            or info.get('Id') != state['container_id']
            or info.get('Image') != state['image_id']
            or labels.get('project') != 'nav2-phase-a'
            or labels.get('nav2.phase_a.run_id') != state['run_id']
            or labels.get('nav2.phase_a.role') != ROLE
            or info.get('HostConfig', {}).get('NetworkMode') != 'none'):
        raise ValueError('Container identity/ownership/isolation mismatch')


def verify_authorization(record, manifest, *, now_unix=None):
    """An explicit, short-lived operator record; never manufactured approval."""
    now_unix = time.time() if now_unix is None else now_unix
    exact = {'schema_version': 1, 'scope': 'SIMULATION_COMMISSIONING',
             'motion_authorized': True, 'area_confirmed_clear': True,
             'run_id': manifest['run_id'], 'source_commit': manifest['source']['commit'],
             'image_id': manifest['image_id'], 'model_sha256': manifest['model_sha256'],
             'protocol_sha256': manifest['protocol_sha256'], 'navigation_profile': 'candidate20kmh'}
    if (type(record.get('schema_version')) is not int
            or any(record.get(key) is not True for key in ('motion_authorized', 'area_confirmed_clear'))
            or any(record.get(key) != value for key, value in exact.items())):
        raise ValueError('Authorization must bind this exact prepared run, image, model and protocol')
    expiry = record.get('expires_at_unix')
    if (not isinstance(expiry, (int, float)) or isinstance(expiry, bool)
            or not math.isfinite(expiry) or not 0 < expiry - now_unix <= 3600):
        raise ValueError('Authorization needs an explicit expiry within one hour')
    if (record.get('trial') not in ('straight', 'left', 'right')
            or any(not isinstance(record.get(k), str) or not record[k].strip()
                   for k in ('operator', 'authorization_reference'))):
        raise ValueError('Explicit trial, operator and authorization reference required')
    if record['trial'] != 'straight' and record.get('straight_trial_review_accepted') is not True:
        raise ValueError('Curves require a separately accepted straight-trial review')
    return record
