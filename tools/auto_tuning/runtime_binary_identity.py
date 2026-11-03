"""Read-only Linux identities of selected executables and mapped ROS libraries.

Mapping a file proves neither that a particular method ran nor equivalence with
upstream source. No environment, command line, process memory or ROS API is read.
"""

from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import re
import stat
import time


EXECUTABLES = ('gzserver', 'component_container_isolated', 'navsat_transform_node', 'ekf_node')
LIBRARIES = {
    'ackermann_drive': ('libgazebo_ros_ackermann_drive.so', ('gzserver',)),
    'gazebo_joint_state': ('libgazebo_ros_joint_state_publisher.so', ('gzserver',)),
    'smac_hybrid': ('libnav2_smac_planner.so', ('component_container_isolated',)),
    'mppi_controller': ('libmppi_controller.so', ('component_container_isolated',)),
    'robot_localization': ('librl_lib.so', ('navsat_transform_node', 'ekf_node')),
}
MAX_BINARY_BYTES = 256 * 1024 * 1024


def _error(error):
    return {'type': type(error).__name__, 'errno': getattr(error, 'errno', None),
            'message': str(error)}


def _stat_identity(value):
    return {'device_major': os.major(value.st_dev), 'device_minor': os.minor(value.st_dev),
            'inode': value.st_ino, 'size_bytes': value.st_size,
            'mtime_ns': value.st_mtime_ns, 'ctime_ns': value.st_ctime_ns}


def _identify_file(path, device, inode):
    """Open once, match the mapping to that descriptor, and hash stable bytes."""
    result = {'path': str(path), 'status': 'UNVERIFIED', 'sha256': None,
              'expected_device': {'major': device[0], 'minor': device[1]},
              'expected_inode': inode}
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        with os.fdopen(descriptor, 'rb') as stream:
            before = os.fstat(stream.fileno())
            result['file_stat'] = _stat_identity(before)
            if not stat.S_ISREG(before.st_mode):
                result['status'] = 'NOT_REGULAR_FILE'
                return result
            if (os.major(before.st_dev), os.minor(before.st_dev), before.st_ino) != (*device, inode):
                result['status'] = 'MAPPING_IDENTITY_MISMATCH'
                return result
            if before.st_size > MAX_BINARY_BYTES:
                result['status'] = 'SIZE_LIMIT'
                return result
            digest = hashlib.sha256()
            size = 0
            while True:
                block = stream.read(min(1024 * 1024, MAX_BINARY_BYTES + 1 - size))
                if not block:
                    break
                size += len(block)
                if size > MAX_BINARY_BYTES:
                    result['status'] = 'SIZE_LIMIT'
                    return result
                digest.update(block)
            after = os.fstat(stream.fileno())
            result['file_stat_after'] = _stat_identity(after)
            if result['file_stat'] != result['file_stat_after'] or size != before.st_size:
                result['status'] = 'FILE_CHANGED_DURING_READ'
                return result
            result.update(status='IDENTIFIED', sha256=digest.hexdigest())
    except OSError as error:
        result.update(status='FILE_READ_ERROR', error=_error(error))
    return result


def _selected_library(path, executable):
    basename = Path(path).name
    for role, (library, executables) in LIBRARIES.items():
        if executable in executables and re.fullmatch(re.escape(library) + r'(?:\.\d+)*', basename):
            return role
    return None


def _mapped_libraries(text, executable):
    """Deduplicate ELF segments, retaining all mapped identity variants."""
    groups = {}
    errors = []
    for line_number, line in enumerate(text.splitlines(), 1):
        fields = line.split(maxsplit=5)
        if len(fields) < 6:
            continue  # Anonymous mappings have no file path.
        address, permissions, offset, device_text, inode_text, raw_path = fields
        deleted = raw_path.endswith(' (deleted)')
        path = raw_path.removesuffix(' (deleted)') if deleted else raw_path
        role = _selected_library(path, executable)
        if role is None:
            continue
        try:
            device = tuple(int(part, 16) for part in device_text.split(':'))
            inode = int(inode_text)
            start, end = (int(part, 16) for part in address.split('-'))
            int(offset, 16)
            if len(device) != 2 or inode <= 0 or start >= end or not Path(path).is_absolute():
                raise ValueError('Invalid file mapping identity')
        except ValueError as error:
            errors.append({'status': 'MALFORMED_RELEVANT_MAPPING', 'line': line_number,
                           'target': role, 'path': raw_path, 'error': _error(error)})
            continue
        key = (role, path, device, inode, deleted)
        entry = groups.setdefault(key, {'target': role, 'path': path, 'deleted': deleted,
            'mapping_device': {'major': device[0], 'minor': device[1]}, 'mapping_inode': inode,
            'segment_count': 0, 'executable_mapping': False, 'ranges': []})
        entry['segment_count'] += 1
        entry['executable_mapping'] |= 'x' in permissions
        entry['ranges'].append({'address': address, 'permissions': permissions, 'offset': offset})
    result = []
    for (_, path, device, inode, deleted), entry in groups.items():
        if deleted:
            entry.update(status='DELETED_MAPPING', sha256=None)
        else:
            identity = _identify_file(path, device, inode)
            entry.update(identity)
            if identity['status'] == 'IDENTIFIED' and not entry['executable_mapping']:
                entry['status'] = 'NO_EXECUTABLE_MAPPING'
        result.append(entry)
    return result, errors


def inspect_runtime_binaries(proc_root=Path('/proc')):
    """Return identities observed in a bounded set of relevant Linux processes.

    MAPPED_IDENTIFIED means a stable file hash matched the device/inode recorded
    by /proc/maps. It is not ACTUALLY_USED evidence or upstream equivalence.
    librl_lib can be linked statically; executable identity is separate evidence.
    """
    started = time.monotonic()
    result = {'schema_version': 1, 'observed_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'SELECTED_EXECUTABLE_AND_MAPPED_FILE_IDENTITIES_ONLY',
        'collection_status': 'UNAVAILABLE', 'processes': [], 'targets': {}, 'errors': [],
        'upstream_equivalence_verified': False, 'plugin_consumption_verified': False,
        'limits': ['A matching hash identifies a mapped file, not execution of a particular method.',
                   'The snapshot can race process lifecycle; exited, changed or inaccessible processes remain explicit.',
                   'Missing librl_lib may mean static linkage; inspect navsat/ekf executable identities separately.']}
    proc_root = Path(proc_root)
    try:
        directories = sorted((path for path in proc_root.iterdir() if path.name.isdecimal()),
                             key=lambda path: int(path.name))
    except OSError as error:
        result['errors'].append({'operation': 'enumerate_processes', 'error': _error(error)})
        directories = []
    for directory in directories:
        executable_link = directory / 'exe'
        try:
            executable_path = os.readlink(executable_link)
        except OSError as error:
            # PID may exit during enumeration; never assume an unreadable PID is irrelevant.
            result['errors'].append({'pid': int(directory.name), 'operation': 'read_executable',
                                     'error': _error(error)})
            continue
        deleted = executable_path.endswith(' (deleted)')
        path = executable_path.removesuffix(' (deleted)') if deleted else executable_path
        executable = Path(path).name
        if executable not in EXECUTABLES:
            continue
        process = {'pid': int(directory.name), 'executable_name': executable,
                   'process_status': 'OBSERVED', 'maps_status': 'UNAVAILABLE', 'libraries': [], 'errors': []}
        result['processes'].append(process)
        if deleted:
            process['executable'] = {'path': path, 'status': 'DELETED_EXECUTABLE', 'sha256': None}
        else:
            try:
                identity = executable_link.stat()
                process['executable'] = _identify_file(path,
                    (os.major(identity.st_dev), os.minor(identity.st_dev)), identity.st_ino)
            except OSError as error:
                process['executable'] = {'path': path, 'status': 'EXECUTABLE_STAT_ERROR',
                                         'sha256': None, 'error': _error(error)}
        try:
            process['libraries'], errors = _mapped_libraries((directory / 'maps').read_text(), executable)
            process['errors'].extend(errors)
            process['maps_status'] = 'READ' if not errors else 'PARTIAL'
        except (OSError, UnicodeError) as error:
            process['maps_status'] = 'READ_ERROR'
            process['errors'].append({'operation': 'read_maps', 'error': _error(error)})
        try:
            if os.readlink(executable_link) != executable_path:
                process['process_status'] = 'EXECUTABLE_CHANGED'
        except OSError as error:
            process['process_status'] = 'EXITED_OR_INACCESSIBLE'
            process['errors'].append({'operation': 'recheck_executable', 'error': _error(error)})

    discovered = {process['executable_name'] for process in result['processes']}
    result['required_processes'] = {name: {'status': 'OBSERVED' if name in discovered else 'NOT_OBSERVED',
        'pids': [process['pid'] for process in result['processes'] if process['executable_name'] == name]}
        for name in EXECUTABLES}
    incomplete = bool(result['errors'])
    for process in result['processes']:
        incomplete |= (process['process_status'] != 'OBSERVED' or process['maps_status'] != 'READ'
                       or process['executable']['status'] != 'IDENTIFIED'
                       or any(item['status'] != 'IDENTIFIED' for item in process['libraries']))
    for role in LIBRARIES:
        records = [{**item, 'pid': process['pid'], 'executable_name': process['executable_name'],
                    'process_status': process['process_status']}
                   for process in result['processes'] for item in process['libraries'] if item['target'] == role]
        identified = records and all(item['status'] == 'IDENTIFIED' and item['process_status'] == 'OBSERVED'
                                     for item in records)
        status = ('MAPPED_IDENTIFIED' if identified else 'MAPPED_UNVERIFIED' if records else
                  'DISCOVERY_INCOMPLETE' if incomplete else 'NOT_OBSERVED')
        result['targets'][role] = {'status': status, 'records': records}
    result['collection_status'] = ('UNAVAILABLE' if not result['processes'] else 'PARTIAL'
                                   if incomplete or set(EXECUTABLES) - discovered else 'COMPLETE')
    result['duration_wall_seconds'] = time.monotonic() - started
    return result
