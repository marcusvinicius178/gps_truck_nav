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


def _executable_role(basename):
    if basename in EXECUTABLES:
        return basename
    # gazebo11_11.10.2 gz_install_executable sets VERSION=GAZEBO_VERSION_FULL.
    # CMake installs a versioned executable and unversioned symlink on Linux.
    # This is discovery only; inode/hash and launch-PID checks remain mandatory.
    if re.fullmatch(r'gzserver-11\.[0-9]+\.[0-9]+', basename):
        return 'gzserver'
    return None


def _process_identity(directory):
    """Read Linux stat fields 4/22; comm may contain spaces or parentheses."""
    text = (directory / 'stat').read_text()
    prefix, closing, tail = text.rpartition(')')
    pid, opening, _ = prefix.partition(' (')
    fields = tail.split()
    if not closing or not opening or int(pid) != int(directory.name) or len(fields) < 20:
        raise ValueError('Malformed /proc stat identity')
    parent, started = int(fields[1]), int(fields[19])
    if parent < 0 or started < 0:
        raise ValueError('Invalid process parent/start time')
    return {'parent_pid': parent, 'start_time_ticks': started}


def _launch_evidence(path):
    result = {'status': 'NOT_PROVIDED', 'pids': {},
        'scope': 'PID correlation only; the launch log does not record kernel process start time.'}
    if path is None:
        return result
    result['path'] = str(path)
    try:
        data = Path(path).read_bytes()
        result['sha256'] = hashlib.sha256(data).hexdigest()
        pattern = (r'^\[INFO\] \[(gzserver|component_container_isolated|navsat_transform_node|ekf_node)'
                   r'-[0-9]+\]: process started with pid \[([1-9][0-9]*)\]\s*$')
        for role, pid in re.findall(pattern, data.decode('utf-8'), re.MULTILINE):
            result['pids'].setdefault(role, []).append(int(pid))
        result['pids'] = {role: sorted(set(pids)) for role, pids in result['pids'].items()}
        result['status'] = 'PARSED' if result['pids'] else 'NO_RECOGNIZED_PROCESS_LINES'
    except (OSError, UnicodeError) as error:
        result.update(status='READ_ERROR', error=_error(error))
    return result


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


def inspect_runtime_binaries(proc_root=Path('/proc'), *, launch_log=None):
    """Return identities observed in a bounded set of relevant Linux processes.

    MAPPED_IDENTIFIED means a stable file hash matched the device/inode recorded
    by /proc/maps. It is not ACTUALLY_USED evidence or upstream equivalence.
    librl_lib can be linked statically; executable identity is separate evidence.
    """
    started = time.monotonic()
    result = {'schema_version': 2, 'observed_at_utc': datetime.now(timezone.utc).isoformat(),
        'scope': 'SELECTED_EXECUTABLE_AND_MAPPED_FILE_IDENTITIES_ONLY',
        'collection_status': 'UNAVAILABLE', 'processes': [], 'targets': {}, 'errors': [],
        'executable_inventory': [], 'launch_evidence': _launch_evidence(launch_log),
        'upstream_equivalence_verified': False, 'plugin_consumption_verified': False,
        'limits': ['A matching hash identifies a mapped file, not execution of a particular method.',
                   'The snapshot can race process lifecycle; exited, changed or inaccessible processes remain explicit.',
                   'Missing librl_lib may mean static linkage; inspect navsat/ekf executable identities separately.',
                   'Kernel start time detects PID reuse within this snapshot; a prior launch-log PID match alone cannot exclude earlier reuse.']}
    proc_root = Path(proc_root)
    try:
        directories = sorted((path for path in proc_root.iterdir() if path.name.isdecimal()),
                             key=lambda path: int(path.name))
    except OSError as error:
        result['errors'].append({'operation': 'enumerate_processes', 'error': _error(error)})
        directories = []
    for directory in directories:
        executable_link = directory / 'exe'
        inventory = {'pid': int(directory.name)}
        result['executable_inventory'].append(inventory)
        try:
            executable_path = os.readlink(executable_link)
        except OSError as error:
            # PID may exit during enumeration; never assume an unreadable PID is irrelevant.
            result['errors'].append({'pid': int(directory.name), 'operation': 'read_executable',
                                     'error': _error(error)})
            inventory.update(status='READ_ERROR', error=_error(error))
            continue
        deleted = executable_path.endswith(' (deleted)')
        path = executable_path.removesuffix(' (deleted)') if deleted else executable_path
        executable = Path(path).name
        role = _executable_role(executable)
        inventory.update(status='READ', executable_symlink=executable_path,
                         executable_name=executable, executable_role=role)
        if role is None:
            continue
        process = {'pid': int(directory.name), 'executable_name': executable,
                   'executable_role': role,
                   'process_status': 'OBSERVED', 'maps_status': 'UNAVAILABLE', 'libraries': [], 'errors': []}
        result['processes'].append(process)
        expected_pids = result['launch_evidence']['pids'].get(role, [])
        process['launch_correlation'] = ('NOT_REQUESTED' if launch_log is None else
            'PID_MATCH_ONLY' if process['pid'] in expected_pids else
            'PID_NOT_IN_LAUNCH' if expected_pids else 'EXPECTED_PID_UNAVAILABLE')
        try:
            process['kernel_identity_before'] = _process_identity(directory)
        except (OSError, UnicodeError, ValueError) as error:
            process['process_status'] = 'IDENTITY_UNVERIFIED'
            process['errors'].append({'operation': 'read_process_identity', 'error': _error(error)})
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
            process['libraries'], errors = _mapped_libraries((directory / 'maps').read_text(), role)
            process['errors'].extend(errors)
            process['maps_status'] = 'READ' if not errors else 'PARTIAL'
        except (OSError, UnicodeError) as error:
            process['maps_status'] = 'READ_ERROR'
            process['errors'].append({'operation': 'read_maps', 'error': _error(error)})
        try:
            if os.readlink(executable_link) != executable_path:
                process['process_status'] = 'EXECUTABLE_CHANGED'
            process['kernel_identity_after'] = _process_identity(directory)
            if (process.get('kernel_identity_before') is not None
                    and process['kernel_identity_after']['start_time_ticks'] !=
                        process['kernel_identity_before']['start_time_ticks']):
                process['process_status'] = 'PID_REUSED_DURING_SNAPSHOT'
            if not deleted and process['executable']['status'] == 'IDENTIFIED':
                after = _stat_identity(executable_link.stat())
                if after != process['executable']['file_stat']:
                    process['process_status'] = 'EXECUTABLE_CHANGED'
        except OSError as error:
            process['process_status'] = 'EXITED_OR_INACCESSIBLE'
            process['errors'].append({'operation': 'recheck_executable', 'error': _error(error)})
        except (UnicodeError, ValueError) as error:
            process['process_status'] = 'IDENTITY_UNVERIFIED'
            process['errors'].append({'operation': 'recheck_process_identity', 'error': _error(error)})

    discovered = {process['executable_role'] for process in result['processes']}
    result['required_processes'] = {name: {'status': 'OBSERVED' if name in discovered else 'NOT_OBSERVED',
        'pids': [process['pid'] for process in result['processes'] if process['executable_role'] == name],
        'launch_pids': result['launch_evidence']['pids'].get(name, [])}
        for name in EXECUTABLES}
    incomplete = bool(result['errors']) or (launch_log is not None and result['launch_evidence']['status'] != 'PARSED')
    for process in result['processes']:
        incomplete |= (process['process_status'] != 'OBSERVED' or process['maps_status'] != 'READ'
                       or process['executable']['status'] != 'IDENTIFIED'
                       or process['launch_correlation'] not in ('NOT_REQUESTED', 'PID_MATCH_ONLY')
                       or any(item['status'] != 'IDENTIFIED' for item in process['libraries']))
    for role in LIBRARIES:
        records = [{**item, 'pid': process['pid'], 'executable_name': process['executable_name'],
                    'executable_role': process['executable_role'],
                    'launch_correlation': process['launch_correlation'],
                    'process_status': process['process_status']}
                   for process in result['processes'] for item in process['libraries'] if item['target'] == role]
        identified = records and all(item['status'] == 'IDENTIFIED' and item['process_status'] == 'OBSERVED'
                                     and item['launch_correlation'] in ('NOT_REQUESTED', 'PID_MATCH_ONLY')
                                     for item in records)
        status = ('MAPPED_IDENTIFIED' if identified else 'MAPPED_UNVERIFIED' if records else
                  'DISCOVERY_INCOMPLETE' if incomplete else 'NOT_OBSERVED')
        result['targets'][role] = {'status': status, 'records': records}
    result['collection_status'] = ('UNAVAILABLE' if not result['processes'] else 'PARTIAL'
                                   if incomplete or set(EXECUTABLES) - discovered else 'COMPLETE')
    result['duration_wall_seconds'] = time.monotonic() - started
    return result
