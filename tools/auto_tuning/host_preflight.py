#!/usr/bin/env python3
"""Collect read-only host evidence for the prospective Iron/Gazebo environment.

Python 3 standard library only. No installation, image pull, build, ROS launch,
container start, or historical-data read is performed. Docker daemon queries
require --probe-docker and a verified local Unix-socket endpoint. Exit 0 means
the diagnostic completed, never that Gate 0 or Gate 1 passed.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import re
import shutil
import stat
import subprocess
import sys


OUTPUT_LIMIT = 16384
LFS_HEADER = b'version https://git-lfs.github.com/spec/v1\n'


def run_command(argv, timeout=10, env=None):
    """Run bounded, non-shell commands; preserve failures as evidence."""
    record = {'argv': list(argv), 'timeout_seconds': timeout}
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                                env=env, check=False)
        record.update(returncode=result.returncode, timed_out=False,
                      stdout=result.stdout[:OUTPUT_LIMIT], stderr=result.stderr[:OUTPUT_LIMIT],
                      output_truncated=len(result.stdout) > OUTPUT_LIMIT or len(result.stderr) > OUTPUT_LIMIT)
    except subprocess.TimeoutExpired:
        record.update(returncode=None, timed_out=True, stdout='', stderr='Command timed out',
                      output_truncated=False)
    except OSError as error:
        record.update(returncode=None, timed_out=False, stdout='', stderr=str(error),
                      output_truncated=False)
    return record


def read_text(path):
    try:
        return Path(path).read_text(encoding='utf-8').strip()
    except (OSError, UnicodeError):
        return None


def parse_memory_limit(value):
    """cgroup 'max' and v1's practically-unlimited sentinel are not capacities."""
    try:
        number = int(value)
        return number if 0 < number < 2 ** 60 else None
    except (ValueError, TypeError):
        return None


def parse_cpu_max(value):
    if value is None:
        return None
    fields = value.split()
    if len(fields) != 2 or fields[0] == 'max':
        return None
    try:
        quota, period = (int(item) for item in fields)
        return quota / period if quota > 0 and period > 0 else None
    except ValueError:
        return None


def resource_evidence(disk_path):
    memory = {}
    for line in (read_text('/proc/meminfo') or '').splitlines():
        if line.startswith(('MemTotal:', 'MemAvailable:')):
            key, value = line.split(':', 1)
            try:
                memory[key] = int(value.split()[0]) * 1024
            except (ValueError, IndexError):
                pass
    cpu_quota = parse_cpu_max(read_text('/sys/fs/cgroup/cpu.max'))
    quota_source = '/sys/fs/cgroup/cpu.max'
    if cpu_quota is None:
        quota = read_text('/sys/fs/cgroup/cpu/cpu.cfs_quota_us')
        period = read_text('/sys/fs/cgroup/cpu/cpu.cfs_period_us')
        cpu_quota = parse_cpu_max('{} {}'.format(quota, period))
        quota_source = '/sys/fs/cgroup/cpu/cpu.cfs_{quota,period}_us'
    mem_limit = parse_memory_limit(read_text('/sys/fs/cgroup/memory.max'))
    mem_current = parse_memory_limit(read_text('/sys/fs/cgroup/memory.current'))
    mem_source = '/sys/fs/cgroup/memory.max'
    if mem_limit is None:
        mem_limit = parse_memory_limit(read_text('/sys/fs/cgroup/memory/memory.limit_in_bytes'))
        mem_current = parse_memory_limit(read_text('/sys/fs/cgroup/memory/memory.usage_in_bytes'))
        mem_source = '/sys/fs/cgroup/memory/memory.limit_in_bytes'
    try:
        affinity_count = len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        affinity_count = None
    cpu_caps = [value for value in (os.cpu_count(), affinity_count, cpu_quota) if value is not None]
    memory_caps = [value for value in (memory.get('MemTotal'), mem_limit) if value is not None]
    disk = shutil.disk_usage(disk_path)
    return {
        'cpu_logical_count': os.cpu_count(), 'cpu_affinity_count': affinity_count,
        'cgroup_cpu_quota_cores': cpu_quota,
        'cgroup_cpu_quota_source': quota_source if cpu_quota is not None else None,
        'effective_cpu_upper_bound_cores': min(cpu_caps) if cpu_caps else None,
        'memory_total_bytes': memory.get('MemTotal'),
        'memory_available_bytes': memory.get('MemAvailable'),
        'cgroup_memory_limit_bytes': mem_limit, 'cgroup_memory_current_bytes': mem_current,
        'cgroup_memory_source': mem_source if mem_limit is not None else None,
        'effective_memory_capacity_upper_bound_bytes': min(memory_caps) if memory_caps else None,
        'disk': {'path': str(disk_path), 'total_bytes': disk.total,
                 'used_bytes': disk.used, 'free_bytes': disk.free},
        'limitation': 'Visible cgroup root limits and affinity only; hidden ancestor limits may be stricter. These are capacities, not measured simulator throughput.',
    }


def validate_local_endpoint(endpoint):
    """Reject TCP, SSH and ambiguous Unix URLs before any daemon request."""
    if not isinstance(endpoint, str) or not endpoint.startswith('unix:///'):
        return False
    socket_path = endpoint[len('unix://'):]
    return bool(socket_path and socket_path.startswith('/') and
                not any(character in socket_path for character in ('\n', '\r', '?', '#', '\x00')))


def docker_evidence(probe_daemon=False, environ=None):
    environ = dict(os.environ if environ is None else environ)
    executable = shutil.which('docker')
    result = {'cli_path': executable, 'daemon_probe_requested': probe_daemon,
              'daemon_status': 'NOT_PROBED', 'runtime_verified': False}
    if executable is None:
        result['daemon_status'] = 'CLI_UNAVAILABLE'
        return result
    result['client_version'] = run_command([executable, '--version'])
    result['compose_version'] = run_command([executable, 'compose', 'version'])
    explicit_host = environ.get('DOCKER_HOST', '')
    explicit_context = environ.get('DOCKER_CONTEXT', '')
    result['docker_host_override_present'] = bool(explicit_host)
    result['docker_context_override_present'] = bool(explicit_context)
    # A nonlocal override is refused even when Docker context precedence would
    # otherwise ignore it. Never contact an accidental remote Docker host.
    if explicit_host and not validate_local_endpoint(explicit_host):
        result['daemon_status'] = 'REMOTE_OR_INVALID_HOST_REFUSED'
        return result
    if explicit_host and not explicit_context:
        endpoint = explicit_host
        result['endpoint_source'] = 'DOCKER_HOST'
    else:
        context_cmd = run_command([executable, 'context', 'show'])
        result['context_show'] = context_cmd
        if context_cmd['returncode'] != 0:
            result['daemon_status'] = 'CONTEXT_UNRESOLVED'
            return result
        context = explicit_context or context_cmd['stdout'].strip()
        if not context or '\n' in context or context.startswith('-'):
            result['daemon_status'] = 'CONTEXT_UNRESOLVED'
            return result
        inspect = run_command([executable, 'context', 'inspect', context,
                               '--format', '{{json .Endpoints.docker.Host}}'])
        result['context_name'] = context
        result['endpoint_source'] = 'docker_context'
        if inspect['returncode'] != 0:
            result['daemon_status'] = 'CONTEXT_UNRESOLVED'
            result['context_error'] = inspect['stderr']
            return result
        try:
            endpoint = json.loads(inspect['stdout'])
        except (ValueError, TypeError):
            result['daemon_status'] = 'CONTEXT_UNRESOLVED'
            return result
    if not validate_local_endpoint(endpoint):
        result['daemon_status'] = 'REMOTE_OR_INVALID_CONTEXT_REFUSED'
        return result
    result['local_endpoint'] = endpoint
    if not probe_daemon:
        return result
    socket_path = Path(endpoint[len('unix://'):])
    try:
        is_socket = stat.S_ISSOCK(socket_path.stat().st_mode)
    except OSError:
        is_socket = False
    if not is_socket:
        result['daemon_status'] = 'LOCAL_SOCKET_UNAVAILABLE'
        return result
    clean_env = dict(environ)
    for key in ('DOCKER_HOST', 'DOCKER_CONTEXT', 'DOCKER_TLS_VERIFY', 'DOCKER_CERT_PATH'):
        clean_env.pop(key, None)
    # --host is explicit and Unix-local; --format avoids exposing registry auth
    # configuration or an unnecessary inventory of unrelated containers.
    info = run_command([executable, '--host', endpoint, 'info', '--format',
                        '{"server_version":{{json .ServerVersion}},"os_type":{{json .OSType}},"architecture":{{json .Architecture}},"cpus":{{json .NCPU}},"memory_bytes":{{json .MemTotal}}}'],
                       timeout=15, env=clean_env)
    result['daemon_info'] = info
    result['daemon_status'] = 'LOCAL_DAEMON_RESPONDED' if info['returncode'] == 0 else 'LOCAL_DAEMON_QUERY_FAILED'
    return result


def lfs_pointer(path):
    """Read only the small header, never decode a model or historical bag."""
    try:
        with Path(path).open('rb') as stream:
            header = stream.read(1024)
    except OSError:
        return None
    if not header.startswith(LFS_HEADER):
        return None
    oid = re.search(rb'^oid sha256:([0-9a-f]{64})$', header, re.MULTILINE)
    size = re.search(rb'^size ([0-9]+)$', header, re.MULTILINE)
    return {'sha256': oid.group(1).decode() if oid else None,
            'expected_asset_bytes': int(size.group(1)) if size else None,
            'valid_pointer': bool(oid and size)}


def repository_evidence(repo):
    repo = Path(repo).resolve()
    result = {'path': str(repo), 'exists': repo.is_dir(), 'assets_scanned': 0,
              'unmaterialized_lfs_assets': [], 'asset_read_errors': []}
    if not repo.is_dir():
        result['status'] = 'REPOSITORY_DIRECTORY_MISSING'
        return result
    if shutil.which('git'):
        prefix = ['git', '-c', 'core.fsmonitor=false', '-C', str(repo)]
        result['head'] = run_command(prefix + ['rev-parse', 'HEAD'])
        result['working_tree'] = run_command(prefix + ['status', '--porcelain=v1', '--untracked-files=normal'])
    models = repo / 'src/gps_truck_nav/truck_bringup/models'
    result['model_directory_exists'] = models.is_dir()
    unique_objects = {}
    if models.is_dir():
        for path in sorted(models.rglob('*')):
            if not path.is_file():
                continue
            try:
                path.resolve().relative_to(repo)
            except ValueError:
                result['asset_read_errors'].append({'path': str(path.relative_to(repo)), 'reason': 'symlink_outside_repository_skipped'})
                continue
            result['assets_scanned'] += 1
            pointer = lfs_pointer(path)
            if pointer is not None:
                pointer['path'] = str(path.relative_to(repo))
                result['unmaterialized_lfs_assets'].append(pointer)
                if pointer['valid_pointer']:
                    unique_objects[pointer['sha256']] = pointer['expected_asset_bytes']
    result['unique_missing_lfs_payload_bytes'] = sum(unique_objects.values())
    result['status'] = 'LFS_MATERIALIZATION_REQUIRED' if result['unmaterialized_lfs_assets'] else 'STATIC_INSPECTION_ONLY'
    return result


def collect_report(repo=None, probe_docker=False):
    try:
        os_release = platform.freedesktop_os_release()
    except (AttributeError, OSError):
        os_release = {}
    tools = {}
    for name, version_args in [('git', ['--version']), ('python3', ['--version']),
                               ('gzserver', ['--version']), ('gazebo', ['--version'])]:
        executable = shutil.which(name)
        tools[name] = {'path': executable}
        if executable:
            tools[name]['version'] = run_command([executable] + version_args)
    if tools['git']['path']:
        tools['git']['lfs_version'] = run_command([tools['git']['path'], 'lfs', 'version'])
    tools['ros2'] = {'path': shutil.which('ros2')}
    tools['colcon'] = {'path': shutil.which('colcon')}
    installed_ros = Path('/opt/ros')
    report = {
        'schema_version': 1, 'collected_at_utc': datetime.now(timezone.utc).isoformat(),
        'evidence_level': 'VERIFIED BY EXECUTION — diagnostic only',
        'gate0': 'NOT YET VERIFIED', 'gate1': 'NOT YET VERIFIED',
        'host': {'system': platform.system(), 'kernel': platform.release(),
                 'architecture': platform.machine(),
                 'os_release': {key: os_release.get(key) for key in ('ID', 'VERSION_ID', 'PRETTY_NAME')},
                 'python_version': platform.python_version()},
        'resources': resource_evidence(Path.cwd()), 'tools': tools,
        'ros_distributions_on_host': sorted(p.name for p in installed_ros.iterdir() if p.is_dir()) if installed_ros.is_dir() else [],
        'gpu_device_paths': sorted(str(p) for pattern in ('/dev/dri/renderD*', '/dev/nvidia[0-9]*') for p in Path('/').glob(pattern.lstrip('/'))),
        'docker': docker_evidence(probe_docker),
        'notes': [
            'No software was installed and no image was pulled, built, or started.',
            'Host ROS/Gazebo absence does not block a Docker-based installation.',
            'A local Unix endpoint can still forward to a VM; Docker capacities are recorded separately when queried.',
            'CPU/RAM/disk values do not establish minimum viable simulator resources.',
        ],
    }
    if repo is not None:
        report['repository'] = repository_evidence(repo)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, help='Optional gps_truck_nav checkout for read-only SHA/status/LFS checks')
    parser.add_argument('--probe-docker', action='store_true', help='Query Docker info only through a verified local Unix endpoint; never start containers')
    parser.add_argument('--output', type=Path, help='New JSON file; refuses overwrite. Default: stdout')
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error('Output already exists; choose a new path to preserve prior evidence')
    try:
        report = collect_report(args.repo, args.probe_docker)
        text = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
        if args.output is None:
            sys.stdout.write(text)
        else:
            # Exclusive creation also protects against races after exists().
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(text)
            print('Diagnostic saved: {}. Gate 0 and Gate 1 remain NOT YET VERIFIED.'.format(args.output))
    except (OSError, ValueError) as error:
        print('Preflight diagnostic failed: {}'.format(error), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
