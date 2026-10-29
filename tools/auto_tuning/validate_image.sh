#!/usr/bin/env bash
# Image smoke checks only. Run inside the newly built container after /entrypoint.sh.
# No simulator, ROS node, mission, recorder or historical data is started/read.
# All temporary paths are under /tmp, permitting a read-only container rootfs.
set -Eeuo pipefail
if [[ $# -ne 0 ]]; then
  echo 'Usage: /entrypoint.sh bash /opt/auto_tuning/tools/validate_image.sh' >&2
  exit 2
fi
export PYTHONDONTWRITEBYTECODE=1
exec python3 - <<'PY'
import datetime
import importlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

checks = []


def add(name, passed, **details):
    checks.append({'name': name, 'passed': bool(passed), **details})


def output_text(value):
    # TimeoutExpired may contain bytes even when subprocess.run uses text=True.
    return value.decode('utf-8', errors='replace') if isinstance(value, bytes) else value or ''


def command(name, argv, timeout=20):
    try:
        completed = subprocess.run(argv, capture_output=True, text=True,
                                   timeout=timeout, check=False)
        record = {'name': name, 'argv': argv, 'returncode': completed.returncode,
                  'stdout': completed.stdout, 'stderr': completed.stderr,
                  'timed_out': False}
    except subprocess.TimeoutExpired as error:
        record = {'name': name, 'argv': argv, 'returncode': None,
                  'stdout': output_text(error.stdout), 'stderr': output_text(error.stderr),
                  'timed_out': True, 'error': 'Command timed out'}
    except OSError as error:
        record = {'name': name, 'argv': argv, 'returncode': None,
                  'stdout': '', 'stderr': str(error), 'timed_out': False}
    record['process_succeeded'] = record['returncode'] == 0
    record['passed'] = record['process_succeeded']
    checks.append(record)
    return record


def gazebo_version_check():
    record = command('gazebo_version', ['gzserver', '--version'])
    # Classic 11.10.2 Server::ParseArgs prints the version and returns false;
    # server_main.cc returns -1 (255 on Linux), before server->Run().
    # This exception belongs ONLY to this version query, not command() generally.
    # https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/Server.cc
    # https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/server_main.cc
    banner = re.compile(r'Gazebo multi-robot simulator, version ([0-9]+\.[0-9]+\.[0-9]+)')
    versions = {match.group(1) for stream in (record['stdout'], record['stderr'])
                for line in stream.splitlines() if (match := banner.fullmatch(line.strip()))}
    version = next(iter(versions)) if len(versions) == 1 else None
    major_11 = version is not None and version.split('.')[0] == '11'
    accepted = (not record['timed_out'] and version is not None and
                (record['returncode'] == 0 or (record['returncode'] == 255 and major_11)))
    record.update(
        passed=accepted, detected_version=version, version_identified=accepted,
        accepted_nonzero_exit=accepted and record['returncode'] == 255,
        validation_scope='VERSION_IDENTIFICATION_ONLY', simulator_runtime_verified=False)
    add('gazebo_major_11', accepted and major_11, detected_version=version,
        observed=(record['stdout'] + '\n' + record['stderr']).strip())
    return record


with tempfile.TemporaryDirectory(prefix='nav2-image-smoke-', dir='/tmp') as temporary:
    os.environ['HOME'] = temporary
    os.environ['ROS_LOG_DIR'] = str(Path(temporary) / 'ros_logs')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    try:
        values = {}
        for line in Path('/etc/os-release').read_text().splitlines():
            if '=' in line and not line.startswith('#'):
                key, value = line.split('=', 1)
                values[key] = value.strip('"')
        add('ubuntu_22_04', values.get('ID') == 'ubuntu' and values.get('VERSION_ID') == '22.04',
            os_release=values)
    except OSError as error:
        add('ubuntu_22_04', False, reason=str(error))
    add('ros_distro_iron', os.environ.get('ROS_DISTRO') == 'iron',
        observed=os.environ.get('ROS_DISTRO'))
    for executable in ('ros2', 'gzserver', 'xvfb-run', 'Xvfb', 'xauth', 'ldd'):
        add('executable_' + executable, shutil.which(executable) is not None,
            path=shutil.which(executable))

    gazebo_version_check()

    # These checks do not call rclpy.init() or create DDS participants.
    for module in ('rclpy', 'rclpy.serialization', 'rosbag2_py', 'rosidl_runtime_py.utilities',
                   'numpy', 'yaml', 'nav_msgs.msg', 'geometry_msgs.msg', 'sensor_msgs.msg',
                   'tf2_msgs.msg', 'rosgraph_msgs.msg', 'nav2_msgs.action', 'robot_localization.srv'):
        try:
            imported = importlib.import_module(module)
            add('import_' + module, True, source=getattr(imported, '__file__', None))
        except Exception as error:
            add('import_' + module, False, reason=f'{type(error).__name__}: {error}')
    try:
        import rosbag2_py
        readers = sorted(rosbag2_py.get_registered_readers())
        writers = sorted(rosbag2_py.get_registered_writers())
        add('mcap_storage_registered', 'mcap' in readers and 'mcap' in writers,
            readers=readers, writers=writers,
            api_source='ros2/rosbag2@4a1997280d03bc8db96259adfb360b867d3d4fc0')
    except Exception as error:
        add('mcap_storage_registered', False, reason=f'{type(error).__name__}: {error}')

    prefixes = {}
    for package in ('truck_bringup', 'nav2_core', 'nav2_smac_planner', 'nav2_mppi_controller',
                    'nav2_bt_navigator', 'nav2_velocity_smoother', 'robot_localization',
                    'gazebo_ros', 'gazebo_plugins', 'rosbag2_storage_mcap'):
        result = command('package_' + package, ['ros2', 'pkg', 'prefix', package])
        if result['passed']:
            prefixes[package] = Path(result['stdout'].strip())

    if 'truck_bringup' in prefixes:
        prefix = prefixes['truck_bringup']
        add('truck_overlay_prefix', str(prefix).startswith('/ws/install/'), observed=str(prefix))
        plugin = prefix / 'lib/libfront_cabin_goal_checker.so'
        add('front_cabin_library_present', plugin.is_file(), path=str(plugin))
        if plugin.is_file():
            linked = command('front_cabin_linked_libraries', ['ldd', str(plugin)])
            add('front_cabin_no_missing_linked_libraries', linked['passed'] and
                'not found' not in linked['stdout'] + linked['stderr'])
        try:
            xml = prefix / 'share/truck_bringup/front_cabin_plugin.xml'
            root = ET.parse(xml).getroot()
            classes = [entry.attrib for entry in root.findall('class')]
            add('front_cabin_plugin_declaration', root.get('path') == 'front_cabin_goal_checker' and
                any(entry.get('name') == 'truck_bringup/FrontCabinGoalChecker' and
                    entry.get('base_class_type') == 'nav2_core::GoalChecker' for entry in classes),
                path=str(xml), classes=classes)
        except (OSError, ET.ParseError) as error:
            add('front_cabin_plugin_declaration', False, reason=str(error))
    else:
        add('front_cabin_library_present', False, reason='truck_bringup prefix unavailable')

    if 'gazebo_plugins' in prefixes:
        ackermann = prefixes['gazebo_plugins'] / 'lib/libgazebo_ros_ackermann_drive.so'
        add('gazebo_ackermann_library_present', ackermann.is_file(), path=str(ackermann))
        if ackermann.is_file():
            linked = command('gazebo_ackermann_linked_libraries', ['ldd', str(ackermann)])
            add('gazebo_ackermann_no_missing_linked_libraries', linked['passed'] and
                'not found' not in linked['stdout'] + linked['stderr'])

    launch = command('launch_show_args', ['ros2', 'launch', 'truck_bringup',
                                        'gps_waypoint_follower.launch.py', '--show-args'], timeout=30)
    arguments = launch['stdout'] + launch['stderr']
    for name in ('use_gzclient', 'use_rviz', 'use_mapviz', 'use_xcb',
                 'publish_legacy_cloud_tf', 'localization_params_file'):
        add('launch_argument_' + name, launch['passed'] and name in arguments)
    manifest = Path('/ws/installed-packages.tsv')
    add('installed_package_manifest', manifest.is_file() and manifest.stat().st_size > 0,
        path=str(manifest))
    # Gazebo's Debian package name varies (gazebo/gazebo11); the executable's
    # major version is checked above and the complete dpkg manifest is retained.
    command('key_installed_versions', ['dpkg-query', '-W', 'ros-iron-gazebo-ros',
                                      'ros-iron-gazebo-plugins', 'ros-iron-navigation2',
                                      'ros-iron-nav2-mppi-controller', 'ros-iron-robot-localization',
                                      'ros-iron-rosbag2-py', 'ros-iron-rosbag2-storage-mcap'])

passed = all(check['passed'] for check in checks)
print(json.dumps({
    'schema_version': 1,
    'status': 'IMAGE_SMOKE_VALIDATED' if passed else 'IMAGE_SMOKE_FAILED',
    'completed_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED', 'checks': checks,
    'limits': [
        'Gazebo version identification may accept exit 255 with an exact Classic 11 banner; the original code/output remain evidence and do not prove simulator operation.',
        'No Gazebo server or Nav2 node was launched; --show-args only parsed the launch description.',
        'Library presence/ldd and plugin XML do not prove pluginlib loading or correct runtime semantics.',
        'MCAP registration does not prove recording/reading a real bag.',
        'Xvfb executable presence does not prove camera/software-rendering operation.',
        'Ackermann yaw-rate/steering, TF/localization, sensors, BT and mission/reference blockers remain open.'
    ]
}, ensure_ascii=False))
raise SystemExit(0 if passed else 1)
PY
