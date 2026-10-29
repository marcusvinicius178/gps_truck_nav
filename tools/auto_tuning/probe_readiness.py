#!/usr/bin/env python3
"""Bounded, read-only ROS observations for Phase A. Never sends motion or goals."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import yaml

from readiness_contract import LIFECYCLE_NODES, assess, costmap_bounds, expected_parameters


def stamp(value):
    return value.sec + value.nanosec / 1_000_000_000


def xyz(value):
    return {key: float(getattr(value, key)) for key in ('x', 'y', 'z')}


def quaternion(value):
    return {**xyz(value), 'w': float(value.w)}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Preserve nonfinite evidence as explicit JSON string tokens.
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def write_new(path, value):
    with path.open('x') as stream:
        json.dump(json_safe(value), stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def collect(output, duration, observed, expected):
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from robot_localization.srv import FromLL
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix
    from tf2_ros import Buffer, TransformListener

    share = Path(get_package_share_directory('truck_bringup'))
    canonical = share / 'params/truck_nav2_params.yaml'
    localization = Path('/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml')
    expected.update(expected_parameters(yaml.safe_load(canonical.read_text())))
    localization_config = yaml.safe_load(localization.read_text())
    datum = localization_config['navsat_transform']['ros__parameters']['datum']
    for name, keys in {
        'navsat_transform': ('datum', 'wait_for_datum', 'zero_altitude', 'yaw_offset',
                             'magnetic_declination_radians', 'use_odometry_yaw'),
        'ekf_filter_node_odom': ('world_frame', 'map_frame', 'odom_frame', 'base_link_frame',
                                 'publish_tf', 'two_d_mode', 'odom0', 'imu0', 'imu0_config'),
        'ekf_filter_node_map': ('world_frame', 'map_frame', 'odom_frame', 'base_link_frame',
                                'publish_tf', 'two_d_mode', 'odom0', 'odom1', 'odom1_config'),
    }.items():
        configured = localization_config[name]['ros__parameters']
        expected['/' + name] = {key: configured[key] for key in keys}
    observed['source_files'] = {str(path): digest(path) for path in (canonical, localization)}
    observed['expected_parameters'] = expected
    observed['datum_configuration'] = {'latitude': datum[0], 'longitude': datum[1], 'yaw_radians': datum[2],
                                       'query_altitude_m': 0.0,
                                       'note': 'Third datum value is yaw, not altitude; query uses altitude zero with zero_altitude profile.'}
    obstacle_topics = sorted({value for node, values in expected.items() for key, value in values.items()
                              if 'costmap' in node and key.endswith('.topic')})
    observed['obstacle_source_topics'] = obstacle_topics
    library = Path(get_package_prefix('gazebo_plugins')) / 'lib/libgazebo_ros_ackermann_drive.so'
    observed['ackermann_library'] = {'path': str(library), 'sha256': digest(library)}
    for path in (share / 'models/arocs_truck/model.sdf', share / 'urdf/arocs_truck.urdf'):
        observed['source_files'][str(path)] = digest(path)

    rclpy.init()
    node = rclpy.create_node('phase_a_readiness_' + str(os.getpid()),
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscriptions = []
    jobs = []
    process = None
    process_log = None
    started = time.monotonic()
    clock = observed['clock']

    def clock_callback(message):
        value = stamp(message.clock)
        received_offset = time.monotonic() - started
        if clock['count']:
            clock['forward_steps'] += value > clock['last']
            clock['backward_steps'] += value < clock['last']
            if value > clock['last']:
                clock['last_advance_wall_offset'] = received_offset
        else:
            clock['first'] = value
        clock.update(last=value, count=clock['count'] + 1, last_received_wall_offset=received_offset)

    def summarize(message, kind):
        result = {'frame_id': message.header.frame_id}
        if kind == 'gps':
            result.update(latitude=message.latitude, longitude=message.longitude, altitude=message.altitude,
                          status=message.status.status, position_covariance=list(message.position_covariance),
                          position_covariance_type=message.position_covariance_type)
        elif kind == 'imu':
            result.update(orientation=quaternion(message.orientation), angular_velocity=xyz(message.angular_velocity),
                          linear_acceleration=xyz(message.linear_acceleration))
            for name in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
                result[name] = list(getattr(message, name))
        elif kind == 'odom':
            result.update(child_frame_id=message.child_frame_id, position=xyz(message.pose.pose.position),
                          orientation=quaternion(message.pose.pose.orientation), pose_covariance=list(message.pose.covariance),
                          linear_velocity=xyz(message.twist.twist.linear), angular_velocity=xyz(message.twist.twist.angular))
        elif kind == 'scan':
            finite_ranges = [value for value in message.ranges if math.isfinite(value)]
            result.update(range_count=len(message.ranges), finite_range_count=len(finite_ranges),
                          minimum_finite_range=min(finite_ranges) if finite_ranges else None,
                          range_min=message.range_min, range_max=message.range_max)
        else:
            result.update(width=message.info.width, height=message.info.height, resolution=message.info.resolution,
                          origin_position=xyz(message.info.origin.position),
                          origin_orientation=quaternion(message.info.origin.orientation))
        return result

    def callback(topic, kind):
        def receive(message):
            value = stamp(message.header.stamp)
            entry = observed['topics'].setdefault(topic, {'count': 0, 'forward_steps': 0, 'backward_steps': 0})
            if entry['count']:
                entry['forward_steps'] += value > entry['last_stamp']
                entry['backward_steps'] += value < entry['last_stamp']
            else:
                entry['first_stamp'] = value
            entry.update(count=entry['count'] + 1, last_stamp=value,
                         last_received_wall_offset=time.monotonic() - started, last=summarize(message, kind))
        return receive

    def queue(name, service_type, request, category, key, decode):
        client = node.create_client(service_type, name)
        record = {'status': 'PENDING', 'service': name}
        observed[category][key] = record
        jobs.append({'client': client, 'request': request, 'record': record, 'decode': decode, 'future': None})
        return record

    try:
        subscriptions.append(node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data))
        descriptions = [(NavSatFix, '/gps/fix', 'gps'), (Imu, '/imu', 'imu'), (LaserScan, '/sick_scan_2D', 'scan')]
        descriptions += [(Odometry, name, 'odom') for name in ('/odom', '/odometry/local', '/odometry/global', '/odometry/gps')]
        descriptions += [(OccupancyGrid, '/' + name + '/costmap', 'costmap') for name in ('global_costmap', 'local_costmap')]
        for message_type, topic, kind in descriptions:
            qos = (QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL) if kind == 'costmap' else qos_profile_sensor_data)
            subscriptions.append(node.create_subscription(message_type, topic, callback(topic, kind), qos))
        for target, values in expected.items():
            keys = list(values)
            request = GetParameters.Request(names=keys)
            def decode_parameters(response, names=keys):
                if len(response.values) != len(names):
                    raise ValueError('GetParameters result count differs from request')
                return {'values': dict(zip(names, [parameter_value_to_python(value) for value in response.values]))}
            queue(target + '/get_parameters', GetParameters, request, 'parameters', target, decode_parameters)
        for target in LIFECYCLE_NODES:
            queue('/' + target + '/get_state', GetState, GetState.Request(), 'lifecycle', '/' + target,
                  lambda response: {'id': response.current_state.id, 'label': response.current_state.label})

        def from_ll(key, latitude, longitude, altitude, **metadata):
            request = FromLL.Request()
            request.ll_point.latitude = float(latitude)
            request.ll_point.longitude = float(longitude)
            request.ll_point.altitude = float(altitude)
            record = queue('/fromLL', FromLL, request, 'from_ll', key,
                           lambda response: {'map_point': xyz(response.map_point)})
            record.update(input={'latitude': latitude, 'longitude': longitude, 'altitude': altitude}, **metadata)
        from_ll('datum', datum[0], datum[1], 0.0, datum_yaw_radians=datum[2])
        process_log = (output / 'tf_authority_process.log').open('x')
        try:
            process = subprocess.Popen(['/opt/auto_tuning/bin/tf_authority_probe', '--duration', str(duration),
                                        '--output', str(output / 'tf_authority.json')],
                                       stdout=process_log, stderr=subprocess.STDOUT)
        except OSError as error:
            observed['tf_authority_error'] = str(error)

        while time.monotonic() - started < duration:
            elapsed = time.monotonic() - started
            gps = observed['topics'].get('/gps/fix', {})
            if elapsed >= max(0, duration - 5) and 'live_gps' not in observed['from_ll'] and gps:
                point = gps['last']
                if all(math.isfinite(point[key]) for key in ('latitude', 'longitude', 'altitude')):
                    from_ll('live_gps', point['latitude'], point['longitude'], point['altitude'],
                            input_sensor_stamp=gps['last_stamp'], input_frame_id=point['frame_id'])
            for job in jobs:
                record = job['record']
                if record['status'] != 'PENDING':
                    continue
                try:
                    if job['future'] is None and job['client'].service_is_ready():
                        job['future'] = job['client'].call_async(job['request'])
                    if job['future'] is not None and job['future'].done():
                        record.update(job['decode'](job['future'].result()), status='OK')
                except Exception as error:
                    record.update(status='ERROR', error=str(error))
            rclpy.spin_once(node, timeout_sec=0.05)
        if process is not None:
            try:
                returncode = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
                returncode = process.returncode
            observed['tf_authority_returncode'] = returncode
            if (output / 'tf_authority.json').is_file():
                observed['tf_authority'] = json.loads((output / 'tf_authority.json').read_text())
            if returncode != 0:
                observed['tf_authority']['observation_complete'] = False
        # The C++ window can finish slightly after the Python window. Refresh
        # /clock and buffered samples before comparing its final TF timestamps.
        refresh_deadline = time.monotonic() + 0.5
        while time.monotonic() < refresh_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        observed['observation_window_end_wall_offset'] = time.monotonic() - started
        observed['clock_reference'] = 'Refreshed after the TF authority observation window.'
        for job in jobs:
            if job['record']['status'] == 'PENDING':
                try:
                    if job['future'] is not None and job['future'].done():
                        job['record'].update(job['decode'](job['future'].result()), status='OK')
                    else:
                        job['record']['status'] = 'TIMEOUT'
                except Exception as error:
                    job['record'].update(status='ERROR', error=str(error))
        for topic in sorted(set(obstacle_topics + ['/tf', '/tf_static'])):
            try:
                observed['topic_publishers'][topic] = [
                    {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
                     'gid': bytes(endpoint.endpoint_gid).hex(), 'topic_type': endpoint.topic_type}
                    for endpoint in node.get_publishers_info_by_topic(topic)]
            except Exception as error:
                observed['topic_discovery_errors'][topic] = str(error)

        for parent, child in (('map', 'base_footprint'), ('odom', 'base_footprint'),
                              ('base_footprint', 'base_link'), ('base_footprint', 'gps_link'),
                              ('base_footprint', 'imu_link'), ('map', 'gps_link')):
            key = parent + '->' + child
            try:
                transform = buffer.lookup_transform(parent, child, Time())
                observed['transforms'][key] = {'status': 'OK', 'stamp': stamp(transform.header.stamp),
                    'translation': xyz(transform.transform.translation), 'rotation': quaternion(transform.transform.rotation)}
            except Exception as error:
                observed['transforms'][key] = {'status': 'UNAVAILABLE', 'error': str(error)}
        for name, frame in (('global_costmap', 'map'), ('local_costmap', 'odom')):
            metadata = observed['topics'].get('/' + name + '/costmap', {}).get('last', {})
            pose = observed['transforms'].get(frame + '->base_footprint', {})
            observed['costmap_bounds'][name] = costmap_bounds(metadata, pose, frame)

        gps_odom = observed['topics'].get('/odometry/gps', {})
        for name, position, source_stamp, source_frame, reference_key in (
            ('from_ll_gps_vs_map_gps_link', observed['from_ll'].get('live_gps', {}).get('map_point'),
             observed['from_ll'].get('live_gps', {}).get('input_sensor_stamp'), 'map', 'map->gps_link'),
            ('gps_odometry_vs_map_base_footprint', gps_odom.get('last', {}).get('position'),
             gps_odom.get('last_stamp'), gps_odom.get('last', {}).get('frame_id'), 'map->base_footprint')):
            transform = observed['transforms'].get(reference_key, {})
            reference = transform.get('translation')
            if source_frame == 'map' and position is not None and reference is not None:
                delta = {key: position[key] - reference[key] for key in ('x', 'y', 'z')}
                observed['alignment'][name] = {'delta_m': delta, 'horizontal_distance_m': math.hypot(delta['x'], delta['y']),
                    'source_stamp': source_stamp, 'reference_stamp': transform.get('stamp'), 'comparison_frame': 'map',
                    'assessment': 'REVIEW_REQUIRED', 'note': 'Samples may have different stamps; inspect saved stamps and lever arms. No geometric acceptance threshold assumed.'}
            else:
                observed['alignment'][name] = {'assessment': 'UNAVAILABLE_OR_FRAME_MISMATCH', 'source_frame': source_frame}
        observed['collection_status'] = ('COMPLETE' if all(job['record']['status'] == 'OK' for job in jobs)
                                         and observed['tf_authority'].get('observation_complete') else 'PARTIAL')
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        if process_log is not None:
            process_log.close()
        observed['observed_duration_wall_seconds'] = time.monotonic() - started
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args(argv)
    if not 10 <= args.duration <= 45:
        parser.error('duration must be 10–45 wall seconds')
    output = args.output_directory.resolve(strict=True)
    if not output.is_dir():
        parser.error('output_directory must already exist')
    for name in ('readiness.json', 'readiness_observations.json', 'tf_authority.json', 'tf_authority_process.log'):
        if (output / name).exists() or (output / name).is_symlink():
            parser.error('Refusing to replace earlier evidence: ' + name)
    observed = {'schema_version': 1, 'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'requested_duration_wall_seconds': args.duration, 'collection_status': 'FAILED',
        'clock': {'count': 0, 'forward_steps': 0, 'backward_steps': 0}, 'topics': {},
        'parameters': {}, 'lifecycle': {}, 'from_ll': {}, 'transforms': {}, 'tf_authority': {},
        'topic_publishers': {}, 'topic_discovery_errors': {}, 'costmap_bounds': {}, 'alignment': {},
        'nonfinite_json_encoding': 'Nonfinite numbers are preserved as string tokens nan/inf/-inf.'}
    expected = {}
    try:
        collect(output, args.duration, observed, expected)
    except Exception as error:
        observed.update(collection_status='FAILED', error=f'{type(error).__name__}: {error}')
    observed['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    try:
        result = assess(observed, expected)
    except Exception as error:
        observed.update(collection_status='FAILED', assessment_error=f'{type(error).__name__}: {error}')
        result = {'schema_version': 1, 'collection_status': 'FAILED', 'diagnostic_status': 'INCOMPLETE',
                  'checks': [{'name': 'assessment_completed', 'status': 'FAIL', 'error': observed['assessment_error']}],
                  'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    write_new(output / 'readiness_observations.json', observed)
    write_new(output / 'readiness.json', result)
    print(json.dumps({key: result[key] for key in ('collection_status', 'diagnostic_status', 'mission_ready', 'gate0', 'gate1')}))
    return 0 if observed['collection_status'] != 'FAILED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
