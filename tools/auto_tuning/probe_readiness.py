#!/usr/bin/env python3
"""Bounded, read-only ROS observations for Phase A. Never sends motion or goals."""

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import yaml

from readiness_contract import LIFECYCLE_NODES, assess, costmap_bounds, expected_parameters, candidate_expected_parameters
from localization_alignment import analyze_alignment
from parameter_probe import PluginParameterProbe
from plugin_parameter_contract import installed_plugin_provenance, parse_configuration_logs, assess_plugin_parameters
from runtime_binary_identity import inspect_runtime_binaries
from audit_ackermann_geometry import audit_model
from prepare_navigation_profiles import load_selected_profile
from speed_chain_evidence import speed_chain_evidence


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


def finish_plugin_capture(observed, plugin_probe, histories):
    """Retain partial readbacks even when another observer or ROS callback fails."""
    observed['localization_sample_history'] = {name: list(samples) for name, samples in histories.items()}
    try:
        observed['plugin_parameters'] = plugin_probe.finish()
    except Exception as error:
        observed['plugin_parameters'] = plugin_probe.report
        observed['plugin_parameters']['finalization_error'] = f'{type(error).__name__}: {error}'
        observed['plugin_parameters']['collection_status'] = 'PARTIAL'
        observed['collection_status'] = 'FAILED'


def collect(output, duration, observed, expected):
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.clock import ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from robot_localization.srv import FromLL
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState, LaserScan, NavSatFix, PointCloud2
    from tf2_ros import Buffer, TransformListener

    share = Path(get_package_share_directory('truck_bringup'))
    canonical = share / 'params/truck_nav2_params.yaml'
    localization = Path('/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml')
    canonical_config = yaml.safe_load(canonical.read_text())
    profile_name = os.environ.get('NAV2_PHASE_A_PROFILE', 'canonical')
    candidate_selected = profile_name in ('candidate20kmh', 'candidate25kmh')
    if profile_name not in ('canonical', '20kmh', '25kmh', 'candidate20kmh', 'candidate25kmh'):
        raise ValueError('Unknown NAV2_PHASE_A_PROFILE')
    profile_directory = Path('/opt/auto_tuning/config/navigation_profiles')
    if candidate_selected:
        from prepare_plant_candidate import load_selected_profile as load_plant_profile
        profile_directory = Path('/opt/auto_tuning/config/plant_candidate')
        canonical_config, profile_entry = load_plant_profile(canonical, profile_directory, profile_name)
        observed['plant_candidate'] = json.loads((profile_directory / 'manifest.json').read_text())
    elif profile_name != 'canonical':
        canonical_config, profile_entry = load_selected_profile(canonical, profile_directory, profile_name)
    else:
        profile_entry = {'scope': 'Canonical configuration readback; historical behavior not inferred from YAML.'}
    observed['navigation_profile'] = {'name': profile_name, 'source': str(canonical),
        'source_sha256': digest(canonical), 'entry': profile_entry,
        'scope': 'PARAMETER_VERIFICATION_ONLY', 'motion_authorized': False}
    expected.update((candidate_expected_parameters if candidate_selected else expected_parameters)(canonical_config))
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
    if profile_name != 'canonical':
        profile_file = (profile_directory / 'navigation' / (profile_name + '.yaml') if candidate_selected
                        else profile_directory / (profile_name + '.yaml'))
        for path in (profile_file, profile_directory / 'manifest.json'):
            observed['source_files'][str(path)] = digest(path)
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
    profile = (profile_directory / 'manifest.json' if candidate_selected
               else Path('/opt/auto_tuning/config/tf_sensor_model_variant.json'))
    observed['model_variant'] = json.loads(profile.read_text())
    observed['source_files'][str(profile)] = digest(profile)
    for relative in observed['model_variant']['variant_sha256']:
        path = Path('/opt/auto_tuning/config') / relative
        observed['source_files'][str(path)] = digest(path)
    observed['runtime_binary_identity'] = inspect_runtime_binaries(launch_log=output.parent / 'bringup_before_probe.log')
    observed['plugin_provenance'] = installed_plugin_provenance()
    observed['plugin_provenance']['runtime_binary_identity'] = observed['runtime_binary_identity']
    model = (profile_directory / 'models/arocs_truck_candidate/model.sdf' if candidate_selected
             else profile.parent / 'models/arocs_truck_tf_sensors/model.sdf')
    observed['ackermann_geometry'] = audit_model(
        model,
        canonical_config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'])
    observed['ackermann_geometry']['requested_radius_source'] = 'Selected navigation YAML; effective values are queried separately.'

    rclpy.init()
    node = rclpy.create_node('phase_a_readiness_' + str(os.getpid()),
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    plugin_probe = PluginParameterProbe(node, canonical_config)
    observed['plugin_parameters'] = plugin_probe.report
    histories = {name: deque(maxlen=128) for name in ('/gps/fix', '/odometry/gps')}
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
                          nan_range_count=sum(math.isnan(value) for value in message.ranges),
                          positive_infinite_range_count=sum(value == math.inf for value in message.ranges),
                          invalid_range_count=sum(value == -math.inf or
                              (math.isfinite(value) and not message.range_min <= value <= message.range_max)
                              for value in message.ranges),
                          minimum_finite_range=min(finite_ranges) if finite_ranges else None,
                          range_min=message.range_min, range_max=message.range_max)
        elif kind == 'joints':
            result.update(names=list(message.name), positions=list(message.position),
                          velocities=list(message.velocity), efforts=list(message.effort))
        elif kind == 'cloud':
            result.update(width=message.width, height=message.height, point_step=message.point_step,
                          row_step=message.row_step, is_dense=message.is_dense,
                          data_bytes=len(message.data),
                          fields=[{'name': field.name, 'offset': field.offset,
                                   'datatype': field.datatype, 'count': field.count} for field in message.fields])
        else:
            result.update(width=message.info.width, height=message.info.height, resolution=message.info.resolution,
                          origin_position=xyz(message.info.origin.position),
                          origin_orientation=quaternion(message.info.origin.orientation))
        return result

    def callback(topic, kind):
        def receive(message):
            value = stamp(message.header.stamp)
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            entry = observed['topics'].setdefault(topic, {'count': 0, 'forward_steps': 0, 'backward_steps': 0})
            if entry['count']:
                entry['forward_steps'] += value > entry['last_stamp']
                entry['backward_steps'] += value < entry['last_stamp']
            else:
                entry['first_stamp'] = value
            entry.update(count=entry['count'] + 1, last_stamp=value,
                         last_stamp_ns=stamp_ns,
                         last_received_wall_offset=time.monotonic() - started, last=summarize(message, kind))
            if topic in histories:
                histories[topic].append({'stamp_ns': stamp_ns,
                    'received_wall_offset': entry['last_received_wall_offset'], **entry['last']})
        return receive

    def queue(name, service_type, request, category, key, decode):
        client = node.create_client(service_type, name)
        record = {'status': 'PENDING', 'service': name}
        observed[category][key] = record
        jobs.append({'client': client, 'request': request, 'record': record, 'decode': decode, 'future': None})
        return record

    try:
        subscriptions.append(node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data))
        descriptions = [(NavSatFix, '/gps/fix', 'gps'), (Imu, '/imu', 'imu'),
                        (LaserScan, '/sick_scan_2D', 'scan'), (JointState, '/joint_states', 'joints'),
                        (PointCloud2, '/external_area_pointcloud', 'cloud')]
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
            record.update(input={'latitude': latitude, 'longitude': longitude, 'altitude': altitude},
                          output_frame_id='map', output_frame_source='Configured global odometry/map frame',
                          service_accepts_timestamp=False, **metadata)
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
                            input_sensor_stamp=gps['last_stamp'], input_sensor_stamp_ns=gps['last_stamp_ns'],
                            input_received_wall_offset=gps['last_received_wall_offset'], input_frame_id=point['frame_id'])
            plugin_probe.poll()
            for job in jobs:
                record = job['record']
                if record['status'] != 'PENDING':
                    continue
                try:
                    if job['future'] is None and job['client'].service_is_ready():
                        job['future'] = job['client'].call_async(job['request'])
                        record['request_sent_wall_offset'] = time.monotonic() - started
                    if job['future'] is not None and job['future'].done():
                        record.update(job['decode'](job['future'].result()), status='OK',
                                      response_received_wall_offset=time.monotonic() - started)
                except Exception as error:
                    record.update(status='ERROR', error=str(error))
            rclpy.spin_once(node, timeout_sec=0.05)
        # Close the RPC window before waiting for the independent C++ observer.
        # A late reply must not become valid merely because process.wait delayed us.
        observed['plugin_parameters'] = plugin_probe.finish()
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
        # Refresh Python sensor/TF snapshots only. TF freshness uses exclusively
        # the C++ observer's own clock; parameter RPCs are already closed.
        refresh_deadline = time.monotonic() + 0.5
        while time.monotonic() < refresh_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        observed['observation_window_end_wall_offset'] = time.monotonic() - started
        observed['clock_reference'] = 'Python sensor snapshot refreshed after C++; TF timing uses the independent C++ clock.'
        for job in jobs:
            if job['record']['status'] == 'PENDING':
                try:
                    if job['future'] is not None and job['future'].done():
                        job['record'].update(job['decode'](job['future'].result()), status='OK')
                    else:
                        job['record']['status'] = 'TIMEOUT'
                except Exception as error:
                    job['record'].update(status='ERROR', error=str(error))
        observed['plugin_parameters'] = plugin_probe.finish()
        observed['plugin_parameters']['provenance'] = observed['plugin_provenance']
        configure_log = output.parent / 'bringup_before_probe.log'
        if configure_log.is_file():
            observed['plugin_parameters']['configure_log_observations'] = parse_configuration_logs(configure_log.read_text())
            observed['plugin_parameters']['configure_log_sha256'] = digest(configure_log)
        else:
            observed['plugin_parameters']['configure_log_error'] = 'No frozen bringup log before this probe.'
        observed['plugin_parameters']['assessment'] = assess_plugin_parameters(observed['plugin_parameters'])
        command_topics = ['/actuation/steering_command'] if candidate_selected else []
        for topic in sorted(set(obstacle_topics + command_topics + ['/tf', '/tf_static', '/joint_states', '/cmd_vel_nav', '/cmd_vel'])):
            try:
                observed['topic_publishers'][topic] = [
                    {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
                     'gid': bytes(endpoint.endpoint_gid).hex(), 'topic_type': endpoint.topic_type}
                    for endpoint in node.get_publishers_info_by_topic(topic)]
                observed['topic_subscribers'][topic] = [
                    {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
                     'gid': bytes(endpoint.endpoint_gid).hex(), 'topic_type': endpoint.topic_type}
                    for endpoint in node.get_subscriptions_info_by_topic(topic)]
            except Exception as error:
                observed['topic_discovery_errors'][topic] = str(error)

        transforms = [('map', 'base_footprint'), ('odom', 'base_footprint'),
                              ('base_footprint', 'base_link'), ('base_footprint', 'gps_link'),
                              ('base_footprint', 'imu_link'), ('map', 'gps_link')]
        if candidate_selected:
            transforms += [('base_footprint', 'rear_axle_nav'), ('map', 'rear_axle_nav'), ('odom', 'rear_axle_nav')]
        for parent, child in transforms:
            key = parent + '->' + child
            try:
                transform = buffer.lookup_transform(parent, child, Time())
                observed['transforms'][key] = {'status': 'OK', 'stamp': stamp(transform.header.stamp),
                    'translation': xyz(transform.transform.translation), 'rotation': quaternion(transform.transform.rotation)}
            except Exception as error:
                observed['transforms'][key] = {'status': 'UNAVAILABLE', 'error': str(error)}
        for name, frame in (('global_costmap', 'map'), ('local_costmap', 'odom')):
            metadata = observed['topics'].get('/' + name + '/costmap', {}).get('last', {})
            base = canonical_config[name][name]['ros__parameters']['robot_base_frame']
            pose = observed['transforms'].get(frame + '->' + base, {})
            observed['costmap_bounds'][name] = costmap_bounds(metadata, pose, frame)

        observed['localization_sample_history'] = {name: list(samples) for name, samples in histories.items()}
        live_fix = observed['from_ll'].get('live_gps', {})
        target_ns = live_fix.get('input_sensor_stamp_ns')
        aligned_tf = {}
        if isinstance(target_ns, int) and target_ns > 0:
            for parent, child in (('map', 'gps_link'), ('map', 'base_footprint'), ('base_footprint', 'gps_link')):
                key = parent + '->' + child
                record = {'parent': parent, 'child': child, 'requested_stamp_ns': target_ns,
                          'lookup_mode': 'EXACT', 'is_static': parent == 'base_footprint'}
                try:
                    transform = buffer.lookup_transform(parent, child,
                        Time(nanoseconds=target_ns, clock_type=ClockType.ROS_TIME))
                    record.update(status='OK',
                        stamp_ns=transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec,
                        translation=xyz(transform.transform.translation), rotation=quaternion(transform.transform.rotation))
                except Exception as error:
                    record.update(status='UNAVAILABLE', error=str(error))
                aligned_tf[key] = record
        def exact_sample(topic):
            return next((sample for sample in histories[topic] if sample['stamp_ns'] == target_ns), {})
        observed['alignment'] = analyze_alignment(live_fix, exact_sample('/gps/fix'),
                                                  exact_sample('/odometry/gps'), aligned_tf)
        observed['collection_status'] = ('COMPLETE' if all(job['record']['status'] == 'OK' for job in jobs)
                                         and observed['tf_authority'].get('observation_complete')
                                         and observed['plugin_parameters'].get('collection_status') == 'COMPLETE' else 'PARTIAL')
    finally:
        finish_plugin_capture(observed, plugin_probe, histories)
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
    for name in ('readiness.json', 'readiness_observations.json', 'tf_authority.json', 'tf_authority_process.log',
                 'plugin_parameters.json', 'runtime_binary_identity.json', 'localization_alignment.json', 'ackermann_geometry.json',
                 'navigation_profile.json', 'speed_chain.json', 'plant_candidate.json'):
        if (output / name).exists() or (output / name).is_symlink():
            parser.error('Refusing to replace earlier evidence: ' + name)
    observed = {'schema_version': 1, 'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'requested_duration_wall_seconds': args.duration, 'collection_status': 'FAILED',
        'clock': {'count': 0, 'forward_steps': 0, 'backward_steps': 0}, 'topics': {},
        'parameters': {}, 'lifecycle': {}, 'from_ll': {}, 'transforms': {}, 'tf_authority': {},
        'topic_publishers': {}, 'topic_subscribers': {}, 'topic_discovery_errors': {}, 'costmap_bounds': {}, 'alignment': {},
        'nonfinite_json_encoding': 'Nonfinite numbers are preserved as string tokens nan/inf/-inf.'}
    expected = {}
    try:
        collect(output, args.duration, observed, expected)
    except Exception as error:
        observed.update(collection_status='FAILED', error=f'{type(error).__name__}: {error}')
    observed['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    try:
        observed['speed_chain'] = speed_chain_evidence(observed)
        result = assess(observed, expected)
    except Exception as error:
        observed.update(collection_status='FAILED', assessment_error=f'{type(error).__name__}: {error}')
        result = {'schema_version': 1, 'collection_status': 'FAILED', 'diagnostic_status': 'INCOMPLETE',
                  'checks': [{'name': 'assessment_completed', 'status': 'FAIL', 'error': observed['assessment_error']}],
                  'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    write_new(output / 'readiness_observations.json', observed)
    write_new(output / 'readiness.json', result)
    for filename, key in (('plugin_parameters.json', 'plugin_parameters'),
                          ('runtime_binary_identity.json', 'runtime_binary_identity'),
                          ('localization_alignment.json', 'alignment'),
                          ('navigation_profile.json', 'navigation_profile'),
                          ('plant_candidate.json', 'plant_candidate'),
                          ('speed_chain.json', 'speed_chain'),
                          ('ackermann_geometry.json', 'ackermann_geometry')):
        write_new(output / filename, observed.get(key, {'status': 'UNAVAILABLE'}))
    print(json.dumps({key: result[key] for key in ('collection_status', 'diagnostic_status', 'mission_ready', 'gate0', 'gate1')}))
    return 0 if observed['collection_status'] != 'FAILED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
