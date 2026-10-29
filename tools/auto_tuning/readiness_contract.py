"""Read-only Phase A diagnostic criteria. Passing these never authorizes motion."""

import ast
import math


LIFECYCLE_NODES = ('bt_navigator', 'planner_server', 'controller_server', 'behavior_server',
                   'smoother_server', 'velocity_smoother', 'waypoint_follower',
                   'global_costmap/global_costmap', 'local_costmap/local_costmap')
REQUIRED_TOPICS = ('/gps/fix', '/imu', '/odom', '/odometry/local', '/odometry/global',
                   '/odometry/gps', '/sick_scan_2D')
BT_PATHS = {
    'default_nav_to_pose_bt_xml': '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_to_pose.xml',
    'default_nav_through_poses_bt_xml': '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_through_poses.xml',
}
POLICY = {'minimum_samples': 2, 'maximum_age_sim_seconds': 1.0,
          'maximum_future_sim_seconds': 0.25,
          'maximum_clock_silence_wall_seconds': 2.0,
          'scope': 'Diagnostic thresholds only; not an experimental acceptance gate.'}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stamp_seconds(value):
    if not isinstance(value, dict):
        return None
    seconds, nanoseconds = value.get('sec'), value.get('nanosec')
    if not finite(seconds) or not finite(nanoseconds) or not 0 <= nanoseconds < 1_000_000_000:
        return None
    return seconds + nanoseconds / 1_000_000_000


def flatten(value, prefix=''):
    result = {}
    for key, item in value.items():
        name = prefix + key
        if isinstance(item, dict):
            result.update(flatten(item, name + '.'))
        else:
            result[name] = item
    return result


def expected_parameters(canonical):
    """Compare declared/effective costmap keys, not obsolete unused YAML keys."""
    result = {}
    core = ('width', 'height', 'resolution', 'rolling_window', 'global_frame',
            'robot_base_frame', 'footprint', 'plugins', 'update_frequency',
            'publish_frequency', 'transform_tolerance')
    layer_keys = ('plugin', 'enabled', 'observation_sources', 'inflation_radius',
                  'cost_scaling_factor', 'z_resolution', 'z_voxels')
    source_keys = ('topic', 'data_type', 'marking', 'clearing', 'obstacle_max_range',
                   'obstacle_min_range', 'raytrace_max_range', 'raytrace_min_range',
                   'max_obstacle_height', 'inf_is_valid')
    for name in ('global_costmap', 'local_costmap'):
        params = canonical[name][name]['ros__parameters']
        wanted = {key: params[key] for key in core}
        wanted['use_sim_time'] = True  # Explicit simulation launch contract.
        for layer in params['plugins']:
            values = params[layer]
            for key in layer_keys:
                if key in values:
                    wanted[layer + '.' + key] = values[key]
            for source in values.get('observation_sources', '').split():
                for key in source_keys:
                    if key in values[source]:
                        wanted[layer + '.' + source + '.' + key] = values[source][key]
        result['/' + name + '/' + name] = wanted
    bt = canonical['bt_navigator']['ros__parameters']
    result['/bt_navigator'] = {**BT_PATHS, 'use_sim_time': True, 'navigators': bt['navigators'],
                              **{name + '.plugin': bt[name]['plugin'] for name in bt['navigators']}}
    for name in ('planner_server', 'controller_server'):
        result['/' + name] = {'use_sim_time': True}
    return result


def equivalent(name, expected, actual):
    if name == 'footprint':
        try:
            expected = ast.literal_eval(expected) if isinstance(expected, str) else expected
            actual = ast.literal_eval(actual) if isinstance(actual, str) else actual
        except (ValueError, SyntaxError):
            return False
    if name.endswith('observation_sources') and isinstance(actual, str):
        return expected.split() == actual.split()
    if isinstance(expected, bool):
        return isinstance(actual, bool) and expected == actual
    if finite(expected):
        return finite(actual) and math.isclose(expected, actual, rel_tol=1e-9, abs_tol=1e-9)
    return type(expected) is type(actual) and expected == actual


def covariance_status(values, unavailable=False):
    if unavailable:
        return 'BLOCKED_REVIEW', 'Sensor reports this estimate unavailable.'
    if not isinstance(values, list) or len(values) != 9 or not all(finite(v) for v in values):
        return 'FAIL', 'Covariance must contain nine finite entries.'
    if all(v == 0 for v in values):
        return 'BLOCKED_REVIEW', 'All-zero covariance requires a documented noise/unknown-data interpretation.'
    a, b, c, d, e, f, g, h, i = values
    symmetric = all(math.isclose(x, y, rel_tol=1e-6, abs_tol=1e-12)
                    for x, y in ((b, d), (c, g), (f, h)))
    minors = (a, e, i, a * e - b * d, a * i - c * g, e * i - f * h,
              a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g))
    if not symmetric or any(v < -1e-12 for v in minors):
        return 'FAIL', 'Covariance is not symmetric positive semidefinite.'
    return 'PASS', 'Finite PSD covariance observed; physical noise model still requires review.'


def costmap_bounds(metadata, transform, expected_frame):
    record = {'metadata': metadata, 'robot_transform': transform, 'inside': None,
              'scope': 'Vehicle origin only; not complete footprint or route containment.'}
    origin = metadata.get('origin_position', {})
    q = metadata.get('origin_orientation', {})
    position = transform.get('translation', {})
    valid = transform.get('status') == 'OK' and metadata.get('frame_id') == expected_frame
    valid = valid and all(finite(item.get(key)) for item in (origin, position) for key in ('x', 'y', 'z'))
    valid = valid and all(finite(q.get(key)) for key in ('x', 'y', 'z', 'w'))
    valid = valid and all(finite(metadata.get(key)) and metadata[key] > 0 for key in ('width', 'height', 'resolution'))
    valid = valid and abs(math.sqrt(sum(q[key] ** 2 for key in ('x', 'y', 'z', 'w'))) - 1) <= 1e-3
    valid = valid and abs(q['x']) <= 1e-6 and abs(q['y']) <= 1e-6
    if not valid:
        record['reason'] = 'Missing, nonfinite, wrong-frame or unsupported nonplanar costmap metadata/transform.'
        return record
    yaw = math.atan2(2 * q['w'] * q['z'], 1 - 2 * q['z'] ** 2)
    dx, dy = position['x'] - origin['x'], position['y'] - origin['y']
    x, y = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    record.update(robot_in_grid_m={'x': x, 'y': y}, inside=0 <= x < metadata['width'] * metadata['resolution']
                  and 0 <= y < metadata['height'] * metadata['resolution'])
    return record


def assess(observed, expected):
    checks = []
    def add(name, status, **details):
        checks.append({'name': name, 'status': status, **details})

    add('parameter_contract_loaded', 'PASS' if expected else 'UNKNOWN')
    for node, wanted in expected.items():
        record = observed.get('parameters', {}).get(node, {})
        if record.get('status') != 'OK':
            add('parameters:' + node, 'UNKNOWN', reason='Parameter service did not return a complete response.')
            continue
        actual = record.get('values', {})
        differences = {key: {'expected': value, 'actual': actual.get(key)}
                       for key, value in wanted.items() if not equivalent(key, value, actual.get(key))}
        add('parameters:' + node, 'FAIL' if differences else 'PASS', differences=differences)
    for name in LIFECYCLE_NODES:
        record = observed.get('lifecycle', {}).get('/' + name, {})
        status = 'UNKNOWN' if record.get('status') != 'OK' else 'PASS' if record.get('id') == 3 else 'FAIL'
        add('lifecycle:/' + name, status, observed=record)

    clock = observed.get('clock', {})
    window_end = observed.get('observation_window_end_wall_offset')
    received = clock.get('last_received_wall_offset')
    advanced_at = clock.get('last_advance_wall_offset')
    reception_age = window_end - received if finite(window_end) and finite(received) else None
    advance_age = window_end - advanced_at if finite(window_end) and finite(advanced_at) else None
    live_clock = all(age is not None and 0 <= age <= POLICY['maximum_clock_silence_wall_seconds']
                     for age in (reception_age, advance_age))
    advanced = (clock.get('count', 0) >= 3 and clock.get('forward_steps', 0) >= 2
                and clock.get('backward_steps', 0) == 0 and finite(clock.get('last')) and live_clock)
    add('clock_advancing', 'PASS' if advanced else 'FAIL', observed=clock,
        last_reception_age_wall_seconds=reception_age, last_advance_age_wall_seconds=advance_age)
    for topic in REQUIRED_TOPICS:
        record = observed.get('topics', {}).get(topic, {})
        last = record.get('last_stamp')
        age = clock['last'] - last if finite(clock.get('last')) and finite(last) else None
        healthy = (advanced and record.get('count', 0) >= POLICY['minimum_samples']
                   and record.get('forward_steps', 0) >= 1 and record.get('backward_steps', 0) == 0
                   and age is not None and -POLICY['maximum_future_sim_seconds'] <= age <= POLICY['maximum_age_sim_seconds'])
        add('freshness:' + topic, 'PASS' if healthy else 'FAIL', count=record.get('count', 0),
            age_sim_seconds=age, first_stamp=record.get('first_stamp'), last_stamp=last)

    topics = observed.get('topics', {})
    for topic, frame in (('/gps/fix', 'gps_link'), ('/imu', 'imu_link')):
        actual = topics.get(topic, {}).get('last', {}).get('frame_id')
        add('sensor_frame:' + topic, 'PASS' if actual == frame else 'BLOCKED_REVIEW',
            expected=frame, actual=actual, reason='URDF/SDF sensor offset must not be silently treated as the vehicle origin.')
    gps = topics.get('/gps/fix', {}).get('last', {})
    valid_fix = (isinstance(gps.get('status'), int) and gps['status'] >= 0
                 and all(finite(gps.get(key)) for key in ('latitude', 'longitude', 'altitude'))
                 and -90 <= gps['latitude'] <= 90 and -180 <= gps['longitude'] <= 180)
    add('gnss_fix_valid', 'PASS' if valid_fix else 'FAIL', fix_status=gps.get('status'))
    status, reason = covariance_status(gps.get('position_covariance'), gps.get('position_covariance_type') == 0)
    add('gps_covariance', status, reason=reason, type=gps.get('position_covariance_type'))
    imu = topics.get('/imu', {}).get('last', {})
    for field in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
        values = imu.get(field)
        status, reason = covariance_status(values, bool(values and values[0] == -1))
        add('imu:' + field, status, reason=reason)
    for topic in ('/imu', '/odom', '/odometry/local', '/odometry/global'):
        data = topics.get(topic, {}).get('last', {})
        q = data.get('orientation', {})
        valid = all(finite(q.get(key)) for key in ('x', 'y', 'z', 'w'))
        norm = math.sqrt(sum(q[key] ** 2 for key in ('x', 'y', 'z', 'w'))) if valid else None
        add('quaternion:' + topic, 'PASS' if norm is not None and abs(norm - 1) <= 1e-3 else 'FAIL', norm=norm)
    for topic, frame in (('/odom', 'odom'), ('/odometry/local', 'odom'),
                          ('/odometry/global', 'map'), ('/odometry/gps', 'map')):
        data = topics.get(topic, {}).get('last', {})
        valid = data.get('frame_id') == frame and all(finite(data.get('position', {}).get(key)) for key in ('x', 'y', 'z'))
        if topic != '/odometry/gps':
            valid = valid and data.get('child_frame_id') == 'base_footprint'
        add('odometry_frame_and_position:' + topic, 'PASS' if valid else 'FAIL',
            expected_frame=frame, actual_frame=data.get('frame_id'), child=data.get('child_frame_id'))
    covariance = topics.get('/odometry/gps', {}).get('last', {}).get('pose_covariance', [])
    position_covariance = [covariance[index] for index in (0, 1, 2, 6, 7, 8, 12, 13, 14)] if len(covariance) == 36 else None
    status, reason = covariance_status(position_covariance)
    add('gps_odometry_position_covariance', status, reason=reason)

    authority = observed.get('tf_authority', {})
    edges = authority.get('edges', [])
    parents = {}
    for edge in edges:
        parents.setdefault(edge.get('child'), set()).add(edge.get('parent'))
    conflicts = {child: sorted(values) for child, values in parents.items() if len(values) > 1}
    duplicate = [edge for edge in edges if len(set(edge.get('publisher_gids', []))) > 1]
    # Also detect the same edge published separately on /tf and /tf_static.
    by_pair = {}
    for edge in edges:
        by_pair.setdefault((edge.get('parent'), edge.get('child')), set()).update(edge.get('publisher_gids', []))
    duplicate_pairs = [list(pair) for pair, gids in by_pair.items() if len(gids) > 1]
    unknown = [edge for edge in edges if not edge.get('publisher_gids') or edge.get('publisher_gid_unknown_count', 0)]
    missing = [list(pair) for pair in (('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'base_link')) if pair not in by_pair]
    nonfinite = [edge for edge in edges if edge.get('nonfinite_transform_count', 0)]
    endpoints = {(entry.get('topic'), entry.get('gid')) for entry in authority.get('publishers', [])}
    unmapped = sorted({gid for edge in edges for gid in edge.get('publisher_gids', [])
                       if (edge.get('topic'), gid) not in endpoints})
    status = ('FAIL' if conflicts or duplicate or duplicate_pairs or nonfinite else
              'UNKNOWN' if missing or unknown or unmapped or authority.get('endpoint_discovery_errors')
              or not authority.get('observation_complete') else 'PASS')
    add('tf_authority_per_edge', status, parent_conflicts=conflicts,
        duplicate_pairs=duplicate_pairs, missing_required_edges=missing,
        unknown_gid_edges=len(unknown), unmapped_publisher_gids=unmapped, nonfinite_edges=len(nonfinite),
        scope='Observed time window only; topic publisher count alone is not an authority check.')
    required_pairs = {('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'base_link')}
    invalid_edges = []
    for edge in edges:
        pair = (edge.get('parent'), edge.get('child'))
        if pair not in required_pairs:
            continue
        transform = edge.get('last_transform', {})
        q = transform.get('rotation', {})
        valid = all(finite(transform.get('translation', {}).get(axis)) for axis in ('x', 'y', 'z'))
        valid = valid and all(finite(q.get(axis)) for axis in ('x', 'y', 'z', 'w'))
        valid = valid and abs(math.sqrt(sum(q[axis] ** 2 for axis in ('x', 'y', 'z', 'w'))) - 1) <= 1e-3
        age = None
        if pair != ('base_footprint', 'base_link'):
            timestamp = stamp_seconds(edge.get('last_stamp'))
            first_timestamp = stamp_seconds(edge.get('first_stamp'))
            count = edge.get('count', 0)
            progressing = (isinstance(count, int) and not isinstance(count, bool) and count >= 2
                           and finite(first_timestamp) and finite(timestamp) and timestamp > first_timestamp)
            age = clock['last'] - timestamp if finite(clock.get('last')) and finite(timestamp) else None
            valid = valid and advanced and progressing and edge.get('topic') == '/tf' and age is not None and -POLICY['maximum_future_sim_seconds'] <= age <= POLICY['maximum_age_sim_seconds']
        if not valid:
            invalid_edges.append({'parent': pair[0], 'child': pair[1], 'age_sim_seconds': age})
    add('required_tf_numeric_and_fresh', 'UNKNOWN' if missing else 'FAIL' if invalid_edges else 'PASS',
        invalid_edges=invalid_edges, note='Zero stamps on static base transforms are valid; dynamic navigation edges must advance with simulation time.')
    for key in ('map->base_footprint', 'odom->base_footprint', 'base_footprint->base_link',
                'base_footprint->gps_link', 'base_footprint->imu_link', 'map->gps_link'):
        record = observed.get('transforms', {}).get(key, {})
        valid = record.get('status') == 'OK' and all(finite(record.get('translation', {}).get(axis)) for axis in ('x', 'y', 'z'))
        q = record.get('rotation', {})
        valid = valid and all(finite(q.get(axis)) for axis in ('x', 'y', 'z', 'w'))
        norm = math.sqrt(sum(q[axis] ** 2 for axis in ('x', 'y', 'z', 'w'))) if valid else None
        valid = valid and abs(norm - 1) <= 1e-3
        age = None
        if key.startswith(('map->', 'odom->')) and valid:
            age = clock['last'] - record['stamp'] if finite(clock.get('last')) and finite(record.get('stamp')) else None
            valid = advanced and age is not None and -POLICY['maximum_future_sim_seconds'] <= age <= POLICY['maximum_age_sim_seconds']
        add('tf_lookup:' + key, 'PASS' if valid else 'UNKNOWN', age_sim_seconds=age, observed=record)

    for key in ('datum', 'live_gps'):
        record = observed.get('from_ll', {}).get(key, {})
        point = record.get('map_point', {})
        valid = record.get('status') == 'OK' and all(finite(point.get(k)) for k in ('x', 'y', 'z'))
        add('from_ll:' + key, 'PASS' if valid else 'UNKNOWN', observed=record)
    for name in ('global_costmap', 'local_costmap'):
        record = observed.get('costmap_bounds', {}).get(name, {})
        add('robot_center_in:' + name, 'PASS' if record.get('inside') is True else
            'FAIL' if record.get('inside') is False else 'UNKNOWN', observed=record)
    for topic in observed.get('obstacle_source_topics', []):
        publishers = observed.get('topic_publishers', {}).get(topic, [])
        add('obstacle_source_publishers:' + topic, 'PASS' if publishers else 'BLOCKED_REVIEW',
            publishers=publishers, reason='Publisher discovery alone does not prove fresh usable obstacle data.')
    library = observed.get('ackermann_library', {})
    add('ackermann_binary_identity', 'PASS' if library.get('sha256') else 'UNKNOWN', observed=library)
    add('localization_geometry_review', 'BLOCKED_REVIEW',
        reason='Review lever arms, datum and map/GNSS/odometry deltas; no geometry or covariance has been retuned.')
    add('ackermann_command_interface', 'BLOCKED_REVIEW',
        reason='MPPI yaw rate versus Gazebo steering semantics must be resolved and validated before movement.')
    statuses = {entry['status'] for entry in checks}
    return {'schema_version': 1, 'collection_status': observed.get('collection_status', 'INCOMPLETE'),
            'diagnostic_status': 'BLOCKED' if 'FAIL' in statuses else
                                 'INCOMPLETE' if 'UNKNOWN' in statuses else 'REVIEW_REQUIRED',
            'checks': checks, 'policy': POLICY, 'mission_ready': False,
            'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
            'limits': ['No goals, velocity commands, recorder, reset or optimizer were started.',
                       'ACTIVE and matching parameters do not establish full Nav2 health or mission readiness.']}
