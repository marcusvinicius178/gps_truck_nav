"""Runtime evidence for the explicit TF/sensor variant; never authorizes motion."""

import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def assess_variant_runtime(observed, clock_healthy, policy):
    checks = []
    def add(name, status, **details):
        checks.append({'name': name, 'status': status, **details})

    variant = observed.get('model_variant', {})
    files = observed.get('source_files', {})
    hashes = variant.get('variant_sha256', {})
    differences = {name: {'expected': expected, 'actual': files.get('/opt/auto_tuning/config/' + name)}
                   for name, expected in hashes.items()
                   if files.get('/opt/auto_tuning/config/' + name) != expected}
    add('prospective_variant_files', 'UNKNOWN' if not hashes else 'FAIL' if differences else 'PASS',
        profile_id=variant.get('profile_id'), differences=differences,
        scope='File identity only; runtime behavior is checked separately.')

    authority = observed.get('tf_authority', {})
    edges = authority.get('edges', [])
    endpoints = {(item.get('topic'), item.get('gid')): item for item in authority.get('publishers', [])}
    expected_edges = variant.get('expected_wheel_edges', [])
    missing, wrong, unknown = [], [], []
    for expected in expected_edges:
        child = expected['child']
        found = [edge for edge in edges if edge.get('child') == child]
        if not found:
            missing.append(child)
            continue
        for edge in found:
            if edge.get('parent') != expected['parent']:
                wrong.append({'child': child, 'expected_parent': expected['parent'], 'actual_parent': edge.get('parent')})
            if not edge.get('publisher_gids') or edge.get('publisher_gid_unknown_count', 0):
                unknown.append(child)
            for gid in edge.get('publisher_gids', []):
                publisher = endpoints.get((edge.get('topic'), gid))
                if publisher is None:
                    unknown.append(child)
                elif publisher.get('node_name') != expected['expected_publisher']:
                    wrong.append({'child': child, 'actual_publisher': publisher.get('node_name')})
    status = ('FAIL' if wrong else 'UNKNOWN' if not expected_edges or missing or unknown
              or not authority.get('observation_complete') or authority.get('endpoint_discovery_errors') else 'PASS')
    add('wheel_tf_urdf_authority', status, missing=missing, wrong=wrong, unknown=sorted(set(unknown)),
        scope='All six articulated children must be observed with their URDF parents and robot_state_publisher.')

    odom_edges = [edge for edge in edges if (edge.get('parent'), edge.get('child')) == ('odom', 'base_footprint')]
    odom_gids = {gid for edge in odom_edges for gid in edge.get('publisher_gids', [])}
    names = {endpoints.get((edge.get('topic'), gid), {}).get('node_name')
             for edge in odom_edges for gid in edge.get('publisher_gids', [])}
    add('ackermann_odometry_tf_preserved', 'UNKNOWN' if not odom_edges or None in names
        or any(edge.get('publisher_gid_unknown_count', 0) for edge in odom_edges)
        or not authority.get('observation_complete') else
        'PASS' if len(odom_gids) == 1 and names == {'ackermann_drive'} else 'FAIL',
        publisher_names=sorted(name for name in names if name is not None),
        scope='Authority only; independent timing and /odom freshness checks remain required.')

    joints = observed.get('topics', {}).get('/joint_states', {}).get('last', {})
    names, positions = joints.get('names', []), joints.get('positions', [])
    expected_joints = variant.get('joint_names', [])
    missing_joints = sorted(set(expected_joints) - set(names))
    valid_positions = (len(names) == len(set(names)) and len(names) == len(positions)
                       and all(finite(value) for value in positions))
    add('joint_states_contract', 'UNKNOWN' if not expected_joints or not joints else
        'PASS' if not missing_joints and valid_positions else 'FAIL',
        missing_joint_names=missing_joints, observed_joint_names=names,
        scope='The six joint positions must remain available; freshness is checked independently.')

    for frame, expected in variant.get('sensor_lever_arms', {}).items():
        tf = observed.get('transforms', {}).get('base_footprint->' + frame, {})
        xyz = tf.get('translation', {})
        q = tf.get('rotation', {})
        valid = tf.get('status') == 'OK' and all(finite(xyz.get(k)) for k in ('x', 'y', 'z'))
        valid = valid and all(math.isclose(xyz[k], value, abs_tol=1e-9, rel_tol=0)
                              for k, value in zip(('x', 'y', 'z'), expected['translation_m']))
        valid = valid and all(finite(q.get(k)) for k in ('x', 'y', 'z', 'w'))
        valid = valid and all(abs(q[k]) <= 1e-9 for k in ('x', 'y', 'z')) and abs(abs(q['w']) - 1) <= 1e-9
        add('sensor_lever_arm:' + frame, 'UNKNOWN' if tf.get('status') != 'OK' else 'PASS' if valid else 'FAIL',
            expected=expected, observed=tf,
            scope='Numerical equality to the existing fixed geometry; not a localization accuracy tolerance.')

    alignment = observed.get('alignment', {})
    status = alignment.get('status')
    if status not in ('PASS', 'FAIL', 'UNKNOWN'):
        status = 'UNKNOWN'
    add('localization_time_alignment', status, evidence=alignment,
        scope='Exact-stamp observations only. Navsat internal timing, covariance and geometric accuracy still require review.')

    targets = observed.get('runtime_binary_identity', {}).get('targets', {})
    for target in ('ackermann_drive', 'gazebo_joint_state', 'smac_hybrid', 'mppi_controller'):
        record = targets.get(target, {})
        add('loaded_binary:' + target, 'PASS' if record.get('status') == 'MAPPED_IDENTIFIED' else 'UNKNOWN',
            observed=record, scope='Mapped file identity, not binary equivalence to an upstream source tag.')
    installed_hash = observed.get('ackermann_library', {}).get('sha256')
    mapped = targets.get('ackermann_drive', {})
    mapped_hashes = {record.get('sha256') for record in mapped.get('records', [])}
    add('loaded_ackermann_matches_installed', 'UNKNOWN' if not installed_hash
        or mapped.get('status') != 'MAPPED_IDENTIFIED' or not mapped_hashes else
        'PASS' if mapped_hashes == {installed_hash} else 'FAIL',
        installed_sha256=installed_hash, mapped_sha256=sorted(value for value in mapped_hashes if value),
        scope='Correlates the inspected prefix library with the file mapped into gzserver.')

    scan = observed.get('topics', {}).get('/sick_scan_2D', {}).get('last', {})
    keys = ('range_count', 'finite_range_count', 'positive_infinite_range_count', 'nan_range_count', 'invalid_range_count')
    present = all(isinstance(scan.get(key), int) and not isinstance(scan[key], bool) for key in keys)
    valid = (present and scan['range_count'] > 0 and scan['nan_range_count'] == 0
             and scan['invalid_range_count'] == 0
             and scan['range_count'] == scan['finite_range_count'] + scan['positive_infinite_range_count'])
    add('obstacle_scan_payload', 'UNKNOWN' if not present else 'PASS' if valid else 'FAIL', observed=scan,
        obstacle_detection_validated=False,
        note='Positive infinity may mean no return within range; it does not validate detection of a known obstacle.')

    for topic in observed.get('obstacle_source_topics', []):
        if topic == '/sick_scan_2D':
            continue
        record = observed.get('topics', {}).get(topic, {})
        age = observed.get('clock', {}).get('last', 0) - record['last_stamp'] if finite(record.get('last_stamp')) else None
        live = (clock_healthy and record.get('count', 0) >= policy['minimum_samples']
                and record.get('forward_steps', 0) >= 1 and record.get('backward_steps', 0) == 0
                and age is not None and -policy['maximum_future_sim_seconds'] <= age <= policy['maximum_age_sim_seconds'])
        add('obstacle_source_samples:' + topic, 'BLOCKED_REVIEW' if not record else 'PASS' if live else 'FAIL',
            count=record.get('count', 0), age_sim_seconds=age, metadata=record.get('last'),
            scope='Sample liveness only; no synthetic source, remap or Virtual Lanes was enabled.')
    return checks
