"""Pure, exact-timestamp GNSS/TF comparisons for a read-only diagnostic probe.

This verifies the identities, frames and timestamps of the supplied observations.
It does not verify navsat_transform's internal time handling, covariance model,
geometric localization accuracy, independent ground truth or permission to move.
No ROS imports, interpolation, nearest-sample selection or latest-TF fallback.
"""

import math


TRANSFORMS = {
    'map->gps_link': ('map', 'gps_link', False),
    'map->base_footprint': ('map', 'base_footprint', False),
    'base_footprint->gps_link': ('base_footprint', 'gps_link', True),
}
QUATERNION_NORM_TOLERANCE = 1e-3  # Numerical validity, not a position tolerance.


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _stamp(value):
    # ROS Time(0) requests latest data, so it cannot establish this contract.
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _rotate(vector, quaternion):
    """Quaternion rotation; norm division is explicit and raw data is preserved."""
    x, y, z, w = (quaternion[key] for key in ('x', 'y', 'z', 'w'))
    vx, vy, vz = (vector[key] for key in ('x', 'y', 'z'))
    scale = 2.0 / (x * x + y * y + z * z + w * w)
    return {
        'x': (1 - scale * (y * y + z * z)) * vx
             + scale * (x * y - z * w) * vy + scale * (x * z + y * w) * vz,
        'y': scale * (x * y + z * w) * vx
             + (1 - scale * (x * x + z * z)) * vy + scale * (y * z - x * w) * vz,
        'z': scale * (x * z - y * w) * vx + scale * (y * z + x * w) * vy
             + (1 - scale * (x * x + y * y)) * vz,
    }


def _residual(left, right):
    delta = {axis: left[axis] - right[axis] for axis in ('x', 'y')}
    return {
        'delta_m': delta,
        'horizontal_distance_m': math.hypot(delta['x'], delta['y']),
        'vertical_difference_m_not_evaluated': left['z'] - right['z'],
    }


def analyze_alignment(from_ll_record, gps_fix, gps_odometry, transforms_at_fix_stamp):
    """Return measurements only after an exact, positive integer-ns match.

    ``gps_fix`` is a flat saved NavSatFix summary with ``stamp_ns`` and its
    original frame/latitude/longitude/altitude/status. ``gps_odometry`` is the
    flat /odometry/gps summary with ``stamp_ns``, frame and position.

    The /fromLL record supplies ``input_sensor_stamp_ns``, ``input_frame_id``,
    the exact request ``input`` coordinates, ``status='OK'``, ``map_point`` and
    explicitly recorded ``output_frame_id='map'`` (the response has no header).

    Transform entries require status, parent, child, requested_stamp_ns,
    stamp_ns, lookup_mode='EXACT', translation and rotation. A zero response
    stamp is permitted only for the fixed base_footprint->gps_link chain with
    is_static=True, while its requested stamp must still match the fix.

    PASS means these observations can be compared at one timestamp. Residual
    size never determines PASS; geometric acceptance remains a separate review.
    Horizontal comparisons reflect the separately checked zero_altitude profile.
    """
    issues = []
    norms = {}

    def issue(field, reason, status='FAIL'):
        issues.append({'field': field, 'status': status, 'reason': reason})

    def mapping(value, name):
        if not isinstance(value, dict):
            issue(name, 'Observation unavailable or not an object.', 'UNKNOWN')
            return {}
        return value

    def vector(value, name, axes=('x', 'y', 'z')):
        if not isinstance(value, dict):
            issue(name, 'Vector unavailable.', 'UNKNOWN')
            return False
        value = mapping(value, name)
        if any(axis not in value for axis in axes):
            issue(name, 'Vector components missing.', 'UNKNOWN')
            return False
        if not all(_finite(value.get(axis)) for axis in axes):
            issue(name, 'All requested components must be finite numbers.')
            return False
        return True

    conversion = mapping(from_ll_record, 'from_ll')
    fix = mapping(gps_fix, 'gps_fix')
    odometry = mapping(gps_odometry, 'gps_odometry')
    transforms = mapping(transforms_at_fix_stamp, 'transforms')
    source_stamp = conversion.get('input_sensor_stamp_ns')
    result = {
        'schema_version': 1,
        'status': 'UNKNOWN',
        'reference_stamp_ns': source_stamp,
        'comparison_frame': 'map',
        'comparison_axes': ['x', 'y'],
        'timestamp_match_policy': 'Exact positive integer nanoseconds; no nearest/latest fallback.',
        'zero_altitude_policy': 'Horizontal residuals only; the runtime zero_altitude profile is checked separately.',
        'synchronized_measurements_verified': False,
        'geometric_validated': False,
        'internal_navsat_time_compensation_verified': False,
        'mission_ready': False,
        'issues': issues,
        'quaternion_norms': norms,
        'residuals': {},
        'limits': [
            '/fromLL has no request timestamp or response frame header; the saved input fix and explicit map-frame contract are checked here.',
            'These comparisons do not prove that navsat_transform internally used the same timestamp or avoided a latest-TF fallback.',
            'GPS odometry and EKF/TF are not independent ground truth; a small residual does not prove localization accuracy.',
            'No geometric acceptance tolerance, noise variance, datum or model geometry is selected by this function.',
        ],
    }
    if issues:
        return result
    if conversion.get('status') != 'OK':
        issue('from_ll.status', 'Conversion response unavailable.', 'UNKNOWN')
        return result
    if not _stamp(source_stamp):
        issue('from_ll.input_sensor_stamp_ns', 'Positive integer nanoseconds required; float seconds and zero/latest are rejected.')
    if conversion.get('output_frame_id') != 'map':
        issue('from_ll.output_frame_id', 'Explicit map output-frame contract required.')
    if conversion.get('input_frame_id') != 'gps_link' or fix.get('frame_id') != 'gps_link':
        issue('gps_fix.frame_id', 'Both conversion input and original fix must identify gps_link.')
    for name, sample in (('gps_fix', fix), ('gps_odometry', odometry)):
        if not _stamp(sample.get('stamp_ns')) or sample.get('stamp_ns') != source_stamp:
            issue(name + '.stamp_ns', 'Exact fix/conversion/odometry timestamp match required.')
    if odometry.get('frame_id') != 'map':
        issue('gps_odometry.frame_id', 'GPS odometry position must be expressed in map.')
    if odometry.get('child_frame_id') not in ('', 'base_footprint'):
        issue('gps_odometry.child_frame_id', 'Unexpected child frame for navsat GPS odometry.')
    fix_status = fix.get('status')
    if not isinstance(fix_status, int) or isinstance(fix_status, bool) or fix_status < 0:
        issue('gps_fix.status', 'A valid original GNSS fix is required.')
    coordinates = mapping(conversion.get('input'), 'from_ll.input')
    for coordinate in ('latitude', 'longitude', 'altitude'):
        if (not _finite(fix.get(coordinate)) or not _finite(coordinates.get(coordinate))
                or fix.get(coordinate) != coordinates.get(coordinate)):
            issue('from_ll.input.' + coordinate, 'Conversion must use the exact saved fix coordinates.')
    if (_finite(fix.get('latitude')) and not -90 <= fix['latitude'] <= 90
            or _finite(fix.get('longitude')) and not -180 <= fix['longitude'] <= 180):
        issue('gps_fix.coordinates', 'GNSS latitude/longitude outside physical bounds.')
    vector(conversion.get('map_point'), 'from_ll.map_point')
    vector(odometry.get('position'), 'gps_odometry.position')

    for key, (parent, child, static) in TRANSFORMS.items():
        record = mapping(transforms.get(key), key)
        if record.get('status') != 'OK':
            issue(key + '.status', 'Exact-time TF lookup unavailable.', 'UNKNOWN')
            continue
        if record.get('parent') != parent or record.get('child') != child:
            issue(key + '.frames', 'Transform parent/child differs from the comparison contract.')
        if record.get('lookup_mode') != 'EXACT':
            issue(key + '.lookup_mode', 'Only an explicit exact-time lookup is admissible.')
        if (not _stamp(record.get('requested_stamp_ns'))
                or record.get('requested_stamp_ns') != source_stamp):
            issue(key + '.requested_stamp_ns', 'Lookup request must use the exact original fix timestamp.')
        stamp = record.get('stamp_ns')
        static_zero = static and record.get('is_static') is True and type(stamp) is int and stamp == 0
        if not static_zero and (not _stamp(stamp) or stamp != source_stamp):
            issue(key + '.stamp_ns', 'Lookup response is not at the requested time; only certified fixed-chain zero is exempt.')
        if not static and record.get('is_static') is True:
            issue(key + '.is_static', 'Navigation transforms cannot use the static timestamp exception.')
        vector(record.get('translation'), key + '.translation')
        quaternion = record.get('rotation')
        if vector(quaternion, key + '.rotation', ('x', 'y', 'z', 'w')):
            norm = math.hypot(*(quaternion[axis] for axis in ('x', 'y', 'z', 'w')))
            norms[key] = norm
            if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
                issue(key + '.rotation', 'Quaternion is not unit length within numerical tolerance.')

    if issues:
        result['status'] = 'FAIL' if any(item['status'] == 'FAIL' for item in issues) else 'UNKNOWN'
        return result

    map_gps = transforms['map->gps_link']['translation']
    map_base = transforms['map->base_footprint']['translation']
    lever_arm = transforms['base_footprint->gps_link']['translation']
    map_lever_arm = _rotate(lever_arm, transforms['map->base_footprint']['rotation'])
    gps_point = conversion['map_point']
    odometry_point = odometry['position']
    observed_lever_arm = {axis: gps_point[axis] - odometry_point[axis] for axis in ('x', 'y', 'z')}
    composed_map_gps = {axis: map_base[axis] + map_lever_arm[axis] for axis in ('x', 'y', 'z')}
    residuals = {
        'from_ll_gps_vs_map_gps_link': _residual(gps_point, map_gps),
        'gps_odometry_vs_map_base_footprint': _residual(odometry_point, map_base),
        'from_ll_minus_gps_odometry_vs_rotated_lever_arm': _residual(observed_lever_arm, map_lever_arm),
        'tf_chain_composition': _residual(map_gps, composed_map_gps),
    }
    computed_numbers = list(map_lever_arm.values())
    for residual in residuals.values():
        computed_numbers.extend(residual['delta_m'].values())
        computed_numbers.extend((residual['horizontal_distance_m'], residual['vertical_difference_m_not_evaluated']))
    if not all(_finite(number) for number in computed_numbers):
        issue('residuals', 'Finite input arithmetic overflowed; no finite comparison can be reported.')
        result['status'] = 'FAIL'
        return result
    result.update(
        status='PASS', synchronized_measurements_verified=True,
        quaternion_rotation_policy='Norm-divided quaternion rotation; original observed quaternion retained by caller.',
        sensor_lever_arm_base_m=dict(lever_arm), sensor_lever_arm_map_m=map_lever_arm,
        residuals=residuals,
    )
    return result
