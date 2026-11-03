#!/usr/bin/env python3
"""Read-only SDF geometry audit; no ROS, simulation, command output or calibration.

Reproduce the initial collision-center distances used by gazebo_plugins 3.7.0.
The mathematical helpers are offline contract checks, not a drive adapter.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


UPSTREAM = {
    'repository': 'ros-simulation/gazebo_ros_pkgs', 'ref': '3.7.0',
    'path': 'gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp',
    'blob_sha': '8f536598f837acef619337f4b72b626e7021dfa8',
    'url': 'https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp',
}
WHEELS = ('front_left_joint', 'front_right_joint', 'rear_left_joint', 'rear_right_joint')
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class GeometryError(ValueError):
    pass


def number(value, label):
    if isinstance(value, bool):
        raise GeometryError(f'{label}: boolean is not a physical quantity')
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise GeometryError(f'{label}: missing or invalid number') from error
    if not math.isfinite(result):
        raise GeometryError(f'{label}: nonfinite number')
    return result


def multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def rotate(matrix, vector):
    return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def compose(parent, child):
    p, rotation = parent
    q, orientation = child
    moved = rotate(rotation, q)
    return tuple(x + y for x, y in zip(p, moved)), multiply(rotation, orientation)


def parse_pose(element):
    if element is None:
        return (0.0, 0.0, 0.0), IDENTITY
    if element.get('degrees', 'false').lower() not in ('false', '0'):
        raise GeometryError('Degree-valued poses are outside this audited SDF contract')
    if element.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise GeometryError('Only six-value Euler RPY poses are supported')
    values = [number(v, 'pose') for v in (element.text or '').split()]
    if len(values) != 6:
        raise GeometryError('Pose must contain six finite x y z roll pitch yaw values')
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1, 0, 0), (0, cr, -sr), (0, sr, cr))
    ry = ((cp, 0, sp), (0, 1, 0), (-sp, 0, cp))
    rz = ((cy, -sy, 0), (sy, cy, 0), (0, 0, 1))
    return (x, y, z), multiply(rz, multiply(ry, rx))


class PoseGraph:
    """Resolve the local, unnested SDF graph without pretending to be libsdformat."""

    def __init__(self, model):
        if model.findall('include') or model.findall('model'):
            raise GeometryError('Includes/nested models require libsdformat inspection; refusing a partial graph')
        self.model_name = model.get('name')
        self.nodes = {}
        for item in model:
            if item.tag in ('link', 'joint', 'frame'):
                name = item.get('name')
                if not name or name in self.nodes:
                    raise GeometryError('Missing or duplicate graph entity name')
                self.nodes[name] = item
        self.cache = {'__model__': ((0.0, 0.0, 0.0), IDENTITY)}

    def resolve(self, name, stack=()):
        if name == self.model_name:
            name = '__model__'
        if name in self.cache:
            return self.cache[name]
        if name in stack:
            raise GeometryError('Cyclic pose references: ' + ' -> '.join((*stack, name)))
        if name not in self.nodes:
            raise GeometryError(f'Unknown or external pose frame: {name}')
        item = self.nodes[name]
        pose = item.find('pose')
        relative = pose.get('relative_to') if pose is not None else None
        if not relative:
            relative = (item.findtext('child') if item.tag == 'joint' else
                        item.get('attached_to', '__model__') if item.tag == 'frame' else '__model__')
        result = compose(self.resolve(relative, (*stack, name)), parse_pose(pose))
        self.cache[name] = result
        return result

    def collision_pose(self, link_name, collision):
        pose = collision.find('pose')
        relative = pose.get('relative_to', link_name) if pose is not None else link_name
        return compose(self.resolve(relative), parse_pose(pose))


def steering_targets(virtual_angle, wheelbase, separation):
    """The upstream target equations only; this does not actuate any joint."""
    angle = number(virtual_angle, 'virtual steering')
    length, width = number(wheelbase, 'wheelbase'), number(separation, 'track')
    if length <= 0 or width <= 0 or abs(angle) >= math.pi / 2:
        raise GeometryError('Invalid geometry or steering branch')
    tangent = math.tan(angle)
    ratio = width / (2 * length)
    return {'left': math.atan2(tangent, 1 - ratio * tangent),
            'right': math.atan2(tangent, 1 + ratio * tangent)}


def ideal_plugin_input(speed, yaw_rate, wheelbase):
    """Offline ideal bicycle inverse, before any limits/dynamics/watchdog.

    Never suitable as a standalone runtime converter. At exact zero, only the
    zero-motion pair has a defined stop policy; signed zero is normalized.
    """
    speed, yaw_rate = number(speed, 'speed'), number(yaw_rate, 'yaw rate')
    length = number(wheelbase, 'wheelbase')
    if length <= 0:
        raise GeometryError('Wheelbase must be positive')
    if speed == 0:
        if yaw_rate != 0:
            raise GeometryError('In-place rotation is not realizable by the Ackermann bicycle model')
        return 0.0
    numerator = length * yaw_rate
    if not math.isfinite(numerator):
        raise GeometryError('Nonfinite intermediate bicycle calculation')
    return math.atan2(numerator, abs(speed))


def audit_model(model_path, requested_turning_radius=None):
    path = Path(model_path)
    payload = path.read_bytes()
    root = ET.fromstring(payload)
    models = root.findall('model')
    if root.tag != 'sdf' or root.get('version') != '1.7' or len(models) != 1:
        raise GeometryError('Expected one top-level SDF 1.7 model')
    model = models[0]
    graph = PoseGraph(model)
    plugins = [p for p in model.findall('plugin') if p.get('filename') == 'libgazebo_ros_ackermann_drive.so']
    if len(plugins) != 1:
        raise GeometryError('Expected exactly one Gazebo Ackermann plugin')
    plugin = plugins[0]
    wheels = {}
    for key in WHEELS:
        joint_name = plugin.findtext(key)
        joint = graph.nodes.get(joint_name)
        if joint is None or joint.tag != 'joint':
            raise GeometryError(f'Missing selected wheel joint: {key}')
        child = joint.findtext('child')
        link = graph.nodes.get(child)
        if link is None or link.tag != 'link':
            raise GeometryError(f'Missing wheel child link: {child}')
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise GeometryError(f'{child}: expected one collision, matching upstream geometry assumption')
        collision = collisions[0]
        shapes = list(collision.find('geometry')) if collision.find('geometry') is not None else []
        if len(shapes) != 1 or shapes[0].tag not in ('cylinder', 'sphere'):
            raise GeometryError(f'{child}: unsupported wheel collision shape')
        radius = number(shapes[0].findtext('radius'), child + ' radius')
        if radius <= 0:
            raise GeometryError('Wheel radius must be positive')
        center, _ = graph.collision_pose(child, collision)
        wheels[key] = {'joint': joint_name, 'child_link': child, 'collision': collision.get('name'),
                       'collision_center_model_m': list(center), 'collision_radius_m': radius}
    centers = {key: item['collision_center_model_m'] for key, item in wheels.items()}
    front = [(a + b) / 2 for a, b in zip(centers['front_left_joint'], centers['front_right_joint'])]
    rear = [(a + b) / 2 for a, b in zip(centers['rear_left_joint'], centers['rear_right_joint'])]
    length = math.dist(front, rear)
    width = math.dist(centers['front_left_joint'], centers['front_right_joint'])
    if not math.isfinite(length) or not math.isfinite(width) or length <= 0 or width <= 0:
        raise GeometryError('Coincident centers cannot define wheelbase or track')
    max_steer = number(plugin.findtext('max_steer', '0.6'), 'max_steer')
    max_speed = number(plugin.findtext('max_speed', '20.0'), 'max_speed')
    if not 0 < max_steer < math.pi / 2 or max_speed <= 0:
        raise GeometryError('Invalid plugin speed/steering limit')
    limits = {}
    for side in ('left', 'right'):
        name = plugin.findtext(side + '_steering_joint')
        joint = graph.nodes.get(name)
        if joint is None or joint.tag != 'joint':
            raise GeometryError('Missing steering joint: ' + str(name))
        axis = joint.find('axis/xyz')
        if axis is None:
            raise GeometryError('Missing steering axis')
        if joint.findtext('axis/use_parent_model_frame', 'false').lower() not in ('false', '0'):
            raise GeometryError('Legacy axis frame flag requires separate libsdformat inspection')
        values = [number(v, 'steering axis') for v in (axis.text or '').split()]
        if len(values) != 3:
            raise GeometryError('Steering axis requires three numbers')
        axis_frame = axis.get('expressed_in', name)
        _, rotation = graph.resolve(axis_frame)
        resolved_axis = rotate(rotation, values)
        if any(abs(v - target) > 1e-6 for v, target in zip(resolved_axis, (0, 0, 1))):
            raise GeometryError('Steering axes must resolve to positive model Z for these equations')
        lower = number(joint.findtext('axis/limit/lower'), name + ' lower limit')
        upper = number(joint.findtext('axis/limit/upper'), name + ' upper limit')
        if not -math.pi / 2 < lower < 0 < upper < math.pi / 2:
            raise GeometryError('Expected steering limits straddling zero within the principal branch')
        limits[side] = {'joint': name, 'child_link': joint.findtext('child'),
                        'lower_rad': lower, 'upper_rad': upper, 'axis_model': list(resolved_axis)}
    common_limit = min(min(-item['lower_rad'], item['upper_rad']) for item in limits.values())
    # Conservative common symmetric bound. The inner wheel has the larger angle.
    joint_bound = math.atan(math.tan(common_limit) / (1 + width / (2 * length) * math.tan(common_limit)))
    bound = min(joint_bound, max_steer)
    maximum_targets = {name: steering_targets(sign * max_steer, length, width)
                       for name, sign in (('positive', 1), ('negative', -1))}
    violations = []
    for direction, targets in maximum_targets.items():
        for side, angle in targets.items():
            if not limits[side]['lower_rad'] <= angle <= limits[side]['upper_rad']:
                violations.append({'direction': direction, 'joint': limits[side]['joint'],
                                   'target_rad': angle, 'limits': limits[side]})
    radius = None if requested_turning_radius is None else number(requested_turning_radius, 'requested radius')
    if radius is not None and radius <= 0:
        raise GeometryError('Requested turning radius must be positive')
    requested_angle = None if radius is None else math.atan(length / radius)
    base_frame = plugin.findtext('robot_base_frame', 'base_footprint')
    base_position, _ = graph.resolve(base_frame)
    return {
        'schema_version': 1, 'status': 'STATIC_MODEL_AUDITED',
        'scope': 'INITIAL_SDF_COLLISION_GEOMETRY_AND_UPSTREAM_EQUATIONS_ONLY',
        'source': {'path': str(path), 'sha256': hashlib.sha256(payload).hexdigest()},
        'upstream': UPSTREAM, 'model': model.get('name'), 'sdf_version': root.get('version'),
        'wheels': wheels, 'front_axle_center_model_m': front, 'rear_axle_center_model_m': rear,
        'wheelbase_euclidean_m': length, 'front_track_euclidean_m': width,
        'rear_right_radius_m': wheels['rear_right_joint']['collision_radius_m'],
        'plugin_max_speed_mps': max_speed, 'plugin_max_steer_rad': max_steer,
        'steering_joint_limits': limits, 'targets_at_plugin_max_steer_rad': maximum_targets,
        'plugin_max_steer_joint_limit_violations': violations,
        'conservative_symmetric_virtual_bound_rad': bound,
        'conservative_ideal_rear_axle_minimum_radius_m': length / math.tan(bound),
        'requested_turning_radius_m': radius, 'requested_virtual_steer_rad': requested_angle,
        'requested_radius_fits_static_joint_bound': None if radius is None else requested_angle <= bound,
        'base_frame': base_frame, 'base_origin_model_m': list(base_position),
        'base_origin_minus_rear_axle_model_m': [a - b for a, b in zip(base_position, rear)],
        'front_left_minus_right_x_m': centers['front_left_joint'][0] - centers['front_right_joint'][0],
        'runtime_binary_verified': False, 'runtime_geometry_validated': False, 'motion_authorized': False,
        'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
        'limitations': [
            'Distances reproduce initial SDF poses; they are not measured loaded Gazebo collision centers.',
            'A common rigid world spawn pose does not change distances; joint dynamics and contact still require validation.',
            'The nonrectangular front axle and base origin offset require model/reference-point review.',
            'The conservative bound is an offline equation result, not a replacement vehicle limit.',
            'No parameter, model, datum, actuator or command topic is changed by this audit.',
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_sdf', type=Path)
    parser.add_argument('--requested-turning-radius', type=float)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if args.output and (args.output.exists() or args.output.is_symlink()):
        parser.error('Refusing to overwrite existing evidence: ' + str(args.output))
    try:
        result = audit_model(args.model_sdf, args.requested_turning_radius)
        code = 0
    except (OSError, ET.ParseError, GeometryError) as error:
        result = {'schema_version': 1, 'status': 'STATIC_MODEL_AUDIT_FAILED',
                  'error': f'{type(error).__name__}: {error}', 'runtime_geometry_validated': False,
                  'motion_authorized': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
        code = 2
    rendered = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(rendered)
    print(rendered, end='')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
