#!/usr/bin/env python3
"""Receive-only ROS 2 shadow of the prospective Classic Ackermann adapter.

No command publisher, live mode or arming path exists here. JSON candidates are
conditional ideal-rolling calculations, never actuator commands or stop proof.
"""

import argparse
from collections import Counter
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import time

from actuation_contract import ContractError, Geometry, Policy, convert, finite
from audit_ackermann_geometry import audit_model


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def load_policy(path):
    payload = Path(path).read_bytes()
    document = json.loads(payload)
    if document.get('schema_version') != 1 or document.get('mode') != 'SHADOW_ONLY':
        raise ContractError('Policy must explicitly specify schema_version=1 and mode=SHADOW_ONLY')
    values = document.get('policy', {})
    if set(values) != {field.name for field in fields(Policy)}:
        raise ContractError('Policy requires every explicit Policy field and no unknown fields')
    return Policy(**values), {'path': str(path), 'sha256': hashlib.sha256(payload).hexdigest(),
                              'document': document}


def rear_right_offset(audit):
    """Signed lateral offset in the existing model's forward/left axes."""
    center = audit['rear_axle_center_model_m']
    right = audit['wheels']['rear_right_joint']['collision_center_model_m']
    left = audit['wheels']['rear_left_joint']['collision_center_model_m']
    if (not all(finite(v) for p in (center, right, left) for v in p)
            or not right[1] < center[1] < left[1]
            or abs(right[0] - left[0]) > 1e-6 or abs(right[2] - left[2]) > 1e-6):
        raise ContractError('Rear axle must be lateral and ordered right/center/left in model axes')
    return right[1] - center[1]


def convert_classic(speed_mps, yaw_rate_rps, audit, policy):
    """Center Twist -> virtual steering and right-rear rolling-speed target.

    Classic 3.7.0 feeds back REAR_RIGHT joint velocity, NOT axle-center speed,
    and applies identical PID torque to both rear wheels. The feedforward
    v_right=v_center-omega*y_right is only an ideal no-slip target relation.
    Joint sign, slip and closed-loop dynamics still require measured validation.
    """
    geometry = Geometry.from_static_audit(audit)
    y_right = rear_right_offset(audit)
    result = convert(speed_mps, yaw_rate_rps, geometry, policy)
    result['scope'] = 'SHADOW_IDEAL_ROLLING_CALCULATION_ONLY'
    result['rear_right_lateral_offset_m'] = y_right
    result['plugin_linear_semantics'] = 'RIGHT_REAR_JOINT_ROLLING_SPEED_TARGET'
    result['runtime_authorized'] = False
    result['reference_frame_validated'] = False
    result['rolling_and_joint_sign_validated'] = False
    result['physical_stop_verified'] = False
    candidate = result.pop('candidate')
    result['candidate'] = {'rear_axle_speed_mps': candidate['linear_x_mps'],
                           'yaw_rate_rps': candidate['yaw_rate_rps'],
                           'plugin_linear_x_mps': 0.0, 'plugin_angular_z_rad': 0.0}
    if result['status'] not in ('ACCEPTED', 'SCALED'):
        return result
    v, w = candidate['linear_x_mps'], candidate['yaw_rate_rps']
    right_speed = v - w * y_right
    if not finite(right_speed) or right_speed * math.copysign(1.0, v) <= 0:
        result.update(status='REJECTED', reason='RIGHT_WHEEL_TARGET_INVALID_OR_REVERSES_SIGN')
        result['candidate'] = dict.fromkeys(result['candidate'], 0.0)
        return result
    # Center profile caps and plugin right-wheel cap are different constraints.
    factor = min(1.0, geometry.plugin_max_speed_mps / abs(right_speed))
    if factor < 1:
        if policy.speed_policy == 'REJECT':
            result.update(status='REJECTED', reason='RIGHT_WHEEL_PLUGIN_SPEED_LIMIT')
            result['candidate'] = dict.fromkeys(result['candidate'], 0.0)
            return result
        v, w, right_speed = factor * v, factor * w, factor * right_speed
        if abs(v) <= policy.stop_band_mps:
            result.update(status='REJECTED', reason='RIGHT_WHEEL_SATURATION_BELOW_STOP_BAND')
            result['candidate'] = dict.fromkeys(result['candidate'], 0.0)
            return result
        result.update(status='SCALED', reason='CURVATURE_PRESERVED_WITH_RIGHT_WHEEL_CAP')
        result['scale_factor'] *= factor
    result['right_wheel_cap_scale_factor'] = factor
    result['candidate'].update(rear_axle_speed_mps=v, yaw_rate_rps=w,
        plugin_linear_x_mps=right_speed,
        plugin_angular_z_rad=math.copysign(1.0, right_speed) * result['virtual_angle_rad'])
    return result


class ShadowObserver:
    """Transport-independent recorder. It cannot arm, publish or stop a plant."""

    def __init__(self, audit, policy, emit, *, started_at, policy_provenance=None):
        policy.validate(Geometry.from_static_audit(audit))
        rear_right_offset(audit)
        if not finite(started_at):
            raise ContractError('Finite monotonic start time required')
        self.audit, self.policy, self.emit = audit, policy, emit
        self.started_at, self.policy_provenance = started_at, policy_provenance
        self.counts = Counter()
        self.clock = self.odom = self.graph = self.last_command = None
        self.clock_regressions = 0
        self.unexpected_nonzero = False
        self.invalid_command_observed = False

    def observe_clock(self, sim_seconds, *, now):
        if not finite(sim_seconds) or sim_seconds < 0 or not finite(now):
            self.counts['invalid_clock'] += 1
            return
        advancing = self.clock is None or sim_seconds > self.clock['sim_seconds']
        if self.clock and sim_seconds < self.clock['sim_seconds']:
            self.clock_regressions += 1
        last_advance = now if advancing else self.clock['last_advance_at']
        self.clock = {'sim_seconds': sim_seconds, 'received_at': now, 'last_advance_at': last_advance}
        self.counts['clock'] += 1

    def observe_odom(self, speed_mps, yaw_rate_rps, child_frame, *, stamp_seconds, now):
        self.counts['odom'] += 1
        self.odom = {'longitudinal_speed_mps': speed_mps, 'yaw_rate_rps': yaw_rate_rps,
                     'child_frame_id': child_frame, 'stamp_seconds': stamp_seconds, 'received_at': now}

    def observe_graph(self, publishers, subscribers, *, now, error=None):
        self.counts['graph'] += 1
        self.graph = {'publishers': publishers, 'subscribers': subscribers,
                      'received_at': now, 'error': error}

    def health(self, now):
        reasons = []
        if not finite(now) or now < self.started_at:
            return ['MONOTONIC_TIME_INVALID']
        for key, item in (('CLOCK', self.clock), ('ODOMETRY', self.odom), ('GRAPH', self.graph)):
            if item is None:
                reasons.append(key + '_MISSING')
            elif not 0 <= now - item['received_at'] < self.policy.observation_timeout_s:
                reasons.append(key + '_STALE_OR_FUTURE')
        if self.clock and now - self.clock['last_advance_at'] >= self.policy.clock_stall_timeout_s:
            reasons.append('SIM_CLOCK_STALLED')
        if self.clock_regressions:
            reasons.append('SIM_CLOCK_REGRESSED')
        if self.odom:
            if not all(finite(self.odom[key]) for key in ('longitudinal_speed_mps', 'yaw_rate_rps', 'stamp_seconds')):
                reasons.append('ODOMETRY_NONFINITE')
            elif self.clock and not 0 <= self.clock['sim_seconds'] - self.odom['stamp_seconds'] < self.policy.observation_timeout_s:
                reasons.append('ODOMETRY_SIM_STAMP_OUTSIDE_CLOCK_WINDOW')
        if self.graph:
            if self.graph['error']:
                reasons.append('GRAPH_QUERY_FAILED')
            elif len(self.graph['publishers']) != 1:
                reasons.append('INPUT_AUTHORITY_NOT_UNIQUE')
        if self.last_command is None:
            reasons.append('NO_COMMANDS_OBSERVED_EXPECTED_WITHOUT_GOAL')
        elif not 0 <= now - self.last_command['received_at'] < self.policy.command_timeout_s:
            reasons.append('COMMAND_RECEIPT_EXPIRED')
        return reasons

    def observe_command(self, components, *, now, publisher_gid=None, middleware_info=None):
        self.counts['commands'] += 1
        expected_keys = {'linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z'}
        if set(components) != expected_keys:
            raise ContractError('All six Twist components must be recorded')
        nonzero = any(finite(value) and value != 0 for value in components.values())
        self.unexpected_nonzero |= nonzero
        self.invalid_command_observed |= any(not finite(value) for value in components.values())
        self.counts['nonzero_commands'] += nonzero
        result = convert_classic(components['linear_x'], components['angular_z'], self.audit, self.policy)
        unsupported = [key for key in ('linear_y', 'linear_z', 'angular_x', 'angular_y')
                       if not finite(components[key]) or components[key] != 0]
        if unsupported:
            result.update(status='REJECTED', reason='UNSUPPORTED_TWIST_COMPONENTS', unsupported=unsupported)
            result['candidate'] = dict.fromkeys(result['candidate'], 0.0)
        self.counts[result['status']] += 1
        self.last_command = {'received_at': now, 'wall_offset_s': now - self.started_at,
            'publisher_gid': publisher_gid, 'twist': dict(components), 'conversion': result,
            'middleware_info': middleware_info,
            'command_source_authority_verified': False,
            'timestamp_scope': 'OBSERVER_MONOTONIC_RECEIPT_ONLY_TWIST_HAS_NO_SOURCE_STAMP'}
        self.last_command['observation_reasons'] = self.health(now)
        if publisher_gid is None:
            self.last_command['observation_reasons'].append('SOURCE_GID_UNAVAILABLE_IN_RCLPY_IRON')
        elif self.graph and publisher_gid not in [item.get('gid') for item in self.graph['publishers']]:
            self.last_command['observation_reasons'].append('SOURCE_GID_NOT_IN_OBSERVED_GRAPH')
        self.emit(json_safe(self.last_command))
        return result

    def summary(self, now):
        return json_safe({'schema_version': 1, 'status': 'SHADOW_COMPLETE',
            'diagnostic_status': 'BLOCKED_ACTUATION_REVIEW', 'mode': 'SHADOW_ONLY',
            'command_publishers_created': 0, 'runtime_authorized': False,
            'command_source_authority_verified': False,
            'physical_stop_verified': False, 'mission_ready': False,
            'conversion_status': 'OBSERVED_INPUTS_CALCULATED' if self.counts['commands'] else 'NOT_EXERCISED',
            'unexpected_nonzero_observed': self.unexpected_nonzero,
            'invalid_command_observed': self.invalid_command_observed,
            'counts': dict(self.counts), 'elapsed_wall_s': now - self.started_at,
            'clock': self.clock, 'odometry': self.odom, 'input_graph': self.graph,
            'last_command': self.last_command, 'observation_reasons': self.health(now),
            'model_audit': self.audit, 'policy_source': self.policy_provenance,
            'limitations': ['No destination watchdog: this recorder cannot stop a simulator or physical plant.',
                'No live publisher or enable path; process death produces no stop command.',
                'Rear-axle reference and ideal rolling/joint-sign assumptions require measured validation.',
                'Twist receipt freshness does not establish producer timestamp freshness.',
                'Iron rclpy message-info dictionaries expose timestamps/sequences but no publisher GID; graph alone does not identify a command source.',
                'No command during a no-goal probe is expected and does not verify dynamic conversion.'],
            'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'})


def gid_text(value):
    try:
        return bytes(value).hex() or None
    except (TypeError, ValueError):
        return None


def message_info_evidence(info):
    """Preserve Iron's actual dict; do not invent a GID from graph endpoints.

    ros2/rclpy iron subscription.cpp blob 24b18fc701a1735d30536156ef57821dde052327
    returns these four fields without publisher_gid. Later transports may expose
    a real GID, which can be recorded and compared without granting authority.
    Middleware timestamps are retained verbatim, not interpreted as ROS /clock.
    """
    keys = ('source_timestamp', 'received_timestamp',
            'publication_sequence_number', 'reception_sequence_number')
    if isinstance(info, dict):
        fields = {key: info[key] for key in keys if key in info}
        raw_gid = info.get('publisher_gid')
    else:
        fields = {key: getattr(info, key) for key in keys if hasattr(info, key)}
        raw_gid = getattr(info, 'publisher_gid', None)
    return gid_text(raw_gid), {'fields': fields,
        'representation': 'DICT' if isinstance(info, dict) else 'OBJECT',
        'timestamp_scope': 'MIDDLEWARE_REPORTED_NO_ROS_CLOCK_EQUIVALENCE_ASSUMED',
        'publisher_gid_available': raw_gid is not None}


def attach_subscriptions(node, observer, types, qos, *, input_topic, odom_topic, monotonic=time.monotonic):
    def command(message, info):
        components = {kind + '_' + axis: float(getattr(getattr(message, kind), axis))
                      for kind in ('linear', 'angular') for axis in ('x', 'y', 'z')}
        publisher_gid, middleware_info = message_info_evidence(info)
        observer.observe_command(components, now=monotonic(), publisher_gid=publisher_gid,
                                 middleware_info=middleware_info)

    def clock(message):
        observer.observe_clock(message.clock.sec + message.clock.nanosec / 1e9, now=monotonic())

    def odom(message):
        observer.observe_odom(float(message.twist.twist.linear.x), float(message.twist.twist.angular.z),
            message.child_frame_id, stamp_seconds=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
            now=monotonic())

    return [node.create_subscription(types['Twist'], input_topic, command, qos),
            node.create_subscription(types['Clock'], '/clock', clock, qos),
            node.create_subscription(types['Odometry'], odom_topic, odom, qos)]


def run_shadow(model, policy_path, output_dir, *, duration=30.0, input_topic='/cmd_vel', odom_topic='/odom'):
    if not finite(duration) or not 1 <= duration <= 1800:
        raise ContractError('Duration must be finite within [1, 1800] wall seconds')
    if any(not topic.startswith('/') or topic == '/' for topic in (input_topic, odom_topic)):
        raise ContractError('Explicit absolute input and odometry topic names required')
    audit = audit_model(model)
    policy, provenance = load_policy(policy_path)
    policy.validate(Geometry.from_static_audit(audit))
    output = Path(output_dir)
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    started = time.monotonic()
    node = None
    rclpy = None
    initialized = False
    status_error = None
    with (output / 'shadow_commands.jsonl').open('x') as stream:
        def emit(record):
            # Bound disk use even under unexpected high-rate producers; counters continue.
            if observer.counts['commands'] <= 10000:
                stream.write(json.dumps(record, allow_nan=False) + '\n')
                stream.flush()
            else:
                observer.counts['command_records_dropped'] += 1
        observer = ShadowObserver(audit, policy, emit, started_at=started, policy_provenance=provenance)
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from rclpy.qos import qos_profile_sensor_data
            rclpy.init(args=[])
            initialized = True
            node = rclpy.create_node('phase_a_ackermann_shadow')
            subscriptions = attach_subscriptions(node, observer,
                {'Twist': Twist, 'Odometry': Odometry, 'Clock': Clock}, qos_profile_sensor_data,
                input_topic=input_topic, odom_topic=odom_topic)
            next_graph = started
            while rclpy.ok() and time.monotonic() - started < duration:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now >= next_graph:
                    try:
                        def endpoint(item):
                            return {'node': item.node_namespace.rstrip('/') + '/' + item.node_name,
                                    'gid': gid_text(item.endpoint_gid), 'type': item.topic_type}
                        observer.observe_graph([endpoint(x) for x in node.get_publishers_info_by_topic(input_topic)],
                            [endpoint(x) for x in node.get_subscriptions_info_by_topic(input_topic)], now=now)
                    except Exception as error:
                        observer.observe_graph([], [], now=now, error=f'{type(error).__name__}: {error}')
                    next_graph = now + min(0.2, policy.observation_timeout_s / 2)
                if observer.unexpected_nonzero or observer.invalid_command_observed:
                    # Report immediately to the owning probe; no stop is claimed or sent.
                    break
            if not rclpy.ok():
                status_error = 'ROS_CONTEXT_STOPPED_BEFORE_DURATION'
        except BaseException as error:
            status_error = f'{type(error).__name__}: {error}'
            raise
        finally:
            result = observer.summary(time.monotonic())
            result.update(input_topic=input_topic, odom_topic=odom_topic, error=status_error)
            if status_error:
                result['status'] = 'SHADOW_INCOMPLETE'
            elif observer.unexpected_nonzero or observer.invalid_command_observed:
                result['status'] = 'SHADOW_ABORTED_UNEXPECTED_INPUT'
            with (output / 'shadow_summary.json').open('x') as summary:
                json.dump(result, summary, indent=2, allow_nan=False)
                summary.write('\n')
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
    return 4 if observer.unexpected_nonzero or observer.invalid_command_observed else 0 if status_error is None else 3


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--input-topic', default='/cmd_vel')
    parser.add_argument('--odom-topic', default='/odom')
    args = parser.parse_args(argv)
    return run_shadow(args.model, args.policy, args.output_dir, duration=args.duration,
                      input_topic=args.input_topic, odom_topic=args.odom_topic)


if __name__ == '__main__':
    raise SystemExit(main())
