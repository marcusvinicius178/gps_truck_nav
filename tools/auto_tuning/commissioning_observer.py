#!/usr/bin/env python3
"""Read-only commissioning telemetry with explicit frames and time matching.

/Odom describes base_footprint, 3.502 m ahead of rear_axle_nav. The rigid planar
transform gives p_rear=p_base+R(yaw)*(-3.502,0) and vy_rear=vy_base-3.502*w.
No geometry is changed. Curvature is reported only for |vx|>0.05 m/s. Joint and
stamped reference samples must be within 0.05 simulation seconds of odometry;
Twist receipt times are never silently treated as source simulation stamps.
Absolute map accuracy, no-slip behavior and braking are NOT established here.
The final actuator topic is intentionally not subscribed: published targets and
saturations are read from adapter_events.jsonl, preserving the plugin as its
sole DDS subscriber. This observer measures the achieved odometry/joint states.
"""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import signal
import time

from ackermann_adapter import json_safe, message_info_evidence, private_evidence_file
from commissioning_source import (TRIALS, atomic_json, command_at, finite, private_directory,
                                  protocol, read_small_json)


BASE_TO_REAR_X_M = -3.502
MATCH_WINDOW_S = 0.05
CURVATURE_MIN_SPEED_MPS = 0.05
STEERING_JOINTS = ('front_left_steer_joint', 'front_right_steer_joint')
WHEEL_JOINTS = ('first_wheel_left_joint', 'first_wheel_right_joint',
                'fourth_wheel_left_joint', 'fourth_wheel_right_joint')
ALL_JOINTS = STEERING_JOINTS + WHEEL_JOINTS


class TelemetryError(ValueError):
    pass


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def rear_state(x, y, yaw, vx, vy, yaw_rate):
    values = (x, y, yaw, vx, vy, yaw_rate)
    if not all(finite(value) for value in values):
        raise TelemetryError('Pose and planar twist must be finite')
    return {'x_m': x + BASE_TO_REAR_X_M * math.cos(yaw),
            'y_m': y + BASE_TO_REAR_X_M * math.sin(yaw), 'yaw_rad': yaw,
            'longitudinal_speed_mps': vx,
            'lateral_speed_mps': vy + BASE_TO_REAR_X_M * yaw_rate,
            'yaw_rate_rps': yaw_rate,
            'curvature_inv_m': yaw_rate / vx if abs(vx) > CURVATURE_MIN_SPEED_MPS else None}


def quaternion_yaw(values):
    if len(values) != 4 or not all(finite(value) for value in values):
        raise TelemetryError('Quaternion must contain four finite values')
    x, y, z, w = values
    norm = sum(value * value for value in values)
    if not 0.99 <= norm <= 1.01:
        raise TelemetryError('Quaternion is not normalized within the observation tolerance')
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def closest(samples, stamp, window=MATCH_WINDOW_S):
    valid = [item for item in samples if item.get('valid') and abs(item['stamp_s'] - stamp) <= window + 1e-12]
    if not valid:
        return None
    return min(valid, key=lambda item: (abs(item['stamp_s'] - stamp), item['stamp_s']))


def requested_pose(trial, elapsed, step=0.01):
    """Numerically integrate the versioned request, never the measured motion.

    Midpoint integration at <=10 ms handles the curvature ramps. This reference
    is a requested rear-axle bicycle path, not an independent ground truth.
    """
    protocol(trial)
    if not finite(elapsed) or elapsed < 0 or elapsed > 30:
        raise TelemetryError('Reference elapsed time outside bounded trial')
    n = max(1, math.ceil(elapsed / step))
    dt = elapsed / n
    x = y = yaw = 0.0
    for i in range(n):
        v, w = command_at(trial, (i + 0.5) * dt)
        heading = yaw + 0.5 * w * dt
        x += v * math.cos(heading) * dt
        y += v * math.sin(heading) * dt
        yaw += w * dt
    return x, y, yaw


def trajectory_errors(rear, anchor, trial, elapsed):
    rx, ry, heading = requested_pose(trial, elapsed)
    c, s = math.cos(anchor['yaw_rad']), math.sin(anchor['yaw_rad'])
    expected = {'x_m': anchor['x_m'] + c * rx - s * ry,
                'y_m': anchor['y_m'] + s * rx + c * ry,
                'yaw_rad': wrap_angle(anchor['yaw_rad'] + heading)}
    dx, dy = rear['x_m'] - expected['x_m'], rear['y_m'] - expected['y_m']
    c, s = math.cos(expected['yaw_rad']), math.sin(expected['yaw_rad'])
    return {'requested_pose': expected, 'along_track_error_m': c * dx + s * dy,
            'cross_track_error_m': -s * dx + c * dy,
            'position_error_m': math.hypot(dx, dy),
            'heading_error_rad': wrap_angle(rear['yaw_rad'] - expected['yaw_rad']),
            'reference_scope': 'REQUESTED_REAR_AXLE_PATH_IN_LOCAL_ODOM_ANCHORED_AT_TRIAL_START'}


class Telemetry:
    """Transport-independent raw recorder and strictly stamped matcher."""

    def __init__(self, trial, run_id, emit_raw, emit_matched):
        self.trial, self.run_id = trial, run_id
        protocol(trial)
        self.emit_raw, self.emit_matched = emit_raw, emit_matched
        self.counts = Counter()
        self.joints, self.references, self.odom_history = deque(maxlen=2000), deque(maxlen=2000), deque(maxlen=2000)
        self.pending = deque()
        self.clock = self.latest_odom = self.latest_joints = None
        self.last_stamps = {}
        self.start_sim = self.anchor = None
        self.anchor_stamp = None
        self.maxima = Counter()
        self.last_clock_raw_at = -math.inf

    def raw(self, kind, data):
        self.counts[kind] += 1
        self.emit_raw(json_safe({'kind': kind, 'run_id': self.run_id, **data}))

    def stamp_valid(self, kind, stamp, received):
        if not finite(stamp) or stamp < 0 or not finite(received):
            self.counts[kind + '_invalid_stamp'] += 1
            return False
        previous = self.last_stamps.get(kind)
        if previous is not None and stamp <= previous:
            self.counts[kind + '_nonadvancing_stamp'] += 1
            return False
        self.last_stamps[kind] = stamp
        return True

    def observe_clock(self, stamp, now):
        self.counts['clock_received'] += 1
        if not finite(stamp) or stamp < 0 or not finite(now):
            self.counts['clock_invalid'] += 1
            return
        if self.clock and stamp < self.clock['sim_seconds']:
            self.counts['clock_regressions'] += 1
        advanced = now if not self.clock or stamp > self.clock['sim_seconds'] else self.clock['advanced_at']
        self.clock = {'sim_seconds': stamp, 'received_at': now, 'advanced_at': advanced}
        if now - self.last_clock_raw_at >= 0.05:
            self.raw('clock', self.clock)
            self.last_clock_raw_at = now
        self.flush()

    def observe_joints(self, names, positions, velocities, stamp, now):
        valid = self.stamp_valid('joints', stamp, now)
        if (len(names) != len(set(names)) or len(positions) != len(names)
                or len(velocities) != len(names) or not set(ALL_JOINTS) <= set(names)):
            valid = False
        values = {}
        if valid:
            values = {name: {'position_rad': positions[names.index(name)],
                             'velocity_rad_s': velocities[names.index(name)]} for name in ALL_JOINTS}
            valid = all(finite(value) for item in values.values() for value in item.values())
        item = {'stamp_s': stamp, 'received_at': now, 'valid': valid, 'joints': values,
                'raw_names': names, 'raw_positions': positions, 'raw_velocities': velocities}
        self.raw('joint_states', item)
        self.latest_joints = item
        if valid:
            self.joints.append(item)
        else:
            self.counts['joints_invalid'] += 1

    def observe_reference(self, stamp, frame, speed, yaw_rate, now):
        valid = (self.stamp_valid('reference', stamp, now) and frame == 'rear_axle_nav'
                 and finite(speed) and finite(yaw_rate))
        item = {'stamp_s': stamp, 'frame_id': frame, 'received_at': now, 'valid': valid,
                'requested_speed_mps': speed, 'requested_yaw_rate_rps': yaw_rate}
        self.raw('stamped_reference', item)
        if valid:
            self.references.append(item)
        else:
            self.counts['reference_invalid'] += 1

    def observe_twist(self, topic, values, now, metadata=None):
        self.raw('unstamped_twist', {'topic': topic, 'received_at': now, 'components': values,
            'middleware_info': metadata, 'source_sim_stamp': None,
            'time_scope': 'MONOTONIC_RECEIPT_ONLY_NOT_SOURCE_SIMULATION_TIME'})

    def observe_odom(self, stamp, frame, child, x, y, yaw, vx, vy, yaw_rate, now):
        valid = (self.stamp_valid('odometry', stamp, now) and frame == 'odom' and child == 'base_footprint')
        try:
            rear = rear_state(x, y, yaw, vx, vy, yaw_rate)
        except TelemetryError:
            valid, rear = False, None
        item = {'stamp_s': stamp, 'received_at': now, 'frame_id': frame, 'child_frame_id': child,
                'valid': valid, 'base': {'x_m': x, 'y_m': y, 'yaw_rad': yaw,
                                       'vx_mps': vx, 'vy_mps': vy, 'yaw_rate_rps': yaw_rate},
                'rear_axle_nav': rear}
        self.raw('odometry', item)
        self.latest_odom = item
        if valid:
            self.odom_history.append(item)
            self.pending.append(item)
            if len(self.pending) > 2000:
                self.pending.popleft()
                self.counts['pending_odometry_dropped'] += 1
        else:
            self.counts['odometry_invalid'] += 1
        self.flush()

    def set_trial_start(self, value):
        if not finite(value) or value < 0:
            raise TelemetryError('Invalid source trial start')
        if self.start_sim is not None and self.start_sim != value:
            raise TelemetryError('Source trial start changed; refusing a shifted reference')
        self.start_sim = value
        if self.anchor is None:
            item = closest(self.odom_history, value)
            if item is not None:
                self.anchor, self.anchor_stamp = item['rear_axle_nav'], item['stamp_s']

    def flush(self, force=False):
        if self.start_sim is not None and self.anchor is None:
            self.set_trial_start(self.start_sim)
        while self.pending and (force or self.clock and self.pending[0]['stamp_s'] <= self.clock['sim_seconds'] - 0.1):
            item = self.pending.popleft()
            stamp, rear = item['stamp_s'], item['rear_axle_nav']
            joint = closest(self.joints, stamp)
            reference = closest(self.references, stamp)
            reasons = []
            if joint is None:
                reasons.append('NO_JOINT_SAMPLE_WITHIN_50MS')
                self.counts['unmatched_joint'] += 1
            if reference is None:
                reasons.append('NO_STAMPED_REQUEST_WITHIN_50MS')
                self.counts['unmatched_reference'] += 1
            result = {'schema_version': 1, 'run_id': self.run_id, 'odom_stamp_s': stamp,
                      'match_window_s': MATCH_WINDOW_S, 'observed': rear,
                      'joint_stamp_s': joint['stamp_s'] if joint else None,
                      'joint_dt_s': joint['stamp_s'] - stamp if joint else None,
                      'joints': joint['joints'] if joint else None,
                      'requested_stamp_s': reference['stamp_s'] if reference else None,
                      'requested_dt_s': reference['stamp_s'] - stamp if reference else None,
                      'requested_speed_mps': reference['requested_speed_mps'] if reference else None,
                      'requested_yaw_rate_rps': reference['requested_yaw_rate_rps'] if reference else None,
                      'yaw_rate_error_rps': rear['yaw_rate_rps'] - reference['requested_yaw_rate_rps'] if reference else None,
                      'speed_error_mps': rear['longitudinal_speed_mps'] - reference['requested_speed_mps'] if reference else None,
                      'trajectory': None, 'reasons': reasons}
            if self.start_sim is not None and stamp >= self.start_sim:
                if self.anchor is None:
                    reasons.append('NO_ODOMETRY_ANCHOR_WITHIN_50MS_OF_TRIAL_START')
                elif stamp - self.start_sim <= 30:
                    result['trajectory'] = {**trajectory_errors(rear, self.anchor, self.trial, stamp - self.start_sim),
                        'anchor_stamp_s': self.anchor_stamp, 'trial_start_sim_s': self.start_sim,
                        'anchor_dt_s': self.anchor_stamp - self.start_sim}
                    self.counts['trajectory_metrics'] += 1
                    for key in ('cross_track_error_m', 'position_error_m', 'heading_error_rad'):
                        self.maxima['max_abs_' + key] = max(self.maxima['max_abs_' + key], abs(result['trajectory'][key]))
            self.counts['synchronized_records'] += 1
            if self.start_sim is not None and stamp >= self.start_sim:
                self.counts['trial_synchronized_records'] += 1
                if joint and reference and result['trajectory'] is not None:
                    self.counts['trial_joint_request_trajectory_matched'] += 1
            if joint:
                self.counts['joint_matched'] += 1
            if joint and reference:
                self.counts['joint_and_request_matched'] += 1
            self.maxima['max_abs_longitudinal_speed_mps'] = max(self.maxima['max_abs_longitudinal_speed_mps'], abs(rear['longitudinal_speed_mps']))
            self.maxima['max_abs_yaw_rate_rps'] = max(self.maxima['max_abs_yaw_rate_rps'], abs(rear['yaw_rate_rps']))
            self.emit_matched(json_safe(result))

    def health(self, now):
        reasons = []
        for key, item in (('clock', self.clock), ('odometry', self.latest_odom), ('joint_states', self.latest_joints)):
            if item is None:
                reasons.append(key.upper() + '_MISSING')
            elif not finite(now) or not 0 <= now - item['received_at'] < 0.5:
                reasons.append(key.upper() + '_STALE_WALL_RECEIPT')
            elif item.get('valid') is False:
                reasons.append(key.upper() + '_INVALID')
        if self.clock and now - self.clock['advanced_at'] >= 1:
            reasons.append('CLOCK_STALLED')
        if self.counts['clock_regressions']:
            reasons.append('CLOCK_REGRESSED')
        return reasons

    def summary(self, now):
        return {'schema_version': 1, 'run_id': self.run_id, 'trial': self.trial,
                'status': 'TELEMETRY_COLLECTED', 'counts': dict(self.counts), 'maxima': dict(self.maxima),
                'trial_start_sim_s': self.start_sim, 'anchor_stamp_s': self.anchor_stamp,
                'trajectory_status': 'MEASURED_RELATIVE_TO_REQUEST' if self.counts['trajectory_metrics'] else 'NOT_EXERCISED',
                'health_reasons': self.health(now), 'dynamic_validation_complete': False,
                'physical_stop_verified': False, 'mission_ready': False,
                'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
                'limitations': ['Plugin odometry is not independent ground truth or absolute localization validation.',
                    'A timestamp match is bounded correspondence; no interpolation or missing-sample fabrication.',
                    'Requested path follows the versioned source and is anchored in local odom, not a surveyed map.',
                    'Unstamped Twist has receipt time only. The source stamped reference supplies requested simulation time.',
                    'Published plugin targets, target angles, rejected inputs and saturations are recorded in adapter_events.jsonl; this observer records achieved joint angles without subscribing to the final actuator topic.']}


def attach_subscriptions(node, recorder, types, qos, *, monotonic=time.monotonic):
    """Receive input/reference/sensors only; never add an actuator subscriber."""
    stamp = lambda value: value.sec + value.nanosec / 1e9

    def odom(message):
        pose, twist = message.pose.pose, message.twist.twist
        try:
            yaw = quaternion_yaw((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        except TelemetryError:
            yaw = math.nan
        recorder.observe_odom(stamp(message.header.stamp), message.header.frame_id, message.child_frame_id,
            pose.position.x, pose.position.y, yaw, twist.linear.x, twist.linear.y, twist.angular.z,
            monotonic())

    def twist_callback(topic):
        def callback(message, info):
            _, metadata = message_info_evidence(info)
            values = {part + '_' + axis: float(getattr(getattr(message, part), axis))
                      for part in ('linear', 'angular') for axis in ('x', 'y', 'z')}
            recorder.observe_twist(topic, values, monotonic(), metadata)
        return callback

    handles = [node.create_subscription(types['Clock'], '/clock',
        lambda message: recorder.observe_clock(stamp(message.clock), monotonic()), qos),
        node.create_subscription(types['Odometry'], '/odom', odom, qos),
        node.create_subscription(types['JointState'], '/joint_states', lambda message: recorder.observe_joints(
            list(message.name), list(message.position), list(message.velocity), stamp(message.header.stamp),
            monotonic()), qos),
        node.create_subscription(types['TwistStamped'], '/commissioning/reference', lambda message: recorder.observe_reference(
            stamp(message.header.stamp), message.header.frame_id, message.twist.linear.x,
            message.twist.angular.z, monotonic()), qos)]
    for topic in ('/commissioning/cmd_vel',):
        handles.append(node.create_subscription(types['Twist'], topic, twist_callback(topic), qos))
    return handles


def run(args):
    output = Path(args.evidence_dir)
    owner = private_directory(output)
    stop_requested = False

    def interrupted(_signal, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    started = time.monotonic()
    node = rclpy = None
    initialized = False
    error = None
    with private_evidence_file(output / 'telemetry_raw.jsonl', owner) as raw_stream, \
            private_evidence_file(output / 'telemetry_synchronized.jsonl', owner) as matched_stream:
        raw_count = 0

        def emit_raw(record):
            nonlocal raw_count
            raw_count += 1
            if raw_count > 200000:
                raise TelemetryError('Raw telemetry bound exceeded; evidence is not silently truncated')
            raw_stream.write(json.dumps(record, allow_nan=False) + '\n')
            raw_stream.flush()

        def emit_matched(record):
            matched_stream.write(json.dumps(record, allow_nan=False) + '\n')
            matched_stream.flush()

        recorder = Telemetry(args.trial, args.run_id, emit_raw, emit_matched)
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import JointState
            from rosgraph_msgs.msg import Clock
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            # The supervisor requests a flush with SIGINT after killing the plant.
            # Keep our handlers: ROS defaults would shut down the context first.
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            node = rclpy.create_node('phase_a_commissioning_observer')
            handles = attach_subscriptions(node, recorder,
                {'Clock': Clock, 'Odometry': Odometry, 'JointState': JointState,
                 'TwistStamped': TwistStamped, 'Twist': Twist}, qos_profile_sensor_data)
            seq = 0
            next_heartbeat = started
            while rclpy.ok() and not stop_requested and time.monotonic() - started < args.duration:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if args.source_heartbeat and args.source_heartbeat.is_file():
                    record = read_small_json(args.source_heartbeat)
                    if record.get('run_id') != args.run_id:
                        raise TelemetryError('Source session identity does not match observer')
                    if record.get('run_start_sim_seconds') is not None:
                        recorder.set_trial_start(record['run_start_sim_seconds'])
                if now >= next_heartbeat:
                    seq += 1
                    atomic_json(output / 'observer_heartbeat.json', {'schema_version': 1, 'run_id': args.run_id,
                        'monotonic_s': now, 'seq': seq, 'state': 'OBSERVING_READ_ONLY',
                        'counts': dict(recorder.counts),
                        'clock_count': recorder.counts['clock_received'],
                        'odom_count': recorder.counts['odometry'],
                        'joint_count': recorder.counts['joint_states'],
                        'valid_sync_count': recorder.counts['joint_matched'],
                        'health_reasons': recorder.health(now),
                        'command_publishers_created': 0, 'fault': None}, owner)
                    next_heartbeat = now + 0.1
            if not rclpy.ok():
                raise TelemetryError('ROS context stopped unexpectedly')
            if not stop_requested:
                raise TelemetryError('Observer duration expired without a requested stop')
        except BaseException as exception:
            error = f'{type(exception).__name__}: {exception}'
        finally:
            recorder.flush(force=True)
            summary = recorder.summary(time.monotonic())
            invalid = {key: value for key, value in recorder.counts.items() if value and
                       (key.endswith(('_invalid', '_invalid_stamp')) or
                        key in ('clock_regressions', 'pending_odometry_dropped'))}
            if invalid and error is None:
                error = 'TelemetryError: Invalid telemetry recorded: ' + json.dumps(invalid, sort_keys=True)
            summary['error'] = error
            if error:
                summary['status'] = 'TELEMETRY_INCOMPLETE'
            with private_evidence_file(output / 'telemetry_summary.json', owner) as summary_stream:
                json.dump(summary, summary_stream, indent=2, allow_nan=False)
                summary_stream.write('\n')
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
    return 3 if error else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--trial', choices=tuple(TRIALS), required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--source-heartbeat', type=Path)
    parser.add_argument('--duration', type=float, default=600)
    args = parser.parse_args(argv)
    if not finite(args.duration) or not 1 <= args.duration <= 1800:
        parser.error('duration must be finite in [1,1800] wall seconds')
    if len(args.run_id) != 32 or any(char not in '0123456789abcdef' for char in args.run_id):
        parser.error('run-id must be 32 lowercase hex characters')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
