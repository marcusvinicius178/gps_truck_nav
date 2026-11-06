#!/usr/bin/env python3
"""One explicitly authorized, bounded simulation commissioning input.

Preparation is offline and creates no ROS context or publisher. Execution is
only for the isolated commissioning session, after its separate authorization
checks. Nominal 0.3 m/s is an experimental command, not a validated vehicle
capability. Stopping publication is NOT a physical stop: the independent
session supervisor must end the simulator on any failure.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import time

from ackermann_adapter import inherit_evidence_owner, private_evidence_file


TRIALS = {'straight': 0.0, 'left': 0.01, 'right': -0.01}
NOMINAL_SPEED_MPS = 0.3
RAMP_ACCEL_MPS2 = 0.1
RAMP_SECONDS = 3.0
HOLD_SECONDS = 8.0
STOP_SECONDS = 3.0
COMMAND_SECONDS = 2 * RAMP_SECONDS + HOLD_SECONDS
COMPLETE_SECONDS = COMMAND_SECONDS + STOP_SECONDS
MAX_RUN_SIM_SECONDS = 30.0
MAX_RUN_WALL_SECONDS = 45.0
LEASE_TIMEOUT_SECONDS = 0.75
HEARTBEAT_TIMEOUT_SECONDS = 0.5
CLOCK_STALL_SECONDS = 1.0
MAX_SIM_CLOCK_STEP_SECONDS = 0.25


class SourceError(ValueError):
    pass


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def protocol(trial):
    if trial not in TRIALS:
        raise SourceError('Unreviewed trial; only straight, left or right is supported')
    return {'schema_version': 1, 'trial': trial, 'scope': 'SIMULATION_COMMISSIONING',
            'nominal_rear_axle_speed_mps': NOMINAL_SPEED_MPS,
            'command_ramp_mps2': RAMP_ACCEL_MPS2, 'ramp_seconds': RAMP_SECONDS,
            'hold_seconds': HOLD_SECONDS, 'stop_observation_seconds': STOP_SECONDS,
            'plateau_curvature_inv_m': TRIALS[trial], 'curvature_ramp_seconds': 1.0,
            'curvature_during_speed_ramps_inv_m': 0.0, 'reverse_allowed': False,
            'nominal_command_distance_m': NOMINAL_SPEED_MPS * (HOLD_SECONDS + RAMP_SECONDS),
            'max_run_sim_seconds': MAX_RUN_SIM_SECONDS, 'max_run_wall_seconds': MAX_RUN_WALL_SECONDS,
            'max_consecutive_clock_step_s': MAX_SIM_CLOCK_STEP_SECONDS,
            'input_topic': '/commissioning/cmd_vel', 'reference_topic': '/commissioning/reference',
            'input_reference': 'rear_axle_nav', 'physical_stop_verified': False,
            'dynamic_validation_complete': False, 'mission_ready': False,
            'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}


def command_at(trial, elapsed):
    """Deterministic trapezoid. Zero band preserves the no-yaw-at-stop contract.

    Ramps and integral are requested inputs only; their achieved acceleration
    and distance must be measured. No discontinuous division by a speed epsilon.
    """
    protocol(trial)
    if not finite(elapsed) or elapsed < 0:
        raise SourceError('Finite nonnegative elapsed simulation time required')
    if elapsed < RAMP_SECONDS:
        speed = RAMP_ACCEL_MPS2 * elapsed
    elif elapsed < RAMP_SECONDS + HOLD_SECONDS:
        speed = NOMINAL_SPEED_MPS
    elif elapsed < COMMAND_SECONDS:
        speed = RAMP_ACCEL_MPS2 * (COMMAND_SECONDS - elapsed)
    else:
        speed = 0.0
    speed = min(NOMINAL_SPEED_MPS, max(0.0, speed))
    if speed <= 0.01:
        speed = 0.0
    plateau = elapsed - RAMP_SECONDS
    curvature_factor = max(0.0, min(1.0, plateau, HOLD_SECONDS - plateau))
    return speed, TRIALS[trial] * curvature_factor * speed


def private_directory(path):
    path = Path(path)
    parent = path.parent.stat()
    owner = (parent.st_uid, parent.st_gid)
    path.mkdir(mode=0o700, exist_ok=False)
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        inherit_evidence_owner(fd, owner)
        os.fchmod(fd, 0o700)
    finally:
        os.close(fd)
    return owner


def atomic_json(path, payload, owner):
    path = Path(path)
    temporary = path.with_name(path.name + '.new')
    # Never follow a stale or hostile temporary file; no evidence overwrite.
    with private_evidence_file(temporary, owner) as stream:
        json.dump(payload, stream, allow_nan=False, sort_keys=True)
        stream.write('\n')
    temporary.replace(path)


def read_small_json(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 65536:
        raise SourceError('Expected a small regular JSON control file')
    result = json.loads(path.read_text())
    if not isinstance(result, dict):
        raise SourceError('Control record must be an object')
    return result


def approval_record(path, run_id, trial):
    if re.fullmatch(r'[0-9a-f]{32}', run_id or '') is None:
        raise SourceError('Explicit 32-hex session identity required')
    record = read_small_json(path)
    if (record.get('schema_version') != 1 or record.get('run_id') != run_id
            or record.get('trial') != trial or record.get('scope') != 'SIMULATION_COMMISSIONING'
            or record.get('motion_authorized') is not True):
        raise SourceError('Missing or mismatched explicit commissioning authorization')
    return {'document': record, 'sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest()}


def lease_fresh(path, now=None):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 128:
        raise SourceError('Independent supervisor lease is missing or malformed')
    try:
        stamp = float(path.read_text().strip())
    except (ValueError, UnicodeError) as error:
        raise SourceError('Independent supervisor lease is malformed') from error
    # Read first: a concurrent atomic writer may publish a timestamp later
    # than a monotonic value captured before this read. No future tolerance is
    # needed when the runtime observation time is sampled after the payload.
    now = time.monotonic() if now is None else now
    if not finite(now) or not finite(stamp) or not 0 <= now - stamp < LEASE_TIMEOUT_SECONDS:
        raise SourceError('Independent supervisor lease expired or is in the future')
    return stamp


def adapter_state(path, run_id, now=None):
    record = read_small_json(path)
    stamp = record.get('monotonic_s')
    now = time.monotonic() if now is None else now
    if (record.get('run_id') != run_id or not finite(stamp)
            or not 0 <= now - stamp < HEARTBEAT_TIMEOUT_SECONDS):
        raise SourceError('Adapter heartbeat identity or freshness failed')
    state = record.get('state')
    if state not in ('ARMING', 'ARMED') or record.get('fault'):
        raise SourceError('Adapter is not in a commissionable state')
    return state


class SourceTimeline:
    """Pure clock/phase model; never publishes or authorizes a test."""

    def __init__(self, trial, started_wall):
        self.plan = protocol(trial)
        if not finite(started_wall):
            raise SourceError('Finite monotonic start time required')
        self.started_wall = self.last_wall = started_wall
        self.sim = self.clock_received = self.clock_advanced = self.run_start_sim = None
        self.phase = 'WAITING_FOR_ADAPTER'
        self.last_tick_sim = None

    def observe_clock(self, sim, now):
        if not finite(sim) or sim < 0 or not finite(now) or now < self.last_wall:
            raise SourceError('Clock is nonfinite, negative or monotonic time regressed')
        if self.sim is not None and sim < self.sim:
            raise SourceError('Simulation clock regressed')
        if self.sim is not None and sim - self.sim > MAX_SIM_CLOCK_STEP_SECONDS:
            raise SourceError('Simulation clock jumped forward beyond the command sampling bound')
        if self.sim is None or sim > self.sim:
            self.clock_advanced = now
        self.sim, self.clock_received = sim, now
        self.last_wall = now

    def tick(self, now, adapter_armed):
        if not finite(now) or now < self.last_wall:
            raise SourceError('Monotonic clock regressed')
        self.last_wall = now
        if now - self.started_wall >= MAX_RUN_WALL_SECONDS:
            raise SourceError('Source wall-duration bound reached')
        if self.sim is None:
            if now - self.started_wall >= CLOCK_STALL_SECONDS:
                raise SourceError('Simulation clock missing')
            return 0.0, 0.0
        if now - self.clock_received >= 0.5 or now - self.clock_advanced >= CLOCK_STALL_SECONDS:
            raise SourceError('Simulation clock stale or stalled')
        if self.run_start_sim is None:
            if not adapter_armed:
                return 0.0, 0.0
            self.run_start_sim = self.sim
        elif not adapter_armed:
            raise SourceError('Adapter disarmed during trial; no automatic restart')
        elapsed = self.sim - self.run_start_sim
        if elapsed >= MAX_RUN_SIM_SECONDS:
            raise SourceError('Source simulation-duration bound reached')
        self.phase = ('RUNNING' if elapsed < COMMAND_SECONDS else
                      'STOPPING' if elapsed < COMPLETE_SECONDS else 'COMPLETE')
        return command_at(self.plan['trial'], elapsed)


def execute(args):
    evidence = approval_record(args.approval_file, args.run_id, args.trial)
    lease_fresh(args.lease_file)
    output = Path(args.evidence_dir)
    owner = private_directory(output)
    stopped = False

    def interrupt(_signal, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    timeline = SourceTimeline(args.trial, time.monotonic())
    count = sequence = 0
    error = None
    node = rclpy = publisher = None
    initialized = False
    subscriptions = []
    with private_evidence_file(output / 'source_commands.jsonl', owner) as stream:
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped
            from rosgraph_msgs.msg import Clock
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
            rclpy.init(args=[])
            initialized = True
            node = rclpy.create_node('phase_a_commissioning_source')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            publisher = node.create_publisher(Twist, '/commissioning/cmd_vel', qos)
            reference = node.create_publisher(TwistStamped, '/commissioning/reference', qos)

            def clock(message):
                timeline.observe_clock(message.clock.sec + message.clock.nanosec / 1e9, time.monotonic())

            subscriptions.append(node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data))
            next_send = time.monotonic()
            while rclpy.ok() and not stopped:
                rclpy.spin_once(node, timeout_sec=0.01)
                lease_fresh(args.lease_file)
                now = time.monotonic()
                if now < next_send:
                    continue
                # Allow the adapter to start, but never command motion before a
                # current ARMED record. Startup itself is wall-bounded above.
                heartbeat_path = Path(args.adapter_heartbeat)
                if not heartbeat_path.exists() and timeline.run_start_sim is None:
                    armed = False
                    if now - timeline.started_wall >= 5.0:
                        raise SourceError('Adapter heartbeat missing at startup')
                else:
                    armed = adapter_state(heartbeat_path, args.run_id) == 'ARMED'
                now = time.monotonic()  # Timeline sees time after every control-file read.
                speed, yaw = timeline.tick(now, armed)
                command = Twist()
                command.linear.x, command.angular.z = speed, yaw
                publisher.publish(command)
                if timeline.sim is not None:
                    stamped = TwistStamped()
                    ns = round(timeline.sim * 1e9)
                    stamped.header.stamp.sec, stamped.header.stamp.nanosec = divmod(ns, 1_000_000_000)
                    stamped.header.frame_id = 'rear_axle_nav'
                    stamped.twist = command
                    reference.publish(stamped)
                count += 1
                sequence += 1
                record = {'schema_version': 1, 'run_id': args.run_id, 'seq': sequence,
                          'monotonic_s': now, 'sim_seconds': timeline.sim,
                          'run_start_sim_seconds': timeline.run_start_sim, 'state': timeline.phase,
                          'requested_speed_mps': speed, 'requested_yaw_rate_rps': yaw,
                          'command_count': count, 'fault': None,
                          'physical_stop_verified': False, 'mission_ready': False}
                stream.write(json.dumps(record, allow_nan=False) + '\n')
                stream.flush()
                atomic_json(output / 'source_heartbeat.json', record, owner)
                next_send = now + 0.05
            if not stopped:
                raise SourceError('ROS context ended before supervisor request')
        except BaseException as exception:
            error = f'{type(exception).__name__}: {exception}'
        finally:
            # Best effort is evidence only: the parent must independently kill
            # the simulation on process loss/fault, regardless of this message.
            if publisher is not None:
                try:
                    from geometry_msgs.msg import Twist
                    publisher.publish(Twist())
                except Exception:
                    pass
            summary = {'schema_version': 1, 'run_id': args.run_id,
                       'status': 'SOURCE_FAULT' if error else 'SOURCE_STOPPED_BY_SUPERVISOR',
                       'last_phase': timeline.phase, 'protocol': timeline.plan,
                       'approval': evidence, 'command_count': count, 'error': error,
                       'physical_stop_verified': False, 'dynamic_validation_complete': False,
                       'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
            if error:
                atomic_json(output / 'source_heartbeat.json', {
                    'schema_version': 1, 'run_id': args.run_id, 'seq': sequence + 1,
                    'state': 'FAULT', 'monotonic_s': time.monotonic(), 'fault': error,
                    'command_count': count, 'physical_stop_verified': False}, owner)
            with private_evidence_file(output / 'source_summary.json', owner) as summary_stream:
                json.dump(summary, summary_stream, indent=2, allow_nan=False)
                summary_stream.write('\n')
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
    return 3 if error else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--prepare', action='store_true')
    mode.add_argument('--execute', action='store_true')
    parser.add_argument('--trial', choices=tuple(TRIALS), required=True)
    for name in ('evidence-dir', 'approval-file', 'lease-file', 'adapter-heartbeat'):
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--run-id')
    args = parser.parse_args(argv)
    if args.prepare:
        print(json.dumps({**protocol(args.trial), 'status': 'OFFLINE_PROTOCOL_PREPARED',
                          'command_publishers_created': 0, 'motion_authorized': False}, indent=2))
        return 0
    for key in ('evidence_dir', 'approval_file', 'lease_file', 'adapter_heartbeat', 'run_id'):
        if getattr(args, key) is None:
            parser.error('--execute requires --' + key.replace('_', '-'))
    return execute(args)


if __name__ == '__main__':
    raise SystemExit(main())
