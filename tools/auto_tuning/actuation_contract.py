"""Offline Ackermann conversion and supervisory model, without ROS or I/O.

Nothing in this module sends a command, provides a runtime watchdog or grants
motion permission. Review assertions in tests are synthetic. A future adapter
must supply independently verified identity, geometry, routing and stop proof.
"""

from dataclasses import dataclass
import math
import re

from audit_ackermann_geometry import steering_targets


REVIEW_REQUIREMENTS = (
    'runtime_geometry_approved', 'reference_frame_approved',
    'units_and_reverse_semantics_approved', 'physical_limits_approved',
    'routing_approved', 'destination_watchdog_verified',
    'test_authorization_recorded',
)


class ContractError(ValueError):
    pass


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sha256(value):
    return isinstance(value, str) and re.fullmatch(r'[0-9a-f]{64}', value) is not None


@dataclass(frozen=True)
class Geometry:
    wheelbase_m: float
    track_m: float
    plugin_max_speed_mps: float
    plugin_max_steer_rad: float
    left_lower_rad: float
    left_upper_rad: float
    right_lower_rad: float
    right_upper_rad: float
    model_sha256: str

    def validate(self):
        values = tuple(value for key, value in vars(self).items() if key != 'model_sha256')
        if not all(finite(value) for value in values) or not sha256(self.model_sha256):
            raise ContractError('Finite geometry and an explicit model SHA-256 are required')
        if min(self.wheelbase_m, self.track_m, self.plugin_max_speed_mps) <= 0:
            raise ContractError('Geometry and plugin speed limit must be positive')
        if not 0 < self.plugin_max_steer_rad < math.pi / 2:
            raise ContractError('Plugin steering limit must use the principal steering branch')
        for low, high in ((self.left_lower_rad, self.left_upper_rad),
                          (self.right_lower_rad, self.right_upper_rad)):
            if not -math.pi / 2 < low < 0 < high < math.pi / 2:
                raise ContractError('Joint limits must straddle zero within the principal branch')

    @classmethod
    def from_static_audit(cls, audit):
        """Read the static result; this does not upgrade its validation status."""
        joints = audit['steering_joint_limits']
        result = cls(audit['wheelbase_euclidean_m'], audit['front_track_euclidean_m'],
                     audit['plugin_max_speed_mps'], audit['plugin_max_steer_rad'],
                     joints['left']['lower_rad'], joints['left']['upper_rad'],
                     joints['right']['lower_rad'], joints['right']['upper_rad'],
                     audit['source']['sha256'])
        result.validate()
        return result

    def virtual_bound_rad(self):
        self.validate()
        common = min(-self.left_lower_rad, self.left_upper_rad,
                     -self.right_lower_rad, self.right_upper_rad)
        if not math.isfinite(2 * self.wheelbase_m):
            raise ContractError('Wheelbase arithmetic overflow')
        ratio = self.track_m / (2 * self.wheelbase_m)
        denominator = 1 + ratio * math.tan(common)
        if not math.isfinite(denominator) or denominator <= 0:
            raise ContractError('Unsupported or overflowing geometry ratio')
        return min(self.plugin_max_steer_rad, math.atan(math.tan(common) / denominator))


@dataclass(frozen=True)
class Policy:
    # All physical/timing choices are explicit; there is no operational default.
    forward_speed_mps: float
    reverse_speed_mps: float
    yaw_rate_rps: float
    stop_band_mps: float
    resume_speed_mps: float
    command_timeout_s: float
    observation_timeout_s: float
    clock_stall_timeout_s: float
    speed_policy: str

    def validate(self, geometry):
        values = tuple(value for key, value in vars(self).items() if key != 'speed_policy')
        if not all(finite(value) for value in values):
            raise ContractError('All policy values must be explicit finite numbers')
        if not 0 < self.stop_band_mps < self.resume_speed_mps < self.forward_speed_mps:
            raise ContractError('Explicit stop/resume hysteresis must fit the forward envelope')
        if self.reverse_speed_mps < 0 or (self.reverse_speed_mps and self.reverse_speed_mps <= self.resume_speed_mps):
            raise ContractError('Reverse must be disabled at zero or exceed the resume threshold')
        if max(self.forward_speed_mps, self.reverse_speed_mps) > geometry.plugin_max_speed_mps:
            raise ContractError('Profile speed must not exceed the plugin speed limit')
        if min(self.yaw_rate_rps, self.command_timeout_s, self.observation_timeout_s,
               self.clock_stall_timeout_s) <= 0:
            raise ContractError('Yaw and monotonic timing limits must be positive')
        if self.speed_policy not in ('REJECT', 'SCALE_PRESERVE_CURVATURE'):
            raise ContractError('Speed policy must explicitly reject or preserve curvature')


def decision(status, reason, **details):
    return {'status': status, 'reason': reason, 'scope': 'OFFLINE_MODEL_ONLY',
            'candidate': {'linear_x_mps': 0.0, 'plugin_angular_z_rad': 0.0,
                          'yaw_rate_rps': 0.0},
            'runtime_authorized': False, 'physical_stop_verified': False, **details}


def convert(speed_mps, yaw_rate_rps, geometry, policy, *, held_at_stop=False):
    """Map a rear-axle bicycle (v, omega) to the upstream plugin's input.

    An impossible curvature is rejected, never clipped. Approved overspeed
    scaling scales v and omega together. This is not a physical vehicle model.
    """
    geometry.validate()
    policy.validate(geometry)
    if not finite(speed_mps) or not finite(yaw_rate_rps):
        return decision('REJECTED', 'NONFINITE_INPUT')
    requested = {'linear_x_mps': speed_mps, 'yaw_rate_rps': yaw_rate_rps}
    if abs(speed_mps) <= policy.stop_band_mps:
        return decision('STOP' if yaw_rate_rps == 0 else 'REJECTED',
                        'ZERO_OR_STOP_BAND' if yaw_rate_rps == 0 else 'UNREALIZABLE_NEAR_ZERO_YAW',
                        requested=requested)
    if speed_mps < 0 and policy.reverse_speed_mps == 0:
        return decision('REJECTED', 'REVERSE_DISABLED', requested=requested)
    curvature = yaw_rate_rps / speed_mps
    product = geometry.wheelbase_m * curvature
    if not math.isfinite(curvature) or not math.isfinite(product):
        return decision('REJECTED', 'NUMERIC_OVERFLOW', requested=requested)
    virtual_angle = math.atan(product)
    bound = geometry.virtual_bound_rad()
    if abs(virtual_angle) > bound:
        return decision('REJECTED', 'CURVATURE_EXCEEDS_STATIC_JOINT_ENVELOPE',
                        requested=requested, requested_curvature_inv_m=curvature,
                        virtual_angle_rad=virtual_angle, virtual_bound_rad=bound)
    targets = steering_targets(virtual_angle, geometry.wheelbase_m, geometry.track_m)
    if not (geometry.left_lower_rad <= targets['left'] <= geometry.left_upper_rad
            and geometry.right_lower_rad <= targets['right'] <= geometry.right_upper_rad):
        return decision('REJECTED', 'JOINT_TARGET_OUTSIDE_LIMIT', requested=requested, targets=targets)
    cap = policy.forward_speed_mps if speed_mps > 0 else policy.reverse_speed_mps
    factor = min(1.0, cap / abs(speed_mps))
    if yaw_rate_rps:
        factor = min(factor, policy.yaw_rate_rps / abs(yaw_rate_rps))
    if factor < 1 and policy.speed_policy == 'REJECT':
        return decision('REJECTED', 'SPEED_OR_YAW_LIMIT', requested=requested)
    applied_speed, applied_yaw = factor * speed_mps, factor * yaw_rate_rps
    if not all(finite(value) for value in (applied_speed, applied_yaw)) or abs(applied_speed) <= policy.stop_band_mps:
        return decision('REJECTED', 'SATURATION_BELOW_STOP_BAND', requested=requested)
    if held_at_stop and abs(applied_speed) < policy.resume_speed_mps:
        return decision('HOLD', 'STOP_HYSTERESIS', requested=requested)
    # Gazebo multiplies its input by sign(v); compensate once, including reverse.
    plugin_input = math.copysign(1.0, applied_speed) * virtual_angle
    return decision('SCALED' if factor < 1 else 'ACCEPTED', 'CURVATURE_PRESERVED',
                    requested=requested,
                    candidate={'linear_x_mps': applied_speed, 'plugin_angular_z_rad': plugin_input,
                               'yaw_rate_rps': applied_yaw},
                    scale_factor=factor, curvature_inv_m=curvature,
                    virtual_angle_rad=virtual_angle, joint_targets_rad=targets,
                    model_sha256=geometry.model_sha256,
                    input_reference='REAR_AXLE_BICYCLE_LONGITUDINAL_SPEED',
                    slew_rates_and_physical_dynamics_verified=False)


class OfflineSupervisor:
    """A deterministic model for tests, not a running final-stage watchdog.

    Snapshot timestamps and command receipt times are monotonic wall seconds.
    Calls cannot guard against this model/process itself ceasing execution.
    """

    def __init__(self, geometry, policy):
        geometry.validate()
        geometry.virtual_bound_rad()
        policy.validate(geometry)
        self.geometry, self.policy = geometry, policy
        self.state = 'DISARMED'
        self.last_now = self.last_sim = self.last_clock_advance = None
        self.last_command_at = None
        self.last_direction = 0
        self.stop_requested_at = None
        self.stop_confirmed = False
        self.armed_identity = None
        self.last_result = decision('INHIBITED', 'EXPLICIT_ARM_REQUIRED')

    def _result(self, result):
        result['model_state'] = self.state
        self.last_result = result
        return result

    def _fault(self, reason):
        self.state = 'FAULT'
        self.stop_confirmed = False
        return self._result(decision('INHIBITED', reason))

    def _observe_standstill(self, snapshot):
        # A past standstill is not a lease to reverse after the plant rolls again.
        self.stop_confirmed = (self.stop_requested_at is not None
                               and snapshot['odom_received_at'] >= self.stop_requested_at
                               and abs(snapshot['measured_speed_mps']) <= self.policy.stop_band_mps)

    def _health(self, snapshot, now):
        if not isinstance(snapshot, dict):
            return 'OBSERVATION_MALFORMED'
        if not finite(now) or (self.last_now is not None and now < self.last_now):
            return 'MONOTONIC_CLOCK_INVALID_OR_REGRESSED'
        self.last_now = now
        sim = snapshot.get('sim_seconds')
        if not finite(sim) or sim < 0:
            return 'SIM_CLOCK_INVALID'
        if self.last_sim is not None and sim < self.last_sim:
            return 'SIM_CLOCK_REGRESSED'
        if self.last_sim is None or sim > self.last_sim:
            self.last_clock_advance = now
        self.last_sim = sim
        if now - self.last_clock_advance >= self.policy.clock_stall_timeout_s:
            return 'SIM_CLOCK_STALLED'
        for name in ('clock_received_at', 'odom_received_at', 'graph_received_at'):
            received = snapshot.get(name)
            if not finite(received) or not 0 <= now - received < self.policy.observation_timeout_s:
                return 'STALE_OR_INVALID_' + name.upper()
        if not finite(snapshot.get('measured_speed_mps')):
            return 'MEASURED_SPEED_INVALID'
        if not -self.policy.reverse_speed_mps <= snapshot['measured_speed_mps'] <= self.policy.forward_speed_mps:
            return 'MEASURED_OVERSPEED'
        expected = snapshot.get('expected_actuator_publisher')
        if not isinstance(expected, str) or not expected or snapshot.get('actuator_publishers') != [expected]:
            return 'ACTUATOR_AUTHORITY_NOT_UNIQUE'
        upstream = snapshot.get('expected_input_publisher')
        if not isinstance(upstream, str) or not upstream or snapshot.get('input_publishers') != [upstream]:
            return 'INPUT_AUTHORITY_NOT_UNIQUE'
        installed, mapped = snapshot.get('installed_library_sha256'), snapshot.get('mapped_library_sha256')
        if not sha256(installed) or not sha256(mapped) or installed != mapped:
            return 'BINARY_IDENTITY_UNVERIFIED_OR_CHANGED'
        if snapshot.get('loaded_model_sha256') != self.geometry.model_sha256:
            return 'MODEL_IDENTITY_MISMATCH'
        if not isinstance(snapshot.get('review_record_id'), str) or not snapshot['review_record_id']:
            return 'REVIEW_RECORD_MISSING'
        reviews = snapshot.get('reviews', {})
        if not isinstance(reviews, dict) or any(reviews.get(key) is not True for key in REVIEW_REQUIREMENTS):
            return 'REVIEW_PREREQUISITES_MISSING'
        if snapshot.get('input_reference') != 'REAR_AXLE_BICYCLE_LONGITUDINAL_SPEED':
            return 'KINEMATIC_REFERENCE_UNRESOLVED'
        identity = (expected, upstream, mapped, snapshot['review_record_id'])
        if self.state == 'ARMED' and identity != self.armed_identity:
            return 'ARMED_IDENTITY_CHANGED'
        return None

    def arm(self, snapshot, *, now):
        """Explicit simulated rearming; assertions are not independently checked."""
        error = self._health(snapshot, now)
        if error:
            return self._fault(error)
        if abs(snapshot['measured_speed_mps']) > self.policy.stop_band_mps:
            return self._fault('ARM_REQUIRES_OBSERVED_STANDSTILL')
        self.state = 'ARMED'
        self.armed_identity = (snapshot['expected_actuator_publisher'], snapshot['expected_input_publisher'],
                               snapshot['mapped_library_sha256'], snapshot['review_record_id'])
        self.last_direction = 0
        self.last_command_at = now
        self.stop_requested_at = now
        self.stop_confirmed = True
        return self._result(decision('STOP', 'OFFLINE_ARMED_AT_STANDSTILL'))

    def submit(self, speed_mps, yaw_rate_rps, snapshot, *, received_at, now, publisher_gid):
        if self.state != 'ARMED':
            return self._result(decision('INHIBITED', 'EXPLICIT_ARM_REQUIRED'))
        error = self._health(snapshot, now)
        if error:
            return self._fault(error)
        if now - self.last_command_at >= self.policy.command_timeout_s:
            return self._fault('COMMAND_WATCHDOG_EXPIRED')
        if publisher_gid != snapshot['expected_input_publisher']:
            return self._fault('COMMAND_SOURCE_MISMATCH')
        self._observe_standstill(snapshot)
        if (not finite(received_at) or not 0 <= now - received_at < self.policy.command_timeout_s
                or (self.last_command_at is not None and received_at < self.last_command_at)):
            return self._fault('STALE_FUTURE_OR_REORDERED_COMMAND')
        result = convert(speed_mps, yaw_rate_rps, self.geometry, self.policy,
                         held_at_stop=self.stop_requested_at is not None)
        if result['status'] == 'REJECTED':
            rejected = result
            result = self._fault(rejected['reason'])
            result['rejected_request'] = rejected
            return result
        speed = result['candidate']['linear_x_mps']
        direction = 1 if speed > 0 else -1 if speed < 0 else 0
        measured = snapshot['measured_speed_mps']
        if direction and abs(measured) > self.policy.stop_band_mps and direction * measured < 0:
            return self._fault('COMMAND_OPPOSES_CURRENT_MEASURED_MOTION')
        if direction and self.last_direction and direction != self.last_direction and not self.stop_confirmed:
            return self._fault('REVERSAL_REQUIRES_STOP_REQUEST_AND_OBSERVED_STANDSTILL')
        self.last_command_at = received_at
        if direction:
            self.last_direction = direction
            self.stop_requested_at = None
            self.stop_confirmed = False
        else:
            if self.stop_requested_at is None:
                self.stop_requested_at = received_at
            self._observe_standstill(snapshot)
        return self._result(result)

    def poll(self, snapshot, *, now):
        if self.state != 'ARMED':
            return self._result(decision('INHIBITED', 'EXPLICIT_ARM_REQUIRED'))
        error = self._health(snapshot, now)
        if error:
            return self._fault(error)
        if now - self.last_command_at >= self.policy.command_timeout_s:
            return self._fault('COMMAND_WATCHDOG_EXPIRED')
        self._observe_standstill(snapshot)
        return self.last_result

    def inhibit(self, reason='OPERATOR_STOP_REQUEST'):
        """Model an inhibition request; it neither cancels actions nor stops a plant."""
        return self._fault(reason)
