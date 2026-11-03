"""Synthetic offline contracts. Fixture limits/assertions are not vehicle approvals."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
from actuation_contract import (ContractError, Geometry, OfflineSupervisor, Policy,
                                REVIEW_REQUIREMENTS, convert)
from audit_ackermann_geometry import audit_model


class ActuationContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audit = audit_model(ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf', 11.6)
        cls.geometry = Geometry.from_static_audit(cls.audit)

    def setUp(self):
        # Artificial thresholds used only to test branches, never a deployed policy.
        self.policy = Policy(20 / 3.6, 2.25, 0.4, 0.05, 0.1, 0.25, 0.3, 0.5,
                             'SCALE_PRESERVE_CURVATURE')
        self.supervisor = OfflineSupervisor(self.geometry, self.policy)

    def snapshot(self, now=0.0, speed=0.0, **changes):
        result = {'sim_seconds': 100 + now, 'clock_received_at': now,
                  'odom_received_at': now, 'graph_received_at': now,
                  'measured_speed_mps': speed, 'actuator_publishers': ['adapter-gid'],
                  'expected_actuator_publisher': 'adapter-gid',
                  'input_publishers': ['smoother-gid'], 'expected_input_publisher': 'smoother-gid',
                  'installed_library_sha256': 'a' * 64, 'mapped_library_sha256': 'a' * 64,
                  'loaded_model_sha256': self.geometry.model_sha256,
                  'review_record_id': 'SYNTHETIC_FIXTURE_NOT_APPROVAL',
                  'reviews': {name: True for name in REVIEW_REQUIREMENTS},
                  'input_reference': 'REAR_AXLE_BICYCLE_LONGITUDINAL_SPEED'}
        result.update(changes)
        return result

    def submit(self, speed, yaw, now=0.01, observed_speed=0.0, **kwargs):
        return self.supervisor.submit(speed, yaw, self.snapshot(now, observed_speed),
                                      received_at=now, now=now, publisher_gid='smoother-gid', **kwargs)

    def arm(self):
        result = self.supervisor.arm(self.snapshot(), now=0.0)
        self.assertEqual(result['model_state'], 'ARMED')
        self.assertFalse(result['runtime_authorized'])

    def test_static_audit_preserves_unapproved_geometry(self):
        self.assertAlmostEqual(self.geometry.virtual_bound_rad(), 0.4799568628770338)
        self.assertFalse(self.audit['runtime_geometry_validated'])
        self.assertFalse(self.audit['requested_radius_fits_static_joint_bound'])

    def test_four_quadrants_reconstruct_yaw_with_plugin_reverse_sign_once(self):
        for speed in (-1.0, 1.0):
            for yaw in (-0.04, 0.04):
                with self.subTest(speed=speed, yaw=yaw):
                    result = convert(speed, yaw, self.geometry, self.policy)
                    candidate = result['candidate']
                    delta = candidate['plugin_angular_z_rad'] * math.copysign(1, speed)
                    reconstructed = speed / self.geometry.wheelbase_m * math.tan(delta)
                    self.assertEqual(result['status'], 'ACCEPTED')
                    self.assertAlmostEqual(reconstructed, yaw)
                    self.assertEqual(math.copysign(1, candidate['plugin_angular_z_rad']), math.copysign(1, yaw))

    def test_signed_zero_normalized(self):
        for zero in (0.0, -0.0):
            result = convert(zero, -0.0, self.geometry, self.policy)
            self.assertEqual(result['status'], 'STOP')
            self.assertTrue(all(math.copysign(1, value) == 1 for value in result['candidate'].values()))

    def test_zero_or_near_zero_yaw_is_rejected_not_divided_by_epsilon(self):
        for speed in (0.0, -0.0, 0.01, -0.01, self.policy.stop_band_mps):
            result = convert(speed, 0.001, self.geometry, self.policy)
            self.assertEqual(result['reason'], 'UNREALIZABLE_NEAR_ZERO_YAW')
            self.assertEqual(result['candidate']['linear_x_mps'], 0)

    def test_stop_hysteresis_distinguishes_moving_and_held(self):
        held = convert(0.08, 0, self.geometry, self.policy, held_at_stop=True)
        moving = convert(0.08, 0, self.geometry, self.policy, held_at_stop=False)
        resume = convert(0.1, 0, self.geometry, self.policy, held_at_stop=True)
        self.assertEqual(held['status'], 'HOLD')
        self.assertEqual(moving['status'], 'ACCEPTED')
        self.assertEqual(resume['status'], 'ACCEPTED')

    def test_impossible_curvature_not_hidden_by_speed_reduction(self):
        result = convert(10.0, 10.0 / 11.6, self.geometry, self.policy)
        self.assertEqual(result['reason'], 'CURVATURE_EXCEEDS_STATIC_JOINT_ENVELOPE')
        self.assertEqual(result['candidate']['linear_x_mps'], 0)

    def test_speed_saturation_scales_both_components_and_preserves_curvature(self):
        result = convert(10, 0.3, self.geometry, self.policy)
        c = result['candidate']
        self.assertEqual(result['status'], 'SCALED')
        self.assertAlmostEqual(c['linear_x_mps'], 20 / 3.6)
        self.assertAlmostEqual(c['yaw_rate_rps'] / c['linear_x_mps'], 0.03)

    def test_yaw_saturation_also_scales_linear_speed(self):
        p = replace(self.policy, forward_speed_mps=20)
        result = convert(10, 0.6, self.geometry, p)
        self.assertEqual(result['status'], 'SCALED')
        self.assertAlmostEqual(result['candidate']['linear_x_mps'], 10 * 0.4 / 0.6)
        self.assertAlmostEqual(result['candidate']['yaw_rate_rps'], 0.4)

    def test_policy_can_reject_overspeed_instead_of_scaling(self):
        result = convert(10, 0.3, self.geometry, replace(self.policy, speed_policy='REJECT'))
        self.assertEqual(result['reason'], 'SPEED_OR_YAW_LIMIT')

    def test_reverse_disabled_and_reverse_cap(self):
        result = convert(-1, 0, self.geometry, replace(self.policy, reverse_speed_mps=0))
        self.assertEqual(result['reason'], 'REVERSE_DISABLED')
        result = convert(-4, 0.1, self.geometry, self.policy)
        self.assertAlmostEqual(result['candidate']['linear_x_mps'], -2.25)
        self.assertAlmostEqual(result['candidate']['yaw_rate_rps'], 0.1 * 2.25 / 4)

    def test_nonfinite_boolean_and_overflow_fail_closed(self):
        for value in (math.nan, math.inf, -math.inf, True, '1'):
            self.assertEqual(convert(value, 0, self.geometry, self.policy)['status'], 'REJECTED')
            self.assertEqual(convert(1, value, self.geometry, self.policy)['status'], 'REJECTED')
        self.assertEqual(convert(0.2, 1e308, self.geometry, self.policy)['reason'], 'NUMERIC_OVERFLOW')

    def test_geometry_and_policy_require_explicit_valid_values(self):
        for g in (replace(self.geometry, wheelbase_m=0), replace(self.geometry, track_m=math.inf),
                  replace(self.geometry, left_lower_rad=0), replace(self.geometry, model_sha256='unknown')):
            with self.assertRaises(ContractError):
                OfflineSupervisor(g, self.policy)
        for p in (replace(self.policy, stop_band_mps=0), replace(self.policy, resume_speed_mps=0.04),
                  replace(self.policy, command_timeout_s=0), replace(self.policy, forward_speed_mps=25),
                  replace(self.policy, speed_policy='CLIP_STEERING')):
            with self.assertRaises(ContractError):
                OfflineSupervisor(self.geometry, p)

    def test_joint_limits_hold_over_signed_feasible_curvature_sweep(self):
        kmax = math.tan(self.geometry.virtual_bound_rad()) / self.geometry.wheelbase_m
        for sign in (-1, 1):
            for step in range(-99, 100):
                result = convert(sign, sign * step * kmax / 100, self.geometry, self.policy)
                self.assertEqual(result['status'], 'ACCEPTED')
                targets = result['joint_targets_rad']
                self.assertLessEqual(abs(targets['left']), 0.524)
                self.assertLessEqual(abs(targets['right']), 0.524)

    def test_disarmed_model_never_accepts_motion(self):
        result = self.submit(1, 0)
        self.assertEqual(result['status'], 'INHIBITED')
        self.assertFalse(result['runtime_authorized'])

    def test_each_review_prerequisite_is_mandatory(self):
        for name in REVIEW_REQUIREMENTS:
            with self.subTest(name=name):
                snapshot = self.snapshot()
                snapshot['reviews'][name] = False
                self.assertEqual(OfflineSupervisor(self.geometry, self.policy).arm(snapshot, now=0)['reason'],
                                 'REVIEW_PREREQUISITES_MISSING')

    def test_current_unreviewed_reference_cannot_arm(self):
        result = self.supervisor.arm(self.snapshot(input_reference='base_footprint'), now=0)
        self.assertEqual(result['reason'], 'KINEMATIC_REFERENCE_UNRESOLVED')
        result = self.supervisor.arm(self.snapshot(speed=1), now=0)
        self.assertEqual(result['reason'], 'ARM_REQUIRES_OBSERVED_STANDSTILL')

    def test_graph_bypass_and_missing_publishers_inhibit(self):
        for publishers in ([], ['adapter-gid', 'behavior-gid'], ['other-gid']):
            s = OfflineSupervisor(self.geometry, self.policy)
            self.assertEqual(s.arm(self.snapshot(actuator_publishers=publishers), now=0)['reason'],
                             'ACTUATOR_AUTHORITY_NOT_UNIQUE')
        self.assertEqual(self.supervisor.arm(self.snapshot(input_publishers=['smoother-gid','behavior-gid']),now=0)['reason'],
                         'INPUT_AUTHORITY_NOT_UNIQUE')

    def test_actual_command_source_must_match_observed_input_authority(self):
        self.arm()
        result = self.supervisor.submit(1, 0, self.snapshot(.01), received_at=.01, now=.01,
                                        publisher_gid='behavior-bypass')
        self.assertEqual(result['reason'], 'COMMAND_SOURCE_MISMATCH')

    def test_identity_change_requires_explicit_rearm(self):
        self.arm()
        snapshot = self.snapshot(.01, mapped_library_sha256='b'*64)
        self.assertEqual(self.supervisor.poll(snapshot,now=.01)['reason'], 'BINARY_IDENTITY_UNVERIFIED_OR_CHANGED')
        self.assertEqual(self.supervisor.arm(self.snapshot(.02),now=.02)['model_state'],'ARMED')
        snapshot = self.snapshot(.03, installed_library_sha256='b'*64,mapped_library_sha256='b'*64)
        self.assertEqual(self.supervisor.poll(snapshot,now=.03)['reason'], 'ARMED_IDENTITY_CHANGED')

    def test_model_identity_change_inhibits(self):
        self.assertEqual(self.supervisor.arm(self.snapshot(loaded_model_sha256='b'*64),now=0)['reason'],
                         'MODEL_IDENTITY_MISMATCH')

    def test_reversal_requires_stop_request_then_measured_standstill(self):
        self.arm()
        self.submit(1, 0, now=.01)
        self.assertIn('REVERSAL', self.submit(-1, 0, now=.02, observed_speed=0)['reason'])
        self.supervisor = OfflineSupervisor(self.geometry, self.policy)
        self.arm()
        self.submit(1,0,now=.01)
        self.submit(0,0,now=.02,observed_speed=.3)
        self.assertFalse(self.supervisor.stop_confirmed)
        self.supervisor.poll(self.snapshot(.03,speed=0),now=.03)
        self.assertTrue(self.supervisor.stop_confirmed)
        self.assertEqual(self.submit(-1, .02,now=.04)['status'],'ACCEPTED')

    def test_previous_standstill_is_not_permission_to_reverse_after_rolling_again(self):
        self.arm()
        self.submit(1,0,now=.01)
        self.submit(0,0,now=.02,observed_speed=.3)
        self.supervisor.poll(self.snapshot(.03,speed=0),now=.03)
        self.assertTrue(self.supervisor.stop_confirmed)
        result=self.submit(-1,0,now=.04,observed_speed=.3)
        self.assertEqual(result['reason'],'COMMAND_OPPOSES_CURRENT_MEASURED_MOTION')
        self.assertFalse(self.supervisor.stop_confirmed)

    def test_first_command_after_arming_cannot_oppose_current_rolling_direction(self):
        for requested,measured in ((-1,.3),(1,-.3)):
            self.supervisor=OfflineSupervisor(self.geometry,self.policy)
            self.arm()
            result=self.submit(requested,0,now=.01,observed_speed=measured)
            self.assertEqual(result['reason'],'COMMAND_OPPOSES_CURRENT_MEASURED_MOTION')

    def test_rollaway_invalidates_stop_confirmation_before_next_command(self):
        self.arm()
        self.assertTrue(self.supervisor.stop_confirmed)
        self.supervisor.poll(self.snapshot(.01,speed=.3),now=.01)
        self.assertFalse(self.supervisor.stop_confirmed)

    def test_direction_change_needs_current_standstill_even_if_rolling_in_new_direction(self):
        self.arm()
        self.submit(1,0,now=.01)
        self.submit(0,0,now=.02,observed_speed=0)
        result=self.submit(-1,0,now=.03,observed_speed=-.3)
        self.assertEqual(result['reason'],'REVERSAL_REQUIRES_STOP_REQUEST_AND_OBSERVED_STANDSTILL')

    def test_receipt_time_stale_future_or_reordered_inhibits(self):
        for received in (-1, .2, math.nan, True):
            s=OfflineSupervisor(self.geometry,self.policy)
            s.arm(self.snapshot(),now=0)
            result=s.submit(1,0,self.snapshot(.1),received_at=received,now=.1,publisher_gid='smoother-gid')
            self.assertEqual(result['reason'],'STALE_FUTURE_OR_REORDERED_COMMAND')

    def test_watchdog_deadline_and_late_command_do_not_automatically_rearm(self):
        self.arm()
        self.assertEqual(self.supervisor.poll(self.snapshot(.25),now=.25)['reason'],'COMMAND_WATCHDOG_EXPIRED')
        self.assertEqual(self.submit(1,0,now=.26)['status'],'INHIBITED')
        self.supervisor=OfflineSupervisor(self.geometry,self.policy)
        self.arm()
        self.assertEqual(self.submit(1,0,now=.25)['reason'],'COMMAND_WATCHDOG_EXPIRED')

    def test_sim_clock_stall_is_separate_from_recent_clock_receipt(self):
        self.arm()
        for now in (.1,.2,.3,.4):
            result=self.supervisor.submit(1,0,self.snapshot(now,sim_seconds=100),
                received_at=now,now=now,publisher_gid='smoother-gid')
            self.assertEqual(result['status'],'ACCEPTED')
        self.assertEqual(self.supervisor.poll(self.snapshot(.5,sim_seconds=100),now=.5)['reason'],'SIM_CLOCK_STALLED')

    def test_sim_or_wall_clock_regression_inhibits(self):
        self.arm()
        self.assertEqual(self.supervisor.poll(self.snapshot(.1,sim_seconds=99),now=.1)['reason'],'SIM_CLOCK_REGRESSED')
        self.supervisor=OfflineSupervisor(self.geometry,self.policy)
        self.arm()
        self.assertEqual(self.supervisor.poll(self.snapshot(-.1),now=-.1)['reason'],'MONOTONIC_CLOCK_INVALID_OR_REGRESSED')

    def test_stale_observation_inhibits_even_with_new_command(self):
        for key in ('clock_received_at','odom_received_at','graph_received_at'):
            s=OfflineSupervisor(self.geometry,self.policy);s.arm(self.snapshot(),now=0)
            snapshot=self.snapshot(.1,**{key:-1})
            self.assertEqual(s.poll(snapshot,now=.1)['reason'],'STALE_OR_INVALID_'+key.upper())

    def test_stop_request_is_latched_and_never_claims_physical_stop(self):
        self.arm();self.submit(1,.02)
        stopped=self.supervisor.inhibit()
        self.assertEqual(stopped['candidate']['linear_x_mps'],0)
        self.assertFalse(stopped['physical_stop_verified'])
        self.assertEqual(self.submit(1,0,now=.02)['model_state'],'FAULT')

    def test_malformed_snapshot_and_measured_overspeed_inhibit(self):
        self.arm()
        self.assertEqual(self.supervisor.poll(None,now=.01)['reason'],'OBSERVATION_MALFORMED')
        self.supervisor=OfflineSupervisor(self.geometry,self.policy)
        self.arm()
        self.assertEqual(self.supervisor.poll(self.snapshot(.01,speed=6),now=.01)['reason'],'MEASURED_OVERSPEED')


if __name__ == '__main__':
    unittest.main()
