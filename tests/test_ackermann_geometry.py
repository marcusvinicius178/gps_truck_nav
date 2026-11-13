"""Offline SDF/equation contracts. No ROS, Twist, simulation or motion is used."""

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/gps_truck_nav/truck_bringup/models/arocs_truck/model.sdf'
SPEC = importlib.util.spec_from_file_location('ackermann_geometry', ROOT / 'tools/auto_tuning/audit_ackermann_geometry.py')
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class AckermannGeometryTests(unittest.TestCase):
    def changed_model(self, mutate):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / 'model.sdf'
        root = ET.parse(MODEL).getroot()
        mutate(root.find('model'))
        ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
        return path

    def test_collision_centers_resolve_rotated_right_wheel_offset(self):
        result = audit.audit_model(MODEL, 11.6)
        centers = result['wheels']
        self.assertAlmostEqual(centers['front_left_joint']['collision_center_model_m'][0], 3.302)
        self.assertAlmostEqual(centers['front_right_joint']['collision_center_model_m'][0], 3.327)
        self.assertAlmostEqual(result['wheelbase_euclidean_m'], 6.8165000733513414)
        self.assertAlmostEqual(result['front_track_euclidean_m'], 2.5981201902242046)
        self.assertNotAlmostEqual(result['wheelbase_euclidean_m'], 6.804)
        self.assertEqual(result['rear_right_radius_m'], 0.536)
        self.assertEqual(result['base_origin_minus_rear_axle_model_m'], [3.502, 0.0, -0.536])

    def test_plugin_max_steer_exceeds_both_physical_joint_targets(self):
        result = audit.audit_model(MODEL, 11.6)
        self.assertEqual(len(result['plugin_max_steer_joint_limit_violations']), 4)
        positive = result['targets_at_plugin_max_steer_rad']['positive']
        self.assertAlmostEqual(positive['left'], 0.7216428186006472)
        self.assertAlmostEqual(positive['right'], 0.5826482087500514)
        self.assertFalse(result['requested_radius_fits_static_joint_bound'])
        self.assertAlmostEqual(result['conservative_symmetric_virtual_bound_rad'], 0.4799568628770338)
        self.assertAlmostEqual(result['conservative_ideal_rear_axle_minimum_radius_m'], 13.094652363938684)
        for sign in (-1, 1):
            targets = audit.steering_targets(sign * result['conservative_symmetric_virtual_bound_rad'],
                                            result['wheelbase_euclidean_m'], result['front_track_euclidean_m'])
            self.assertTrue(all(abs(value) <= 0.524 + 1e-12 for value in targets.values()))

    def test_static_pass_is_never_runtime_geometry_or_motion_approval(self):
        result = audit.audit_model(MODEL)
        self.assertEqual(result['status'], 'STATIC_MODEL_AUDITED')
        self.assertIsNone(result['requested_radius_fits_static_joint_bound'])
        for field in ('runtime_binary_verified', 'runtime_geometry_validated', 'motion_authorized'):
            self.assertIs(result[field], False)
        self.assertEqual(result['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(result['gate1'], 'NOT_YET_VERIFIED')

    def test_common_translation_and_yaw_do_not_change_plugin_distances(self):
        def change(model):
            base = model.find("link[@name='base_footprint']")
            ET.SubElement(base, 'pose').text = '5 -2 3 0 0 0.7'
        result = audit.audit_model(self.changed_model(change))
        baseline = audit.audit_model(MODEL)
        for field in ('wheelbase_euclidean_m', 'front_track_euclidean_m'):
            self.assertAlmostEqual(result[field], baseline[field])

    def test_missing_joint_unknown_frame_and_cycle_fail_closed(self):
        changes = (
            lambda m: m.find("plugin[@name='ackermann_drive']/front_left_joint").__setattr__('text', 'missing'),
            lambda m: m.find("link[@name='front_left_steering_link']/pose").set('relative_to', 'missing'),
            lambda m: m.find("link[@name='front_left_steering_link']/pose").set('relative_to', 'first_wheel_left_link'),
        )
        for change in changes:
            with self.subTest(change=change):
                with self.assertRaises(audit.GeometryError):
                    audit.audit_model(self.changed_model(change))

    def test_nonfinite_pose_and_unsupported_nested_model_fail_closed(self):
        for change in (
            lambda m: m.find("link[@name='front_left_steering_link']/pose").__setattr__('text', 'nan 0 0 0 0 0'),
            lambda m: ET.SubElement(m, 'include'),
            lambda m: m.find("link[@name='front_left_steering_link']/pose").set('rotation_format', 'quat_xyzw'),
        ):
            with self.subTest(change=change), self.assertRaises(audit.GeometryError):
                audit.audit_model(self.changed_model(change))

    def test_unsupported_collision_and_axis_are_not_silently_approximated(self):
        def collision(model):
            geometry = model.find("link[@name='fourth_wheel_right_link']/collision/geometry")
            geometry.clear()
            ET.SubElement(geometry, 'mesh')
        def axis(model):
            model.find("joint[@name='front_left_steer_joint']/axis/xyz").text = '0 0 -1'
        for change in (collision, axis):
            with self.subTest(change=change), self.assertRaises(audit.GeometryError):
                audit.audit_model(self.changed_model(change))

    def test_requested_radius_is_an_explicit_audit_input(self):
        self.assertTrue(audit.audit_model(MODEL, 20.0)['requested_radius_fits_static_joint_bound'])
        for radius in (0, -1, float('inf'), float('nan')):
            with self.subTest(radius=radius), self.assertRaises(audit.GeometryError):
                audit.audit_model(MODEL, radius)

    def test_ideal_inverse_accounts_for_plugin_reverse_sign(self):
        length = 6.8165000733513414
        for speed in (-2.0, 2.0):
            for yaw_rate in (-0.1, 0.1):
                with self.subTest(speed=speed, yaw_rate=yaw_rate):
                    plugin_input = audit.ideal_plugin_input(speed, yaw_rate, length)
                    virtual_angle = plugin_input * math.copysign(1, speed)
                    realized_ideal_yaw = speed * math.tan(virtual_angle) / length
                    self.assertAlmostEqual(realized_ideal_yaw, yaw_rate)
                    self.assertLess(abs(plugin_input), math.pi / 2)
        self.assertEqual(audit.ideal_plugin_input(-2, 0.1, length), audit.ideal_plugin_input(2, 0.1, length))

    def test_signed_zero_is_normalized_and_pure_rotation_is_rejected(self):
        for speed in (0.0, -0.0):
            self.assertEqual(math.copysign(1, audit.ideal_plugin_input(speed, 0, 6)), 1)
            for yaw in (-0.1, 0.1):
                with self.assertRaises(audit.GeometryError):
                    audit.ideal_plugin_input(speed, yaw, 6)

    def test_nonfinite_and_overflowing_equation_inputs_are_rejected(self):
        for args in ((float('nan'), 0.1, 6), (1, float('inf'), 6), (1, 0.1, 0),
                     (True, 0.1, 6), (1, 1e308, 1e308)):
            with self.subTest(args=args), self.assertRaises(audit.GeometryError):
                audit.ideal_plugin_input(*args)

    def test_steering_targets_swap_under_sign_change(self):
        positive = audit.steering_targets(0.3, 6.8, 2.6)
        negative = audit.steering_targets(-0.3, 6.8, 2.6)
        self.assertAlmostEqual(positive['left'], -negative['right'])
        self.assertAlmostEqual(positive['right'], -negative['left'])
        self.assertGreater(positive['left'], positive['right'])

    def test_cli_preserves_existing_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'audit.json'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(audit.main([str(MODEL), '--requested-turning-radius', '11.6', '--output', str(output)]), 0)
            original = output.read_bytes()
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                audit.main([str(MODEL), '--output', str(output)])
            self.assertEqual(caught.exception.code, 2)
            self.assertEqual(output.read_bytes(), original)

    def test_cli_invalid_model_records_failure_without_motion_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'failed.json'
            with contextlib.redirect_stdout(io.StringIO()):
                code = audit.main([str(Path(directory) / 'missing.sdf'), '--output', str(output)])
            record = json.loads(output.read_text())
            self.assertEqual(code, 2)
            self.assertEqual(record['status'], 'STATIC_MODEL_AUDIT_FAILED')
            self.assertFalse(record['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
