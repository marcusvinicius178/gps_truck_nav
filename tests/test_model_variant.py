"""Prospective model contracts; mesh fixtures do not run Gazebo or claim geometry validation."""

import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('model_variant', ROOT / 'tools/auto_tuning/validate_model_variant.py')
variant = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(variant)


class ModelVariantTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.repo = Path(temporary.name)
        self.share = self.repo / 'src/gps_truck_nav/truck_bringup'
        self.config = self.repo / 'config/auto_tuning'
        for relative in variant.CANONICAL_SHA256:
            target = self.share / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / 'src/gps_truck_nav/truck_bringup' / relative, target)
        for relative in (*variant.VARIANT_FILES, 'tf_sensor_model_variant.json'):
            target = self.config / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / 'config/auto_tuning' / relative, target)
        # Asset byte identities belong to the existing LFS/source-manifest check.
        # Here exercise resolution and pointer rejection with small synthetic files.
        model = ET.parse(self.share / 'models/arocs_truck/model.sdf')
        for item in model.findall('.//mesh/uri'):
            path = self.share / 'models' / item.text.removeprefix('model://')
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'offline synthetic mesh fixture\n')
        self.manifest_path = self.config / 'tf_sensor_model_variant.json'

    def rewrite_manifest(self, change):
        manifest = json.loads(self.manifest_path.read_text())
        change(manifest)
        self.manifest_path.write_text(json.dumps(manifest))

    def replace_variant_and_update_hash(self, relative, before, after):
        path = self.config / relative
        original = path.read_text()
        self.assertIn(before, original)
        path.write_text(original.replace(before, after, 1))
        self.rewrite_manifest(lambda record: record['variant_sha256'].update({
            relative: hashlib.sha256(path.read_bytes()).hexdigest()}))

    def test_complete_variant_matches_pinned_sources_and_fixed_sensor_offsets(self):
        observed = variant.validate(self.share, self.config)
        self.assertEqual(observed['model_name'], 'arocs_truck')
        self.assertEqual(observed['sensor_lever_arms']['gps_link']['translation_m'], [3.8205, 0.0, 3.062])
        self.assertEqual(observed['sensor_lever_arms']['imu_link']['translation_m'], [3.7205, 0.0, 2.595])
        self.assertEqual(len(observed['expected_wheel_edges']), 6)
        self.assertEqual(len(observed['joint_names']), 6)
        self.assertEqual(observed['canonical_mesh_references_checked'], 9)

    def test_only_three_sdf_changes_and_one_world_uri_change_are_versioned(self):
        canonical = (self.share / 'models/arocs_truck/model.sdf').read_text()
        expected = variant.exact_replacements(canonical, variant.REPLACEMENTS)
        self.assertEqual((self.config / variant.VARIANT_FILES[0]).read_text(), expected)
        canonical_world = (self.share / 'worlds/ctvi.world').read_text()
        expected_world = canonical_world.replace('model://arocs_truck', 'model://arocs_truck_tf_sensors')
        self.assertEqual((self.config / variant.VARIANT_FILES[2]).read_text(), expected_world)

    def test_odom_tf_joint_states_noise_and_remaps_survive_unchanged(self):
        canonical = ET.parse(self.share / 'models/arocs_truck/model.sdf')
        proposed = ET.parse(self.config / variant.VARIANT_FILES[0])
        drive = proposed.find("./model/plugin[@name='ackermann_drive']")
        self.assertEqual(drive.findtext('publish_wheel_tf'), '0')
        self.assertEqual(drive.findtext('publish_odom_tf'), '1')
        self.assertEqual(drive.findtext('publish_odom'), '1')
        for selector in ("./model/plugin[@name='truck_joint_state']", './/imu',
                         ".//sensor[@name='arocs_gps']/plugin/ros", ".//sensor[@name='imu_sensor']/plugin/ros"):
            self.assertEqual(ET.tostring(canonical.find(selector)), ET.tostring(proposed.find(selector)))

    def test_changed_geometry_is_rejected_even_with_updated_variant_hash(self):
        self.replace_variant_and_update_hash(variant.VARIANT_FILES[0], '<mass>1108.1</mass>', '<mass>1108.2</mass>')
        with self.assertRaisesRegex(ValueError, 'beyond the three'):
            variant.validate(self.share, self.config)

    def test_disabled_odometry_is_rejected_even_with_updated_variant_hash(self):
        self.replace_variant_and_update_hash(variant.VARIANT_FILES[0], '<publish_odom_tf>1</publish_odom_tf>', '<publish_odom_tf>0</publish_odom_tf>')
        with self.assertRaisesRegex(ValueError, 'beyond the three'):
            variant.validate(self.share, self.config)

    def test_changed_datum_or_spawn_is_rejected_even_with_updated_world_hash(self):
        self.replace_variant_and_update_hash(variant.VARIANT_FILES[2], '<heading_deg>180</heading_deg>', '<heading_deg>0</heading_deg>')
        with self.assertRaisesRegex(ValueError, 'World changed'):
            variant.validate(self.share, self.config)

    def test_canonical_drift_is_rejected_and_cannot_be_hidden_by_manifest(self):
        source = self.share / 'models/arocs_truck/model.sdf'
        source.write_text(source.read_text() + '\n')
        with self.assertRaisesRegex(ValueError, 'Hash mismatch'):
            variant.validate(self.share, self.config)
        self.rewrite_manifest(lambda record: record['canonical_sha256'].update({
            'models/arocs_truck/model.sdf': hashlib.sha256(source.read_bytes()).hexdigest()}))
        with self.assertRaisesRegex(ValueError, 'source pins changed'):
            variant.validate(self.share, self.config)

    def test_missing_wheel_contract_is_not_accepted_as_conflict_resolution(self):
        self.rewrite_manifest(lambda record: record.update(expected_wheel_edges=[]))
        with self.assertRaisesRegex(ValueError, 'Wheel TF contract'):
            variant.validate(self.share, self.config)

    def test_changed_lever_arm_metadata_is_rejected(self):
        self.rewrite_manifest(lambda record: record['sensor_lever_arms']['gps_link'].update(translation_m=[0, 0, 0]))
        with self.assertRaisesRegex(ValueError, 'lever arms'):
            variant.validate(self.share, self.config)

    def test_missing_and_unhydrated_canonical_meshes_fail(self):
        path = self.share / 'models/arocs_truck/meshes/cabine_chassi.dae'
        path.unlink()
        with self.assertRaisesRegex(ValueError, 'does not resolve'):
            variant.validate(self.share, self.config)
        path.write_text('version https://git-lfs.github.com/spec/v1\n')
        with self.assertRaisesRegex(ValueError, 'Unhydrated'):
            variant.validate(self.share, self.config)

    def test_manifest_cannot_claim_runtime_or_mission_approval(self):
        self.rewrite_manifest(lambda record: record.update(runtime_verified=True))
        with self.assertRaisesRegex(ValueError, 'cannot approve'):
            variant.validate(self.share, self.config)

    def test_repo_and_installed_cli_report_static_scope_without_ros(self):
        for arguments in (['--repo', str(self.repo)],
                          ['--installed-share', str(self.share), '--config', str(self.config)]):
            with self.subTest(arguments=arguments), contextlib.redirect_stdout(io.StringIO()) as output:
                code = variant.main(arguments)
            self.assertEqual(code, 0)
            result = json.loads(output.getvalue())
            self.assertEqual(result['status'], 'MODEL_VARIANT_VALIDATED')
            self.assertFalse(result['runtime_verified'])
            self.assertFalse(result['mission_ready'])
            self.assertEqual(result['gate0'], 'NOT_YET_VERIFIED')

    def test_cli_failure_is_machine_readable_and_has_nonzero_exit(self):
        self.manifest_path.unlink()
        with contextlib.redirect_stdout(io.StringIO()) as output:
            code = variant.main(['--repo', str(self.repo)])
        result = json.loads(output.getvalue())
        self.assertEqual(code, 1)
        self.assertEqual(result['status'], 'MODEL_VARIANT_FAILED')
        self.assertIn('FileNotFoundError', result['error'])
        self.assertFalse(result['mission_ready'])


if __name__ == '__main__':
    unittest.main()
