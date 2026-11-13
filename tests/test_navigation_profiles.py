"""Offline speed-profile contracts; no ROS, runtime approval or movement."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import prepare_navigation_profiles as profiles
from prepare_candidates import differences


class NavigationProfilesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.source = self.directory / 'canonical.yaml'
        self.original = (ROOT / profiles.SOURCE_PATH).read_bytes()
        self.source.write_bytes(self.original)
        self.output = self.directory / 'new_profiles'

    def generate(self):
        return profiles.prepare_profiles(self.source, self.output)

    def test_complete_profiles_preserve_every_other_value_and_original_bytes(self):
        manifest = self.generate()
        baseline = yaml.safe_load(self.original)
        for name, item in manifest['profiles'].items():
            content = (self.output / item['file']).read_bytes()
            parsed = yaml.safe_load(content)
            self.assertEqual(set(differences(baseline, parsed)), set(profiles.ALLOWED_PATHS))
            self.assertEqual(set(parsed), set(baseline))
            self.assertEqual(parsed['planner_server'], baseline['planner_server'])
            self.assertEqual(parsed['local_costmap'], baseline['local_costmap'])
            self.assertEqual(parsed['global_costmap'], baseline['global_costmap'])
            self.assertEqual(parsed['bt_navigator'], baseline['bt_navigator'])
            self.assertEqual(hashlib.sha256(content).hexdigest(), item['sha256'])
        self.assertEqual(self.source.read_bytes(), self.original)

    def test_caps_are_si_conversions_and_match_declared_pipeline_envelopes(self):
        manifest = self.generate()
        for name, cap in [('20kmh', 50 / 9), ('25kmh', 125 / 18)]:
            config, entry = profiles.load_selected_profile(self.source, self.output, name)
            follow = config['controller_server']['ros__parameters']['FollowPath']
            smoother = config['velocity_smoother']['ros__parameters']
            self.assertEqual(follow['vx_max'], cap)
            self.assertEqual(smoother['max_velocity'], [cap, 0.0, 0.4])
            self.assertEqual(smoother['min_velocity'], [-2.25, 0.0, -0.4])
            self.assertEqual(follow['vx_min'], -2.25)
            self.assertEqual(follow['wz_max'], 0.4)
            self.assertIs(type(follow['vx_max']), float)
            self.assertEqual(entry['model_horizon_s'], 2.0)
            self.assertAlmostEqual(entry['straight_horizon_at_cap_m'], cap * 2)
            self.assertAlmostEqual(entry['radius_at_speed_and_yaw_caps_m'], cap / 0.4)
        self.assertEqual(manifest['profiles']['20kmh']['exact_speed_fraction_m_s'], {'numerator': 50, 'denominator': 9})
        self.assertEqual(manifest['profiles']['25kmh']['exact_speed_fraction_m_s'], {'numerator': 125, 'denominator': 18})

    def test_static_profile_never_claims_geometry_motion_or_runtime_approval(self):
        manifest = self.generate()
        self.assertEqual(manifest['status'], 'PREPARED_DIAGNOSTIC_ONLY')
        self.assertEqual(manifest['readiness'], 'BLOCKED_ACTUATION_REVIEW')
        self.assertFalse(manifest['motion_authorized'])
        self.assertEqual(manifest['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(manifest['gate1'], 'NOT_YET_VERIFIED')
        for name in manifest['profiles']:
            config, entry = profiles.load_selected_profile(self.source, self.output, name)
            self.assertFalse(entry['runtime_values_verified'])
            self.assertFalse(entry['actuation_authorized'])
            self.assertEqual(config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'], 11.6)
            self.assertEqual(config['controller_server']['ros__parameters']['FollowPath']['AckermannConstraints']['min_turning_r'], 11.6)
        self.assertIn('11.6', manifest['blocked_reasons'][0])

    def test_runtime_contract_covers_smoother_and_both_plugins(self):
        manifest = self.generate()
        expected = manifest['profiles']['20kmh']['expected_parameters']
        self.assertEqual(expected['/controller_server']['FollowPath.vx_max'], 50 / 9)
        self.assertEqual(expected['/velocity_smoother']['max_velocity'], [50 / 9, 0.0, 0.4])
        self.assertEqual(expected['/velocity_smoother']['min_velocity'], [-2.25, 0.0, -0.4])
        self.assertEqual(expected['/planner_server']['GridBased.motion_model_for_search'], 'DUBIN')
        self.assertNotIn('FollowPath.wz_min', expected['/controller_server'])

    def test_existing_output_and_dangling_symlink_are_not_overwritten(self):
        self.output.mkdir()
        sentinel = self.output / 'keep'
        sentinel.write_text('previous evidence')
        with self.assertRaises(profiles.ProfileError):
            self.generate()
        self.assertEqual(sentinel.read_text(), 'previous evidence')
        other = self.directory / 'dangling'
        other.symlink_to(self.directory / 'missing')
        with self.assertRaises(profiles.ProfileError):
            profiles.prepare_profiles(self.source, other)
        self.assertTrue(other.is_symlink())

    def test_changed_source_is_not_implicitly_rebased(self):
        self.source.write_bytes(self.original.replace(b'vx_max: 25.0', b'vx_max: 2.5'))
        with self.assertRaises(profiles.ProfileError):
            self.generate()
        self.assertFalse(self.output.exists())

    def test_malformed_or_nonfinite_source_is_rejected_before_outputs(self):
        for raw in (b'not: [valid', b'vx_max: .nan\n', b'key: 1\nkey: 2\n'):
            self.source.write_bytes(raw)
            with self.assertRaises(profiles.ProfileError):
                self.generate()
            self.assertFalse(self.output.exists())

    def test_unknown_profile_rejected(self):
        self.generate()
        with self.assertRaises(profiles.ProfileError):
            profiles.load_selected_profile(self.source, self.output, '../../canonical')

    def test_changed_yaml_is_rejected_even_with_self_consistent_manifest_hash(self):
        self.generate()
        path = self.output / '20kmh.yaml'
        parsed = yaml.safe_load(path.read_text())
        parsed['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'] = 13.09
        path.write_text(yaml.safe_dump(parsed))
        manifest_path = self.output / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['profiles']['20kmh']['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaises(profiles.ProfileError):
            profiles.verify_profiles(self.source, self.output)

    def test_manifest_authorization_tamper_is_rejected(self):
        self.generate()
        path = self.output / 'manifest.json'
        manifest = json.loads(path.read_text())
        manifest['motion_authorized'] = True
        path.write_text(json.dumps(manifest))
        with self.assertRaises(profiles.ProfileError):
            profiles.verify_profiles(self.source, self.output)

    def test_symlink_profile_file_or_directory_rejected(self):
        self.generate()
        path = self.output / '20kmh.yaml'
        target = self.directory / 'moved.yaml'
        path.rename(target)
        path.symlink_to(target)
        with self.assertRaises(profiles.ProfileError):
            profiles.verify_profiles(self.source, self.output)
        alias = self.directory / 'alias'
        alias.symlink_to(self.output)
        with self.assertRaises(profiles.ProfileError):
            profiles.verify_profiles(self.source, alias)

    def test_generated_files_are_reproducible_and_match_checked_in_assets(self):
        self.generate()
        checked_in = ROOT / 'config/auto_tuning/navigation_profiles'
        for name in ('20kmh.yaml', '25kmh.yaml', 'manifest.json'):
            self.assertEqual((self.output / name).read_bytes(), (checked_in / name).read_bytes())
        self.assertEqual(profiles.verify_profiles(self.source, self.output)['source']['sha256'], profiles.SOURCE_SHA256)

    def test_cli_verify_is_read_only_and_reports_blocked_status(self):
        self.generate()
        result = subprocess.run([sys.executable, str(ROOT / 'tools/auto_tuning/prepare_navigation_profiles.py'),
            'verify', '--source', str(self.source), '--profiles-dir', str(self.output)],
            capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        reported = json.loads(result.stdout)
        self.assertEqual(reported['readiness'], 'BLOCKED_ACTUATION_REVIEW')
        self.assertFalse(reported['motion_authorized'])


if __name__ == '__main__':
    unittest.main()
