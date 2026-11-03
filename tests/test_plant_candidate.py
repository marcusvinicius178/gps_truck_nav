"""Static prospective-plant contracts; no ROS, command publication or simulation."""

import contextlib
import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import prepare_plant_candidate as plant
from actuation_contract import Geometry, Policy, convert
from audit_ackermann_geometry import audit_model, steering_targets
from prepare_candidates import differences


class PlantCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.outputs, cls.manifest = plant.build_bundle(ROOT)
        cls.bundle = ROOT / plant.BUNDLE_PATH
        cls.baseline = yaml.safe_load((ROOT / plant.SOURCE_PATH).read_bytes())

    def temporary(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        return Path(temp.name)

    def copied_sources(self):
        target = self.temporary()
        for name in plant.SOURCES:
            path = target / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((ROOT / name).read_bytes())
        return target

    def test_committed_bundle_matches_all_pinned_sources_and_deterministic_bytes(self):
        result = plant.verify_bundle(ROOT)
        self.assertEqual(result, self.manifest)
        self.assertEqual(plant.build_bundle(ROOT), (self.outputs, self.manifest))

    def test_only_two_new_sdf_changes_keep_geometry_odom_joints_and_sensors_identical(self):
        original = (ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf').read_bytes()
        candidate = self.outputs[plant.MODEL_PATH]
        original_tree, candidate_tree = ET.fromstring(original), ET.fromstring(candidate)
        plugin = candidate_tree.find("model/plugin[@name='ackermann_drive']")
        self.assertEqual(plugin.findtext('ros/remapping'), 'cmd_vel:=/actuation/steering_command')
        self.assertEqual(plugin.findtext('publish_odom'), '1')
        self.assertEqual(plugin.findtext('publish_odom_tf'), '1')
        self.assertEqual(plugin.findtext('publish_wheel_tf'), '0')
        self.assertEqual(plugin.findtext('max_speed'), '20')
        plugin.find('max_steer').text = '0.6458'
        plugin.find('ros').remove(plugin.find('ros/remapping'))
        # Whitespace surrounding the inserted XML element is immaterial here;
        # a byte-replacement check below establishes preservation outside it.
        reverse = candidate.replace(
            ('<max_steer>' + repr(self.manifest['geometry']['candidate_plugin_max_steer_rad']) + '</max_steer>').encode(),
            b'<max_steer>0.6458</max_steer>')
        reverse = reverse.replace(b'\n        <remapping>cmd_vel:=/actuation/steering_command</remapping>', b'')
        self.assertEqual(reverse, original)
        self.assertEqual(original_tree.findtext("model/link/sensor[@name='arocs_gps']/plugin/frame_name"), 'gps_link')

    def test_no_upstream_plugin_changes_or_new_actuator_publisher_claim(self):
        self.assertEqual(self.manifest['expected_actuator_publisher_count'], 0)
        self.assertEqual(self.manifest['actuator_topic'], '/actuation/steering_command')
        self.assertFalse(any(name.endswith(('.cpp', '.so')) for name in self.outputs))

    def test_radius_and_both_steering_targets_derive_from_unchanged_joint_limits(self):
        geometry = self.manifest['geometry']
        radius = geometry['candidate_radius_m']
        self.assertEqual(radius, 13.1)
        self.assertGreaterEqual(radius, geometry['static_ideal_rear_axle_radius_bound_m'])
        self.assertLess(radius - geometry['static_ideal_rear_axle_radius_bound_m'], 0.01)
        for sign in (-1, 1):
            targets = steering_targets(sign * geometry['candidate_plugin_max_steer_rad'],
                                       geometry['wheelbase_plugin_collision_m'],
                                       geometry['front_track_plugin_collision_m'])
            for side, angle in targets.items():
                limits = geometry['unchanged_joint_limits'][side]
                self.assertLessEqual(limits['lower_rad'], angle)
                self.assertLessEqual(angle, limits['upper_rad'])

    def test_exact_navigation_radius_is_not_rejected_by_rounded_steering_serialization(self):
        geometry = Geometry.from_static_audit(audit_model(self.bundle / plant.MODEL_PATH))
        policy = Policy(**json.loads(self.outputs['policies/candidate20kmh_shadow.json'])['policy'])
        for yaw in (-1 / 13.1, 1 / 13.1):
            result = convert(1.0, yaw, geometry, policy)
            self.assertEqual(result['status'], 'ACCEPTED', result)
        self.assertEqual(convert(1.0, 1 / 11.6, geometry, policy)['status'], 'REJECTED')

    def test_reference_adds_only_one_fixed_leaf_and_preserves_old_urdf(self):
        original = (ROOT / 'src/gps_truck_nav/truck_bringup/urdf/arocs_truck.urdf').read_bytes()
        candidate = ET.fromstring(self.outputs[plant.URDF_PATH])
        new_link = candidate.find("link[@name='rear_axle_nav']")
        new_joint = candidate.find("joint[@name='rear_axle_nav_fixed']")
        self.assertEqual(new_joint.get('type'), 'fixed')
        self.assertEqual(new_joint.find('parent').get('link'), 'base_footprint')
        self.assertEqual(new_joint.find('child').get('link'), 'rear_axle_nav')
        self.assertEqual(new_joint.find('origin').attrib, {'xyz': '-3.502 0 0', 'rpy': '0 0 0'})
        self.assertEqual(list(new_link), [])
        candidate.remove(new_link)
        candidate.remove(new_joint)
        def normalized(element):
            return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
                    [normalized(child) for child in element])
        self.assertEqual(normalized(candidate), normalized(ET.fromstring(original)))
        self.assertEqual(self.manifest['navigation_reference']['rotation_xyzw'], [0, 0, 0, 1])

    def test_world_changes_only_model_uri_not_datum_spawn_physics_or_environment(self):
        original = (ROOT / 'config/auto_tuning/worlds/ctvi_tf_sensors.world').read_bytes()
        self.assertEqual(self.outputs[plant.WORLD_PATH].replace(b'model://arocs_truck_candidate',
                                                               b'model://arocs_truck_tf_sensors'), original)

    def test_translated_footprint_has_same_world_polygon_for_arbitrary_pose(self):
        for profile_id in plant.PROFILES:
            config = yaml.safe_load(self.outputs['navigation/' + profile_id + '.yaml'])
            for node in ('local_costmap', 'global_costmap'):
                before = json.loads(self.baseline[node][node]['ros__parameters']['footprint'])
                after = json.loads(config[node][node]['ros__parameters']['footprint'])
                for yaw in (-2.3, 0.0, 1.2, math.pi):
                    c, s = math.cos(yaw), math.sin(yaw)
                    root_x, root_y = 12.4, -42.0
                    rear_x, rear_y = root_x - 3.502 * c, root_y - 3.502 * s
                    for a, b in zip(before, after):
                        self.assertAlmostEqual(root_x + c * a[0] - s * a[1], rear_x + c * b[0] - s * b[1])
                        self.assertAlmostEqual(root_y + s * a[0] + c * a[1], rear_y + s * b[0] + c * b[1])

    def test_goal_reference_and_readback_use_conventional_checker_not_old_front_offset(self):
        for profile_id, entry in self.manifest['profiles'].items():
            config = yaml.safe_load(self.outputs[entry['file']])
            controller = config['controller_server']['ros__parameters']
            self.assertEqual(controller['goal_checker_plugins'], ['general_goal_checker'])
            self.assertEqual(controller['front_cabin_goal_checker'],
                             self.baseline['controller_server']['ros__parameters']['front_cabin_goal_checker'])
            self.assertEqual(controller['general_goal_checker'],
                             self.baseline['controller_server']['ros__parameters']['general_goal_checker'])
            readback = entry['expected_parameters']['/controller_server']
            self.assertEqual(readback['goal_checker_plugins'], ['general_goal_checker'])
            self.assertEqual(readback['general_goal_checker.xy_goal_tolerance'], 1.0)
            for node in ('bt_navigator', 'behavior_server'):
                self.assertEqual(entry['expected_parameters']['/' + node]['robot_base_frame'], 'rear_axle_nav')

    def test_candidate_speed_envelopes_and_radius_are_mutually_consistent(self):
        for profile_id, kmh in plant.PROFILES.items():
            config = yaml.safe_load(self.outputs['navigation/' + profile_id + '.yaml'])
            follow = config['controller_server']['ros__parameters']['FollowPath']
            smoother = config['velocity_smoother']['ros__parameters']
            planner = config['planner_server']['ros__parameters']['GridBased']
            self.assertAlmostEqual(follow['vx_max'], kmh * 5 / 18)
            self.assertEqual(smoother['max_velocity'], [follow['vx_max'], 0.0, follow['wz_max']])
            self.assertEqual(smoother['min_velocity'], [0.0, 0.0, -follow['wz_max']])
            self.assertEqual(follow['vx_min'], 0.0)
            self.assertEqual(follow['AckermannConstraints']['min_turning_r'], planner['minimum_turning_radius'])
            self.assertEqual(planner['minimum_turning_radius'], 13.1)
            self.assertEqual(follow['motion_model'], 'Ackermann')
            self.assertEqual(planner['motion_model_for_search'], 'DUBIN')
            self.assertGreater(follow['vx_max'] / follow['wz_max'], 13.1)

    def test_exact_16_yaml_leaf_changes_no_noise_or_weight_retuning(self):
        for entry in self.manifest['profiles'].values():
            config = yaml.safe_load(self.outputs[entry['file']])
            changes = differences(self.baseline, config)
            self.assertEqual(len(changes), 16)
            self.assertEqual([list(path) for path in changes], [x['yaml_path'] for x in entry['changes']])
            self.assertEqual(config['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic'],
                             self.baseline['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic'])
            for node in ('local_costmap', 'global_costmap'):
                expected = copy.deepcopy(self.baseline[node][node]['ros__parameters'])
                expected['robot_base_frame'] = 'rear_axle_nav'
                expected['footprint'] = config[node][node]['ros__parameters']['footprint']
                self.assertEqual(config[node][node]['ros__parameters'], expected)

    def test_shadow_policies_never_claim_motion_or_stopping_proof(self):
        for entry in self.manifest['profiles'].values():
            policy = json.loads(self.outputs[entry['policy_file']])
            self.assertEqual(policy['mode'], 'SHADOW_ONLY')
            self.assertEqual(policy['status'], 'SHADOW_DIAGNOSTIC_POLICY')
            self.assertFalse(policy['runtime_authorized'])
            self.assertFalse(policy['physical_stop_verified'])
            self.assertEqual(policy['policy']['reverse_speed_mps'], 0)
            self.assertEqual(policy['policy']['command_timeout_s'], 5 / 20)
            self.assertEqual(policy['policy']['speed_policy'], 'REJECT')

    def test_rear_right_pid_geometry_separates_front_track_from_rear_lever_arm(self):
        geometry = self.manifest['geometry']
        self.assertEqual(geometry['rear_track_planar_m'], 2.598)
        self.assertEqual(geometry['rear_right_lateral_offset_from_axle_m'], -1.299)
        self.assertNotEqual(geometry['front_track_plugin_collision_m'], geometry['rear_track_planar_m'])

    def test_pinned_inputs_include_confirmed_variant_manifest(self):
        root = self.copied_sources()
        path = root / 'config/auto_tuning/tf_sensor_model_variant.json'
        path.write_bytes(path.read_bytes() + b'\n')
        with self.assertRaisesRegex(plant.PlantCandidateError, 'SHA-256'):
            plant.build_bundle(root)

    def test_generation_refuses_to_overwrite_previous_candidate(self):
        target = self.temporary() / 'candidate'
        plant.prepare_bundle(ROOT, target)
        original = (target / 'manifest.json').read_bytes()
        with self.assertRaises(plant.PlantCandidateError):
            plant.prepare_bundle(ROOT, target)
        self.assertEqual((target / 'manifest.json').read_bytes(), original)

    def test_tampered_missing_extra_and_symlink_files_are_rejected(self):
        for mutation in ('tampered', 'missing', 'extra', 'symlink'):
            with self.subTest(mutation=mutation):
                target = self.temporary() / 'candidate'
                plant.prepare_bundle(ROOT, target)
                file = target / plant.MODEL_PATH
                if mutation == 'tampered':
                    file.write_bytes(file.read_bytes() + b'\n')
                elif mutation == 'missing':
                    file.unlink()
                elif mutation == 'extra':
                    (target / 'extra.json').write_text('{}')
                else:
                    file.unlink()
                    file.symlink_to(self.bundle / plant.MODEL_PATH)
                with self.assertRaises(plant.PlantCandidateError):
                    plant.verify_bundle(ROOT, target)

    def test_installed_image_layout_regenerates_same_contract_and_loads_profile(self):
        temp = self.temporary()
        share, config = temp / 'ws/install/truck_bringup/share/truck_bringup', temp / 'opt/auto_tuning/config'
        paths = plant.source_paths(installed_share=share, config_root=config)
        for name, path in paths.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((ROOT / name).read_bytes())
        bundle = config / 'plant_candidate'
        bundle.mkdir(parents=True)
        for name, raw in self.outputs.items():
            path = bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        self.assertEqual(plant.verify_bundle(installed_share=share, config_root=config), self.manifest)
        result, entry = plant.load_selected_profile(share / 'params/truck_nav2_params.yaml', bundle, 'candidate20kmh')
        self.assertEqual(entry, self.manifest['profiles']['candidate20kmh'])
        self.assertEqual(result['bt_navigator']['ros__parameters']['robot_base_frame'], 'rear_axle_nav')
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(plant.main(['verify', '--installed-share', str(share), '--config', str(config)]), 0)

    def test_repository_loader_rejects_unknown_profile(self):
        config, _ = plant.load_selected_profile(ROOT / plant.SOURCE_PATH, self.bundle, 'candidate25kmh')
        self.assertEqual(config['controller_server']['ros__parameters']['FollowPath']['vx_min'], 0.0)
        with self.assertRaises(plant.PlantCandidateError):
            plant.load_selected_profile(ROOT / plant.SOURCE_PATH, self.bundle, 'canonical')

    def test_incomplete_or_mixed_runtime_locations_fail_with_a_readable_contract_error(self):
        for kwargs in ({}, {'installed_share': ROOT}, {'repo': ROOT, 'config_root': ROOT}):
            with self.subTest(kwargs=kwargs), self.assertRaises(plant.PlantCandidateError):
                plant.verify_bundle(**kwargs)

    def test_manifest_never_upgrades_static_contract_to_gate_approval(self):
        for key in ('runtime_verified', 'runtime_geometry_validated', 'motion_authorized', 'mission_ready'):
            self.assertIs(self.manifest[key], False)
        self.assertEqual(self.manifest['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(self.manifest['gate1'], 'NOT_YET_VERIFIED')


if __name__ == '__main__':
    unittest.main()
