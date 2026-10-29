"""Static safety contracts for the proposed localization profile, not ROS tests."""

import ast
from copy import deepcopy
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src/gps_truck_nav/truck_bringup'


class Gate0LocalizationContractTests(unittest.TestCase):
    def setUp(self):
        self.canonical = yaml.safe_load((BRINGUP / 'params/dual_ekf_navsat_params.yaml').read_text())
        self.proposed = yaml.safe_load((ROOT / 'config/auto_tuning/dual_ekf_navsat_gate0.yaml').read_text())

    def test_only_four_declared_localization_fields_change(self):
        expected = deepcopy(self.canonical)
        local = expected['ekf_filter_node_odom']['ros__parameters']
        local['world_frame'] = 'odom'
        local['publish_tf'] = False
        global_filter = expected['ekf_filter_node_map']['ros__parameters']
        global_filter['odom1'] = '/odometry/gps'
        global_filter['odom1_config'] = [True, True] + [False] * 13
        self.assertEqual(self.proposed, expected)

    def test_one_configured_authority_per_navigation_transform(self):
        model = ET.parse(BRINGUP / 'models/arocs_truck/model.sdf')
        drive = model.find('.//plugin[@name="ackermann_drive"]')
        self.assertEqual(drive.findtext('publish_odom_tf'), '1')
        authorities = [(drive.findtext('odometry_frame'), drive.findtext('robot_base_frame'))]
        for name in ('ekf_filter_node_odom', 'ekf_filter_node_map'):
            params = self.proposed[name]['ros__parameters']
            if params['publish_tf']:
                parent = params['world_frame']
                child = params['odom_frame'] if parent == params['map_frame'] else params['base_link_frame']
                authorities.append((parent, child))
        self.assertCountEqual(authorities, [('odom', 'base_footprint'), ('map', 'odom')])

    def test_gps_pose_selected_and_no_unmeasured_gps_twist_fused(self):
        params = self.proposed['ekf_filter_node_map']['ros__parameters']
        self.assertTrue(params['odom1'].endswith('/odometry/gps'))
        self.assertEqual(params['odom1_config'][:3], [True, True, False])
        self.assertFalse(any(params['odom1_config'][3:]))
        self.assertFalse(params['odom1_differential'])

    def test_urdf_cloud_matches_sdf_sensor_without_legacy_transform(self):
        urdf = ET.parse(BRINGUP / 'urdf/arocs_truck.urdf')
        base = urdf.find('.//joint[@name="base_joint"]')
        cloud = urdf.find('.//joint[@name="sick_joint"]')
        self.assertEqual(base.find('parent').get('link'), 'base_footprint')
        self.assertEqual(cloud.find('parent').get('link'), 'base_link')
        self.assertEqual(cloud.find('child').get('link'), 'cloud')
        base_xyz = [float(v) for v in base.find('origin').get('xyz').split()]
        cloud_xyz = [float(v) for v in cloud.find('origin').get('xyz').split()]
        pose = ET.parse(BRINGUP / 'models/arocs_truck/model.sdf').findtext('.//sensor[@name="sick_lidar"]/pose')
        sdf_pose = [float(v) for v in pose.split()]
        for expected, actual in zip(sdf_pose[:3], [a + b for a, b in zip(base_xyz, cloud_xyz)]):
            self.assertAlmostEqual(actual, expected, places=7)
        self.assertEqual([float(v) for v in cloud.find('origin').get('rpy').split()], sdf_pose[3:])

    def test_legacy_tf_default_preserved_and_opt_out_wired(self):
        tree = ast.parse((BRINGUP / 'launch/gps_waypoint_follower.launch.py').read_text())
        declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                        and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                        and n.args and isinstance(n.args[0], ast.Constant)
                        and n.args[0].value == 'publish_legacy_cloud_tf']
        self.assertEqual(len(declarations), 1)
        defaults = [kw.value.value for kw in declarations[0].keywords if kw.arg == 'default_value']
        self.assertEqual(defaults, ['true'])
        nodes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'Node'
                 and any(kw.arg == 'name' and isinstance(kw.value, ast.Constant)
                         and kw.value.value == 'static_transform_cloud' for kw in n.keywords)]
        self.assertEqual(len(nodes), 1)
        condition = next(kw.value for kw in nodes[0].keywords if kw.arg == 'condition')
        self.assertEqual(ast.dump(condition), ast.dump(ast.parse("IfCondition(lc('publish_legacy_cloud_tf'))", mode='eval').body))


if __name__ == '__main__':
    unittest.main()
