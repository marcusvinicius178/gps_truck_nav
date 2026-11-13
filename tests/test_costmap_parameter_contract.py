"""Offline launch wiring contracts; effective ROS parameters require host dumps.

Capture the actual launch description with inert action constructors, never ROS
processes. verify_parameter_transport.py additionally uses installed Iron APIs
inside the image, and the readiness probe checks the running costmap parameters.
"""

import ast
import importlib.util
import json
import os
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / 'src/gps_truck_nav/truck_bringup'
SPEC = importlib.util.spec_from_file_location(
    'parameter_transport', ROOT / 'tools/auto_tuning/verify_parameter_transport.py')
TRANSPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRANSPORT)


class Captured:
    def __init__(self, kind, args, kwargs):
        self.kind, self.args, self.kwargs = kind, args, kwargs
        self.actions = []

    def add_action(self, action):
        self.actions.append(action)


def capture(kind):
    return lambda *args, **kwargs: Captured(kind, args, kwargs)


def description(filename):
    tree = ast.parse((SHARE / 'launch' / filename).read_text())
    # Replace only dependency constructors. Execute the repository's real
    # generate_launch_description, not a duplicate implementation of its wiring.
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    namespace = {'os': os, 'get_package_share_directory': lambda _: str(SHARE)}
    for name in ('LaunchDescription', 'DeclareLaunchArgument', 'IncludeLaunchDescription',
                 'SetEnvironmentVariable', 'SetLaunchConfiguration', 'IfCondition',
                 'PythonLaunchDescriptionSource', 'LaunchConfiguration', 'Node',
                 'ParameterFile', 'RewrittenYaml', 'GroupAction', 'PythonExpression',
                 'LoadComposableNodes', 'SetParameter', 'ComposableNode'):
        namespace[name] = capture(name)
    exec(compile(tree, filename, 'exec'), namespace)
    return namespace['generate_launch_description']()


def configuration_name(value):
    if value.kind != 'LaunchConfiguration':
        raise AssertionError('Expected a LaunchConfiguration')
    return value.args[0]


class CostmapParameterContracts(unittest.TestCase):
    def setUp(self):
        self.full = description('gps_waypoint_follower.launch.py').args[0]
        self.canonical = yaml.safe_load((SHARE / 'params/truck_nav2_params.yaml').read_text())

    def container(self):
        matches = [action for action in self.full if action.kind == 'Node'
                   and action.kwargs.get('executable') == 'component_container_isolated']
        self.assertEqual(len(matches), 1)
        return matches[0]

    def test_container_receives_complete_parameter_file_not_just_clock(self):
        parameters = self.container().kwargs['parameters']
        self.assertEqual(parameters[0].kind, 'ParameterFile')
        self.assertTrue(parameters[0].kwargs['allow_substs'])
        rewritten = parameters[0].args[0]
        self.assertEqual(rewritten.kind, 'RewrittenYaml')
        self.assertEqual(configuration_name(rewritten.kwargs['source_file']), 'params_file')
        self.assertEqual(rewritten.kwargs['root_key'], '')
        self.assertTrue(rewritten.kwargs['convert_types'])
        self.assertEqual(set(rewritten.kwargs['param_rewrites']), {'autostart'})
        self.assertEqual(configuration_name(parameters[1]['use_sim_time']), 'use_sim_time')

    def test_navigation_receives_same_selected_source_as_container(self):
        includes = [action for action in self.full if action.kind == 'IncludeLaunchDescription'
                    and action.args[0].args[0].endswith('/navigation_launch.py')]
        self.assertEqual(len(includes), 1)
        passed = dict(includes[0].kwargs['launch_arguments'])
        self.assertEqual(configuration_name(passed['params_file']), 'params_file')
        self.assertEqual(passed['namespace'], '')

    def test_child_parameterfile_retains_namespace_root_and_source(self):
        launch = description('navigation_launch.py')
        loads = [action for group in launch.actions if group.kind == 'GroupAction'
                 for action in group.kwargs['actions'] if action.kind == 'LoadComposableNodes']
        self.assertEqual(len(loads), 1)
        self.assertEqual(configuration_name(loads[0].kwargs['target_container'][0]), 'namespace')
        nodes = loads[0].kwargs['composable_node_descriptions']
        for name in ('planner_server', 'controller_server'):
            node = next(item for item in nodes if item.kwargs['name'] == name)
            parameter_file = node.kwargs['parameters'][0]
            self.assertEqual(parameter_file.kind, 'ParameterFile')
            rewritten = parameter_file.args[0]
            self.assertEqual(configuration_name(rewritten.kwargs['root_key']), 'namespace')
            self.assertEqual(configuration_name(rewritten.kwargs['source_file']), 'params_file')

    def test_ackermann_opt_in_defaults_off_and_runs_before_container_and_include(self):
        declarations = [action for action in self.full if action.kind == 'DeclareLaunchArgument'
                        and action.args[0] == 'use_ackermann_bt']
        self.assertEqual(len(declarations), 1)
        self.assertEqual(declarations[0].kwargs['default_value'], 'false')
        rewrites = [action for action in self.full if action.kind == 'SetLaunchConfiguration'
                    and action.args[0] == 'params_file']
        self.assertEqual(len(rewrites), 1)
        rewrite = rewrites[0]
        self.assertEqual(configuration_name(rewrite.kwargs['condition'].args[0]), 'use_ackermann_bt')
        self.assertLess(self.full.index(rewrite), self.full.index(self.container()))
        nav = next(action for action in self.full if action.kind == 'IncludeLaunchDescription'
                   and action.args[0].args[0].endswith('/navigation_launch.py'))
        self.assertLess(self.full.index(rewrite), self.full.index(nav))

    def test_opt_in_changes_only_two_bt_paths_and_preserves_custom_params_source(self):
        setter = next(action for action in self.full if action.kind == 'SetLaunchConfiguration')
        rewrite = setter.args[1]
        self.assertEqual(configuration_name(rewrite.kwargs['source_file']), 'params_file')
        self.assertEqual(rewrite.kwargs['param_rewrites'], {
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_to_pose.xml',
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                '/opt/auto_tuning/config/behavior_trees/ackermann_navigate_through_poses.xml',
        })
        self.assertNotIn('root_key', rewrite.kwargs)

    def test_canonical_costmap_geometry_and_layers_are_the_existing_truck_contract(self):
        for name, dimensions, frame in (
                ('global_costmap', (1000, 300), 'map'),
                ('local_costmap', (200, 120), 'odom')):
            with self.subTest(costmap=name):
                params = self.canonical[name][name]['ros__parameters']
                self.assertEqual((params['width'], params['height']), dimensions)
                self.assertTrue(params['rolling_window'])
                self.assertEqual(params['robot_base_frame'], 'base_footprint')
                self.assertEqual(params['global_frame'], frame)
                self.assertEqual(json.loads(params['footprint']), [
                    [-4.8025, -1.739], [4.8025, -1.739],
                    [4.8025, 1.739], [-4.8025, 1.739]])
                self.assertNotIn('static_layer', params['plugins'])
                self.assertIn('obstacle_layer', params['plugins'])
                self.assertIn('inflation_layer', params['plugins'])
                self.assertTrue(params['obstacle_layer']['observation_sources'])

    def test_hierarchy_reader_preserves_footprint_string_in_root_and_namespace(self):
        for namespace in ('', 'parameter_contract_robot', '/fleet/truck'):
            with self.subTest(namespace=namespace):
                document = {namespace: self.canonical} if namespace else self.canonical
                sections = TRANSPORT.node_sections(document)
                for name in ('global_costmap', 'local_costmap'):
                    target = '/'.join(part for part in (namespace.strip('/'), name, name) if part)
                    self.assertEqual(sections[target], self.canonical[name][name]['ros__parameters'])
                    self.assertIsInstance(sections[target]['footprint'], str)
                    self.assertIsInstance(sections[target]['plugins'], list)


if __name__ == '__main__':
    unittest.main()
