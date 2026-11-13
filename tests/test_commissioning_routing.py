"""Execute actual launch construction with inert actions; no ROS or motion.

The existing transport tests replace ROS constructors, not repository logic.
These tests additionally evaluate the real topic substitutions for each choice.
Actual DDS endpoints and publisher GIDs remain runtime commissioning evidence.
"""

import unittest
import xml.etree.ElementTree as ET

from test_costmap_parameter_contract import (
    Captured, ROOT, configuration_name, description,
)


def resolve(value, routing):
    if isinstance(value, str):
        return value
    if not isinstance(value, Captured):
        raise AssertionError('Unexpected substitution type')
    if value.kind == 'LaunchConfiguration':
        if value.args != ('actuation_routing',):
            raise AssertionError('Unexpected routing configuration')
        return routing
    if value.kind == 'PythonExpression':
        expression = ''.join(resolve(part, routing) for part in value.args[0])
        return eval(expression, {'__builtins__': {}})
    raise AssertionError('Unexpected routing substitution: ' + value.kind)


def nodes_by_composition():
    groups = [item for item in description('navigation_launch.py').actions
              if item.kind == 'GroupAction']
    standalone = [item for item in groups[0].kwargs['actions'] if item.kind == 'Node']
    load = next(item for item in groups[1].kwargs['actions']
                if item.kind == 'LoadComposableNodes')
    return standalone, load.kwargs['composable_node_descriptions']


def command_remaps(nodes, name, routing):
    node = next(item for item in nodes
                if item.kwargs.get('name', item.kwargs.get('executable')) == name)
    return {source: resolve(target, routing)
            for source, target in node.kwargs.get('remappings', [])
            if source not in ('/tf', '/tf_static')}


class CommissioningRoutingTests(unittest.TestCase):
    def test_both_entrypoints_default_to_legacy_with_explicit_closed_choices(self):
        for filename in ('navigation_launch.py', 'gps_waypoint_follower.launch.py'):
            launch = description(filename)
            actions = launch.actions if filename == 'navigation_launch.py' else launch.args[0]
            declaration = next(item for item in actions if item.kind == 'DeclareLaunchArgument'
                               and item.args[0] == 'actuation_routing')
            self.assertEqual(declaration.kwargs['default_value'], 'legacy')
            self.assertEqual(declaration.kwargs['choices'], ['legacy', 'isolated'])

    def test_full_launch_forwards_routing_without_changing_parameter_sources(self):
        actions = description('gps_waypoint_follower.launch.py').args[0]
        include = next(item for item in actions if item.kind == 'IncludeLaunchDescription'
                       and item.args[0].args[0].endswith('/navigation_launch.py'))
        arguments = dict(include.kwargs['launch_arguments'])
        self.assertEqual(configuration_name(arguments['actuation_routing']), 'actuation_routing')
        self.assertEqual(configuration_name(arguments['params_file']), 'params_file')

    def test_legacy_topics_remain_unchanged_in_both_composition_modes(self):
        for nodes in nodes_by_composition():
            self.assertEqual(command_remaps(nodes, 'controller_server', 'legacy'),
                             {'cmd_vel': 'cmd_vel_nav'})
            self.assertEqual(command_remaps(nodes, 'velocity_smoother', 'legacy'),
                             {'cmd_vel': 'cmd_vel_nav', 'cmd_vel_smoothed': 'cmd_vel'})
            # Explicit identity remapping preserves the legacy relative topic.
            self.assertEqual(command_remaps(nodes, 'behavior_server', 'legacy'),
                             {'cmd_vel': 'cmd_vel'})

    def test_isolated_chain_keeps_twist_units_and_quarantines_behavior_bypass(self):
        for nodes in nodes_by_composition():
            self.assertEqual(command_remaps(nodes, 'controller_server', 'isolated'),
                             {'cmd_vel': 'cmd_vel_nav'})
            self.assertEqual(command_remaps(nodes, 'velocity_smoother', 'isolated'),
                             {'cmd_vel': 'cmd_vel_nav', 'cmd_vel_smoothed': '/actuation/nav2_twist'})
            self.assertEqual(command_remaps(nodes, 'behavior_server', 'isolated'),
                             {'cmd_vel': '/actuation/behavior_quarantine'})

    def test_nav2_launch_never_connects_to_final_actuator_or_commissioning_source(self):
        forbidden = {'/actuation/steering_command', '/commissioning/cmd_vel'}
        for nodes in nodes_by_composition():
            for node in nodes:
                targets = {resolve(target, 'isolated')
                           for _, target in node.kwargs.get('remappings', [])}
                self.assertFalse(targets & forbidden)
                self.assertNotIn('commissioning_adapter', node.kwargs.values())

    def test_candidate_plant_retains_private_actuator_subscription(self):
        model = ET.parse(ROOT / 'config/auto_tuning/plant_candidate/models/arocs_truck_candidate/model.sdf')
        plugin = model.find(".//plugin[@name='ackermann_drive']")
        self.assertIsNotNone(plugin)
        self.assertEqual([item.text for item in plugin.findall('ros/remapping')],
                         ['cmd_vel:=/actuation/steering_command'])


if __name__ == '__main__':
    unittest.main()
