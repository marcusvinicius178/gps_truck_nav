"""Execute the actual deferred URDF selector with inert launch objects."""
import ast
import os
from pathlib import Path
import tempfile
import unittest

from test_costmap_parameter_contract import Captured, capture, description, configuration_name, SHARE


class ProspectiveDescription(unittest.TestCase):
    def selector(self):
        path = SHARE / 'launch/truck_simulation_launch.py'
        tree = ast.parse(path.read_text())
        tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
        class Configuration:
            def __init__(self, name): self.name = name
            def perform(self, context): return context[self.name]
        namespace = {'os': os, 'Path': Path, 'LaunchConfiguration': Configuration,
                     'Node': capture('Node')}
        exec(compile(tree, str(path), 'exec'), namespace)
        return namespace['robot_state_publisher']

    def test_full_launch_forwards_description_and_preserves_canonical_default(self):
        actions = description('gps_waypoint_follower.launch.py').args[0]
        argument = next(a for a in actions if a.kind == 'DeclareLaunchArgument'
                        and a.args[0] == 'robot_description_file')
        self.assertEqual(argument.kwargs['default_value'], str(SHARE / 'urdf/arocs_truck.urdf'))
        included = next(a for a in actions if a.kind == 'IncludeLaunchDescription'
                        and a.args[0].args[0].endswith('/truck_simulation_launch.py'))
        self.assertEqual(configuration_name(dict(included.kwargs['launch_arguments'])['robot_description_file']),
                         'robot_description_file')

    def test_selected_file_is_read_at_runtime_without_shell_interpolation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'description with spaces $(literal).urdf'
            text = '<robot name="candidate"><link name="rear_axle_nav"/></robot>\n'
            path.write_text(text)
            nodes = self.selector()({'robot_description_file': str(path)})
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].kwargs['name'], 'robot_state_publisher')
            self.assertEqual(nodes[0].kwargs['parameters'][0]['robot_description'], text)

    def test_missing_prospective_file_does_not_fall_back_to_canonical(self):
        with self.assertRaises(FileNotFoundError):
            self.selector()({'robot_description_file': '/not-a-real-plant-description.urdf'})


if __name__ == '__main__':
    unittest.main()
