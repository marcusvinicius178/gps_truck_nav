#!/usr/bin/env python3
"""Check installed launch parameter transport without starting any ROS node.

Uses the installed Iron launch/launch_ros implementations, including Node's
substitution-only expansion hook. No Node.execute, rclpy.init or load service is
called. Effective parameters of running costmaps still require the host probe.
"""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile


def flattened(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flattened(value, name + '.'))
        else:
            result[name] = value
    return result


def node_sections(document, prefix=''):
    """Read node-key hierarchy only; preserve parameter scalar/list values.

    Costmap children read the global CLI file through rcl, not the Python
    component request converter (which can reinterpret footprint strings).
    This function verifies the file transport, not rcl's runtime parsing.
    """
    result = {}
    for key, value in document.items():
        if key == 'ros__parameters':
            result[prefix] = value
        elif isinstance(value, dict):
            path = '/'.join(part for part in (prefix, key.strip('/')) if part)
            result.update(node_sections(value, path))
    return result


def verify():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.descriptions import ParameterFile
    from launch_ros.utilities import to_parameters_list
    from nav2_common.launch import RewrittenYaml

    share = Path(get_package_share_directory('truck_bringup'))
    launch_path = share / 'launch/gps_waypoint_follower.launch.py'
    canonical_path = share / 'params/truck_nav2_params.yaml'
    canonical = yaml.safe_load(canonical_path.read_text())
    specification = importlib.util.spec_from_file_location('parameter_transport_launch', launch_path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    observations = []

    for enabled in (False, True):
        context = LaunchContext()
        context.launch_configurations.update({
            'params_file': str(canonical_path), 'use_sim_time': 'true',
            'use_ackermann_bt': str(enabled).lower(), 'use_composition': 'True'})
        description = module.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
            elif isinstance(action, SetLaunchConfiguration):
                if action.condition is None or action.condition.evaluate(context):
                    action.execute(context)

        containers = [action for action in description.entities if isinstance(action, Node)
                      and action.node_package == 'rclcpp_components'
                      and action.node_executable == 'component_container_isolated']
        if len(containers) != 1:
            raise RuntimeError('Expected one declared component container')
        container = containers[0]
        # launch_ros 0.24.2: expansion produces CLI --params-file arguments but
        # never delegates to ExecuteProcess. Fail closed if this API changes.
        container._perform_substitutions(context)
        arguments = getattr(container, '_Node__expanded_parameter_arguments', None)
        if not arguments:
            raise RuntimeError('Container has no expanded parameter arguments')
        files = [Path(value) for value, is_file in arguments if is_file]
        if not files:
            raise RuntimeError('Container is missing a full parameter file')

        def selected(node_name, namespace, paths):
            return {parameter.name: parameter.value for parameter in
                    to_parameters_list(context, node_name, namespace, paths)}

        def transported(node_name, namespace, paths):
            target = '/'.join(part for part in (namespace.strip('/'), node_name) if part)
            values = {}
            for path in paths:
                sections = node_sections(yaml.safe_load(path.read_text()))
                values.update(flattened(sections.get('**', {})))
                values.update(flattened(sections.get(target, {})))
            return values

        observed = {'ackermann_bt': enabled, 'costmaps': {},
                    'container_parameter_file_sha256': [
                        hashlib.sha256(path.read_bytes()).hexdigest() for path in files]}
        for costmap in ('global_costmap', 'local_costmap'):
            expected = flattened(canonical[costmap][costmap]['ros__parameters'])
            actual = transported(costmap, '/' + costmap, files)
            mismatches = {key: {'expected': value, 'actual': actual.get(key)}
                          for key, value in expected.items() if actual.get(key) != value}
            if mismatches:
                raise RuntimeError(f'Container parameter transport mismatch for {costmap}: {mismatches}')
            observed['costmaps'][costmap] = {key: actual[key] for key in (
                'width', 'height', 'resolution', 'rolling_window', 'global_frame',
                'robot_base_frame', 'footprint', 'plugins')}

        actual_bt = selected('bt_navigator', '/', files)
        expected_bt = canonical['bt_navigator']['ros__parameters']
        for key, filename in (
            ('default_nav_to_pose_bt_xml', 'ackermann_navigate_to_pose.xml'),
            ('default_nav_through_poses_bt_xml', 'ackermann_navigate_through_poses.xml')):
            expected = '/opt/auto_tuning/config/behavior_trees/' + filename if enabled else expected_bt.get(key)
            if actual_bt.get(key) != expected:
                raise RuntimeError(f'BT opt-in transport mismatch: {key}')
            if enabled and not Path(expected).is_file():
                raise RuntimeError(f'Missing prospective BT file: {expected}')

        # Exercise real RewrittenYaml + ParameterFile and inspect resulting
        # namespace hierarchy, without converting child costmap values through
        # the composable-parent API. This is not full simulator namespacing.
        scoped = ParameterFile(RewrittenYaml(
            source_file=context.launch_configurations['params_file'],
            root_key='parameter_contract_robot', param_rewrites={}, convert_types=True),
            allow_substs=True)
        scoped_path = Path(scoped.evaluate(context))
        for costmap in ('global_costmap', 'local_costmap'):
            actual = transported(costmap, '/parameter_contract_robot/' + costmap, [scoped_path])
            expected = flattened(canonical[costmap][costmap]['ros__parameters'])
            if any(actual.get(key) != value for key, value in expected.items()):
                raise RuntimeError(f'Namespaced parameter selection mismatch: {costmap}')
        observed['namespaced_yaml_hierarchy'] = 'PASS'
        observations.append(observed)
    return observations


def main():
    record = {'schema_version': 1, 'status': 'PARAMETER_TRANSPORT_FAILED',
              'scope': 'INSTALLED_LAUNCH_FILE_TRANSPORT_AND_PARENT_PARAMETER_SELECTION_ONLY',
              'child_costmap_rcl_parsing_verified': False,
              'runtime_effective_parameters_verified': False,
              'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    try:
        with tempfile.TemporaryDirectory(prefix='nav2-parameter-transport-') as temporary:
            previous_tempdir = tempfile.tempdir
            tempfile.tempdir = temporary
            try:
                os.environ['ROS_LOG_DIR'] = str(Path(temporary) / 'ros_logs')
                record['observations'] = verify()
            finally:
                tempfile.tempdir = previous_tempdir
        record['status'] = 'PARAMETER_TRANSPORT_VERIFIED'
    except Exception as error:
        record['error'] = f'{type(error).__name__}: {error}'
    print(json.dumps(record, ensure_ascii=False))
    return 0 if record['status'] == 'PARAMETER_TRANSPORT_VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
