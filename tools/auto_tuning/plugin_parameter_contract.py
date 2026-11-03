"""Version-scoped parameter evidence; a node value is not algorithm execution.

Read sites were inspected in Nav2 1.2.10. This module never changes parameters,
launches ROS, sends goals, or treats a configured but unused YAML key as consumed.
"""

import hashlib
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


NAV2_VERSION = '1.2.10'
BASE_URL = 'https://github.com/ros-navigation/navigation2/blob/1.2.10/'
SOURCES = {
    'smac': ('nav2_smac_planner/src/smac_planner_hybrid.cpp', '42dbd4efe0b4d826fbe572164da0a2ef761148a8'),
    'smac_smoother': ('nav2_smac_planner/include/nav2_smac_planner/types.hpp', '1df34851afdd0c0c3fc2bb38889228bc35f6a010'),
    'planner_server': ('nav2_planner/src/planner_server.cpp', '82a2e3adb7d8b340efe918db85bb86ddd958cd8d'),
    'controller_server': ('nav2_controller/src/controller_server.cpp', '9dd6c118eabc038ad1941d537d284526de463d71'),
    'mppi': ('nav2_mppi_controller/src/controller.cpp', '6c83131263334376101fabf75b062b1a0a1434b0'),
    'optimizer': ('nav2_mppi_controller/src/optimizer.cpp', '86359395ef96081b9858ffc1fd1d61fa762a8d72'),
    'handler': ('nav2_mppi_controller/include/nav2_mppi_controller/tools/parameters_handler.hpp', '1667fa6d79d4b2ac4a31fdfc9e767f6f46f38e57'),
    'path': ('nav2_mppi_controller/src/path_handler.cpp', 'ddf2b403b683fbf76b79bb20578168d47cd8900c'),
    'visualizer': ('nav2_mppi_controller/src/trajectory_visualizer.cpp', 'a6531b7d1df7b1566220a18edb397b035d45552e'),
    'motion': ('nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp', '0c5671ea3e56f994c8257d19026c9dfecd8fb34d'),
    'noise': ('nav2_mppi_controller/src/noise_generator.cpp', 'b7c452aa6a48632bf9ff4e6e82a3fc2b63e6e715'),
    'critic_manager': ('nav2_mppi_controller/src/critic_manager.cpp', '2a7a77a234699b0130f67a55a76557712dce0615'),
    'critic_base': ('nav2_mppi_controller/include/nav2_mppi_controller/critic_function.hpp', '88d277c14ee19c0abb23205dbb7dc45005c2a99d'),
    'PathFollowCritic': ('nav2_mppi_controller/src/critics/path_follow_critic.cpp', '01ecb91728e00c1d170582ed327842a08f61cc09'),
    'PathAlignCritic': ('nav2_mppi_controller/src/critics/path_align_critic.cpp', 'b103f5b5f64ddebc767743de3d07691065b590b4'),
    'PathAngleCritic': ('nav2_mppi_controller/src/critics/path_angle_critic.cpp', 'cd701ea093d978b44d5cc6273fd25f2aaf541c3a'),
    'GoalCritic': ('nav2_mppi_controller/src/critics/goal_critic.cpp', 'f61d6fd2b2132be7f06002061e1272ba49edc521'),
    'PreferForwardCritic': ('nav2_mppi_controller/src/critics/prefer_forward_critic.cpp', '08e755dd6affe55a891820eb4eea1bac92f45b78'),
    'ObstaclesCritic': ('nav2_mppi_controller/src/critics/obstacles_critic.cpp', '57b18d6eb8289c42cfa4a7b2d59b6891a69a668f'),
}
SMAC_KEYS = ('downsample_costmap downsampling_factor angle_quantization_bins tolerance allow_unknown '
             'max_iterations max_on_approach_iterations smooth_path minimum_turning_radius '
             'cache_obstacle_heuristic reverse_penalty change_penalty non_straight_penalty '
             'cost_penalty retrospective_penalty analytic_expansion_ratio analytic_expansion_max_length '
             'max_planning_time lookup_table_size debug_visualizations motion_model_for_search').split()
OPTIMIZER_KEYS = ('model_dt time_steps batch_size iteration_count temperature gamma vx_max vx_min '
                  'vy_max wz_max vx_std vy_std wz_std retry_attempt_limit motion_model').split()
PATH_KEYS = ('max_robot_pose_search_dist prune_distance transform_tolerance enforce_path_inversion').split()
CRITIC_KEYS = {
    'PathFollowCritic': 'cost_power cost_weight threshold_to_consider offset_from_furthest',
    'PathAlignCritic': 'cost_power cost_weight max_path_occupancy_ratio offset_from_furthest trajectory_point_step threshold_to_consider use_path_orientations',
    'PathAngleCritic': 'cost_power cost_weight offset_from_furthest threshold_to_consider max_angle_to_furthest mode',
    'GoalCritic': 'cost_power cost_weight threshold_to_consider',
    'PreferForwardCritic': 'cost_power cost_weight threshold_to_consider',
    'ObstaclesCritic': 'cost_power consider_footprint repulsion_weight critical_weight collision_cost collision_margin_distance near_goal_distance inflation_layer_name',
}


def flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        if isinstance(value, dict):
            result.update(flatten(value, prefix + key + '.'))
        else:
            result[prefix + key] = value
    return result


def ros_type(value):
    """Return rcl_interfaces/ParameterType without importing ROS."""
    if isinstance(value, bool):
        return 1
    if isinstance(value, int):
        return 2
    if isinstance(value, float):
        return 3
    if isinstance(value, str):
        return 4
    if isinstance(value, list) and value:
        kinds = {ros_type(item) for item in value}
        if len(kinds) == 1 and next(iter(kinds)) in (1, 2, 3, 4):
            return {1: 6, 2: 7, 3: 8, 4: 9}[next(iter(kinds))]
    raise ValueError('Unsupported or ambiguous parameter value type')


def build_contract(canonical):
    planner = canonical['planner_server']['ros__parameters']
    controller = canonical['controller_server']['ros__parameters']
    if (planner.get('planner_plugins') != ['GridBased'] or
            planner.get('GridBased', {}).get('plugin') != 'nav2_smac_planner/SmacPlannerHybrid' or
            controller.get('controller_plugins') != ['FollowPath'] or
            controller.get('FollowPath', {}).get('plugin') != 'nav2_mppi_controller::MPPIController'):
        raise ValueError('Source audit is scoped to GridBased/SmacPlannerHybrid and FollowPath/MPPIController.')
    nodes = {}
    for node_name in ('planner_server', 'controller_server'):
        configured = flatten(canonical[node_name]['ros__parameters'])
        configured['use_sim_time'] = True
        nodes['/' + node_name] = {
            key: {'expected': value, 'expected_type': ros_type(value), 'origin': 'PROJECT_YAML',
                  'required': False, 'source_consumer': 'NO_READ_SITE_IDENTIFIED',
                  'note': 'No consumer identified in the inspected Nav2 1.2.10 path; declaration is not use.'}
            for key, value in configured.items()}

    def consumed(node, keys, source, *, prefix='', condition=True):
        for suffix in keys:
            key = prefix + suffix
            if key in nodes[node]:
                nodes[node][key].update(required=condition,
                    source_consumer='SOURCE_CONSUMER_IDENTIFIED' if condition else 'CONDITION_DISABLED',
                    source=source, note='' if condition else 'Read site is disabled by this configuration.')

    def default(node, name, value, source, *, required=True):
        if name not in nodes[node]:
            nodes[node][name] = {'expected': value, 'expected_type': ros_type(value),
                'origin': 'UPSTREAM_1_2_10_DEFAULT', 'required': required,
                'source_consumer': 'SOURCE_CONSUMER_IDENTIFIED' if required else 'CONDITION_DISABLED',
                'source': source, 'note': 'Observed only; this probe does not install or set defaults.'}

    p, c = '/planner_server', '/controller_server'
    consumed(p, ('planner_plugins', 'expected_planner_frequency', 'GridBased.plugin'), 'planner_server')
    consumed(p, SMAC_KEYS, 'smac', prefix='GridBased.')
    if canonical['planner_server']['ros__parameters']['GridBased'].get('smooth_path', True):
        for key, value in {'tolerance': 1e-10, 'max_iterations': 1000, 'w_data': 0.2,
                           'w_smooth': 0.3, 'do_refinement': True, 'refinement_num': 2}.items():
            name = 'GridBased.smoother.' + key
            default(p, name, value, 'smac_smoother')
            consumed(p, (name,), 'smac_smoother')
    consumed(c, ('controller_frequency', 'min_x_velocity_threshold', 'min_y_velocity_threshold',
                 'min_theta_velocity_threshold', 'failure_tolerance', 'controller_plugins',
                 'progress_checker_plugins', 'goal_checker_plugins', 'FollowPath.plugin',
                 'front_cabin_goal_checker.plugin', 'progress_checker.plugin'), 'controller_server')
    # Controller plugins outside SMAC/MPPI are captured, but are not falsely
    # classified as unused merely because this review does not audit their code.
    for key, value in nodes[c].items():
        if key.startswith('progress_checker.') and key != 'progress_checker.plugin':
            value.update(source_consumer='OUTSIDE_PLUGIN_REVIEW_SCOPE', note='Separate progress checker contract.')
        if key.startswith('general_goal_checker.'):
            value.update(source_consumer='INACTIVE_PLUGIN_CONFIGURATION', note='Not selected in goal_checker_plugins.')
    consumed(c, OPTIMIZER_KEYS, 'optimizer', prefix='FollowPath.')
    consumed(c, PATH_KEYS, 'path', prefix='FollowPath.')
    consumed(c, ('FollowPath.visualize',), 'mppi')
    consumed(c, ('FollowPath.regenerate_noises',), 'noise')
    consumed(c, ('FollowPath.critics',), 'critic_manager')
    consumed(c, ('FollowPath.AckermannConstraints.min_turning_r',), 'motion')
    follow = canonical['controller_server']['ros__parameters']['FollowPath']
    consumed(c, ('inversion_xy_tolerance', 'inversion_yaw_tolerance'), 'path', prefix='FollowPath.',
             condition=follow.get('enforce_path_inversion', False))
    default(c, 'FollowPath.vy_std', 0.2, 'optimizer')
    for key, value in {'trajectory_step': 5, 'time_step': 3}.items():
        name = 'FollowPath.TrajectoryVisualizer.' + key
        default(c, name, value, 'visualizer')
        consumed(c, (name,), 'visualizer')
        if 'FollowPath.' + key in nodes[c]:
            nodes[c]['FollowPath.' + key]['note'] = 'Actual read site: ' + name + '; this flat key is not read.'
    for critic in follow['critics']:
        if critic not in CRITIC_KEYS:
            continue  # Unknown critic cannot acquire a consumer claim by name.
        consumed(c, CRITIC_KEYS[critic].split(), critic, prefix='FollowPath.' + critic + '.')
        consumed(c, ('FollowPath.' + critic + '.enabled',), 'critic_base')
    default(c, 'FollowPath.PathAlignCritic.trajectory_point_step', 4, 'PathAlignCritic',
            required='PathAlignCritic' in follow['critics'])
    for key in ('cost_scaling_factor', 'inflation_radius'):
        name = 'FollowPath.ObstaclesCritic.' + key
        if name in nodes[c]:
            nodes[c][name]['note'] = ('ObstaclesCritic reads the selected local InflationLayer accessor; '
                                     'the same-named critic YAML entry is not its source.')
    for node in nodes.values():
        node['use_sim_time'].update(required=True, source_consumer='ROS_CLOCK_CONTRACT',
                                    note='Explicit launch clock contract, outside plugin tuning.')
    return {'schema_version': 1, 'nav2_source_version': NAV2_VERSION, 'nodes': nodes,
            'sources': {key: {'url': BASE_URL + path, 'blob_sha': sha} for key, (path, sha) in SOURCES.items()},
            'scope': 'Typed parameter readback plus version-scoped source mapping; no runtime mutation.'}


def same_value(expected, actual):
    if type(expected) is not type(actual):
        return False
    if isinstance(expected, float):
        return math.isfinite(actual) and math.isclose(expected, actual, rel_tol=1e-9, abs_tol=1e-12)
    if isinstance(expected, list):
        return len(expected) == len(actual) and all(same_value(x, y) for x, y in zip(expected, actual))
    return expected == actual


def assess_plugin_parameters(report, provenance=None):
    checks, entries = [], []
    provenance = provenance if provenance is not None else report.get('provenance', {})
    packages = provenance.get('packages', {})
    targets = provenance.get('runtime_binary_identity', {}).get('targets', {})
    associations = {}
    for node, package, target in (('/planner_server', 'nav2_smac_planner', 'smac_hybrid'),
                                  ('/controller_server', 'nav2_mppi_controller', 'mppi_controller')):
        installed = packages.get(package, {}).get('libraries', [])
        mapped = targets.get(target, {})
        records = mapped.get('records', [])
        installed_hashes = {item.get('sha256') for item in installed}
        mapped_hashes = {item.get('sha256') for item in records}
        valid = (installed and records and all(isinstance(value, str) and value for value in installed_hashes | mapped_hashes)
                 and mapped.get('status') == 'MAPPED_IDENTIFIED')
        association = {'package': package, 'runtime_target': target,
            'status': 'UNKNOWN' if not valid else 'PASS' if installed_hashes == mapped_hashes else 'FAIL',
            'installed_sha256': sorted(value for value in installed_hashes if isinstance(value, str)),
            'mapped_sha256': sorted(value for value in mapped_hashes if isinstance(value, str)),
            'scope': 'Installed prefix library matches mapped identity; not equivalence to upstream source.'}
        associations[node] = association
        checks.append({'name': 'plugin_binary_association:' + node, **association})
    def source_version_matches(source):
        package = {'planner_server': 'nav2_planner', 'controller_server': 'nav2_controller',
                   'smac': 'nav2_smac_planner', 'smac_smoother': 'nav2_smac_planner'}.get(
                       source, 'nav2_mppi_controller')
        return packages.get(package, {}).get('version') == NAV2_VERSION
    contract = report.get('contract', {})
    nodes = contract.get('nodes', {})
    if not nodes:
        checks.append({'name': 'plugin_parameter_contract', 'status': 'UNKNOWN', 'reason': 'Contract absent.'})
    for node, expected in nodes.items():
        actual = report.get('nodes', {}).get(node, {})
        plugin_key = 'GridBased.plugin' if node == '/planner_server' else 'FollowPath.plugin'
        plugin_sample = actual.get('values', {}).get(plugin_key, {})
        plugin_identity_matches = (plugin_sample.get('type') == 4 and
                                   plugin_sample.get('value') == expected[plugin_key]['expected'])
        missing, differences, unsupported, optional_unknown = [], {}, [], []
        for key, wanted in expected.items():
            sample = actual.get('values', {}).get(key)
            description = actual.get('descriptors', {}).get(key)
            declaration = actual.get('declarations', {}).get(key, {})
            known_absent = declaration.get('status') == 'UNDECLARED'
            declared = bool(description and description.get('type', 0) != 0)
            matches = bool(sample and sample.get('type') == wanted['expected_type']
                           and same_value(wanted['expected'], sample.get('value')))
            if wanted['required']:
                if sample is None or description is None:
                    missing.append(key)
                elif not declared or description.get('type') != wanted['expected_type'] or not matches:
                    differences[key] = {'expected': wanted, 'actual': sample, 'descriptor': description}
            if wanted['source_consumer'] == 'NO_READ_SITE_IDENTIFIED':
                unsupported.append(key)
            if not wanted['required'] and (declaration.get('status') == 'INCONSISTENT_OBSERVATIONS'
                    or not known_absent and (sample is None or description is None)):
                optional_unknown.append(key)
            version_matches = source_version_matches(wanted.get('source')) if wanted.get('source') else None
            source_consumer = wanted['source_consumer']
            plugin_source = wanted.get('source') not in (None, 'planner_server', 'controller_server')
            if source_consumer == 'SOURCE_CONSUMER_IDENTIFIED' and not version_matches:
                source_consumer = 'UNKNOWN_INSTALLED_VERSION'
            elif source_consumer == 'SOURCE_CONSUMER_IDENTIFIED' and not plugin_identity_matches and plugin_source:
                source_consumer = 'UNKNOWN_PLUGIN_IDENTITY'
            elif source_consumer == 'SOURCE_CONSUMER_IDENTIFIED' and plugin_source and associations[node]['status'] != 'PASS':
                source_consumer = ('BINARY_IDENTITY_MISMATCH' if associations[node]['status'] == 'FAIL'
                                   else 'UNKNOWN_BINARY_ASSOCIATION')
            package_names = (('nav2_planner', 'nav2_smac_planner') if node == '/planner_server'
                             else ('nav2_controller', 'nav2_mppi_controller'))
            unsupported_evidence = (known_absent and wanted['source_consumer'] == 'NO_READ_SITE_IDENTIFIED'
                and all(packages.get(name, {}).get('version') == NAV2_VERSION for name in package_names)
                and plugin_identity_matches and associations[node]['status'] == 'PASS')
            availability = ('UNSUPPORTED_VERSION' if unsupported_evidence else
                'UNDECLARED' if known_absent else declaration.get('status', 'DECLARED' if declared else 'UNKNOWN'))
            entries.append({'node': node, 'name': key, **wanted,
                'source_consumer': source_consumer,
                'SOURCE_AUDIT_VERSION_MATCHED': 'YES' if version_matches else 'UNKNOWN' if version_matches is False else 'NOT_APPLICABLE',
                'PLUGIN_IDENTITY_MATCHED': plugin_identity_matches,
                'INSTALLED_MAPPED_ASSOCIATION': associations[node]['status'] if plugin_source else 'NOT_APPLICABLE',
                'DECLARED': 'NO' if known_absent else 'YES' if declared else 'NOT_SET' if description is not None else 'UNKNOWN',
                'EFFECTIVE': 'NOT_APPLICABLE_UNDECLARED' if known_absent else
                    'MATCH' if matches else 'DIFFERS' if sample is not None else 'UNKNOWN',
                'AVAILABILITY': availability,
                'availability_evidence': declaration,
                'unsupported_version_scope': ('No read site in the inspected 1.2.10 path plus observed undeclared name; '
                    'not a claim about other releases or source/binary equivalence.') if unsupported_evidence else None,
                'tuning_eligible': False,
                'observed': sample, 'descriptor': description,
                'ACTUALLY_USED': 'NOT_VERIFIED',
                'usage_limit': 'Get/DescribeParameters do not expose internal algorithm execution.'})
        checks.append({'name': 'effective_plugin_parameters:' + node,
                       'status': 'FAIL' if differences else 'UNKNOWN' if missing else 'PASS',
                       'missing': missing, 'differences': differences,
                       'scope': 'Required parameters: returned ROS types, node values and descriptors.'})
        checks.append({'name': 'unconsumed_yaml_parameters:' + node,
                       'status': 'BLOCKED_REVIEW' if unsupported else 'PASS', 'names': unsupported,
                       'source_version': NAV2_VERSION,
                       'reason': 'Reconcile before selecting tuning knobs; canonical YAML is preserved.'})
        checks.append({'name': 'optional_parameter_declarations:' + node,
                       'status': 'UNKNOWN' if optional_unknown else 'PASS',
                       'unresolved_names': optional_unknown,
                       'known_undeclared_names': sorted(key for key, item in actual.get('declarations', {}).items()
                                                       if item.get('status') == 'UNDECLARED'),
                       'scope': 'Resolved observation outcomes, not optional parameters accepted as useful tuning knobs.'})
    identities_ok = all(packages.get(name, {}).get('version') == NAV2_VERSION
                        and packages.get(name, {}).get('libraries')
                        and all(library.get('sha256') for library in packages[name]['libraries'])
                        for name in ('nav2_smac_planner', 'nav2_mppi_controller'))
    checks.append({'name': 'plugin_version_and_binary_identity', 'status': 'PASS' if identities_ok else 'UNKNOWN',
                   'source_version': NAV2_VERSION, 'observed': provenance,
                   'limit': 'Installed identity is not proof of equivalence to upstream source or execution.'})
    checks.append({'name': 'plugin_parameters_actually_used', 'status': 'BLOCKED_REVIEW',
                   'reason': 'No planning/control computation or controlled parameter intervention was executed.',
                   'configure_log_observations': report.get('configure_log_observations', [])})
    return {'schema_version': 1, 'checks': checks, 'parameters': entries, 'binary_associations': associations,
            'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}


def installed_plugin_provenance():
    """Record installed package versions/XML/library hashes; no DDS participant."""
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    result = {'scope': 'INSTALLED_FILES_ONLY_NOT_PROCESS_MAPPINGS', 'packages': {}}
    for name, class_name in (('nav2_smac_planner', 'nav2_smac_planner/SmacPlannerHybrid'),
                             ('nav2_mppi_controller', 'nav2_mppi_controller::MPPIController'),
                             ('nav2_planner', None), ('nav2_controller', None)):
        entry = result['packages'][name] = {'libraries': []}
        try:
            share, prefix = Path(get_package_share_directory(name)), Path(get_package_prefix(name))
            package = share / 'package.xml'
            entry.update(version=ET.parse(package).getroot().findtext('version'),
                         package_xml_sha256=hashlib.sha256(package.read_bytes()).hexdigest())
            if class_name is None:
                continue
            for xml_path in sorted(share.glob('*.xml')):
                tree = ET.parse(xml_path).getroot()
                libraries = [tree] if tree.tag == 'library' else tree.findall('.//library')
                for library in libraries:
                    if not any(item.get('name') == class_name or item.get('type') == class_name
                               or item.get('type', '').replace('::', '/') == class_name
                               for item in library.findall('class')):
                        continue
                    lib_name = library.get('path', '')
                    filename = Path(lib_name).name
                    if not filename.startswith('lib'):
                        filename = 'lib' + filename
                    if not filename.endswith('.so'):
                        filename += '.so'
                    path = prefix / 'lib' / filename
                    entry['libraries'].append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                        'plugin_xml': str(xml_path), 'plugin_xml_sha256': hashlib.sha256(xml_path.read_bytes()).hexdigest(),
                        'class': class_name})
            if not entry['libraries']:
                entry['error'] = 'No matching installed plugin XML/library located.'
        except Exception as error:
            entry['error'] = f'{type(error).__name__}: {error}'
    return result


def parse_configuration_logs(text):
    """Extract only numeric values printed by internal configure code paths."""
    observations = []
    def add(line, node, values):
        observations.append({'evidence': 'CONFIGURE_LOG_OBSERVED', 'node': node, 'values': values,
                             'line': line, 'ACTUALLY_USED': 'NOT_VERIFIED',
                             'scope': 'Internal configuration log; no algorithm/mission execution proof.'})
    for number, line in enumerate(text.splitlines(), 1):
        if '[controller_server]:' in line:
            match = re.search(r'Controller frequency set to ([\d.eE+-]+)Hz', line)
            if match:
                add(number, '/controller_server', {'controller_frequency': float(match[1])})
            match = re.search(r'(PathAlignCritic|PathAngleCritic|PreferForwardCritic|GoalCritic) instantiated with (\d+) power and ([\d.eE+-]+) weight', line)
            if match:
                add(number, '/controller_server', {'FollowPath.' + match[1] + '.cost_power': int(match[2]),
                                                  'FollowPath.' + match[1] + '.cost_weight': float(match[3])})
            # Iron 1.2.10 PathAlignCritic's configure message says
            # "ReferenceTrajectoryCritic". Preserve that spelling as evidence.
            match = re.search(r'ReferenceTrajectoryCritic instantiated with (\d+) power and ([\d.eE+-]+) weight', line)
            if match:
                add(number, '/controller_server', {'FollowPath.PathAlignCritic.cost_power': int(match[1]),
                    'FollowPath.PathAlignCritic.cost_weight': float(match[2])})
                observations[-1]['logged_name'] = 'ReferenceTrajectoryCritic'
                observations[-1]['source_version'] = NAV2_VERSION
            match = re.search(r'ObstaclesCritic instantiated with (\d+) power and ([\d.eE+-]+) / ([\d.eE+-]+) weights', line)
            if match:
                add(number, '/controller_server', {'FollowPath.ObstaclesCritic.cost_power': int(match[1]),
                    'FollowPath.ObstaclesCritic.critical_weight': float(match[2]),
                    'FollowPath.ObstaclesCritic.repulsion_weight': float(match[3])})
        if '[planner_server]:' in line:
            match = re.search(r'Configured plugin GridBased of type SmacPlannerHybrid with maximum iterations (\d+), max on approach iterations (\d+), and (allowing|not allowing) unknown traversal\. Tolerance ([\d.eE+-]+)\.Using motion model: ([\w_]+)', line)
            if match:
                add(number, '/planner_server', {'GridBased.max_iterations': int(match[1]),
                    'GridBased.max_on_approach_iterations': int(match[2]), 'GridBased.allow_unknown': match[3] == 'allowing',
                    'GridBased.tolerance': float(match[4]), 'GridBased.motion_model_for_search': match[5].upper()})
    return observations
