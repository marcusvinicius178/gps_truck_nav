"""Read-only parameter evidence regressions, with no ROS/DDS or movement."""

import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools/auto_tuning'
SPEC = importlib.util.spec_from_file_location('plugin_parameter_contract', TOOLS / 'plugin_parameter_contract.py')
contract = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(contract)
PROBE_SPEC = importlib.util.spec_from_file_location('parameter_probe', TOOLS / 'parameter_probe.py')
probe = importlib.util.module_from_spec(PROBE_SPEC)
with mock.patch.dict(sys.modules, {'plugin_parameter_contract': contract}):
    PROBE_SPEC.loader.exec_module(probe)
CANONICAL = yaml.safe_load((ROOT / 'src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml').read_text())


def fixture():
    expected = contract.build_contract(CANONICAL)
    report = {'contract': expected, 'nodes': {}, 'provenance': {'packages': {}}}
    for node, parameters in expected['nodes'].items():
        report['nodes'][node] = {
            'values': {key: {'type': value['expected_type'], 'value': copy.deepcopy(value['expected'])}
                       for key, value in parameters.items()},
            'descriptors': {key: {'name': key, 'type': value['expected_type']}
                            for key, value in parameters.items()}}
    for name in ('nav2_planner', 'nav2_controller', 'nav2_smac_planner', 'nav2_mppi_controller'):
        report['provenance']['packages'][name] = {'version': '1.2.10', 'libraries': [{'sha256': 'a' * 64}]}
    report['provenance']['runtime_binary_identity'] = {'targets': {
        target: {'status': 'MAPPED_IDENTIFIED', 'records': [{'sha256': 'a' * 64}]}
        for target in ('smac_hybrid', 'mppi_controller')}}
    return report


def check(report, name):
    return next(item for item in contract.assess_plugin_parameters(report)['checks'] if item['name'] == name)


class ParameterContractTests(unittest.TestCase):
    def setUp(self):
        self.report = fixture()

    def entry(self, name):
        return next(item for item in contract.assess_plugin_parameters(self.report)['parameters'] if item['name'] == name)

    def test_all_configured_keys_are_captured_with_ros_types(self):
        for node in ('planner_server', 'controller_server'):
            actual = self.report['contract']['nodes']['/' + node]
            for key, value in contract.flatten(CANONICAL[node]['ros__parameters']).items():
                self.assertEqual(actual[key]['expected'], value)
                self.assertEqual(actual[key]['expected_type'], contract.ros_type(value))

    def test_smac_penalties_limits_and_mppi_nested_critics_have_read_sites(self):
        for key in ('GridBased.non_straight_penalty', 'GridBased.max_iterations', 'GridBased.minimum_turning_radius',
                    'FollowPath.time_steps', 'FollowPath.batch_size', 'FollowPath.model_dt',
                    'FollowPath.AckermannConstraints.min_turning_r', 'FollowPath.PathAlignCritic.cost_weight',
                    'FollowPath.ObstaclesCritic.repulsion_weight'):
            item = self.entry(key)
            self.assertTrue(item['required'])
            self.assertEqual(item['source_consumer'], 'SOURCE_CONSUMER_IDENTIFIED')
            self.assertEqual(item['SOURCE_AUDIT_VERSION_MATCHED'], 'YES')

    def test_full_effective_readback_is_never_algorithm_use_or_gate_approval(self):
        result = contract.assess_plugin_parameters(self.report)
        for node in ('/planner_server', '/controller_server'):
            self.assertEqual(check(self.report, 'effective_plugin_parameters:' + node)['status'], 'PASS')
        self.assertEqual(check(self.report, 'plugin_parameters_actually_used')['status'], 'BLOCKED_REVIEW')
        self.assertTrue(all(item['ACTUALLY_USED'] == 'NOT_VERIFIED' for item in result['parameters']))
        self.assertFalse(result['mission_ready'])
        self.assertEqual(result['gate0'], 'NOT_YET_VERIFIED')
        self.assertEqual(result['gate1'], 'NOT_YET_VERIFIED')

    def test_unconsumed_yaml_can_be_declared_and_match_without_consumption_pass(self):
        for key in ('planner_patience', 'GridBased.transform_tolerance', 'GridBased.costmap.width',
                    'GridBased.analytic_expansion_max_cost', 'GridBased.allow_primitive_interpolation',
                    'FollowPath.wz_min', 'FollowPath.reset_period', 'FollowPath.trajectory_step',
                    'FollowPath.ObstaclesCritic.inflation_radius'):
            item = self.entry(key)
            self.assertEqual(item['DECLARED'], 'YES')
            self.assertEqual(item['EFFECTIVE'], 'MATCH')
            self.assertFalse(item['required'])
            self.assertEqual(item['source_consumer'], 'NO_READ_SITE_IDENTIFIED')
        self.assertEqual(check(self.report, 'unconsumed_yaml_parameters:/planner_server')['status'], 'BLOCKED_REVIEW')

    def test_actual_visualizer_namespace_and_smoother_defaults_are_queried_without_mutation(self):
        for key, expected in (('FollowPath.TrajectoryVisualizer.trajectory_step', 5),
                              ('FollowPath.TrajectoryVisualizer.time_step', 3),
                              ('FollowPath.PathAlignCritic.trajectory_point_step', 4),
                              ('GridBased.smoother.refinement_num', 2),
                              ('GridBased.smoother.tolerance', 1e-10)):
            item = self.entry(key)
            self.assertEqual(item['expected'], expected)
            self.assertEqual(item['origin'], 'UPSTREAM_1_2_10_DEFAULT')
        self.assertNotIn('TrajectoryVisualizer', CANONICAL['controller_server']['ros__parameters']['FollowPath'])

    def test_inversion_tolerances_are_conditional_and_inactive_goal_checker_not_claimed(self):
        for key in ('FollowPath.inversion_xy_tolerance', 'FollowPath.inversion_yaw_tolerance'):
            self.assertEqual(self.entry(key)['source_consumer'], 'CONDITION_DISABLED')
            self.assertFalse(self.entry(key)['required'])
        self.assertEqual(self.entry('general_goal_checker.xy_goal_tolerance')['source_consumer'], 'INACTIVE_PLUGIN_CONFIGURATION')
        changed = copy.deepcopy(CANONICAL)
        changed['controller_server']['ros__parameters']['FollowPath']['enforce_path_inversion'] = True
        entry = contract.build_contract(changed)['nodes']['/controller_server']['FollowPath.inversion_xy_tolerance']
        self.assertTrue(entry['required'])

    def test_empty_dump_and_clock_only_readback_do_not_pass(self):
        for node in ('/planner_server', '/controller_server'):
            self.report['nodes'][node]['values'] = {'use_sim_time': {'type': 1, 'value': True}}
            self.assertEqual(check(self.report, 'effective_plugin_parameters:' + node)['status'], 'UNKNOWN')
        self.report['nodes'] = {}
        self.assertEqual(check(self.report, 'effective_plugin_parameters:/controller_server')['status'], 'UNKNOWN')

    def test_wrong_namespace_cannot_satisfy_planner_and_candidate_change_is_detected(self):
        values = self.report['nodes']['/planner_server']['values']
        values['GridBased.non_straight_penalty']['value'] = 2.6
        mismatch = check(self.report, 'effective_plugin_parameters:/planner_server')
        self.assertEqual(mismatch['status'], 'FAIL')
        self.assertIn('GridBased.non_straight_penalty', mismatch['differences'])
        moved = values.pop('GridBased.non_straight_penalty')
        self.report['nodes']['/controller_server']['values']['GridBased.non_straight_penalty'] = moved
        self.assertEqual(check(self.report, 'effective_plugin_parameters:/planner_server')['status'], 'UNKNOWN')

    def test_missing_descriptor_wrong_ros_type_and_not_set_are_not_passes(self):
        target = self.report['nodes']['/planner_server']
        target['descriptors'].pop('GridBased.non_straight_penalty')
        self.assertEqual(check(self.report, 'effective_plugin_parameters:/planner_server')['status'], 'UNKNOWN')
        self.report = fixture()
        target = self.report['nodes']['/planner_server']
        target['values']['GridBased.max_iterations']['type'] = 3
        self.assertEqual(check(self.report, 'effective_plugin_parameters:/planner_server')['status'], 'FAIL')
        target['descriptors']['GridBased.max_iterations']['type'] = 0
        target['values']['GridBased.max_iterations'] = {'type': 0, 'value': None}
        self.assertEqual(self.entry('GridBased.max_iterations')['DECLARED'], 'NOT_SET')
        self.assertEqual(check(self.report, 'effective_plugin_parameters:/planner_server')['status'], 'FAIL')

    def test_types_finiteness_and_arrays_are_strict(self):
        for left, right in ((True, 1), (1, 1.0), ([1.0], [1]), (1.0, float('nan')), (1.0, float('inf'))):
            self.assertFalse(contract.same_value(left, right))
        self.assertTrue(contract.same_value(1.0, 1.0 + 1e-12))
        for value in ([], [True, 1], {'x': 1}, None):
            with self.assertRaises(ValueError):
                contract.ros_type(value)

    def test_missing_or_different_installed_version_suspends_source_consumer_claim(self):
        self.report['provenance']['packages']['nav2_mppi_controller']['version'] = '1.3.0'
        item = self.entry('FollowPath.batch_size')
        self.assertEqual(item['source_consumer'], 'UNKNOWN_INSTALLED_VERSION')
        self.assertEqual(item['SOURCE_AUDIT_VERSION_MATCHED'], 'UNKNOWN')
        self.assertEqual(check(self.report, 'plugin_version_and_binary_identity')['status'], 'UNKNOWN')
        self.assertEqual(item['EFFECTIVE'], 'MATCH')
        self.assertEqual(self.entry('GridBased.non_straight_penalty')['source_consumer'], 'SOURCE_CONSUMER_IDENTIFIED')

    def test_wrong_plugin_identity_never_borrows_mppi_source_consumption(self):
        self.report['nodes']['/controller_server']['values']['FollowPath.plugin']['value'] = 'some/OtherController'
        self.assertEqual(self.entry('FollowPath.batch_size')['source_consumer'], 'UNKNOWN_PLUGIN_IDENTITY')
        changed = copy.deepcopy(CANONICAL)
        changed['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'some/OtherController'
        with self.assertRaises(ValueError):
            contract.build_contract(changed)

    def test_missing_mapped_identity_suspends_source_claim_without_losing_readback(self):
        self.report['provenance'].pop('runtime_binary_identity')
        item = self.entry('FollowPath.batch_size')
        self.assertEqual(item['source_consumer'], 'UNKNOWN_BINARY_ASSOCIATION')
        self.assertEqual(item['INSTALLED_MAPPED_ASSOCIATION'], 'UNKNOWN')
        self.assertEqual(item['EFFECTIVE'], 'MATCH')
        self.assertEqual(check(self.report, 'plugin_binary_association:/controller_server')['status'], 'UNKNOWN')

    def test_different_mapped_overlay_hash_fails_association_and_suspends_consumer(self):
        targets = self.report['provenance']['runtime_binary_identity']['targets']
        targets['mppi_controller']['records'][0]['sha256'] = 'b' * 64
        self.assertEqual(check(self.report, 'plugin_binary_association:/controller_server')['status'], 'FAIL')
        self.assertEqual(self.entry('FollowPath.batch_size')['source_consumer'], 'BINARY_IDENTITY_MISMATCH')
        self.assertEqual(self.entry('GridBased.non_straight_penalty')['source_consumer'], 'SOURCE_CONSUMER_IDENTIFIED')
        # Matching one of two mapped variants is insufficient.
        targets['mppi_controller']['records'].append({'sha256': 'a' * 64})
        self.assertEqual(check(self.report, 'plugin_binary_association:/controller_server')['status'], 'FAIL')

    def test_unverified_mapping_or_missing_hash_is_unknown_not_identity_pass(self):
        target = self.report['provenance']['runtime_binary_identity']['targets']['smac_hybrid']
        target['status'] = 'MAPPED_UNVERIFIED'
        self.assertEqual(check(self.report, 'plugin_binary_association:/planner_server')['status'], 'UNKNOWN')
        target['status'] = 'MAPPED_IDENTIFIED'
        target['records'][0]['sha256'] = None
        self.assertEqual(check(self.report, 'plugin_binary_association:/planner_server')['status'], 'UNKNOWN')

    def test_observed_undeclared_is_distinct_from_version_support_and_rpc_failure(self):
        node = self.report['nodes']['/controller_server']
        key = 'FollowPath.wz_min'
        node['values'].pop(key)
        node['descriptors'].pop(key)
        node['declarations'] = {key: {'status': 'UNDECLARED',
            'evidence': 'LIST_ABSENT_AND_TWO_SINGLETON_RESPONSES'}}
        item = self.entry(key)
        self.assertEqual(item['DECLARED'], 'NO')
        self.assertEqual(item['EFFECTIVE'], 'NOT_APPLICABLE_UNDECLARED')
        self.assertEqual(item['AVAILABILITY'], 'UNSUPPORTED_VERSION')
        self.assertEqual(item['source_consumer'], 'NO_READ_SITE_IDENTIFIED')
        self.assertFalse(item['tuning_eligible'])
        self.assertEqual(check(self.report, 'unconsumed_yaml_parameters:/controller_server')['status'], 'BLOCKED_REVIEW')
        self.report['provenance']['packages']['nav2_mppi_controller']['version'] = 'unknown'
        self.assertEqual(self.entry(key)['AVAILABILITY'], 'UNDECLARED')
        node['declarations'] = {}
        self.assertEqual(self.entry(key)['DECLARED'], 'UNKNOWN')
        self.assertEqual(check(self.report, 'optional_parameter_declarations:/controller_server')['status'], 'UNKNOWN')

    def test_inactive_and_conditional_absence_are_not_unsupported_version(self):
        node = self.report['nodes']['/controller_server']
        for key in ('FollowPath.inversion_xy_tolerance', 'general_goal_checker.plugin'):
            node['values'].pop(key)
            node['descriptors'].pop(key)
            node.setdefault('declarations', {})[key] = {'status': 'UNDECLARED'}
            self.assertEqual(self.entry(key)['AVAILABILITY'], 'UNDECLARED')
            self.assertNotEqual(self.entry(key)['source_consumer'], 'NO_READ_SITE_IDENTIFIED')

    def test_configure_logs_record_only_printed_subset_and_iron_pathalign_spelling(self):
        text = '\n'.join([
            '[controller_server]: Controller frequency set to 20.0000Hz',
            '[controller_server]: ReferenceTrajectoryCritic instantiated with 1 power and 30.000000 weight',
            '[controller_server]: PathAngleCritic instantiated with 1 power and 15.000000 weight. Mode set to: Consider Feasible Path Orientations',
            '[controller_server]: PreferForwardCritic instantiated with 2 power and 20.000000 weight.',
            '[controller_server]: GoalCritic instantiated with 2 power and 10.000000 weight.',
            '[controller_server]: ObstaclesCritic instantiated with 2 power and 15.000000 / 1.000000 weights.',
            '[planner_server]: Configured plugin GridBased of type SmacPlannerHybrid with maximum iterations 2000000, max on approach iterations 2000, and allowing unknown traversal. Tolerance 1.00.Using motion model: Dubin.',
            '[unrelated]: Controller frequency set to 999.0Hz',
            '[controller_server]: FollowPath.batch_size is not printed by the configure log'])
        result = contract.parse_configuration_logs(text)
        self.assertEqual(len(result), 7)
        values = {key: value for item in result for key, value in item['values'].items()}
        self.assertEqual(values['FollowPath.PathAlignCritic.cost_weight'], 30.0)
        self.assertEqual(values['GridBased.motion_model_for_search'], 'DUBIN')
        self.assertNotIn('FollowPath.batch_size', values)
        self.assertNotIn('GridBased.non_straight_penalty', values)
        self.assertTrue(all(item['evidence'] == 'CONFIGURE_LOG_OBSERVED' and item['ACTUALLY_USED'] == 'NOT_VERIFIED' for item in result))

    def test_installed_xml_versions_and_library_hashes_are_evidence_not_process_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            shares = {}
            (prefix / 'lib').mkdir()
            for name, class_name in (('nav2_smac_planner', 'nav2_smac_planner::SmacPlannerHybrid'),
                                     ('nav2_mppi_controller', 'nav2_mppi_controller::MPPIController'),
                                     ('nav2_planner', None), ('nav2_controller', None)):
                share = prefix / 'share' / name
                share.mkdir(parents=True)
                shares[name] = str(share)
                (share / 'package.xml').write_text('<package><version>1.2.10</version></package>')
                if class_name:
                    lib_path = name if name == 'nav2_smac_planner' else 'lib' + name + '.so'
                    (share / 'plugins.xml').write_text(f'<library path="{lib_path}"><class type="{class_name}"/></library>')
                    (prefix / 'lib' / ('lib' + name + '.so')).write_bytes(b'fixture ELF identity')
            fake = SimpleNamespace(get_package_prefix=lambda name: str(prefix), get_package_share_directory=lambda name: shares[name])
            with mock.patch.dict(sys.modules, {'ament_index_python.packages': fake}):
                result = contract.installed_plugin_provenance()
            self.assertEqual(result['scope'], 'INSTALLED_FILES_ONLY_NOT_PROCESS_MAPPINGS')
            self.assertEqual(result['packages']['nav2_planner']['version'], '1.2.10')
            self.assertEqual(len(result['packages']['nav2_smac_planner']['libraries'][0]['sha256']), 64)


class Future:
    def __init__(self, result=None, pending=False):
        self.response, self.pending, self.cancelled = result, pending, False

    def done(self):
        return not self.pending

    def result(self):
        if isinstance(self.response, Exception):
            raise self.response
        return self.response

    def cancel(self):
        self.cancelled = True
        self.pending = False


class FakeNode:
    def __init__(self, policy=None, ready=True):
        self.expected = contract.build_contract(CANONICAL)['nodes']
        self.policy, self.ready = policy, ready
        self.clients, self.calls, self.futures, self.counts = {}, [], [], {}
        self.undeclared = {target: set() for target in self.expected}

    def create_client(self, kind, service):
        owner = self
        class Client:
            def service_is_ready(self):
                return owner.ready

            def remove_pending_request(self, future):
                pass

            def call_async(self, request):
                target, method = service.rsplit('/', 1)
                names = getattr(request, 'names', None)
                key = (service, tuple(names or []))
                owner.counts[key] = owner.counts.get(key, 0) + 1
                behavior = owner.policy(service, names, owner.counts[key]) if owner.policy else None
                if behavior is None and names and any(name in owner.undeclared[target] for name in names):
                    behavior = 'empty'
                owner.calls.append((service, names))
                if behavior == 'raise':
                    raise RuntimeError('RPC start failed')
                if method == 'get_parameters':
                    values = []
                    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                              6: 'bool_array_value', 7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}
                    for name in names:
                        item = owner.expected[target][name]
                        values.append(SimpleNamespace(type=0) if behavior == 'unset' else
                            SimpleNamespace(type=item['expected_type'], **{fields[item['expected_type']]: item['expected']}))
                    result = SimpleNamespace(values=[] if behavior == 'empty' else values)
                elif method == 'describe_parameters':
                    descriptors = [SimpleNamespace(name='wrong_name' if behavior == 'wrong_descriptor' else name,
                        type=0 if behavior == 'unset' else owner.expected[target][name]['expected_type'],
                        read_only=False, dynamic_typing=False, description='', additional_constraints='') for name in names]
                    result = SimpleNamespace(descriptors=[] if behavior == 'empty' else descriptors)
                else:
                    listed = [name for name in owner.expected[target] if name not in owner.undeclared[target]]
                    if behavior == 'empty_list':
                        listed = []
                    elif behavior == 'duplicate_list':
                        listed += listed[:1]
                    result = SimpleNamespace(result=SimpleNamespace(names=listed, prefixes=[]))
                if behavior == 'error':
                    result = RuntimeError('RPC result failed')
                future = Future(result, pending=behavior == 'timeout')
                owner.futures.append(future)
                return future
        client = Client()
        self.clients[service] = client
        return client


class ParameterProbeTests(unittest.TestCase):
    def make(self, policy=None, ready=True, **options):
        self.now = 0.0
        self.node = FakeNode(policy, ready)
        service = SimpleNamespace(Request=SimpleNamespace)
        return probe.PluginParameterProbe(self.node, CANONICAL, monotonic=lambda: self.now,
            service_types={key: service for key in ('get', 'describe', 'list')}, **options)

    def run_window(self, collector, iterations=100, step=0.1):
        for _ in range(iterations):
            collector.poll()
            self.assertLessEqual(sum(job['future'] is not None for job in collector.jobs), 4)
            self.now += step
        return collector.finish()

    def test_explicit_small_get_describe_batches_complete_independently_of_list(self):
        collector = self.make(lambda service, names, number: 'timeout' if service.endswith('/list_parameters') else None)
        result = self.run_window(collector)
        self.assertEqual(result['collection_status'], 'COMPLETE')
        self.assertFalse(result['mutation_performed'])
        self.assertTrue(any(job['status'] == 'TIMEOUT' for job in result['jobs'] if job['method'] == 'list'))
        for service, names in self.node.calls:
            self.assertIn(service.rsplit('/', 1)[1], ('get_parameters', 'describe_parameters', 'list_parameters'))
            if names is not None:
                self.assertLessEqual(len(names), 16)
        self.assertEqual(result['nodes']['/controller_server']['values']['FollowPath.batch_size'], {'type': 2, 'value': 800})
        self.assertEqual(check(result, 'effective_plugin_parameters:/controller_server')['status'], 'PASS')
        json.dumps(result, allow_nan=False)

    def test_timeout_then_success_preserves_attempt_history(self):
        collector = self.make(lambda service, names, number: 'timeout' if number == 1 and service.endswith('/get_parameters') else None)
        result = self.run_window(collector, iterations=500)
        self.assertEqual(result['collection_status'], 'COMPLETE')
        for job in result['jobs']:
            if job['method'] == 'get':
                self.assertEqual([attempt['status'] for attempt in job['attempts']], ['TIMEOUT', 'OK'])
                self.assertGreaterEqual(job['attempts'][0]['response_latency_wall_seconds'], 2.0)

    def test_late_completed_response_preserves_payload_without_validating_effective_value(self):
        collector = self.make(max_attempts=1)
        collector.poll()
        sent = [job for job in collector.jobs if job['future'] is not None]
        self.now = 7.0  # Executor stalled across the two-second monotonic deadline.
        collector.poll()
        for job in sent:
            record = job['record']
            self.assertEqual(record['status'], 'LATE_RESPONSE')
            attempt = record['attempts'][0]
            self.assertEqual(attempt['response_latency_wall_seconds'], 7.0)
            self.assertIn('response_payload', attempt)
            category = 'values' if record['method'] == 'get' else 'descriptors'
            for name in record['names']:
                self.assertNotIn(name, collector.report['nodes'][record['node']][category])
        result = collector.finish()
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertEqual(check(result, 'effective_plugin_parameters:/planner_server')['status'], 'UNKNOWN')

    def test_completion_at_deadline_and_late_errors_do_not_become_ok(self):
        for behavior in ('error', 'empty'):
            collector = self.make(lambda service, names, number: behavior, max_attempts=1)
            collector.poll()
            self.now = 2.0
            result = collector.finish()
            attempts = [job['attempts'][0] for job in result['jobs'] if job['attempts']]
            self.assertTrue(all(item['status'] == 'LATE_RESPONSE' for item in attempts))
            self.assertTrue(all('late_response_error' in item if behavior == 'error' else 'response_payload' in item for item in attempts))
            self.assertEqual(result['required_collection_status'], 'PARTIAL')

    def test_iron_undeclared_optional_empty_response_never_poisons_required_batch(self):
        expected = contract.build_contract(CANONICAL)['nodes']
        def policy(service, names, number):
            target = service.rsplit('/', 1)[0]
            return 'empty' if names and any(not expected[target][name]['required'] for name in names) else None
        collector = self.make(policy)
        result = self.run_window(collector, iterations=200)
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertEqual(result['required_collection_status'], 'COMPLETE')
        for job in result['jobs']:
            if job['method'] == 'list':
                continue
            if job['contract_required']:
                self.assertEqual(job['status'], 'OK')
            else:
                self.assertEqual(len(job['names']), 1)
                self.assertEqual(job['status'], 'ERROR')
        self.assertEqual(check(result, 'effective_plugin_parameters:/planner_server')['status'], 'PASS')
        self.assertEqual(check(result, 'effective_plugin_parameters:/controller_server')['status'], 'PASS')
        # Empty vector cannot distinguish undeclared from uninitialized or a
        # server bug; never synthesize NOT_SET or an undeclared descriptor.
        self.assertNotIn('planner_patience', result['nodes']['/planner_server']['values'])

    def test_complete_not_set_responses_fail_required_contract_without_faking_timeout(self):
        collector = self.make(lambda service, names, number: 'unset')
        result = self.run_window(collector)
        self.assertEqual(result['collection_status'], 'COMPLETE')
        self.assertEqual(check(result, 'effective_plugin_parameters:/planner_server')['status'], 'FAIL')

    def test_corroborated_optional_absence_is_resolved_once_without_fabricated_values(self):
        collector = self.make()
        absent = {'/planner_server': {'planner_patience', 'GridBased.allow_primitive_interpolation'},
                  '/controller_server': {'FollowPath.wz_min', 'general_goal_checker.plugin',
                                         'FollowPath.inversion_xy_tolerance'}}
        self.node.undeclared = absent
        result = self.run_window(collector)
        self.assertEqual(result['collection_status'], 'COMPLETE')
        self.assertEqual(result['required_collection_status'], 'COMPLETE')
        self.assertEqual(result['readback_collection_status'], 'PARTIAL')
        self.assertEqual(len(result['known_undeclared_parameters']), 5)
        for job in result['jobs']:
            if job['names'] and job['names'][0] in absent[job['node']]:
                self.assertEqual(job['status'], 'UNDECLARED')
                self.assertEqual(len(job['attempts']), 1)
                self.assertEqual(job['attempts'][0]['status'], 'EMPTY_RESPONSE')
                self.assertEqual(job['attempts'][0]['response_payload'],
                                 {'values': []} if job['method'] == 'get' else {'descriptors': []})
        for target, keys in absent.items():
            self.assertEqual(result['nodes'][target]['list_cross_check']['status'], 'CONSISTENT')
            for key in keys:
                self.assertNotIn(key, result['nodes'][target]['values'])
                self.assertNotIn(key, result['nodes'][target]['descriptors'])
                self.assertEqual(result['nodes'][target]['declarations'][key]['status'], 'UNDECLARED')
            self.assertEqual(check(result, 'optional_parameter_declarations:' + target)['status'], 'PASS')
            self.assertEqual(check(result, 'effective_plugin_parameters:' + target)['status'], 'PASS')

    def test_absence_waits_for_bounded_list_retry_without_repeating_singletons(self):
        collector = self.make(lambda service, names, number:
            'timeout' if service.endswith('/list_parameters') and number == 1 else None)
        self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
        result = self.run_window(collector, iterations=200)
        selected = [job for job in result['jobs'] if job['names'] == ['FollowPath.wz_min']]
        self.assertTrue(all(job['status'] == 'UNDECLARED' and len(job['attempts']) == 1 for job in selected))
        self.assertEqual(result['collection_status'], 'COMPLETE')

    def test_empty_or_malformed_list_cannot_establish_absence(self):
        for list_behavior in ('empty_list', 'duplicate_list', 'timeout'):
            with self.subTest(list_behavior=list_behavior):
                collector = self.make(lambda service, names, number:
                    list_behavior if service.endswith('/list_parameters') else None)
                self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
                result = self.run_window(collector, iterations=200)
                self.assertEqual(result['collection_status'], 'PARTIAL')
                self.assertEqual(result['known_undeclared_parameters'], [])
                self.assertEqual(result['required_collection_status'], 'COMPLETE')

    def test_absent_list_with_positive_descriptor_is_inconsistent_not_undeclared(self):
        collector = self.make(lambda service, names, number:
            'present' if names == ['FollowPath.wz_min'] and service.endswith('/describe_parameters') else None)
        self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
        result = self.run_window(collector)
        declaration = result['nodes']['/controller_server']['declarations']['FollowPath.wz_min']
        self.assertEqual(declaration['status'], 'INCONSISTENT_OBSERVATIONS')
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertEqual(check(result, 'optional_parameter_declarations:/controller_server')['status'], 'UNKNOWN')

    def test_failed_rpc_is_not_relabelled_undeclared_by_list_alone(self):
        collector = self.make(lambda service, names, number:
            'error' if names == ['FollowPath.wz_min'] and service.endswith('/describe_parameters') else None)
        self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
        result = self.run_window(collector)
        selected = [job for job in result['jobs'] if job['names'] == ['FollowPath.wz_min']]
        self.assertTrue(all(job['status'] == 'ERROR' for job in selected))
        self.assertTrue(all(len(job['attempts']) == 3 for job in selected))
        self.assertEqual(result['known_undeclared_parameters'], [])

    def test_explicit_not_set_with_consistent_list_is_preserved_as_returned(self):
        collector = self.make(lambda service, names, number:
            'unset' if names == ['FollowPath.wz_min'] else None)
        self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
        result = self.run_window(collector)
        node = result['nodes']['/controller_server']
        self.assertEqual(node['declarations']['FollowPath.wz_min']['status'], 'UNDECLARED')
        self.assertEqual(node['values']['FollowPath.wz_min'], {'type': 0, 'value': None})
        self.assertEqual(result['collection_status'], 'COMPLETE')

    def test_window_close_preserves_unresolved_empty_payload_and_does_not_approve(self):
        collector = self.make(lambda service, names, number:
            'timeout' if service.endswith('/list_parameters') else None)
        self.node.undeclared['/controller_server'] = {'FollowPath.wz_min'}
        result = self.run_window(collector, iterations=100, step=0.001)
        selected = [job for job in result['jobs'] if job['names'] == ['FollowPath.wz_min']]
        self.assertTrue(all(job['status'] == 'UNRESOLVED_EMPTY_RESPONSE' for job in selected))
        self.assertTrue(all('response_payload' in job['attempts'][-1] for job in selected))
        self.assertEqual(result['collection_status'], 'PARTIAL')

    def test_wrong_descriptor_name_is_preserved_as_protocol_error(self):
        collector = self.make(lambda service, names, number: 'wrong_descriptor')
        result = self.run_window(collector)
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertEqual(result['nodes']['/controller_server']['descriptors'], {})
        self.assertTrue(any('different parameter name' in attempt.get('error', '')
                            for job in result['jobs'] for attempt in job['attempts']))

    def test_retries_exhaust_at_three_and_never_report_complete(self):
        collector = self.make(lambda service, names, number: 'timeout')
        result = self.run_window(collector, iterations=500)
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertTrue(all(len(job['attempts']) <= 3 for job in result['jobs']))
        self.assertTrue(any(job['status'] == 'TIMEOUT' and len(job['attempts']) == 3 for job in result['jobs']))
        self.assertEqual(check(result, 'effective_plugin_parameters:/planner_server')['status'], 'UNKNOWN')

    def test_empty_response_is_retryable_protocol_error_not_an_empty_pass(self):
        collector = self.make(lambda service, names, number: 'empty' if service.endswith('/get_parameters') else None)
        result = self.run_window(collector)
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertTrue(all(job['status'] == 'ERROR' and len(job['attempts']) == 3 for job in result['jobs'] if job['method'] == 'get'))
        self.assertEqual(result['nodes']['/controller_server']['values'], {})

    def test_service_unavailable_does_not_block_or_create_requests(self):
        collector = self.make(ready=False)
        result = self.run_window(collector)
        self.assertEqual(self.node.calls, [])
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertTrue(all(job['status'] == 'SERVICE_UNAVAILABLE' for job in result['jobs']))
        self.assertTrue(all(job['service_check_count'] > 0 for job in result['jobs']))

    def test_window_close_does_not_launch_new_attempt_and_is_idempotent(self):
        collector = self.make(lambda service, names, number: 'timeout')
        collector.poll()
        count = len(self.node.calls)
        result = collector.finish()
        self.assertEqual(len(self.node.calls), count)
        self.assertTrue(all(future.cancelled for future in self.node.futures))
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertIs(collector.finish(), result)
        collector.poll()
        self.assertEqual(len(self.node.calls), count)

    def test_remote_exception_and_call_exception_are_bounded(self):
        for behavior in ('error', 'raise'):
            with self.subTest(behavior=behavior):
                collector = self.make(lambda service, names, number: behavior)
                result = self.run_window(collector)
                self.assertEqual(result['collection_status'], 'PARTIAL')
                self.assertTrue(all(len(job['attempts']) == 3 and job['status'] == 'ERROR' for job in result['jobs']))

    def test_decode_not_set_and_array_types(self):
        self.assertEqual(probe.parameter_value(SimpleNamespace(type=0)), {'type': 0, 'value': None})
        self.assertEqual(probe.parameter_value(SimpleNamespace(type=9, string_array_value=('a', 'b'))), {'type': 9, 'value': ['a', 'b']})
        with self.assertRaises(ValueError):
            probe.parameter_value(SimpleNamespace(type=99))

    def test_safety_bounds_reject_unbounded_batches_retries_and_timeouts(self):
        for arguments in ({'batch_size': 17}, {'max_attempts': 4}, {'timeout': 10}, {'timeout': 0}):
            with self.assertRaises(ValueError):
                probe.PluginParameterProbe(None, CANONICAL, **arguments)


if __name__ == '__main__':
    unittest.main()
