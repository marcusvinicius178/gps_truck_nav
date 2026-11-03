"""Bounded read-only Get/Describe parameter batches, independent of CLI discovery.

Only GetParameters, DescribeParameters and ListParameters clients are created.
poll() never spins or waits; the owner's executor supplies progress. finish()
closes the observation window and preserves every attempt, including timeouts.
"""

import argparse
import json
from pathlib import Path
import time

from plugin_parameter_contract import build_contract, parse_configuration_logs


def parameter_value(message):
    fields = {0: None, 1: 'bool_value', 2: 'integer_value', 3: 'double_value',
              4: 'string_value', 5: 'byte_array_value', 6: 'bool_array_value',
              7: 'integer_array_value', 8: 'double_array_value', 9: 'string_array_value'}
    kind = int(message.type)
    if kind not in fields:
        raise ValueError('Unsupported ParameterType ' + str(kind))
    value = None if kind == 0 else getattr(message, fields[kind])
    if kind >= 5:
        value = list(value)
    return {'type': kind, 'value': value}


class PluginParameterProbe:
    def __init__(self, node, canonical, *, timeout=2.0, max_attempts=3, batch_size=16,
                 monotonic=time.monotonic, service_types=None):
        if not 0 < timeout <= 5 or not 1 <= max_attempts <= 3 or not 1 <= batch_size <= 16:
            raise ValueError('Probe bounds: timeout (0,5], attempts [1,3], batch size [1,16]')
        if service_types is None:
            from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters
            service_types = {'get': GetParameters, 'describe': DescribeParameters, 'list': ListParameters}
        self.node, self.clock = node, monotonic
        self.started = monotonic()
        self.timeout, self.max_attempts = timeout, max_attempts
        self.closed = False
        self.jobs = []
        self.report = {'schema_version': 1, 'contract': build_contract(canonical), 'nodes': {},
            'collection_status': 'PENDING', 'jobs': [],
            'policy': {'batch_size': batch_size, 'timeout_wall_seconds': timeout,
                       'maximum_attempts': max_attempts, 'maximum_inflight': 4,
                       'list_parameters_required_for_explicit_readback': False,
                       'optional_parameters_queried_as_singletons': True,
                       'required_batches_precede_optional_singletons': True,
                       'deadline_basis': 'MONOTONIC_FIRST_OBSERVED_COMPLETION',
                       'late_response_payload_used_as_effective_value': False},
            'mutation_performed': False}
        for target, parameters in self.report['contract']['nodes'].items():
            self.report['nodes'][target] = {'values': {}, 'descriptors': {}, 'listed_names': [],
                                           'listed_prefixes': []}
            keys = sorted(key for key, value in parameters.items() if value['required'])
            optional = sorted(key for key, value in parameters.items() if not value['required'])
            for method, service_type in service_types.items():
                suffix = {'get': 'get_parameters', 'describe': 'describe_parameters',
                          'list': 'list_parameters'}[method]
                client = node.create_client(service_type, target + '/' + suffix)
                # rclcpp 21.0.8 returns an empty vector for the whole request if
                # one name is undeclared and allow_undeclared_parameters=false.
                # Optional/inactive names must never poison required batches.
                batches = ([None] if method == 'list' else
                    [keys[i:i + batch_size] for i in range(0, len(keys), batch_size)] + [[key] for key in optional])
                for names in batches:
                    request = service_type.Request()
                    if method == 'list':
                        request.prefixes, request.depth = [], 0
                    else:
                        request.names = names
                    record = {'node': target, 'method': method, 'service': target + '/' + suffix,
                              'names': names, 'status': 'PENDING', 'attempts': [],
                              'required_for_collection': method != 'list',
                              'contract_required': bool(names and all(parameters[key]['required'] for key in names)),
                              'service_ready_observed': False, 'service_check_count': 0}
                    self.report['jobs'].append(record)
                    self.jobs.append({'client': client, 'request': request, 'record': record,
                                      'future': None, 'sent': None})

    def _forget(self, job):
        future = job['future']
        if future is not None:
            try:
                job['client'].remove_pending_request(future)
            except Exception as error:
                job['record'].setdefault('pending_request_cleanup_errors', []).append(str(error))
            if not future.done():
                future.cancel()
        job['future'] = None

    def _response_payload(self, method, response):
        if method == 'get':
            return {'values': [parameter_value(value) for value in response.values]}
        if method == 'describe':
            return {'descriptors': [{'name': value.name, 'type': int(value.type),
                'read_only': bool(value.read_only), 'dynamic_typing': bool(value.dynamic_typing),
                'description': value.description, 'additional_constraints': value.additional_constraints}
                for value in response.descriptors]}
        return {'listed_names': list(response.result.names), 'listed_prefixes': list(response.result.prefixes)}

    def _decode(self, record, response):
        target, method, names = record['node'], record['method'], record['names']
        destination = self.report['nodes'][target]
        payload = self._response_payload(method, response)
        if method == 'get':
            if len(payload['values']) != len(names):
                raise ValueError('GetParameters response count differs from explicit request')
            destination['values'].update(zip(names, payload['values']))
        elif method == 'describe':
            if len(payload['descriptors']) != len(names):
                raise ValueError('DescribeParameters response count differs from explicit request')
            values = {}
            for name, value in zip(names, payload['descriptors']):
                if value['name'] not in (name, '') or (value['type'] != 0 and value['name'] != name):
                    raise ValueError('DescribeParameters returned a different parameter name')
                values[name] = value
            destination['descriptors'].update(values)
        else:
            destination.update(payload)

    def poll(self, *, start_new=True):
        if self.closed:
            return
        now = self.clock()
        # Finish in-flight calls before allocating bounded replacement attempts.
        for job in self.jobs:
            future, record = job['future'], job['record']
            if future is None:
                continue
            attempt = record['attempts'][-1]
            if now - job['sent'] >= self.timeout:
                attempt.update(status='LATE_RESPONSE' if future.done() else 'TIMEOUT',
                               completed_wall_offset=now - self.started,
                               response_latency_wall_seconds=now - job['sent'])
                if future.done():
                    try:
                        attempt['response_payload'] = self._response_payload(record['method'], future.result())
                    except Exception as error:
                        attempt['late_response_error'] = f'{type(error).__name__}: {error}'
                self._forget(job)
            elif future.done():
                try:
                    self._decode(record, future.result())
                    attempt.update(status='OK', completed_wall_offset=now - self.started,
                                   response_latency_wall_seconds=now - job['sent'])
                    record['status'] = 'OK'
                except Exception as error:
                    attempt.update(status='ERROR', error=f'{type(error).__name__}: {error}',
                                   completed_wall_offset=now - self.started)
                self._forget(job)
            if job['future'] is None and record['status'] != 'OK' and len(record['attempts']) >= self.max_attempts:
                record['status'] = record['attempts'][-1]['status']

        if not start_new:
            return
        inflight = sum(job['future'] is not None for job in self.jobs)
        occupied = {(job['record']['node'], job['record']['method'])
                    for job in self.jobs if job['future'] is not None}
        # Explicit queries precede ListParameters; listing never gates Get/Describe.
        ordered = sorted(self.jobs, key=lambda job: (job['record']['method'] == 'list',
                                                    not job['record']['contract_required']))
        for job in ordered:
            record = job['record']
            channel = (record['node'], record['method'])
            if inflight >= 4:
                break
            if record['status'] != 'PENDING' or job['future'] is not None or channel in occupied:
                continue
            try:
                ready = job['client'].service_is_ready()
                record['service_check_count'] += 1
                record['last_service_check_wall_offset'] = now - self.started
                if ready and not record['service_ready_observed']:
                    record['first_service_ready_wall_offset'] = now - self.started
                    record['service_ready_observed'] = True
            except Exception as error:
                record['status'] = 'SERVICE_CHECK_ERROR'
                record['error'] = f'{type(error).__name__}: {error}'
                continue
            if not ready:
                continue
            attempt = {'number': len(record['attempts']) + 1, 'sent_wall_offset': now - self.started,
                       'status': 'PENDING'}
            record['attempts'].append(attempt)
            try:
                job['future'] = job['client'].call_async(job['request'])
                job['sent'] = now
                occupied.add(channel)
                inflight += 1
            except Exception as error:
                attempt.update(status='ERROR', error=f'{type(error).__name__}: {error}',
                               completed_wall_offset=now - self.started)
                if len(record['attempts']) >= self.max_attempts:
                    record['status'] = 'ERROR'

    def finish(self):
        if self.closed:
            return self.report
        self.poll(start_new=False)
        now = self.clock()
        for job in self.jobs:
            record = job['record']
            if record['status'] == 'PENDING':
                record['status'] = ('WINDOW_CLOSED' if record['attempts'] else
                    'SERVICE_UNAVAILABLE' if record['service_check_count'] and not record['service_ready_observed'] else
                    'NOT_ATTEMPTED_WINDOW_CLOSED')
                if job['future'] is not None:
                    record['attempts'][-1].update(status='WINDOW_CLOSED', completed_wall_offset=now - self.started)
                self._forget(job)
        self.closed = True
        required = [job for job in self.report['jobs'] if job['required_for_collection']]
        self.report['collection_status'] = 'COMPLETE' if all(job['status'] == 'OK' for job in required) else 'PARTIAL'
        mandatory = [job for job in self.report['jobs'] if job['contract_required']]
        self.report['required_collection_status'] = 'COMPLETE' if all(job['status'] == 'OK' for job in mandatory) else 'PARTIAL'
        self.report['observed_duration_wall_seconds'] = now - self.started
        return self.report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Read existing startup logs; no ROS, goals or parameter changes.')
    parser.add_argument('--configuration-log', required=True, type=Path)
    args = parser.parse_args(argv)
    text = args.configuration_log.read_text()
    print(json.dumps({'schema_version': 1, 'configuration_log': str(args.configuration_log),
                      'observations': parse_configuration_logs(text), 'ACTUALLY_USED': 'NOT_VERIFIED',
                      'scope': 'Internal configure logs only; no algorithm execution proof.'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
