#!/usr/bin/env python3
"""Host commissioning supervisor. Default preparation has no actuator publisher.

No host ROS installation is used. A new isolated container owns all simulation
processes. Its guard watches this host's lease; this process independently
watches the guard and can kill only the recorded, identity-checked container.
"""

import argparse
from http.server import HTTPServer
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

# Explicit reviewed sibling modules; host invocation may use python3 -I.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import commissioning_supervision as safety
from commissioning_source import protocol
import local_phase_a as phase
import visual_observer as visual


MODEL = 'config/auto_tuning/plant_candidate/models/arocs_truck_candidate/model.sdf'


def protocol_record():
    return {'schema_version': 1, 'trials': {name: protocol(name) for name in ('straight', 'left', 'right')},
            'host_lease_timeout_s': safety.HOST_LEASE_TIMEOUT,
            'process_heartbeat_timeout_s': safety.PROCESS_HEARTBEAT_TIMEOUT,
            'execution_wall_limit_s': safety.EXECUTION_WALL_LIMIT,
            'scope': 'SIMULATION_COMMISSIONING_ONLY_NO_NAV2_GOALS'}


def inspect_bounded(runner, docker, cid, name):
    result = json.loads(runner.run(name, docker + ['inspect', cid], timeout=5))
    if not isinstance(result, list) or len(result) != 1 or not isinstance(result[0], dict):
        raise phase.PhaseError('Invalid Docker inspection')
    return result[0]


def stop_owned(session, state=None, *, runner=None, docker=None):
    """Immediate simulator containment; no graceful stale-command interval."""
    state = state or json.loads((session / 'manifest.json').read_text())
    # Revoke the session independently before any Docker RPC can block. The
    # container guard checks this request; its run ID prevents cross-session use.
    safety.atomic_private(session / 'cancel.request.json', {
        'run_id': state['run_id'], 'monotonic_s': time.monotonic(), 'reason': 'HOST_CONTAINMENT_REQUEST'})
    if runner is None:
        operations = session / 'operations'
        operations.mkdir(mode=0o700, exist_ok=True)
        runner = phase.Runner(Path(tempfile.mkdtemp(prefix='stop.', dir=operations)))
    docker = docker or phase.docker_prefix()
    cid = state.get('container_id') or phase.owned_id(session / 'container.id')
    if cid is None:
        return {'status': 'NO_CONTAINER_CREATED'}
    state = {**state, 'container_id': cid}
    info = inspect_bounded(runner, docker, cid, 'stop_identity')
    safety.verify_container(info, state)
    kill_error = None
    if info.get('State', {}).get('Running') is True:
        try:
            runner.run('kill_owned_simulator', docker + ['kill', '--signal', 'KILL', cid], timeout=5)
        except (OSError, phase.PhaseError, subprocess.SubprocessError) as error:
            kill_error = str(error)
    after = inspect_bounded(runner, docker, cid, 'stopped_identity')
    safety.verify_container(after, state)
    if after.get('State', {}).get('Running') is not False or after.get('State', {}).get('Pid') != 0:
        raise phase.PhaseError('INDEPENDENT_STOP_NOT_CONFIRMED: owned simulator still running')
    result = {'status': 'OWNED_SIMULATOR_TERMINATED', 'container_id': cid, 'running': False,
              'pid': 0, 'kill_error': kill_error, 'physical_braking_verified': False}
    safety.atomic_private(session / 'containment.json', result)
    return result


def verify_preparation(record, source, image):
    if (record.get('mode') != 'prepare' or record.get('read_only_ready_observed') is not True
            or record.get('status') != 'PREPARATION_COLLECTED' or record.get('error') is not None
            or record.get('source') != source or record.get('image_id') != image
            or record.get('actuator_publishers_created') != 0
            or record.get('containment', {}).get('status') != 'OWNED_SIMULATOR_TERMINATED'):
        raise phase.PhaseError('Preparation must demonstrate read-only readiness and owned simulator termination')


def run(args):
    repo, output = args.repo.resolve(), args.output_dir.absolute()
    if (output.exists() or output.is_symlink() or output.resolve().is_relative_to(repo)
            or any(c in str(output) for c in (',', '\r', '\n'))):
        raise phase.PhaseError('Use a new output directory outside the checkout')
    output.mkdir(mode=0o700)
    runner, docker = phase.Runner(output), phase.docker_prefix()
    state = {'schema_version': 1, 'mode': args.operation, 'run_id': uuid.uuid4().hex,
             'navigation_profile': 'candidate20kmh', 'trial': 'straight',
             'status': 'STARTING', 'fault_drill': getattr(args, 'fault_drill', 'none'),
             'duration_wall_s': args.duration, 'mission_ready': False,
             'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    server = thread = None
    operator_started, prompt_started = threading.Event(), False
    reason = 'DURATION_LIMIT_REACHED'
    ready_seen, drill_triggered, drill_time, publisher_count = False, False, None, None
    failure = None
    started = time.monotonic()
    try:
        source = phase.source_snapshot(repo, runner)
        build = json.loads((args.build_dir / 'status.json').read_text())
        probe = json.loads((args.probe_dir / 'status.json').read_text())
        image = build.get('image_id')
        if (build.get('status') != 'IMAGE_VALIDATED' or build.get('stage') != 'build'
                or probe.get('status') != 'OBSERVATIONS_COLLECTED' or probe.get('stage') != 'probe'
                or probe.get('navigation_profile') != 'candidate20kmh'
                or source != build.get('source') or source != probe.get('source')
                or image != probe.get('image_id') or (args.build_dir / 'image_id.txt').read_text().strip() != image):
            raise phase.PhaseError('Exact clean source, IMAGE_VALIDATED and candidate20kmh probe required')
        phase.readiness_result(args.probe_dir)
        phase.image_inspection(runner, docker, image, 'image_identity')
        safety.atomic_private(output / 'protocol.json', protocol_record())
        state.update(source=source, image_id=image, model_sha256=safety.digest(repo / MODEL),
                     protocol_sha256=safety.digest(output / 'protocol.json'))
        if args.operation == 'execute':
            prepared = json.loads((args.prepare_dir / 'manifest.json').read_text())
            report = json.loads((args.prepare_dir / 'commissioning_report.json').read_text())
            verify_preparation(report, source, image)
            other = json.loads((args.watchdog_proof_dir / 'commissioning_report.json').read_text())
            verify_preparation(other, source, image)
            proofs = {item.get('fault_drill'): item for item in (report, other)}
            if set(proofs) != {'adapter', 'session'} or any(x.get('fault_drill_passed') is not True for x in proofs.values()):
                raise phase.PhaseError('Both adapter-loss and session-loss read-only containment proofs are required')
            if (prepared['model_sha256'] != state['model_sha256']
                    or prepared['protocol_sha256'] != state['protocol_sha256']):
                raise phase.PhaseError('Prepared model or protocol changed')
            state['run_id'] = prepared['run_id']
            approval = safety.verify_authorization(json.loads(args.authorization.read_text()), state)
            state.update(trial=approval['trial'], authorization_id=safety.digest(args.authorization))
            # A prepared identity is single-use, including failed launch attempts.
            with (args.prepare_dir / 'authorization.used').open('x') as record:
                record.write(json.dumps({'authorization_sha256': state['authorization_id'], 'output': str(output)}))
            safety.atomic_private(output / 'authorization.json', approval)
        safety.atomic_private(output / 'manifest.json', state)
        safety.atomic_private(output / 'host.lease', f'{time.monotonic():.9f}\n')
        command = docker + phase.container_options(output, state['run_id'], 'commissioning-' + args.operation) + [
            '--label', 'nav2.phase_a.role=commissioning', image,
            'python3', '/opt/auto_tuning/tools/commissioning_session.py']
        runner.run('commissioning_create', command, capture=False)
        state['container_id'] = phase.owned_id(output / 'container.id')
        safety.verify_container(inspect_bounded(runner, docker, state['container_id'], 'created_identity'), state)
        safety.atomic_private(output / 'manifest.json', state)
        safety.atomic_private(output / 'host.lease', f'{time.monotonic():.9f}\n')
        runner.run('commissioning_start', docker + ['start', state['container_id']], timeout=10)
        started = time.monotonic()
        watch = safety.HeartbeatWatch(state['run_id'])
        heartbeat_seen = False
        gui_announced = False
        while time.monotonic() - started < args.duration + 5:
            current = time.monotonic()
            cancellation = output / 'cancel.request.json'
            if cancellation.is_file():
                if json.loads(cancellation.read_text()).get('run_id') != state['run_id']:
                    raise phase.PhaseError('Cancellation identity mismatch')
                reason = 'OPERATOR_CANCELLED'
                break
            # This loop is independent of HTTP/rendering and ROS callbacks.
            safety.atomic_private(output / 'host.lease', f'{current:.9f}\n')
            if (output / 'guard_fault.json').is_file():
                fault = json.loads((output / 'guard_fault.json').read_text())
                reason = fault.get('reason', 'GUARD_FAULT')
                expected = (drill_triggered and state['fault_drill'] == 'adapter'
                    and fault.get('component') == 'adapter'
                    and fault.get('run_id') == state['run_id'] and 'HEARTBEAT' in reason)
                if not expected:
                    failure = reason
                break
            if (output / 'session_summary.json').is_file():
                summary = json.loads((output / 'session_summary.json').read_text())
                if summary.get('run_id') != state['run_id'] or summary.get('mode') != args.operation:
                    raise phase.PhaseError('Session summary identity mismatch')
                reason = summary.get('reason', 'SESSION_ENDED')
                if args.operation == 'execute' and reason != 'TRIAL_COMMANDS_COMPLETE_STANDSTILL_OBSERVED':
                    failure = reason
                elif args.operation == 'prepare' and reason != 'DURATION_LIMIT_REACHED':
                    failure = reason
                break
            guard_file = output / 'guard_heartbeat.json'
            if guard_file.exists():
                heartbeat_seen = True
                try:
                    guard = safety.read_heartbeat(guard_file, watch)
                except (OSError, ValueError) as error:
                    reason = 'SESSION_GUARD_HEARTBEAT_FAILED: ' + str(error)
                    if not drill_triggered or state['fault_drill'] != 'session':
                        failure = reason
                    break
                adapter = guard.get('adapter', {})
                count = adapter.get('publisher_created')
                if type(count) is bool:
                    publisher_count = int(count)
                ready_seen |= guard.get('ready') is True
                if args.operation == 'prepare' and count is True:
                    raise phase.PhaseError('PREPARATION_CREATED_ACTUATOR_PUBLISHER')
                if not gui_announced and (output / 'runtime_status.json').is_file():
                    try:
                        visual.read_frame(output)
                        token = uuid.uuid4().hex
                        server = HTTPServer(('127.0.0.1', args.port),
                                            visual.handler_for(output, token, args.port, capture_frames=3))
                        thread = threading.Thread(target=server.serve_forever, daemon=True)
                        thread.start()
                        print(f'Observação: http://127.0.0.1:{args.port}/{token}/', flush=True)
                        print('Ctrl-C encerra somente esta simulação. Fechar a aba não encerra o ensaio.', flush=True)
                        gui_announced = True
                    except ValueError:
                        pass
                if (args.operation == 'execute' and gui_announced and guard.get('telemetry_ready') is True
                        and adapter.get('observations_ready') is True and not prompt_started):
                    def request_start():
                        try:
                            answer = input('Confirme a imagem ao vivo e a área livre. Digite START para iniciar este único trecho: ')
                            if answer == 'START':
                                operator_started.set()
                        except EOFError:
                            pass
                    threading.Thread(target=request_start, daemon=True).start()
                    prompt_started = True
                if operator_started.is_set() and not (output / 'operator_start.json').exists():
                    safety.atomic_private(output / 'operator_start.json', {
                        'run_id': state['run_id'], 'authorization_id': state['authorization_id'],
                        'monotonic_s': time.monotonic()})
                if ready_seen and state['fault_drill'] != 'none' and not drill_triggered:
                    target = guard.get('adapter_pid') if state['fault_drill'] == 'adapter' else guard.get('pid')
                    if type(target) is not int or target <= 1:
                        raise phase.PhaseError('Invalid owned process identity for read-only fault drill')
                    drill_time = time.monotonic()
                    safety.atomic_private(output / 'fault_injection.json', {
                        'target': state['fault_drill'], 'signal': 'SIGSTOP', 'pid': target,
                        'monotonic_s': drill_time, 'actuator_publishers_created': publisher_count})
                    runner.run('inject_read_only_process_stall', docker + ['exec', state['container_id'],
                        '/bin/kill', '-STOP', str(target)], timeout=3)
                    drill_triggered = True
                if drill_triggered and time.monotonic() - drill_time > 3:
                    raise phase.PhaseError('Fault drill did not terminate supervision within 3 wall seconds')
            elif heartbeat_seen or current - started > safety.STARTUP_TIMEOUT:
                raise phase.PhaseError('Session guard heartbeat missing')
            time.sleep(0.05)
    except KeyboardInterrupt:
        reason = 'OPERATOR_CANCELLED'
    except BaseException as error:
        failure = reason = f'{type(error).__name__}: {error}'
    finally:
        containment = None
        try:
            containment = stop_owned(output, state, runner=runner, docker=docker)
        except Exception as error:
            failure = 'INDEPENDENT_STOP_NOT_CONFIRMED: ' + str(error)
        containment_delay = time.monotonic() - drill_time if drill_time else None
        if drill_triggered and containment_delay >= 3:
            failure = 'FAULT_CONTAINMENT_EXCEEDED_3_WALL_SECONDS'
        if args.operation == 'prepare' and not ready_seen and reason != 'OPERATOR_CANCELLED' and failure is None:
            failure = 'READ_ONLY_READINESS_NOT_OBSERVED'
        if args.operation == 'execute' and reason not in ('TRIAL_COMMANDS_COMPLETE_STANDSTILL_OBSERVED', 'OPERATOR_CANCELLED') and failure is None:
            failure = 'BOUNDED_TRIAL_NOT_COMPLETED'
        if (args.operation == 'prepare' and state['fault_drill'] != 'none' and reason != 'OPERATOR_CANCELLED'
                and failure is None and not (drill_triggered and ready_seen and publisher_count == 0 and 'HEARTBEAT' in reason)):
            failure = 'FAULT_DRILL_DID_NOT_PROVE_EXPECTED_CONTAINMENT'
        # Stop the simulator before waiting for any browser worker or log I/O.
        if server:
            server.shutdown()
            server.server_close()
        if state.get('container_id'):
            try:
                runner.run('container_console', docker + ['logs', state['container_id']], timeout=5, capture=False)
            except Exception:
                pass
        report = {**state, 'status': 'FAILED' if failure else 'CANCELLED' if reason == 'OPERATOR_CANCELLED' else 'PREPARATION_COLLECTED' if args.operation == 'prepare' else 'TRIAL_COLLECTED',
            'reason': reason, 'error': failure, 'read_only_ready_observed': ready_seen,
            'actuator_publishers_created': publisher_count, 'containment': containment,
            'fault_drill_triggered': drill_triggered,
            'fault_drill_passed': bool(drill_triggered and not failure and ready_seen and publisher_count == 0
                and ('HEARTBEAT' in reason)
                and containment and containment.get('status') == 'OWNED_SIMULATOR_TERMINATED'),
            'fault_to_host_containment_wall_s': containment_delay,
            'physical_stop_verified': False, 'dynamic_validation_complete': False,
            'elapsed_wall_s': time.monotonic() - started}
        safety.atomic_private(output / 'commissioning_report.json', report)
        if args.operation == 'prepare' and 'source' in state:
            safety.atomic_private(output / 'authorization.template.json', {
                'schema_version': 1, 'scope': 'SIMULATION_COMMISSIONING', 'run_id': state['run_id'],
                'source_commit': state['source']['commit'], 'image_id': state['image_id'],
                'model_sha256': state['model_sha256'], 'protocol_sha256': state['protocol_sha256'],
                'navigation_profile': 'candidate20kmh', 'trial': 'straight',
                'motion_authorized': False, 'area_confirmed_clear': False,
                'operator': '', 'authorization_reference': '', 'expires_at_unix': 0})
        print(json.dumps(report, indent=2), flush=True)
    return 2 if failure else 130 if reason == 'OPERATOR_CANCELLED' else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    for name in ('prepare', 'execute'):
        p = sub.add_parser(name)
        for option in ('repo', 'build-dir', 'probe-dir', 'output-dir'):
            p.add_argument('--' + option, type=Path, required=True)
        p.add_argument('--port', type=int, default=8765)
        p.add_argument('--duration', type=float, default=120 if name == 'prepare' else 60)
        if name == 'prepare':
            p.add_argument('--fault-drill', choices=('none', 'adapter', 'session'), default='none')
        else:
            for option in ('prepare-dir', 'watchdog-proof-dir', 'authorization'):
                p.add_argument('--' + option, type=Path, required=True)
    for name in ('status', 'stop'):
        p = sub.add_parser(name)
        p.add_argument('--session-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.operation in ('status', 'stop'):
        state = json.loads((args.session_dir / 'manifest.json').read_text())
        if args.operation == 'stop':
            result = stop_owned(args.session_dir, state)
        else:
            operations = args.session_dir / 'operations'
            operations.mkdir(mode=0o700, exist_ok=True)
            runner = phase.Runner(Path(tempfile.mkdtemp(prefix='status.', dir=operations)))
            info = inspect_bounded(runner, phase.docker_prefix(), state['container_id'], 'status_identity')
            safety.verify_container(info, state)
            latest = json.loads((args.session_dir / 'guard_heartbeat.json').read_text()) if (args.session_dir / 'guard_heartbeat.json').is_file() else None
            age = time.monotonic() - latest.get('monotonic_s', 0) if latest else None
            running = info.get('State', {}).get('Running') is True
            result = {'status': 'STOPPED' if not running else 'RUNNING' if age is not None and 0 <= age < safety.PROCESS_HEARTBEAT_TIMEOUT else 'STALE_OR_STARTING',
                      'container_state': info.get('State'), 'guard_age_wall_s': age,
                      'manifest': state, 'last_guard_evidence': latest,
                      'report': json.loads((args.session_dir / 'commissioning_report.json').read_text())
                      if (args.session_dir / 'commissioning_report.json').is_file() else None,
                      'mission_ready': False}
        print(json.dumps(result, indent=2))
        return 0
    limit = safety.PREPARE_WALL_LIMIT if args.operation == 'prepare' else safety.EXECUTION_WALL_LIMIT
    if not 10 <= args.duration <= limit or not 1024 <= args.port <= 65535:
        parser.error('Duration or localhost port outside reviewed bounds')
    return run(args)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, phase.interrupted)
    raise SystemExit(main())
