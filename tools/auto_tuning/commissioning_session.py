#!/usr/bin/env python3
"""Private simulator session with a wall-clock guard independent of ROS.

prepare is read-only: neither test source nor actuator publisher is started.
execute is reachable only through an explicit, bound host authorization record.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ackermann_adapter import rear_right_offset
from audit_ackermann_geometry import audit_model
from commissioning_source import private_directory
import commissioning_supervision as safety
import local_phase_a as phase
import visual_observer_session as visual


EVIDENCE = Path('/evidence')
TOOLS = Path('/opt/auto_tuning/tools')
CONFIG = Path('/opt/auto_tuning/config')


def adapter_arguments(manifest, model, *, evidence=EVIDENCE):
    audit = audit_model(model)
    if audit['source']['sha256'] != manifest['model_sha256']:
        raise ValueError('Prepared candidate model identity changed')
    limits = audit['steering_joint_limits']
    parameters = {
        'mode': manifest['mode'], 'evidence_dir': str(evidence / 'adapter'),
        'supervisor_lease': str(evidence / 'supervisor.lease'),
        'run_id': manifest['run_id'], 'authorization_id': manifest.get('authorization_id', ''),
        'duration_wall_s': float(manifest['duration_wall_s']),
        'model_sha256': audit['source']['sha256'],
        'wheelbase_m': audit['wheelbase_euclidean_m'], 'track_m': audit['front_track_euclidean_m'],
        'rear_right_y_m': rear_right_offset(audit),
        'plugin_max_speed_mps': audit['plugin_max_speed_mps'],
        'plugin_max_steer_rad': audit['plugin_max_steer_rad'],
        'left_lower_rad': limits['left']['lower_rad'], 'left_upper_rad': limits['left']['upper_rad'],
        'right_lower_rad': limits['right']['lower_rad'], 'right_upper_rad': limits['right']['upper_rad'],
        'base_to_rear_x_m': -3.502,
    }
    command = ['/opt/auto_tuning/bin/commissioning_adapter', '--ros-args']
    for key, value in parameters.items():
        # ROS parses string identifiers as YAML; quote even all-numeric hashes.
        rendered = json.dumps(value) if isinstance(value, str) else str(value)
        command.extend(('-p', key + ':=' + rendered))
    return command, parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=EVIDENCE / 'manifest.json')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest.get('mode') not in ('prepare', 'execute') or manifest.get('navigation_profile') != 'candidate20kmh':
        raise ValueError('Explicit commissioning candidate/mode required')
    authorization = None
    if manifest['mode'] == 'execute':
        authorization = safety.verify_authorization(json.loads((EVIDENCE / 'authorization.json').read_text()), manifest)
    duration = manifest['duration_wall_s']
    bound = safety.EXECUTION_WALL_LIMIT if manifest['mode'] == 'execute' else safety.PREPARE_WALL_LIMIT
    if not 10 <= duration <= bound:
        raise ValueError('Session duration outside the reviewed wall bound')
    os.environ.update(DISPLAY=':91', QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1',
                      ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='71',
                      GAZEBO_MASTER_URI='http://127.0.0.1:11345')
    processes, logs = {}, []
    interrupted = threading.Event()
    gui = {'gui_ready': False}
    sequence, ready, code, reason = 0, False, 0, 'DURATION_LIMIT_REACHED'
    fault_component = 'session'
    started = time.monotonic()

    def signal_stop(_signum, _frame):
        interrupted.set()

    signal.signal(signal.SIGTERM, signal_stop)
    signal.signal(signal.SIGINT, signal_stop)

    def start(name, command):
        stream = (EVIDENCE / (name + '.log')).open('xb')
        logs.append(stream)
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                start_new_session=True, env=os.environ.copy())
        processes[name] = proc
        with (EVIDENCE / 'session_commands.jsonl').open('a') as out:
            out.write(json.dumps({'name': name, 'pid': proc.pid, 'argv': command}) + '\n')
        return proc

    def gui_worker():
        while not interrupted.wait(1):
            try:
                windows = visual.arrange_windows_bounded()
                gui.update(gui_ready={'gazebo', 'rviz'} <= windows.keys(), windows=windows)
            except Exception as error:
                gui.update(gui_ready=False, error=str(error))

    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('truck_bringup'))
        selected = phase.navigation_arguments('candidate20kmh')
        model = CONFIG / 'plant_candidate/models/arocs_truck_candidate/model.sdf'
        adapter, parameters = adapter_arguments(manifest, model)
        safety.atomic_private(EVIDENCE / 'adapter_parameters.json', parameters)
        presentation = visual.prepare_presentation_model(CONFIG, EVIDENCE / 'presentation_models',
                                                         'model://arocs_truck_candidate')
        os.environ.update(visual.gazebo_asset_environment(share, EVIDENCE / 'presentation_models', CONFIG, os.environ))
        assets = visual.visual_asset_readback(Path(presentation['presentation_model']), os.environ['GAZEBO_MODEL_PATH'])
        source_world = CONFIG / 'plant_candidate/worlds/ctvi_plant_candidate.world'
        world, addition = visual.observation_world(source_world, 'model://arocs_truck_candidate')
        (EVIDENCE / 'observation_world.world').write_text(world)
        safety.atomic_private(EVIDENCE / 'presentation_manifest.json', {
            'model': presentation, 'visual_assets': assets, 'only_world_addition': addition,
            'physical_model_changed': False, 'navigation_profile': 'candidate20kmh'})
        (EVIDENCE / 'framebuffer').mkdir()
        start('xvfb', ['Xvfb', ':91', '-screen', '0', '2200x900x24', '-nolisten', 'tcp',
                       '-fbdir', str(EVIDENCE / 'framebuffer')])
        while not (EVIDENCE / 'framebuffer/Xvfb_screen0').is_file():
            if interrupted.is_set() or time.monotonic() - started > 10 or processes['xvfb'].poll() is not None:
                raise RuntimeError('Framebuffer startup failed')
            if not safety.read_lease(EVIDENCE / 'host.lease'):
                raise RuntimeError('HOST_LEASE_EXPIRED')
            time.sleep(0.05)
        visual.make_framebuffer_readable(EVIDENCE / 'framebuffer/Xvfb_screen0')
        start('bringup', ['ros2', 'launch', 'truck_bringup', 'gps_waypoint_follower.launch.py',
            'use_gzclient:=false', 'use_rviz:=false', 'use_mapviz:=false', 'use_gpu:=false',
            'use_xcb:=true', 'publish_legacy_cloud_tf:=false', 'use_ackermann_bt:=true',
            'actuation_routing:=isolated', 'world:=' + str(EVIDENCE / 'observation_world.world'),
            'localization_params_file:=/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml']
            + [item for item in selected if not item.startswith('world:=')])
        start('gazebo_gui', ['gzclient', '--verbose'])
        start('rviz_gui', ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(CONFIG / 'observation.rviz'),
                           '--ros-args', '-p', 'use_sim_time:=true'])
        threading.Thread(target=gui_worker, daemon=True).start()
        safety.atomic_private(EVIDENCE / 'supervisor.lease', f'{time.monotonic():.9f}\n')
        private_directory(EVIDENCE / 'adapter')
        start('adapter', adapter)
        start('observer', ['python3', str(TOOLS / 'commissioning_observer.py'),
            '--evidence-dir', str(EVIDENCE / 'observer'), '--trial', manifest['trial'],
            '--run-id', manifest['run_id'], '--duration', str(duration + 5),
            '--source-heartbeat', str(EVIDENCE / 'source/source_heartbeat.json')])
        watches = {name: safety.HeartbeatWatch(manifest['run_id']) for name in ('adapter', 'observer')}
        source_started = False
        first_heartbeat_deadline = time.monotonic() + 15
        adapter_record = {}
        observer_record = {}
        while not interrupted.is_set():
            now = time.monotonic()
            if authorization is not None and time.time() >= authorization['expires_at_unix']:
                raise RuntimeError('EXECUTION_AUTHORIZATION_EXPIRED')
            cancellation = EVIDENCE / 'cancel.request.json'
            if cancellation.is_file():
                if json.loads(cancellation.read_text()).get('run_id') != manifest['run_id']:
                    raise RuntimeError('CANCELLATION_IDENTITY_MISMATCH')
                reason = 'OPERATOR_OR_HOST_STOP'
                break
            if now - started >= duration:
                break
            if not safety.read_lease(EVIDENCE / 'host.lease'):
                raise RuntimeError('HOST_LEASE_EXPIRED')
            exited = [name for name, proc in processes.items() if proc.poll() is not None]
            if exited:
                raise RuntimeError('REQUIRED_PROCESS_EXITED: ' + ','.join(exited))
            heartbeat = EVIDENCE / 'adapter/adapter_heartbeat.json'
            if heartbeat.exists():
                fault_component = 'adapter'
                adapter_record = safety.read_heartbeat(heartbeat, watches['adapter'])
                if adapter_record.get('fault') or adapter_record.get('state') in ('FAULT', 'STOPPED'):
                    raise RuntimeError('ADAPTER_FAULT: ' + str(adapter_record.get('fault')))
            elif now > first_heartbeat_deadline:
                raise RuntimeError('ADAPTER_HEARTBEAT_MISSING')
            observer_file = EVIDENCE / 'observer/observer_heartbeat.json'
            if observer_file.exists():
                fault_component = 'observer'
                observer_record = safety.read_heartbeat(observer_file, watches['observer'])
            elif now > first_heartbeat_deadline:
                raise RuntimeError('OBSERVER_HEARTBEAT_MISSING')
            fault_component = 'session'
            operator_start = EVIDENCE / 'operator_start.json'
            start_authorized = False
            if operator_start.is_file():
                request = json.loads(operator_start.read_text())
                start_authorized = (request.get('run_id') == manifest['run_id']
                    and request.get('authorization_id') == manifest.get('authorization_id')
                    and 0 <= time.monotonic() - request.get('monotonic_s', 0) < 3)
            telemetry_ready = (not observer_record.get('health_reasons', ['MISSING'])
                and observer_record.get('valid_sync_count', 0) > 0)
            if source_started and (not gui['gui_ready'] or not telemetry_ready):
                raise RuntimeError('LIVE_VISUAL_OR_TELEMETRY_LOST')
            if (manifest['mode'] == 'execute' and not source_started and gui['gui_ready'] and telemetry_ready
                    and adapter_record.get('observations_ready') is True and start_authorized):
                start('source', ['python3', str(TOOLS / 'commissioning_source.py'), '--execute',
                    '--evidence-dir', str(EVIDENCE / 'source'), '--trial', manifest['trial'],
                    '--run-id', manifest['run_id'], '--approval-file', str(EVIDENCE / 'authorization.json'),
                    '--lease-file', str(EVIDENCE / 'supervisor.lease'), '--adapter-heartbeat', str(heartbeat)])
                source_started = True
                watches['source'] = safety.HeartbeatWatch(manifest['run_id'])
                source_deadline = now + 10
            if source_started:
                source_file = EVIDENCE / 'source/source_heartbeat.json'
                if source_file.exists():
                    fault_component = 'source'
                    source_record = safety.read_heartbeat(source_file, watches['source'])
                    if source_record.get('state') == 'FAULT':
                        raise RuntimeError('SOURCE_FAULT')
                    if source_record.get('state') == 'COMPLETE':
                        # Commands have ended. Keep measuring zeros until the adapter
                        # reports actual standstill, bounded by both wall deadlines.
                        if adapter_record.get('standstill') is True:
                            reason = 'TRIAL_COMMANDS_COMPLETE_STANDSTILL_OBSERVED'
                            break
                elif now > source_deadline:
                    raise RuntimeError('SOURCE_HEARTBEAT_MISSING')
                fault_component = 'session'
            ready = bool(gui['gui_ready'] and telemetry_ready and adapter_record.get('state') == 'READY_READ_ONLY')
            sequence += 1
            safety.atomic_private(EVIDENCE / 'guard_heartbeat.json', {
                'run_id': manifest['run_id'], 'seq': sequence, 'monotonic_s': now,
                'state': 'READY_READ_ONLY' if ready else 'SUPERVISING',
                'pid': os.getpid(), 'adapter_pid': processes['adapter'].pid,
                'ready': ready, 'adapter': adapter_record, 'telemetry_ready': telemetry_ready,
                'mode': manifest['mode'], 'mission_ready': False})
            safety.atomic_private(EVIDENCE / 'runtime_status.json', {
                'status': 'OBSERVING' if gui['gui_ready'] else 'WAITING_FOR_WINDOWS',
                'updated_at_unix': time.time(), 'gui_ready': gui['gui_ready'],
                'commissioning_mode': manifest['mode'], 'adapter_state': adapter_record.get('state'),
                'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'})
            # Renew only after every independent guard check has completed.
            safety.atomic_private(EVIDENCE / 'supervisor.lease', f'{time.monotonic():.9f}\n')
            time.sleep(0.05)
        if interrupted.is_set():
            reason = 'OPERATOR_OR_HOST_STOP'
    except BaseException as error:
        reason, code = f'{type(error).__name__}: {error}', 2
        safety.atomic_private(EVIDENCE / 'guard_fault.json', {
            'run_id': manifest['run_id'], 'reason': reason, 'monotonic_s': time.monotonic(),
            'component': fault_component,
            'adapter_pid': processes['adapter'].pid if 'adapter' in processes else None,
            'independent_of_adapter': True})
    finally:
        interrupted.set()
        # Immediately terminate the plant process group. Do not give a stale
        # Gazebo target a graceful 20-second opportunity to keep driving.
        for name in ('bringup', 'source', 'adapter', 'gazebo_gui', 'rviz_gui', 'xvfb'):
            proc = processes.get(name)
            if proc and proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        observer = processes.get('observer')
        if observer and observer.poll() is None:
            try:
                os.killpg(observer.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        # Normal completion may need a short telemetry flush. Keep the guard
        # progressing while it supervises already-signalled processes; a hung
        # finalizer still has a hard deadline. Faults are independently visible
        # in guard_fault.json and can trigger immediate host containment.
        finalization_deadline = time.monotonic() + 2
        while any(proc.poll() is None for proc in processes.values()) and time.monotonic() < finalization_deadline:
            sequence += 1
            safety.atomic_private(EVIDENCE / 'guard_heartbeat.json', {
                'run_id': manifest['run_id'], 'seq': sequence, 'monotonic_s': time.monotonic(),
                'state': 'FINALIZING', 'pid': os.getpid(), 'ready': False,
                'adapter': adapter_record if 'adapter_record' in locals() else {},
                'mode': manifest['mode'], 'mission_ready': False})
            time.sleep(0.05)
        for proc in processes.values():
            try:
                proc.wait(timeout=0.01)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for stream in logs:
            stream.close()
        safety.atomic_private(EVIDENCE / 'session_summary.json', {
            'schema_version': 1, 'run_id': manifest['run_id'], 'mode': manifest['mode'],
            'reason': reason, 'read_only_ready_observed': ready,
            'bringup_launcher_terminated': processes.get('bringup') is not None and processes['bringup'].poll() is not None,
            'containment_proof': 'Host must inspect owned container Running=false and Pid=0',
            'physical_stop_verified': False, 'dynamic_validation_complete': False,
            'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'})
        safety.atomic_private(EVIDENCE / 'runtime_status.json', {
            'status': 'STOPPED', 'gui_ready': False, 'updated_at_unix': time.time(), 'reason': reason})
    return code


if __name__ == '__main__':
    raise SystemExit(main())
