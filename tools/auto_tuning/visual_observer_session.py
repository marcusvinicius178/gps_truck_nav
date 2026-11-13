#!/usr/bin/env python3
"""Private Xvfb session: Gazebo/RViz pixels only; no input or motion interface."""

import argparse
import ctypes
import ctypes.util
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


EVIDENCE = Path('/evidence')
CONFIG = Path('/opt/auto_tuning/config')
_phase_spec = importlib.util.spec_from_file_location('visual_session_phase_helpers',
                                                    Path(__file__).with_name('local_phase_a.py'))
phase = importlib.util.module_from_spec(_phase_spec)
_phase_spec.loader.exec_module(phase)


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def make_framebuffer_readable(path):
    # Only this session's regular framebuffer; the host session directory is 0700.
    if path.is_symlink() or not path.is_file():
        raise ValueError('Private framebuffer must be a regular file')
    path.chmod(0o644)


def sensor_presentation_copy(text):
    """Hide GUI-only ray drawings, preserving all other source bytes.

    Classic 11.10.2 Scene::ProcessSensorMsg creates LaserVisual only when the
    sensor message has visualize=true. Ray parameters/plugins remain intact.
    The original model is never edited; only the graphical session uses a copy.
    """
    changed = []

    def hide(match):
        sensor = ET.fromstring(match.group(0))
        if sensor.get('type') != 'ray':
            return match.group(0)
        name = sensor.get('name')
        if name not in ('left_lidar', 'right_lidar', 'sick_lidar'):
            raise ValueError('Unreviewed ray sensor: ' + str(name))
        if len(sensor.findall('visualize')) != 1:
            raise ValueError('Expected one GUI visualization flag per ray sensor')
        replacement, count = re.subn(r'(<visualize>\s*)(?:1|true)(\s*</visualize>)',
                                      r'\g<1>0\2', match.group(0))
        if count != 1:
            raise ValueError('Expected the reviewed enabled ray visualization flag')
        changed.append({'sensor': name, 'type': 'ray', 'field': 'visualize',
                        'source_value': True, 'presentation_value': False})
        return replacement

    result = re.sub(r'<sensor\b[^>]*>.*?</sensor\s*>', hide, text, flags=re.S)
    if sorted(item['sensor'] for item in changed) != ['left_lidar', 'right_lidar', 'sick_lidar']:
        raise ValueError('Expected exactly the three reviewed ray sensors')
    ET.fromstring(result)
    return result, changed


def prepare_presentation_model(config, destination, model_uri):
    selected = {
        'model://arocs_truck_tf_sensors': 'models/arocs_truck_tf_sensors',
        'model://arocs_truck_candidate': 'plant_candidate/models/arocs_truck_candidate',
    }
    if model_uri not in selected:
        raise ValueError('Unknown graphical model selection')
    source = config / selected[model_uri]
    original = (source / 'model.sdf').read_bytes()
    shown, changes = sensor_presentation_copy(original.decode('utf-8'))
    model_config = (source / 'model.config').read_bytes()
    destination.mkdir()  # A new session only; never overwrite prior evidence.
    model = destination / model_uri.removeprefix('model://')
    model.mkdir()
    (model / 'model.sdf').write_text(shown, encoding='utf-8')
    (model / 'model.config').write_bytes(model_config)
    return {
        'model_uri': model_uri, 'source_model': str(source / 'model.sdf'),
        'source_model_sha256': hashlib.sha256(original).hexdigest(),
        'presentation_model': str(model / 'model.sdf'),
        'presentation_model_sha256': hashlib.sha256(shown.encode('utf-8')).hexdigest(),
        'model_config_sha256': hashlib.sha256(model_config).hexdigest(),
        'changes': changes, 'sensor_data_configuration_changed': False,
        'resolution': 'Selected model alias in session search path before config model paths',
    }


def gazebo_asset_environment(share, presentation_models, config, inherited):
    """Give the independently launched gzclient the server's local assets.

    Launch SetEnvironmentVariable actions affect the launch's children only.
    They do not update this supervisor, which also starts gzclient directly.
    Keep the presentation alias first, then expose the installed canonical
    meshes and terrain to both processes. Preserve additional inherited paths.
    """
    def joined(primary, variable):
        paths = [str(path) for path in primary] + inherited.get(variable, '').split(os.pathsep)
        return os.pathsep.join(dict.fromkeys(path for path in paths if path))

    return {
        'GAZEBO_MODEL_PATH': joined([
            presentation_models, share / 'models', '/usr/share/gazebo-11/models',
            config / 'plant_candidate/models', config / 'models'], 'GAZEBO_MODEL_PATH'),
        'GAZEBO_RESOURCE_PATH': joined([
            share / 'worlds', share / 'models', '/usr/share/gazebo-11'], 'GAZEBO_RESOURCE_PATH'),
    }


def visual_asset_readback(model_sdf, model_search_path):
    """Resolve the truck meshes and CTVI resources in the actual GUI paths.

    This is a local file/readability check, not proof of OGRE rendering. A
    missing mesh should fail clearly before showing an incomplete truck.
    """
    tree = ET.parse(model_sdf)
    uris = {node.text.strip() for node in tree.findall('.//mesh/uri') if node.text}
    uris.update(('model://ctvi/model.sdf',
                 'model://ctvi/materials/scripts/MyTerrainMaterial.material',
                 'model://ctvi/materials/textures/ctvi_aerial.png'))
    paths = [Path(item) for item in model_search_path.split(os.pathsep) if item]
    result = []
    for uri in sorted(uris):
        if not uri.startswith('model://'):
            raise ValueError('Unreviewed visual mesh URI: ' + uri)
        relative = Path(uri.removeprefix('model://'))
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Invalid visual asset URI: ' + uri)
        resolved = next((base / relative for base in paths if (base / relative).is_file()), None)
        if resolved is None:
            raise ValueError('Gazebo client cannot resolve visual asset: ' + uri)
        with resolved.open('rb') as stream:
            if stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1'):
                raise ValueError('Gazebo client visual asset is an unhydrated LFS pointer: ' + str(resolved))
        result.append({'uri': uri, 'resolved_file': str(resolved), 'bytes': resolved.stat().st_size})
    return result


def observation_world(source, model_uri='model://arocs_truck_tf_sensors'):
    """Add camera-only presentation settings; retain every source byte otherwise."""
    text = source.read_text()
    tree = ET.fromstring(text)
    world = tree.find('world')
    if world is None or world.find('gui') is not None or text.count('</world>') != 1:
        raise ValueError('Expected one world without an existing GUI section')
    truck = [item for item in world.findall('include')
             if item.findtext('uri') == model_uri]
    if len(truck) != 1:
        raise ValueError('Expected the reviewed TF/sensor model exactly once')
    pose = [float(value) for value in truck[0].findtext('pose', '').split()]
    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise ValueError('Invalid existing truck spawn pose')
    # Camera position only: no vehicle/entity pose, datum, physics or controller edit.
    x, y, z, _, _, yaw = pose
    # Front three-quarter view relative to the truck's actual spawn heading.
    # Aim at the body center, rather than assuming the world +X/+Y directions.
    cx = x + 14 * math.cos(yaw) + 11 * math.sin(yaw)
    cy = y + 14 * math.sin(yaw) - 11 * math.cos(yaw)
    cz, target_z = z + 10.5, z + 1.5
    camera_yaw = math.atan2(y - cy, x - cx)
    camera_pitch = math.atan2(cz - target_z, math.hypot(x - cx, y - cy))
    camera = f'{cx:.9f} {cy:.9f} {cz:.9f} 0 {camera_pitch:.9f} {camera_yaw:.9f}'
    addition = ('    <gui fullscreen="0">\n'
                '      <camera name="phase_a_observation_camera">\n'
                f'        <pose>{camera}</pose>\n'
                '        <view_controller>orbit</view_controller>\n'
                '      </camera>\n'
                '    </gui>\n')
    return text.replace('</world>', addition + '</world>'), addition


def arrange_windows():
    """Move only Gazebo/RViz windows on this container's private display."""
    library = ctypes.util.find_library('X11')
    if not library:
        raise RuntimeError('libX11 unavailable')
    x = ctypes.CDLL(library)
    display_t, window_t = ctypes.c_void_p, ctypes.c_ulong
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]; x.XOpenDisplay.restype = display_t
    x.XDefaultRootWindow.argtypes = [display_t]; x.XDefaultRootWindow.restype = window_t
    x.XQueryTree.argtypes = [display_t, window_t, ctypes.POINTER(window_t), ctypes.POINTER(window_t),
                            ctypes.POINTER(ctypes.POINTER(window_t)), ctypes.POINTER(ctypes.c_uint)]
    x.XFetchName.argtypes = [display_t, window_t, ctypes.POINTER(ctypes.c_char_p)]
    x.XFree.argtypes = [ctypes.c_void_p]
    x.XMoveResizeWindow.argtypes = [display_t, window_t, ctypes.c_int, ctypes.c_int,
                                   ctypes.c_uint, ctypes.c_uint]
    x.XMapRaised.argtypes = [display_t, window_t]
    x.XFlush.argtypes = [display_t]; x.XCloseDisplay.argtypes = [display_t]
    display = x.XOpenDisplay(b':91')
    if not display:
        raise RuntimeError('Private observation display unavailable')
    found = {}
    children = ctypes.POINTER(window_t)()
    try:
        root = x.XDefaultRootWindow(display)
        root_return, parent_return, count = window_t(), window_t(), ctypes.c_uint()
        if not x.XQueryTree(display, root, ctypes.byref(root_return), ctypes.byref(parent_return),
                            ctypes.byref(children), ctypes.byref(count)):
            raise RuntimeError('Cannot inspect private X11 windows')
        for i in range(count.value):
            title = ctypes.c_char_p()
            if not x.XFetchName(display, children[i], ctypes.byref(title)) or not title.value:
                continue
            name = title.value.decode('utf-8', errors='replace')
            x.XFree(ctypes.cast(title, ctypes.c_void_p))
            key = 'gazebo' if name.strip().lower() == 'gazebo' else 'rviz' if 'rviz' in name.lower() else None
            if key is not None:
                x.XMoveResizeWindow(display, children[i], 0 if key == 'gazebo' else 1100, 0, 1100, 900)
                x.XMapRaised(display, children[i])
                found[key] = {'window': int(children[i]), 'title': name}
        x.XFlush(display)
        return found
    finally:
        if children:
            x.XFree(children)
        x.XCloseDisplay(display)


def arrange_windows_bounded(*, runner=subprocess.run):
    """A wedged X server must not trap the supervisor inside a blocking Xlib call."""
    result = runner([sys.executable, str(Path(__file__).resolve()), '--arrange-windows-only'],
                    timeout=2, check=True, capture_output=True, text=True)
    windows = json.loads(result.stdout)
    if not isinstance(windows, dict) or set(windows) - {'gazebo', 'rviz'}:
        raise ValueError('Invalid private window arrangement result')
    return windows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=900)
    parser.add_argument('--arrange-windows-only', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.arrange_windows_only:
        print(json.dumps(arrange_windows()))
        return 0
    if not 60 <= args.duration <= 1800:
        parser.error('duration must be 60..1800 wall seconds')
    if (EVIDENCE / 'runtime_status.json').exists():
        parser.error('Refusing to restart a previous visual session')
    selected_profile = os.environ.get('NAV2_PHASE_A_PROFILE', 'canonical')
    selected_arguments = phase.navigation_arguments(selected_profile)
    os.environ.update(DISPLAY=':91', QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1',
                      ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='71',
                      GAZEBO_MASTER_URI='http://127.0.0.1:11345')
    (EVIDENCE / 'framebuffer').mkdir()
    processes, logs = {}, []
    state = {'schema_version': 1, 'status': 'STARTING', 'gui_ready': False,
             'navigation_profile': selected_profile,
             'mission_ready': False, 'motion_interface_exposed': False,
             'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    stop_requested = False

    def interrupted(_signal, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)

    def start(name, command):
        stream = (EVIDENCE / (name + '.log')).open('xb')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=os.environ.copy())
        processes[name] = process
        with (EVIDENCE / 'session_commands.jsonl').open('a') as out:
            out.write(json.dumps({'name': name, 'pid': process.pid, 'argv': command,
                                  'started_at_unix': time.time()}) + '\n')
        return process

    started = time.monotonic()
    code = 0
    try:
        from ament_index_python.packages import get_package_share_directory
        installed_share = Path(get_package_share_directory('truck_bringup'))
        source = Path(next(item.split(':=', 1)[1] for item in selected_arguments if item.startswith('world:=')))
        model_uri = ('model://arocs_truck_candidate' if selected_profile in phase.CANDIDATE_PROFILES
                     else 'model://arocs_truck_tf_sensors')
        presentation_models = EVIDENCE / 'presentation_models'
        model_presentation = prepare_presentation_model(CONFIG, presentation_models, model_uri)
        asset_environment = gazebo_asset_environment(installed_share, presentation_models, CONFIG, os.environ)
        os.environ.update(asset_environment)
        assets = visual_asset_readback(Path(model_presentation['presentation_model']),
                                       asset_environment['GAZEBO_MODEL_PATH'])
        world, addition = observation_world(source, model_uri)
        path = EVIDENCE / 'observation_world.world'
        path.write_text(world)
        save(EVIDENCE / 'presentation_manifest.json', {
            'source_world': str(source), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'visual_world_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'only_added_content': addition, 'scope': 'GUI_PRESENTATION_ONLY', 'physical_model_changed': False,
            'navigation_profile': selected_profile, 'navigation_arguments': selected_arguments,
            'model_presentation': model_presentation,
            'session_model_search_path': os.environ['GAZEBO_MODEL_PATH'],
            'session_resource_search_path': os.environ['GAZEBO_RESOURCE_PATH'],
            'visual_assets': assets,
            'rviz_config_sha256': hashlib.sha256((CONFIG / 'observation.rviz').read_bytes()).hexdigest()})
        start('xvfb', ['Xvfb', ':91', '-screen', '0', '2200x900x24', '-nolisten', 'tcp',
                       '-fbdir', str(EVIDENCE / 'framebuffer')])
        deadline = time.monotonic() + 8
        while not (EVIDENCE / 'framebuffer/Xvfb_screen0').is_file():
            if stop_requested or processes['xvfb'].poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Private framebuffer did not become available')
            time.sleep(0.1)
        make_framebuffer_readable(EVIDENCE / 'framebuffer/Xvfb_screen0')
        start('bringup', ['ros2', 'launch', 'truck_bringup', 'gps_waypoint_follower.launch.py',
              'use_gzclient:=false', 'use_rviz:=false', 'use_mapviz:=false', 'use_gpu:=false',
              'use_xcb:=true', 'publish_legacy_cloud_tf:=false', 'use_ackermann_bt:=true',
              'world:=' + str(path),
              'localization_params_file:=/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml']
              + [item for item in selected_arguments if not item.startswith('world:=')])
        start('gazebo_gui', ['gzclient', '--verbose'])
        start('rviz_gui', ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(CONFIG / 'observation.rviz'),
                           '--ros-args', '-p', 'use_sim_time:=true'])
        probe_started = False
        while not stop_requested and time.monotonic() - started < args.duration:
            state['processes'] = {name: {'pid': proc.pid, 'returncode': proc.poll()}
                                  for name, proc in processes.items()}
            failed = [name for name, proc in processes.items()
                      if (name not in ('readiness', 'actuation_shadow') and proc.poll() is not None)
                      or (name == 'actuation_shadow' and proc.poll() not in (None, 0))]
            if failed:
                raise RuntimeError('Required GUI/session process exited: ' + ', '.join(failed))
            try:
                windows = arrange_windows_bounded()
                state.update(windows=windows, gui_ready={'gazebo', 'rviz'} <= windows.keys())
                state.pop('window_error', None)
            except Exception as error:
                state.update(gui_ready=False, window_error=f'{type(error).__name__}: {error}')
            if not probe_started and time.monotonic() - started >= 10:
                (EVIDENCE / 'runtime_probe').mkdir()
                # Snapshot existing configure logs, without signals/services/actions.
                (EVIDENCE / 'bringup_before_probe.log').write_bytes((EVIDENCE / 'bringup.log').read_bytes())
                start('readiness', ['python3', '/opt/auto_tuning/tools/probe_readiness.py',
                                    '/evidence/runtime_probe', '--duration', '30'])
                if selected_profile in phase.CANDIDATE_PROFILES:
                    start('actuation_shadow', phase.shadow_arguments(selected_profile))
                probe_started = True
            state.update(status='OBSERVING' if state['gui_ready'] else 'WAITING_FOR_WINDOWS',
                         updated_at_unix=time.time(), elapsed_wall_seconds=time.monotonic() - started)
            save(EVIDENCE / 'runtime_status.json', state)
            time.sleep(0.5)
        state['status'] = 'STOPPED_BY_REQUEST' if stop_requested else 'DURATION_LIMIT_REACHED'
    except Exception as error:
        code = 2
        state.update(status='FAILED', error=f'{type(error).__name__}: {error}')
    finally:
        for proc in reversed(list(processes.values())):
            if proc.poll() is None:
                for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                    try:
                        os.killpg(proc.pid, sig)
                        proc.wait(timeout=timeout)
                        break
                    except ProcessLookupError:
                        break
                    except subprocess.TimeoutExpired:
                        continue
        for stream in logs:
            stream.close()
        state.update(gui_ready=False, updated_at_unix=time.time(),
                     processes={name: {'pid': proc.pid, 'returncode': proc.poll()}
                                for name, proc in processes.items()})
        save(EVIDENCE / 'runtime_status.json', state)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
