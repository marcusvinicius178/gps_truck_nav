import http.client
from http.server import HTTPServer
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import zlib

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools/auto_tuning' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


visual = load('visual_observer')
session = load('visual_observer_session')


def frame(order=0, **changes):
    fields = [100, 7, 2, 24, 2, 1, 0, order, 32, 0, 32, 32, 8, 4,
              0xff0000, 0xff00, 0xff, 8, 256, 0, 2, 1, 0, 0, 0]
    for key, value in changes.items():
        fields[int(key)] = value
    pixels = b'\x00\x00\xff\x00\x00\xff\x00\x00' if order == 0 else b'\x00\xff\x00\x00\x00\x00\xff\x00'
    return struct.pack('>25I', *fields) + pixels


def raw_png(png):
    offset, compressed = 8, b''
    while offset < len(png):
        length, kind = struct.unpack('>I4s', png[offset:offset + 8])
        body = png[offset + 8:offset + 8 + length]
        crc = struct.unpack('>I', png[offset + 8 + length:offset + 12 + length])[0]
        if crc != zlib.crc32(kind + body):
            raise AssertionError('PNG checksum mismatch')
        if kind == b'IDAT':
            compressed += body
        offset += length + 12
    return zlib.decompress(compressed)


class VisualPixelsTests(unittest.TestCase):
    def test_both_endiannesses_preserve_red_green_pixels(self):
        for order in (0, 1):
            self.assertEqual(raw_png(visual.xwd_to_png(frame(order))), b'\0\xff\0\0\0\xff\0')

    def test_window_name_colormap_and_row_padding_are_skipped(self):
        data = frame(**{'0': 104, '12': 12, '19': 1})
        data = data[:100] + b'abc\0' + bytes(12) + data[100:] + bytes(4)
        self.assertEqual(raw_png(visual.xwd_to_png(data)), b'\0\xff\0\0\0\xff\0')

    def test_unknown_layouts_and_excessive_geometry_refused(self):
        for key, value in ((1, 8), (2, 1), (3, 16), (4, 100000), (5, 100000),
                           (6, 1), (7, 4), (11, 24), (12, 4), (13, 5), (14, 0xff), (19, 999999)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                visual.xwd_to_png(frame(**{str(key): value}))

    def test_truncated_or_extra_frame_bytes_refused(self):
        for data in (b'', frame()[:-1], frame() + b'\0'):
            with self.assertRaises(ValueError):
                visual.xwd_to_png(data)

    def test_stale_or_stopped_visual_session_cannot_show_current_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / 'framebuffer').mkdir()
            (p / 'framebuffer/Xvfb_screen0').write_bytes(frame())
            for record in ({'status': 'OBSERVING', 'gui_ready': True, 'updated_at_unix': 90},
                           {'status': 'STOPPED_BY_REQUEST', 'gui_ready': False, 'updated_at_unix': 100},
                           {'status': 'OBSERVING', 'gui_ready': True, 'updated_at_unix': 101}):
                (p / 'runtime_status.json').write_text(json.dumps(record))
                with self.assertRaises(ValueError):
                    visual.read_frame(p, clock=lambda: 100)

    def test_symlink_framebuffer_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            (p / 'framebuffer').mkdir()
            (p / 'real').write_bytes(frame())
            (p / 'framebuffer/Xvfb_screen0').symlink_to(p / 'real')
            (p / 'runtime_status.json').write_text(json.dumps({'status': 'OBSERVING', 'gui_ready': True, 'updated_at_unix': 100}))
            with self.assertRaises(ValueError):
                visual.read_frame(p, clock=lambda: 100)


class VisualHttpTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)
        (self.path / 'framebuffer').mkdir()
        (self.path / 'framebuffer/Xvfb_screen0').write_bytes(frame())
        (self.path / 'runtime_status.json').write_text(json.dumps({
            'status': 'OBSERVING', 'gui_ready': True, 'updated_at_unix': time.time()}))
        self.token = 'a' * 32
        self.server = HTTPServer(('127.0.0.1', 0), visual.handler_for(self.path, self.token, 1))
        self.port = self.server.server_port
        self.server.RequestHandlerClass = visual.handler_for(self.path, self.token, self.port)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join()
        self.directory.cleanup()

    def request(self, method, path, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        connection.request(method, path, headers=headers or {})
        response = connection.getresponse()
        result = response.status, response.read()
        connection.close()
        return result

    def test_page_and_png_are_available_without_input_endpoints(self):
        status, content = self.request('GET', '/' + self.token + '/')
        self.assertEqual(status, 200)
        self.assertIn(b'Sem teclado, mouse, goals', content)
        self.assertNotIn(b'keydown', content)
        self.assertEqual(self.request('GET', '/' + self.token + '/frame.png')[0], 200)
        self.assertFalse((self.path / 'visual_captures').exists())

    def test_write_methods_are_refused(self):
        for method in ('POST', 'PUT', 'PATCH', 'DELETE'):
            self.assertEqual(self.request(method, '/' + self.token + '/frame.png')[0], 405)

    def test_unknown_path_file_traversal_and_wrong_host_refused(self):
        for path in ('/', '/' + self.token + '/status.json', '/' + self.token + '/../status.json', '/wrong/frame.png'):
            self.assertEqual(self.request('GET', path)[0], 404)
        self.assertEqual(self.request('GET', '/' + self.token + '/', {'Host': 'example.org'})[0], 403)

    def test_stopped_gui_returns_unavailable_not_old_image(self):
        (self.path / 'runtime_status.json').write_text(json.dumps({'status': 'FAILED', 'gui_ready': False}))
        self.assertEqual(self.request('GET', '/' + self.token + '/frame.png')[0], 503)

    def test_incomplete_headers_do_not_block_server_indefinitely(self):
        slow = socket.create_connection(('127.0.0.1', self.port), timeout=1)
        try:
            slow.sendall(('GET /' + self.token + '/ HTTP/1.1\r\nHost:').encode())
            # Use the same single-threaded server: the next complete request must
            # progress after the slow connection's bounded timeout.
            connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=4)
            started = time.monotonic()
            connection.request('GET', '/' + self.token + '/')
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            response.read(); connection.close()
            self.assertLess(time.monotonic() - started, 3.5)
        finally:
            slow.close()


class PresentationContractTests(unittest.TestCase):
    def asset_fixture(self, root):
        config = ROOT / 'config/auto_tuning'
        share = root / 'installed/share/truck_bringup'
        # Use the real candidate's complete URIs, with tiny synthetic asset
        # payloads. These test resolution, never claim to render the fixtures.
        source = config / 'plant_candidate/models/arocs_truck_candidate/model.sdf'
        meshes = {node.text for node in ET.parse(source).findall('.//mesh/uri')}
        uris = meshes | {'model://ctvi/model.sdf',
                        'model://ctvi/materials/scripts/MyTerrainMaterial.material',
                        'model://ctvi/materials/textures/ctvi_aerial.png'}
        for uri in uris:
            path = share / 'models' / uri.removeprefix('model://')
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'offline resolution fixture, not renderable data')
        presentation = root / 'presentation_models'
        model = session.prepare_presentation_model(config, presentation, 'model://arocs_truck_candidate')
        return config, share, presentation, Path(model['presentation_model']), uris

    def test_independent_client_gets_installed_mesh_and_resource_paths(self):
        inherited = {'GAZEBO_MODEL_PATH': '/old/models:/old/models:',
                     'GAZEBO_RESOURCE_PATH': '/old/resources:', 'UNRELATED': 'keep'}
        before = inherited.copy()
        environment = session.gazebo_asset_environment(Path('/overlay/share/truck_bringup'),
                                                       Path('/session/presentation_models'),
                                                       Path('/config'), inherited)
        models = environment['GAZEBO_MODEL_PATH'].split(os.pathsep)
        resources = environment['GAZEBO_RESOURCE_PATH'].split(os.pathsep)
        self.assertEqual(models[:2], ['/session/presentation_models', '/overlay/share/truck_bringup/models'])
        self.assertIn('/overlay/share/truck_bringup/worlds', resources)
        self.assertIn('/usr/share/gazebo-11', resources)
        self.assertEqual(models.count('/old/models'), 1)
        self.assertIn('/old/resources', resources)
        self.assertNotIn('', models + resources)
        self.assertEqual(inherited, before)

    def test_previous_gui_paths_reproduce_missing_mesh_and_shared_paths_resolve_all(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, share, presentation, model, uris = self.asset_fixture(Path(tmp))
            previous = os.pathsep.join(map(str, (presentation, config / 'plant_candidate/models', config / 'models')))
            with self.assertRaisesRegex(ValueError, 'cannot resolve visual asset: model://arocs_truck/'):
                session.visual_asset_readback(model, previous)
            environment = session.gazebo_asset_environment(share, presentation, config, {})
            assets = session.visual_asset_readback(model, environment['GAZEBO_MODEL_PATH'])
            self.assertEqual({asset['uri'] for asset in assets}, uris)
            self.assertTrue(all(Path(asset['resolved_file']).is_relative_to(share / 'models') for asset in assets))

    def test_asset_environment_reaches_a_separately_started_client_process(self):
        with tempfile.TemporaryDirectory() as tmp:
            config, share, presentation, model, uris = self.asset_fixture(Path(tmp))
            environment = {**os.environ, **session.gazebo_asset_environment(share, presentation, config, {})}
            code = ('import importlib.util,json,os,sys; '
                    's=importlib.util.spec_from_file_location("session",sys.argv[1]); '
                    'm=importlib.util.module_from_spec(s);s.loader.exec_module(m); '
                    'print(json.dumps(m.visual_asset_readback(sys.argv[2],os.environ["GAZEBO_MODEL_PATH"])))')
            result = subprocess.run([sys.executable, '-c', code,
                str(ROOT / 'tools/auto_tuning/visual_observer_session.py'), str(model)],
                env=environment, check=True, capture_output=True, text=True, timeout=5)
            self.assertEqual({asset['uri'] for asset in json.loads(result.stdout)}, uris)

    def test_missing_or_pointer_mesh_fails_before_presenting_incomplete_truck(self):
        for pointer in (False, True):
            with tempfile.TemporaryDirectory() as tmp:
                config, share, presentation, model, _ = self.asset_fixture(Path(tmp))
                cabin = share / 'models/arocs_truck/meshes/cabine_chassi.dae'
                if pointer:
                    cabin.write_text('version https://git-lfs.github.com/spec/v1\noid sha256:fixture\n')
                else:
                    cabin.unlink()
                environment = session.gazebo_asset_environment(share, presentation, config, {})
                with self.assertRaisesRegex(ValueError, 'unhydrated LFS pointer' if pointer else 'cannot resolve visual asset'):
                    session.visual_asset_readback(model, environment['GAZEBO_MODEL_PATH'])

    def test_private_framebuffer_is_host_readable_without_following_symlinks(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'Xvfb_screen0'
            path.write_bytes(frame()); path.chmod(0o600)
            session.make_framebuffer_readable(path)
            self.assertEqual(path.stat().st_mode & 0o777, 0o644)
            link = Path(tmp) / 'link'; link.symlink_to(path)
            with self.assertRaises(ValueError):
                session.make_framebuffer_readable(link)

    def test_window_arrangement_has_subprocess_deadline(self):
        runner = mock.Mock(return_value=mock.Mock(stdout='{"gazebo": {}, "rviz": {}}'))
        self.assertEqual(set(session.arrange_windows_bounded(runner=runner)), {'gazebo', 'rviz'})
        self.assertEqual(runner.call_args.kwargs['timeout'], 2)
        self.assertIn('--arrange-windows-only', runner.call_args.args[0])
        runner.side_effect = subprocess.TimeoutExpired(['window-helper'], 2)
        with self.assertRaises(subprocess.TimeoutExpired):
            session.arrange_windows_bounded(runner=runner)

    def test_only_camera_content_added_to_world(self):
        source = ROOT / 'config/auto_tuning/worlds/ctvi_tf_sensors.world'
        text, addition = session.observation_world(source)
        self.assertEqual(text.replace(addition, '', 1), source.read_text())
        tree = ET.fromstring(text)
        self.assertIsNotNone(tree.find('world/gui/camera/pose'))
        self.assertEqual(len(tree.findall('world/gui')), 1)

    def test_ambiguous_existing_camera_or_wrong_model_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'world.world'
            source = (ROOT / 'config/auto_tuning/worlds/ctvi_tf_sensors.world').read_text()
            for text in (source.replace('</world>', '<gui/></world>'), source.replace('arocs_truck_tf_sensors', 'another_model')):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    session.observation_world(path)

    def test_candidate_observation_camera_preserves_selected_plant_world(self):
        source = ROOT / 'config/auto_tuning/plant_candidate/worlds/ctvi_plant_candidate.world'
        text, addition = session.observation_world(source, 'model://arocs_truck_candidate')
        self.assertEqual(text.replace(addition, '', 1), source.read_text())
        with self.assertRaises(ValueError):
            session.observation_world(source)  # A candidate cannot masquerade as the prior TF/sensor variant.

    def test_camera_looks_at_body_center_and_follows_actual_spawn_heading(self):
        source = ROOT / 'config/auto_tuning/plant_candidate/worlds/ctvi_plant_candidate.world'
        text, _ = session.observation_world(source, 'model://arocs_truck_candidate')
        tree = ET.fromstring(text)
        pose = [float(value) for value in tree.find("world/include[uri='model://arocs_truck_candidate']/pose").text.split()]
        camera = [float(value) for value in tree.find('world/gui/camera/pose').text.split()]
        dx, dy = camera[0] - pose[0], camera[1] - pose[1]
        # Exact relative front/side offsets, independent of the nonzero yaw.
        self.assertAlmostEqual(math.cos(pose[5]) * dx + math.sin(pose[5]) * dy, 14, places=7)
        self.assertAlmostEqual(-math.sin(pose[5]) * dx + math.cos(pose[5]) * dy, -11, places=7)
        self.assertAlmostEqual(camera[5], math.atan2(-dy, -dx), places=7)
        self.assertAlmostEqual(camera[4], math.atan2(9, math.hypot(dx, dy)), places=7)

    def test_presentation_hides_only_ray_gui_flags_in_both_models(self):
        for relative in ('models/arocs_truck_tf_sensors', 'plant_candidate/models/arocs_truck_candidate'):
            source = ROOT / 'config/auto_tuning' / relative / 'model.sdf'
            original = source.read_text()
            shown, changes = session.sensor_presentation_copy(original)
            before, after = ET.fromstring(original), ET.fromstring(shown)
            self.assertEqual(len(changes), 3)
            self.assertEqual({change['sensor'] for change in changes}, {'left_lidar', 'right_lidar', 'sick_lidar'})
            for sensor in before.findall('.//sensor'):
                if sensor.get('type') == 'ray':
                    sensor.find('visualize').text = '0'
            # This compares every field: enabled state, update rate, ray ranges,
            # noise, ROS plugin, pose, joints, geometry, PID and all other sensors.
            self.assertEqual(ET.tostring(before), ET.tostring(after))
            self.assertEqual(source.read_text(), original)

    def test_session_model_copy_keeps_alias_and_source_config_and_refuses_reuse(self):
        for uri, relative in (
                ('model://arocs_truck_tf_sensors', 'models/arocs_truck_tf_sensors'),
                ('model://arocs_truck_candidate', 'plant_candidate/models/arocs_truck_candidate')):
            with tempfile.TemporaryDirectory() as tmp:
                destination = Path(tmp) / 'presentation_models'
                config = ROOT / 'config/auto_tuning'
                result = session.prepare_presentation_model(config, destination, uri)
                model = destination / uri.removeprefix('model://')
                self.assertEqual((model / 'model.config').read_bytes(), (config / relative / 'model.config').read_bytes())
                self.assertEqual(result['model_uri'], uri)
                self.assertFalse(result['sensor_data_configuration_changed'])
                self.assertNotEqual(result['source_model_sha256'], result['presentation_model_sha256'])
                with self.assertRaises(FileExistsError):
                    session.prepare_presentation_model(config, destination, uri)

    def test_unreviewed_ray_or_missing_visual_flag_refused(self):
        original = (ROOT / 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf').read_text()
        for text in (original.replace("name='left_lidar'", "name='unreviewed_lidar'"),
                     original.replace('<visualize>1</visualize>', '', 1)):
            with self.assertRaises(ValueError):
                session.sensor_presentation_copy(text)

    def test_rviz_has_displays_and_no_action_tools_or_panels(self):
        config = yaml.safe_load((ROOT / 'config/auto_tuning/observation.rviz').read_text())
        self.assertEqual(config['Visualization Manager']['Tools'], [])
        self.assertEqual([p['Class'] for p in config['Panels']], ['rviz_common/Displays'])
        displays = config['Visualization Manager']['Displays']
        classes = {d['Class'] for d in displays}
        self.assertTrue({'rviz_default_plugins/RobotModel', 'rviz_default_plugins/TF',
                         'rviz_default_plugins/Map', 'rviz_default_plugins/LaserScan',
                         'rviz_default_plugins/Polygon', 'rviz_default_plugins/Path'} <= classes)
        topics = {d.get('Topic', {}).get('Value'): d['Class'] for d in displays}
        self.assertEqual(topics['/transformed_global_plan'], 'rviz_default_plugins/Path')
        self.assertEqual(topics['/trajectories'], 'rviz_default_plugins/MarkerArray')
        self.assertNotIn('/local_plan', topics)

    def test_rviz_truck_framing_and_overlays_are_legible_without_new_control_tools(self):
        config = yaml.safe_load((ROOT / 'config/auto_tuning/observation.rviz').read_text())
        manager = config['Visualization Manager']
        self.assertEqual(manager['Views']['Current']['Target Frame'], 'base_footprint')
        self.assertEqual(manager['Views']['Current']['Distance'], 20)
        displays = {d['Name']: d for d in manager['Displays']}
        self.assertLess(displays['Global rolling costmap']['Alpha'], displays['Local rolling costmap']['Alpha'])
        tf = displays['Navigation and sensor frames']
        self.assertFalse(tf['Show Arrows'])
        self.assertFalse(tf['Show Names'])
        self.assertTrue(tf['Frames']['rear_axle_nav']['Value'])
        self.assertTrue(displays['Truck footprint']['Enabled'])
        self.assertTrue(displays['Sick scan']['Enabled'])
        self.assertFalse(displays['Global pose']['Enabled'])
        self.assertFalse(displays['MPPI trajectories (conditional on visualize)']['Enabled'])
        self.assertEqual(manager['Tools'], [])


class VisualDockerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)
        self.repo = self.path / 'repo'; self.repo.mkdir()
        self.build = self.path / 'build'; self.build.mkdir()
        self.probe = self.path / 'probe'; self.probe.mkdir()
        self.output = self.path / 'visual'
        self.source = {'commit': 'f' * 40, 'tree': 'a' * 40, 'input_sha256': {}}
        self.image = 'sha256:' + 'b' * 64
        self.cid = 'c' * 64
        self.commands = []
        self.write_records()

    def tearDown(self):
        self.directory.cleanup()

    def write_records(self):
        (self.build / 'status.json').write_text(json.dumps({'stage': 'build', 'status': 'IMAGE_VALIDATED',
            'source': self.source, 'image_id': self.image}))
        (self.build / 'image_id.txt').write_text(self.image)
        (self.probe / 'status.json').write_text(json.dumps({'stage': 'probe', 'status': 'OBSERVATIONS_COLLECTED',
            'readiness_collection_status': 'COMPLETE', 'source': self.source, 'image_id': self.image}))

    def fake_runner(self, output):
        outer = self
        class Runner:
            def __init__(self): self.output = output
            def run(self, name, command, **_kwargs):
                outer.commands.append((name, command))
                if name == 'visual_create': (output / 'container.id').write_text(outer.cid)
                if name == 'visual_start':
                    (output / 'runtime_status.json').write_text(json.dumps({'status': 'OBSERVING', 'gui_ready': True}))
                return ''
        return Runner()

    def patches(self):
        return mock.patch.multiple(visual.phase,
            Runner=mock.Mock(side_effect=self.fake_runner), docker_prefix=mock.Mock(return_value=['docker', '--host', visual.phase.ENDPOINT]),
            source_snapshot=mock.Mock(return_value=self.source), image_inspection=mock.Mock(return_value={'Id': self.image}),
            readiness_result=mock.Mock(return_value=({}, 'hash')), cleanup_container=mock.Mock(return_value={'status': 'OWNED_CONTAINER_STOPPED'}))

    def test_visual_container_keeps_network_none_and_no_host_display_or_ports(self):
        with self.patches(), mock.patch.object(visual, 'read_frame', return_value=b'png'):
            result = visual.start(self.repo, self.build, self.probe, self.output, 900)
        command = next(cmd for name, cmd in self.commands if name == 'visual_create')
        self.assertEqual(command[command.index('--network') + 1], 'none')
        for forbidden in ('--publish', '--privileged', '--gpus', '--pid', '--ipc'):
            self.assertNotIn(forbidden, command)
        self.assertIn('NAV2_PHASE_A_PROFILE=canonical', command)
        self.assertEqual(command[command.index('--mount') + 1], f'type=bind,src={self.output},dst=/evidence')
        self.assertIn(self.image, command)
        self.assertEqual(result['status'], 'VISUAL_OBSERVATION_RUNNING')
        self.assertFalse(result['mission_ready'])

    def test_different_source_or_image_cannot_open_visual_session(self):
        record = json.loads((self.probe / 'status.json').read_text())
        record['image_id'] = 'sha256:' + 'd' * 64
        (self.probe / 'status.json').write_text(json.dumps(record))
        with self.patches(), self.assertRaises(visual.phase.PhaseError):
            visual.start(self.repo, self.build, self.probe, self.output, 900)
        self.assertFalse(any(name == 'visual_create' for name, _ in self.commands))

    def test_visual_reuses_candidate_selection_from_probe(self):
        record = json.loads((self.probe / 'status.json').read_text())
        record['navigation_profile'] = 'candidate25kmh'
        (self.probe / 'status.json').write_text(json.dumps(record))
        with self.patches(), mock.patch.object(visual, 'read_frame', return_value=b'png'):
            result = visual.start(self.repo, self.build, self.probe, self.output, 900)
        command = next(cmd for name, cmd in self.commands if name == 'visual_create')
        self.assertIn('NAV2_PHASE_A_PROFILE=candidate25kmh', command)
        self.assertNotIn('NAV2_PHASE_A_PROFILE=canonical', command)
        self.assertEqual(result['navigation_profile'], 'candidate25kmh')

    def test_unknown_probe_profile_cannot_silently_open_canonical_visual(self):
        record = json.loads((self.probe / 'status.json').read_text())
        record['navigation_profile'] = 'other'
        (self.probe / 'status.json').write_text(json.dumps(record))
        with self.patches(), self.assertRaises(visual.phase.PhaseError):
            visual.start(self.repo, self.build, self.probe, self.output, 900)
        self.assertFalse(any(name == 'visual_create' for name, _ in self.commands))

    def test_existing_output_and_unbounded_duration_refused(self):
        for duration in (0, 59, 1801):
            with self.assertRaises(visual.phase.PhaseError):
                visual.start(self.repo, self.build, self.probe, self.output, duration)
        self.output.mkdir()
        (self.output / 'earlier.json').write_text('previous evidence')
        with self.assertRaises(visual.phase.PhaseError):
            visual.start(self.repo, self.build, self.probe, self.output, 900)
        self.assertEqual((self.output / 'earlier.json').read_text(), 'previous evidence')

    def test_ownership_mismatch_refuses_status_and_cleanup(self):
        self.output.mkdir()
        (self.output / 'container.id').write_text(self.cid)
        state = {'run_id': 'e' * 32, 'image_id': self.image}
        info = {'Id': self.cid, 'Image': self.image, 'Config': {'Labels': {'project': 'another-project'}},
                'HostConfig': {'NetworkMode': 'none'}}
        with mock.patch.object(visual.phase, 'inspect_container', return_value=info), self.assertRaises(visual.phase.PhaseError):
            visual.verify_owned(self.output, object(), ['docker'], state)


if __name__ == '__main__':
    unittest.main()
