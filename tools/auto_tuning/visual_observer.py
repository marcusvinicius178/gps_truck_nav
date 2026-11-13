#!/usr/bin/env python3
"""Observe private Gazebo/RViz pixels in a localhost browser; no input channel.

Host dependencies: Python stdlib, Git/LFS and the existing local Docker daemon.
No ROS/X11 installation, host-display mount, network port or GUI control reaches
the container. Build/probe must already have completed for this exact source.
"""

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import struct
import tempfile
import time
from urllib.parse import urlsplit
import uuid
import zlib


_spec = importlib.util.spec_from_file_location('phase_a_visual_host_helpers',
                                             Path(__file__).with_name('local_phase_a.py'))
phase = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(phase)
MAX_XWD_BYTES = 40 * 1024 * 1024


def xwd_to_png(data):
    """Decode Xvfb's bounded 24-bit TrueColor ZPixmap; refuse other layouts."""
    if not 100 <= len(data) <= MAX_XWD_BYTES:
        raise ValueError('XWD size outside the observation bounds')
    fields = struct.unpack('>25I', data[:100])
    (header, version, format_, depth, width, height, xoffset, byte_order, bitmap_unit,
     bitmap_bit_order, bitmap_pad, bits_per_pixel, stride, visual_class,
     red, green, blue, bits_per_rgb, colormap_entries, ncolors,
     window_width, window_height, window_x, window_y, border) = fields
    if (version != 7 or format_ != 2 or depth != 24 or bits_per_pixel != 32
            or visual_class != 4 or xoffset != 0 or byte_order not in (0, 1)
            or (red, green, blue) != (0xff0000, 0xff00, 0xff)):
        raise ValueError('Unsupported XWD format; no unverified pixel reinterpretation')
    if not (0 < width <= 4096 and 0 < height <= 2160 and 100 <= header <= 65536
            and 0 <= ncolors <= 65536 and width * 4 <= stride <= 4096 * 4):
        raise ValueError('Invalid or excessive XWD geometry')
    offset = header + ncolors * 12
    expected = offset + stride * height
    if expected != len(data):
        raise ValueError('Incomplete/inconsistent XWD frame')
    output = bytearray((width * 3 + 1) * height)
    for y in range(height):
        row = data[offset + y * stride:offset + y * stride + width * 4]
        rgb = bytearray(width * 3)
        if byte_order == 0:
            rgb[0::3], rgb[1::3], rgb[2::3] = row[2::4], row[1::4], row[0::4]
        else:
            rgb[0::3], rgb[1::3], rgb[2::3] = row[1::4], row[2::4], row[3::4]
        begin = y * (width * 3 + 1)
        output[begin + 1:begin + 1 + width * 3] = rgb
    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind + body))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>2I5B', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(output, 1)) + chunk(b'IEND', b''))


def read_frame(session, *, clock=time.time):
    state = json.loads((session / 'runtime_status.json').read_text())
    age = clock() - state.get('updated_at_unix', 0)
    if state.get('status') != 'OBSERVING' or state.get('gui_ready') is not True or not 0 <= age <= 5:
        raise ValueError('Visual session is not live; refusing to present an old frame as current')
    path = session / 'framebuffer/Xvfb_screen0'
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_XWD_BYTES:
        raise ValueError('Invalid private framebuffer file')
    return xwd_to_png(path.read_bytes())


def handler_for(session, token, port, *, capture_frames=0):
    if not re.fullmatch('[0-9a-f]{32}', token) or not 0 <= capture_frames <= 10:
        raise ValueError('Invalid observation HTTP configuration')
    prefix = '/' + token
    cache = {'at': 0.0, 'png': None, 'captures': 0}
    html = ('''<!doctype html><html lang="pt-BR"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Arocs — observação</title><style>body{margin:0;background:#14212b;color:#edf5fa;font:16px sans-serif}
header{padding:12px 18px}img{width:100%;height:auto}small{display:block;color:#c4d3df}</style>
<header>Gazebo Classic + RViz — somente observação
<small>Atualização aproximada de 1 Hz. Sem teclado, mouse, goals ou comandos de movimento.
As trajetórias ficam vazias enquanto não houver planejamento autorizado.</small>
<span id="state">Carregando imagem...</span></header><img id="frame" alt="Gazebo e RViz">
<script>const f=document.getElementById('frame'),s=document.getElementById('state');
function refresh(){f.src='__FRAME_PATH__?t='+Date.now()}
f.onload=()=>{s.textContent='Imagem recebida às '+new Date().toLocaleTimeString();setTimeout(refresh,1000)};
f.onerror=()=>{s.textContent='Imagem indisponível: confira status/logs da sessão.';setTimeout(refresh,2000)};
refresh();</script></html>''').replace('__FRAME_PATH__', prefix + '/frame.png').encode()

    class Handler(BaseHTTPRequestHandler):
        timeout = 2  # Bound slow/incomplete HTTP headers and writes, not only accept().

        def log_message(self, _format, *_args):
            pass

        def reply(self, status, content, mime='text/plain; charset=utf-8'):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'none'; img-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if self.headers.get('Host') not in (f'127.0.0.1:{port}', f'localhost:{port}'):
                self.reply(403, b'Local observation endpoint only')
                return
            path = urlsplit(self.path).path
            if path in (prefix, prefix + '/'):
                self.reply(200, html, 'text/html; charset=utf-8')
            elif path == prefix + '/frame.png':
                try:
                    current = time.monotonic()
                    if cache['png'] is None or current - cache['at'] >= 1:
                        cache.update(png=read_frame(session), at=current)
                        if cache['captures'] < capture_frames:
                            folder = session / 'visual_captures'
                            folder.mkdir(exist_ok=True)
                            name = uuid.uuid4().hex
                            with (folder / (name + '.png')).open('xb') as out:
                                out.write(cache['png'])
                            phase.save_json(folder / (name + '.json'), {
                                'captured_at_utc': phase.now(), 'sha256': hashlib.sha256(cache['png']).hexdigest(),
                                'scope': 'GUI_PIXELS_ONLY_NOT_SYNCHRONIZED_ROS_METRICS'})
                            cache['captures'] += 1
                    self.reply(200, cache['png'], 'image/png')
                except (OSError, ValueError, KeyError) as error:
                    self.reply(503, ('Frame unavailable: ' + str(error)).encode())
            else:
                self.reply(404, b'Not found')

        def do_POST(self):
            self.reply(405, b'Observation is GET-only; no command endpoint exists')

        do_PUT = do_POST
        do_PATCH = do_POST
        do_DELETE = do_POST

    return Handler


def session_state(session):
    path = session / 'status.json'
    if path.is_symlink():
        raise phase.PhaseError('Session record must be a regular file')
    state = json.loads(path.read_text())
    if (state.get('stage') != 'visual_observation' or not re.fullmatch('[0-9a-f]{32}', state.get('run_id', ''))
            or not phase.IMAGE_RE.fullmatch(state.get('image_id', ''))):
        raise phase.PhaseError('Not an identified Phase A visual session')
    return state


def operation_runner(session, name):
    directory = session / 'operations'
    directory.mkdir(exist_ok=True)
    return phase.Runner(Path(tempfile.mkdtemp(prefix=name + '.', dir=directory)))


def verify_owned(session, runner, docker, state):
    cid = phase.owned_id(session / 'container.id')
    if cid is None:
        raise phase.PhaseError('Visual session has no container identity')
    info = phase.inspect_container(runner, docker, cid, 'visual_inspect')
    labels = info.get('Config', {}).get('Labels', {}) or {}
    if (info.get('Id') != cid or info.get('Image') != state['image_id'] or labels.get('project') != 'nav2-phase-a'
            or labels.get(phase.LABEL) != state['run_id'] or labels.get('nav2.phase_a.role') != 'visual_observation'
            or info.get('HostConfig', {}).get('NetworkMode') != 'none'):
        raise phase.PhaseError('Visual session ownership/image/isolation mismatch')
    return cid, info


def stop(session):
    state = session_state(session)
    runner, docker = operation_runner(session, 'stop'), phase.docker_prefix()
    verify_owned(session, runner, docker, state)
    state['cleanup'] = phase.cleanup_container(runner, docker, session / 'container.id', state['run_id'], 'visual')
    state.update(status='STOPPED', finished_at_utc=phase.now())
    phase.save_json(session / 'status.json', state)
    return state


def start(repo, build_dir, probe_dir, output, duration, *, sleep=time.sleep, monotonic=time.monotonic):
    if not 60 <= duration <= 1800:
        raise phase.PhaseError('Visual duration must be 60..1800 wall seconds')
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to(repo):
        raise phase.PhaseError('Use a new output directory outside the checkout')
    if any(c in str(output) for c in (',', '\r', '\n')):
        raise phase.PhaseError('Output path cannot contain commas or line breaks')
    output.mkdir(mode=0o700)
    runner, docker = phase.Runner(output), None
    state = {'schema_version': 1, 'stage': 'visual_observation', 'status': 'STARTING',
             'run_id': uuid.uuid4().hex, 'started_at_utc': phase.now(), 'duration_wall_seconds': duration,
             'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    phase.save_json(output / 'status.json', state)
    try:
        docker = phase.docker_prefix()
        build = json.loads((build_dir / 'status.json').read_text())
        probe = json.loads((probe_dir / 'status.json').read_text())
        if build.get('stage') != 'build' or build.get('status') != 'IMAGE_VALIDATED':
            raise phase.PhaseError('Visual start requires IMAGE_VALIDATED')
        if (probe.get('stage') != 'probe' or probe.get('status') != 'OBSERVATIONS_COLLECTED'
                or probe.get('readiness_collection_status') == 'FAILED'):
            raise phase.PhaseError('Complete the diagnostic probe before opening the visual session')
        source = phase.source_snapshot(repo, runner)
        image = build.get('image_id', '')
        if (source != build.get('source') or source != probe.get('source') or probe.get('image_id') != image
                or (build_dir / 'image_id.txt').read_text().strip() != image):
            raise phase.PhaseError('Source/image must match both the validated build and completed probe')
        phase.readiness_result(probe_dir)
        selected_profile = probe.get('navigation_profile', 'canonical')
        phase.navigation_arguments(selected_profile)  # Refuse unknown profile before creating a container.
        state.update(source=source, image_id=image, build_directory=str(build_dir), probe_directory=str(probe_dir),
                     navigation_profile=selected_profile,
                     build_status_sha256=phase.file_hash(build_dir / 'status.json'),
                     probe_status_sha256=phase.file_hash(probe_dir / 'status.json'))
        phase.save_json(output / 'source_manifest.json', source)
        phase.save_json(output / 'image_inspect.json', phase.image_inspection(runner, docker, image, 'image_inspect'))
        phase.save_json(output / 'status.json', state)
        command = (docker + phase.container_options(output, state['run_id'], 'visual')
                   + ['--label', 'nav2.phase_a.role=visual_observation', '--env', 'NAV2_PHASE_A_PROFILE=' + selected_profile,
                      image, 'python3', '/opt/auto_tuning/tools/visual_observer_session.py', '--duration', str(duration)])
        runner.run('visual_create', command, capture=False)
        cid = phase.owned_id(output / 'container.id')
        if cid is None:
            raise phase.PhaseError('Visual container identity not recorded')
        runner.run('visual_start', docker + ['start', cid], capture=False)
        deadline = monotonic() + 60
        for index in range(60):
            runtime = output / 'runtime_status.json'
            if runtime.is_file():
                record = json.loads(runtime.read_text())
                if record.get('status') == 'FAILED':
                    raise phase.PhaseError(record.get('error', 'Visual process failed'))
                if record.get('gui_ready'):
                    read_frame(output)
                    break
            info = phase.inspect_container(runner, docker, cid, f'visual_wait_{index:02d}')
            if info.get('State', {}).get('Running') is not True or monotonic() >= deadline:
                raise phase.PhaseError('Visual apps did not become observable')
            sleep(1)
        else:
            raise phase.PhaseError('Gazebo and RViz windows not observed within 60 seconds')
        state.update(status='VISUAL_OBSERVATION_RUNNING', observation_only=True,
                     limitation='GUI pixels only. No input channel. Visual rendering and ROS evidence require host review.')
        phase.save_json(output / 'status.json', state)
        return state
    except BaseException as error:
        state.update(status='FAILED', error=f'{type(error).__name__}: {error}')
        try:
            state['cleanup'] = (phase.cleanup_container(runner, docker, output / 'container.id', state['run_id'], 'visual')
                                if docker is not None else {'status': 'NO_CONTAINER_ID_CREATED'})
        except Exception as cleanup_error:
            state['cleanup_error'] = str(cleanup_error)
        phase.save_json(output / 'status.json', state)
        raise


def serve(session, port, capture_frames):
    state = session_state(session)
    runner, docker = operation_runner(session, 'serve'), phase.docker_prefix()
    _, info = verify_owned(session, runner, docker, state)
    if info.get('State', {}).get('Running') is not True:
        raise phase.PhaseError('Visual container has stopped')
    server = None
    try:
        read_frame(session)
        token = uuid.uuid4().hex
        server = HTTPServer(('127.0.0.1', port), handler_for(session, token, port, capture_frames=capture_frames))
        server.timeout = 1
        print(f'Open on ipt-sim-01: http://127.0.0.1:{port}/{token}/', flush=True)
        print('Ctrl-C closes this viewer and stops only its owned visual container.', flush=True)
        deadline = time.monotonic() + 1800
        while time.monotonic() < deadline:
            server.handle_request()
            record = json.loads((session / 'runtime_status.json').read_text())
            if record.get('status') in ('STOPPED_BY_REQUEST', 'DURATION_LIMIT_REACHED', 'FAILED'):
                break
    finally:
        if server is not None:
            server.server_close()
        stop(session)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    p = sub.add_parser('start')
    for flag in ('repo', 'build-dir', 'probe-dir', 'output-dir'):
        p.add_argument('--' + flag, type=Path, required=True)
    p.add_argument('--duration', type=int, default=900)
    for name in ('status', 'stop', 'serve'):
        p = sub.add_parser(name)
        p.add_argument('--session-dir', type=Path, required=True)
        if name == 'serve':
            p.add_argument('--port', type=int, default=8765)
            p.add_argument('--capture-frames', type=int, default=0)
    args = parser.parse_args(argv)
    try:
        if args.operation == 'start':
            result = start(args.repo.resolve(), args.build_dir.resolve(), args.probe_dir.resolve(),
                           args.output_dir.absolute(), args.duration)
        elif args.operation == 'stop':
            result = stop(args.session_dir.resolve(strict=True))
        elif args.operation == 'status':
            session = args.session_dir.resolve(strict=True)
            result = session_state(session)
            _, info = verify_owned(session, operation_runner(session, 'status'), phase.docker_prefix(), result)
            result = {**result, 'container_state': info.get('State'),
                      'runtime': json.loads((session / 'runtime_status.json').read_text())
                      if (session / 'runtime_status.json').is_file() else None}
            if info.get('State', {}).get('Running') is not True:
                result['status'] = 'STOPPED'
            elif (result['runtime'] or {}).get('status') != 'OBSERVING':
                result['status'] = 'VISUAL_NOT_READY'
        else:
            if not 1024 <= args.port <= 65535 or not 0 <= args.capture_frames <= 10:
                parser.error('port must be 1024..65535; capture-frames must be 0..10')
            serve(args.session_dir.resolve(strict=True), args.port, args.capture_frames)
            return 0
        print(json.dumps(result, indent=2))
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, KeyError, phase.PhaseError) as error:
        print(json.dumps({'status': 'FAILED', 'error': str(error), 'mission_ready': False}))
        return 2


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, phase.interrupted)
    raise SystemExit(main())
