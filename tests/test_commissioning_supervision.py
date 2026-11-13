import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('commissioning_supervision',
    ROOT / 'tools/auto_tuning/commissioning_supervision.py')
safety = importlib.util.module_from_spec(spec)
spec.loader.exec_module(safety)


class SupervisionTests(unittest.TestCase):
    def test_clock_sample_follows_concurrent_file_snapshot(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'heartbeat.json'
            path.write_text('{}')
            order = []
            def read(_path):
                order.append('snapshot')
                return json.dumps({'run_id': 'run', 'seq': 1, 'monotonic_s': 2.0})
            def clock():
                order.append('clock')
                return 2.01
            with patch.object(Path, 'read_text', read), patch.object(safety.time, 'monotonic', clock):
                safety.read_heartbeat(path, safety.HeartbeatWatch('run'))
            self.assertEqual(order, ['snapshot', 'clock'])

    def test_heartbeat_requires_sequence_and_time_progress_not_mtime(self):
        watch = safety.HeartbeatWatch('run', .5)
        watch.check({'run_id': 'run', 'seq': 0, 'monotonic_s': 1}, 1)
        watch.check({'run_id': 'run', 'seq': 0, 'monotonic_s': 1}, 1.1)
        for record in ({'run_id': 'wrong', 'seq': 1, 'monotonic_s': 1.2},
                       {'run_id': 'run', 'seq': 0, 'monotonic_s': 1.2},
                       {'run_id': 'run', 'seq': 1, 'monotonic_s': 1},
                       {'run_id': 'run', 'seq': True, 'monotonic_s': 1.2},
                       {'run_id': 'run', 'seq': -1, 'monotonic_s': 1.2},
                       {'run_id': 'run', 'seq': 1, 'monotonic_s': float('nan')},
                       {'run_id': 'run', 'seq': 1, 'monotonic_s': 2}):
            with self.subTest(record=record), self.assertRaises(ValueError):
                watch.check(record, 1.2)
        with self.assertRaises(ValueError):
            watch.check({'run_id': 'run', 'seq': 0, 'monotonic_s': 1}, 1.5)
        watch.check({'run_id': 'run', 'seq': 1, 'monotonic_s': 1.6}, 1.6)

    def test_lease_rejects_future_stale_nan_large_symlink_and_fifo(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            lease = root / 'lease'
            for value, expected in [('1.5', True), ('1', False), ('3', False), ('nan', False), ('x' * 1024, False)]:
                lease.write_text(value)
                self.assertEqual(safety.read_lease(lease, 2, .75), expected)
            link = root / 'link'
            link.symlink_to(lease)
            self.assertFalse(safety.read_lease(link, 2))
            fifo = root / 'fifo'
            os.mkfifo(fifo)
            self.assertFalse(safety.read_lease(fifo, 2))

    def test_atomic_artifacts_private_and_existing_evidence_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            past = root / 'past.json'
            past.write_text('prior evidence')
            path = root / 'heartbeat.json'
            safety.atomic_private(path, {'seq': 1})
            safety.atomic_private(path, {'seq': 2})
            self.assertEqual(json.loads(path.read_text()), {'seq': 2})
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(path.stat().st_uid, root.stat().st_uid)
            self.assertEqual(past.read_text(), 'prior evidence')
            link = root / 'link'
            link.symlink_to(past)
            with self.assertRaises(ValueError):
                safety.atomic_private(link, {'overwritten': True})
            self.assertEqual(past.read_text(), 'prior evidence')

    def test_kill_requires_exact_container_owner_image_and_network(self):
        state = {'container_id': 'a' * 64, 'image_id': 'sha256:' + 'b' * 64, 'run_id': 'c' * 32}
        info = {'Id': state['container_id'], 'Image': state['image_id'],
            'Config': {'Labels': {'project': 'nav2-phase-a', 'nav2.phase_a.run_id': state['run_id'],
                                  'nav2.phase_a.role': 'commissioning'}},
            'HostConfig': {'NetworkMode': 'none'}}
        safety.verify_container(info, state)
        for key in ('Id', 'Image'):
            bad = copy.deepcopy(info)
            bad[key] = 'other'
            with self.assertRaises(ValueError):
                safety.verify_container(bad, state)
        for key in ('project', 'nav2.phase_a.run_id', 'nav2.phase_a.role'):
            bad = copy.deepcopy(info)
            bad['Config']['Labels'][key] = 'other'
            with self.assertRaises(ValueError):
                safety.verify_container(bad, state)
        bad = copy.deepcopy(info)
        bad['HostConfig']['NetworkMode'] = 'host'
        with self.assertRaises(ValueError):
            safety.verify_container(bad, state)

    def test_authorization_bound_expiring_explicit_and_curves_separate(self):
        manifest = {'run_id': 'a' * 32, 'source': {'commit': 'b' * 40}, 'image_id': 'sha256:' + 'c' * 64,
                    'model_sha256': 'd' * 64, 'protocol_sha256': 'e' * 64}
        approval = {'schema_version': 1, 'scope': 'SIMULATION_COMMISSIONING',
            'motion_authorized': True, 'area_confirmed_clear': True, 'run_id': manifest['run_id'],
            'source_commit': manifest['source']['commit'], 'image_id': manifest['image_id'],
            'model_sha256': manifest['model_sha256'], 'protocol_sha256': manifest['protocol_sha256'],
            'navigation_profile': 'candidate20kmh', 'expires_at_unix': 1100,
            'trial': 'straight', 'operator': 'test fixture', 'authorization_reference': 'synthetic only'}
        safety.verify_authorization(approval, manifest, now_unix=1000)
        for key, value in [('run_id', 'other'), ('motion_authorized', False), ('motion_authorized', 1),
                           ('schema_version', True), ('expires_at_unix', 999), ('expires_at_unix', 5000),
                           ('operator', ''), ('model_sha256', 'other'), ('trial', 'left')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                safety.verify_authorization({**approval, key: value}, manifest, now_unix=1000)
        safety.verify_authorization({**approval, 'trial': 'left', 'straight_trial_review_accepted': True},
                                     manifest, now_unix=1000)

    def test_stalled_independent_process_detected_without_ros_or_docker(self):
        # A real disposable process and SIGSTOP, never a simulator or actuator.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'beat.json'
            script = """import json,os,sys,time
path=sys.argv[1]
seq=0
while True:
 seq+=1
 with open(path+'.tmp','w') as out: json.dump({'run_id':'fixture','seq':seq,'monotonic_s':time.monotonic()},out)
 os.replace(path+'.tmp',path)
 time.sleep(.02)
"""
            child = subprocess.Popen([sys.executable, '-c', script, str(path)])
            try:
                deadline = time.monotonic() + 3
                while not path.exists() and time.monotonic() < deadline:
                    time.sleep(.01)
                watch = safety.HeartbeatWatch('fixture', .2)
                safety.read_heartbeat(path, watch, time.monotonic())
                os.kill(child.pid, signal.SIGSTOP)
                detected = False
                while time.monotonic() < deadline:
                    try:
                        safety.read_heartbeat(path, watch, time.monotonic())
                    except ValueError:
                        detected = True
                        break
                    time.sleep(.01)
                self.assertTrue(detected)
                self.assertIsNone(child.poll())  # Alive does not mean progressing.
            finally:
                child.kill()
                child.wait(timeout=2)


if __name__ == '__main__':
    unittest.main()
