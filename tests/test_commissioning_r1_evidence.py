"""Saved synthetic evidence only. No operational authorization, ROS or Docker."""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/auto_tuning'))
import check_r1_evidence as checker
import commissioning_supervision as safety
from test_commissioning_finalization import summary_fixture


class R1EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        source, image = {'commit': 'd' * 40}, 'sha256:' + 'c' * 64
        (self.root / 'checkout_commit.txt').write_text(source['commit'] + '\n')
        self.write('build/status.json', {'stage': 'build', 'status': 'IMAGE_VALIDATED',
                                       'source': source, 'image_id': image})
        self.write('probe/status.json', {'stage': 'probe', 'status': 'OBSERVATIONS_COLLECTED',
            'navigation_profile': 'candidate20kmh', 'source': source, 'image_id': image})
        for index, (name, drill) in enumerate((('prepare-normal', 'none'), ('prepare-adapter', 'adapter'),
                                               ('prepare-session', 'session'))):
            manifest = {'mode': 'prepare', 'trial': 'straight', 'fault_drill': drill,
                'run_id': str(index) * 32, 'source': source, 'image_id': image,
                'container_id': str(index) * 64, 'model_sha256': 'f' * 64, 'protocol_sha256': 'e' * 64}
            self.write(name + '/manifest.json', manifest)
            report = {**manifest, 'status': 'PREPARATION_COLLECTED', 'error': None,
                'read_only_ready_observed': True, 'actuator_publishers_created': 0,
                'containment': {'status': 'OWNED_SIMULATOR_TERMINATED', 'running': False,
                                'pid': 0, 'container_id': manifest['container_id']}}
            if drill == 'none':
                self.write(name + '/observer/telemetry_summary.json', summary_fixture(manifest['run_id']))
                confirmation = safety.telemetry_confirmation(self.root / name, manifest, 0, False)
                summary = {'run_id': manifest['run_id'], 'mode': 'prepare',
                    'reason': 'DURATION_LIMIT_REACHED', 'telemetry_finalization': confirmation}
                self.write(name + '/session_summary.json', summary)
                report.update(reason='DURATION_LIMIT_REACHED', normal_telemetry_confirmed=True,
                              telemetry_finalization=confirmation)
            else:
                report.update(fault_drill_triggered=True, fault_drill_passed=True,
                              fault_to_host_containment_wall_s=.9)
            self.write(name + '/commissioning_report.json', report)

    def write(self, relative, value):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def test_accepts_normal_finalization_and_two_proofs_without_fault_summaries(self):
        result = checker.check(self.root)
        self.assertEqual(result['status'], 'R1_READ_ONLY_VALIDATED')
        self.assertFalse(result['movement_authorized'])
        self.assertEqual(result['gate0'], 'NOT_YET_VERIFIED')

    def test_missing_normal_summary_prevents_r1_validation(self):
        (self.root / 'prepare-normal/observer/telemetry_summary.json').unlink()
        self.assertEqual(checker.check(self.root)['status'], 'R1_EVIDENCE_FAILED')

    def test_cross_image_and_model_mismatch_fail(self):
        path = self.root / 'prepare-adapter/manifest.json'
        manifest = json.loads(path.read_text())
        for field in ('image_id', 'model_sha256'):
            with self.subTest(field=field):
                self.write(str(path.relative_to(self.root)), {**manifest, field: 'other'})
                self.assertEqual(checker.check(self.root)['status'], 'R1_EVIDENCE_FAILED')

    def test_missing_inputs_produce_machine_readable_failure_and_preserve_previous_output(self):
        (self.root / 'build/status.json').unlink()
        output = self.root / 'result.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(checker.main(['--root', str(self.root), '--output', str(output)]), 2)
        self.assertEqual(json.loads(output.read_text())['status'], 'R1_EVIDENCE_FAILED')
        with self.assertRaises(FileExistsError):
            checker.main(['--root', str(self.root), '--output', str(output)])


if __name__ == '__main__':
    unittest.main()
