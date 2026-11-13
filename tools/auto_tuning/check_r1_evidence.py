#!/usr/bin/env python3
"""Check saved R1 read-only evidence. Never invokes ROS, Docker or motion."""

import argparse
import json
import math
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import commissioning_supervision as safety


def read(path):
    return json.loads(path.read_text())


def check(root):
    root = Path(root)
    checks = []
    identities = []
    build = read(root / 'build/status.json')
    probe = read(root / 'probe/status.json')
    source, image = build.get('source'), build.get('image_id')
    checks.append({'name': 'build_and_candidate_probe_identity', 'passed': bool(
        build.get('status') == 'IMAGE_VALIDATED' and build.get('stage') == 'build'
        and isinstance(source, dict) and isinstance(image, str) and image.startswith('sha256:')
        and source.get('commit') == (root / 'checkout_commit.txt').read_text().strip()
        and probe.get('status') == 'OBSERVATIONS_COLLECTED' and probe.get('stage') == 'probe'
        and probe.get('navigation_profile') == 'candidate20kmh'
        and probe.get('source') == source and probe.get('image_id') == image)})
    for name, drill in (('prepare-normal', 'none'), ('prepare-adapter', 'adapter'), ('prepare-session', 'session')):
        try:
            directory = root / name
            manifest = read(directory / 'manifest.json')
            report = read(directory / 'commissioning_report.json')
            containment = report.get('containment', {})
            if (report.get('status') != 'PREPARATION_COLLECTED' or report.get('error') is not None
                    or report.get('mode') != 'prepare' or manifest.get('mode') != 'prepare'
                    or manifest.get('fault_drill') != drill or report.get('fault_drill') != drill
                    or report.get('run_id') != manifest.get('run_id')
                    or not isinstance(report.get('run_id'), str)
                    or report.get('source') != source or manifest.get('source') != source
                    or report.get('image_id') != image or manifest.get('image_id') != image
                    or not re.fullmatch('[0-9a-f]{64}', manifest.get('container_id', ''))
                    or report.get('read_only_ready_observed') is not True
                    or type(report.get('actuator_publishers_created')) is not int
                    or report['actuator_publishers_created'] != 0
                    or containment.get('status') != 'OWNED_SIMULATOR_TERMINATED'
                    or containment.get('running') is not False or type(containment.get('pid')) is not int
                    or containment['pid'] != 0
                    or containment.get('container_id') != manifest.get('container_id')):
                raise ValueError('Read-only identity/readiness/containment contract failed')
            if drill == 'none':
                if (report.get('reason') != 'DURATION_LIMIT_REACHED'
                        or report.get('normal_telemetry_confirmed') is not True):
                    raise ValueError('Normal completion not confirmed')
                summary = read(directory / 'session_summary.json')
                if (summary.get('run_id') != manifest['run_id'] or summary.get('mode') != 'prepare'
                        or summary.get('reason') != 'DURATION_LIMIT_REACHED'):
                    raise ValueError('Session summary identity/reason failed')
                confirmation = safety.verify_telemetry_confirmation(summary, manifest, directory)
                if report.get('telemetry_finalization') != confirmation:
                    raise ValueError('Host and session telemetry confirmations differ')
            else:
                delay = report.get('fault_to_host_containment_wall_s')
                if (report.get('fault_drill_triggered') is not True or report.get('fault_drill_passed') is not True
                        or not isinstance(delay, (int, float)) or isinstance(delay, bool)
                        or not math.isfinite(delay) or not 0 <= delay < 3):
                    raise ValueError('Injected failure containment proof failed')
                # A SIGSTOP proof may never complete observer finalization.
            for forbidden in ('authorization.json', 'authorization.used', 'operator_start.json'):
                if (directory / forbidden).exists():
                    raise ValueError('Unexpected runtime authorization/start artifact: ' + forbidden)
            if (directory / 'source').exists():
                raise ValueError('Read-only preparation unexpectedly started a command source')
            identities.append((manifest['model_sha256'], manifest['protocol_sha256']))
            if not all(re.fullmatch('[0-9a-f]{64}', value) for value in identities[-1]):
                raise ValueError('Invalid model/protocol digest')
            checks.append({'name': name, 'passed': True, 'run_id': manifest['run_id']})
        except (OSError, ValueError, KeyError, TypeError) as error:
            checks.append({'name': name, 'passed': False, 'error': f'{type(error).__name__}: {error}'})
    checks.append({'name': 'same_model_and_protocol', 'passed': len(identities) == 3 and
                   len(set(identities)) == 1 and all(all(item) for item in identities)})
    return {'schema_version': 1, 'status': 'R1_READ_ONLY_VALIDATED' if all(c['passed'] for c in checks)
            else 'R1_EVIDENCE_FAILED', 'source': source, 'image_id': image, 'checks': checks,
            'mission_ready': False, 'movement_authorized': False, 'dynamic_validation_complete': False,
            'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = check(args.root)
    except (OSError, ValueError, KeyError, TypeError) as error:
        result = {'schema_version': 1, 'status': 'R1_EVIDENCE_FAILED',
            'checks': [{'name': 'required_build_probe_inputs', 'passed': False,
                        'error': f'{type(error).__name__}: {error}'}],
            'mission_ready': False, 'movement_authorized': False,
            'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['status'] == 'R1_READ_ONLY_VALIDATED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
