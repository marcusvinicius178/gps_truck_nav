#!/usr/bin/env python3
"""Prepare/verify complete prospective speed profiles without ROS or motion.

The 20/25 km/h values are experimental forward-speed ceilings, not demonstrated
cruise speeds or physical safety limits. The historical YAML remains immutable.
Profiles may be loaded only by the no-goal diagnostic while actuation is blocked.
"""

import argparse
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import sys

import yaml

from prepare_candidates import CandidateError, differences, load_source


SOURCE_COMMIT = 'a21403fcc67e027e29010d29f6339a580dfb7076'
SOURCE_PATH = 'src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml'
SOURCE_SHA256 = '722169c640ac8a37c10ca1097ff212366bac7ae58ac2f596608bfc7a9454eef7'
SPEEDS_KMH = {'20kmh': 20, '25kmh': 25}
ALLOWED_PATHS = (
    ('controller_server', 'ros__parameters', 'FollowPath', 'vx_max'),
    ('velocity_smoother', 'ros__parameters', 'max_velocity', '0'),
    ('velocity_smoother', 'ros__parameters', 'max_velocity', '2'),
    ('velocity_smoother', 'ros__parameters', 'min_velocity', '0'),
    ('velocity_smoother', 'ros__parameters', 'min_velocity', '2'),
)


class ProfileError(ValueError):
    """The immutable source or generated profile contract was violated."""


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def value_at(tree, path):
    for key in path:
        tree = tree[int(key)] if isinstance(tree, list) else tree[key]
    return tree


def expected_runtime_parameters(config):
    """Exact ROS readback requirements; these are declarations, not motion proof."""
    controller = config['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    planner = config['planner_server']['ros__parameters']['GridBased']
    smoother = config['velocity_smoother']['ros__parameters']
    return {
        '/controller_server': {
            'controller_frequency': controller['controller_frequency'],
            **{'FollowPath.' + key: follow[key] for key in (
                'plugin', 'motion_model', 'vx_max', 'vx_min', 'vy_max', 'wz_max',
                'model_dt', 'time_steps', 'prune_distance')},
            'FollowPath.AckermannConstraints.min_turning_r': follow['AckermannConstraints']['min_turning_r'],
        },
        '/planner_server': {'GridBased.' + key: planner[key] for key in (
            'plugin', 'motion_model_for_search', 'minimum_turning_radius')},
        '/velocity_smoother': {key: smoother[key] for key in (
            'max_velocity', 'min_velocity', 'max_accel', 'max_decel',
            'scale_velocities', 'feedback', 'smoothing_frequency', 'velocity_timeout')},
    }


def build_bundle(source_raw):
    """Return deterministic complete files with the five permitted leaf changes."""
    if sha256(source_raw) != SOURCE_SHA256:
        raise ProfileError('Canonical source hash differs from the reviewed snapshot; refuse implicit rebasing')
    baseline = load_source(source_raw)
    outputs, profiles = {}, {}
    for profile_id, kmh in SPEEDS_KMH.items():
        config = copy.deepcopy(baseline)
        follow = config['controller_server']['ros__parameters']['FollowPath']
        smoother = config['velocity_smoother']['ros__parameters']
        exact_cap = Fraction(kmh * 5, 18)
        cap = float(exact_cap)
        follow['vx_max'] = cap
        # Reuse the existing MPPI reverse/yaw caps, rather than inventing new
        # mechanical capabilities or silently promoting the historical smoother.
        smoother['max_velocity'] = [cap, 0.0, follow['wz_max']]
        smoother['min_velocity'] = [follow['vx_min'], 0.0, -follow['wz_max']]
        changed = differences(baseline, config)
        if set(changed) != set(ALLOWED_PATHS) or len(changed) != len(ALLOWED_PATHS):
            raise ProfileError('A profile changed parameters outside the explicit speed-envelope contract')
        text = ('# Prospective diagnostic profile; NOT approved for actuation.\n'
                '# Complete copy of the reviewed YAML values with five documented leaf changes.\n'
                '# Historical geometry, radius, noise and other parameters remain unchanged.\n'
                + yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
        raw = text.encode('utf-8')
        if differences(config, load_source(raw)):
            raise ProfileError('Serialization changed values or ROS scalar types')
        filename = profile_id + '.yaml'
        outputs[filename] = raw
        horizon = follow['model_dt'] * follow['time_steps']
        profiles[profile_id] = {
            'file': filename,
            'sha256': sha256(raw),
            'requested_test_speed_kmh': kmh,
            'forward_speed_cap_m_s': cap,
            'exact_speed_fraction_m_s': {'numerator': exact_cap.numerator, 'denominator': exact_cap.denominator},
            'speed_semantics': 'EXPERIMENTAL_FORWARD_CEILING_NOT_GUARANTEED_CRUISE',
            'reverse_speed_cap_m_s': abs(follow['vx_min']),
            'yaw_rate_cap_rad_s': follow['wz_max'],
            'model_horizon_s': horizon,
            'straight_horizon_at_cap_m': cap * horizon,
            'radius_at_speed_and_yaw_caps_m': cap / follow['wz_max'],
            'radius_note': 'Kinematic quotient at both caps only; not a minimum physical turning radius.',
            'changes': [{'yaml_path': list(path), 'before': value_at(baseline, path),
                         'after': value_at(config, path)} for path in changed],
            'expected_parameters': expected_runtime_parameters(config),
            'runtime_values_verified': False,
            'actuation_authorized': False,
        }
    manifest = {
        'schema_version': 1,
        'profile_family': 'prospective_speed_envelopes_v1',
        'status': 'PREPARED_DIAGNOSTIC_ONLY',
        'readiness': 'BLOCKED_ACTUATION_REVIEW',
        'source': {'commit': SOURCE_COMMIT, 'path': SOURCE_PATH, 'sha256': SOURCE_SHA256},
        'profiles': profiles,
        'preserved': ['historical YAML bytes', 'all planner/MPPI weights', 'DUBIN and Ackermann models',
                      'planner and MPPI radius 11.6 m (unresolved)', 'sensor noise/covariances and datum',
                      'plant geometry and steering limits', 'all costmap/BT parameters',
                      'existing acceleration/deceleration and timeout settings (not physically validated)'],
        'blocked_reasons': [
            '11.6 m planned radius is unreconciled with the static ideal joint-limit bound near 13.09 m',
            'Actuation conversion, rear-axle/base reference, authority and destination watchdog are not validated',
            'Lowering forward caps alone does not prevent impossible curvature at lower speed',
            'Historical smoother scales components independently; runtime joint-constraint enforcement is still required',
            'Behavior and other command producers may bypass the smoother in the historical command graph',
            'Speed ceilings and inherited acceleration limits are experimental proposals, not vehicle safety ratings',
            'Localization, obstacle detection and review authorization remain prerequisites for movement',
        ],
        'allowed_use': 'No-goal diagnostic parameter verification only; never a motion authorization',
        'motion_authorized': False,
        'gate0': 'NOT_YET_VERIFIED',
        'gate1': 'NOT_YET_VERIFIED',
    }
    outputs['manifest.json'] = (json.dumps(manifest, indent=2, ensure_ascii=False) + '\n').encode('utf-8')
    return outputs, manifest


def prepare_profiles(source, output_dir):
    """Create a new directory only; no edits to source or previous evidence."""
    source, output = Path(source), Path(output_dir)
    if output.exists() or output.is_symlink():
        raise ProfileError('Output already exists; previous profiles/evidence must be preserved')
    raw = source.read_bytes()
    outputs, manifest = build_bundle(raw)
    if source.read_bytes() != raw:
        raise ProfileError('Source changed while preparing profiles')
    output.mkdir()  # Parent must already exist; never create an unintended tree.
    for name, content in outputs.items():
        with (output / name).open('xb') as stream:
            stream.write(content)
    if source.read_bytes() != raw:
        raise ProfileError('Source changed during write; preserve the output for inspection, do not use it')
    return manifest


def verify_profiles(source, directory):
    """Recompute from the pinned canonical source, rather than trusting manifest."""
    source, directory = Path(source), Path(directory)
    if directory.is_symlink() or not directory.is_dir():
        raise ProfileError('Profiles must be an existing real directory')
    outputs, manifest = build_bundle(source.read_bytes())
    for name, expected in outputs.items():
        path = directory / name
        if path.is_symlink() or not path.is_file() or path.read_bytes() != expected:
            raise ProfileError('Profile/manifest differs from the reviewed deterministic contract: ' + name)
    return manifest


def load_selected_profile(source, directory, profile_id):
    """Return (parsed complete YAML, manifest entry) for no-goal diagnostics."""
    if profile_id not in SPEEDS_KMH:
        raise ProfileError('Unknown prospective profile: ' + str(profile_id))
    manifest = verify_profiles(source, directory)
    entry = manifest['profiles'][profile_id]
    return load_source((Path(directory) / entry['file']).read_bytes()), entry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    generate = subparsers.add_parser('generate', help='Write complete profiles to a NEW directory')
    generate.add_argument('--source', type=Path, required=True)
    generate.add_argument('--output-dir', type=Path, required=True)
    verify = subparsers.add_parser('verify', help='Check exact generated bytes; never starts ROS')
    verify.add_argument('--source', type=Path, required=True)
    verify.add_argument('--profiles-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        manifest = (prepare_profiles(args.source, args.output_dir) if args.command == 'generate'
                    else verify_profiles(args.source, args.profiles_dir))
    except (OSError, ValueError, CandidateError, yaml.YAMLError) as error:
        print('Profile contract failed: ' + str(error), file=sys.stderr)
        return 2
    print(json.dumps({'status': manifest['status'], 'readiness': manifest['readiness'],
                      'profiles': list(manifest['profiles']), 'motion_authorized': False,
                      'gate0': manifest['gate0'], 'gate1': manifest['gate1']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
