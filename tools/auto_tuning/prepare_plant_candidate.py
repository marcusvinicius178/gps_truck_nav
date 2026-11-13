#!/usr/bin/env python3
"""Build/verify the isolated Classic Ackermann candidate; never starts ROS.

The candidate is a reproducible static contract, not a physically validated
plant. Historical files and the confirmed TF/sensor variant remain unchanged.
"""

import argparse
import copy
from decimal import Decimal, ROUND_CEILING
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

from audit_ackermann_geometry import audit_model, steering_targets
from prepare_candidates import differences, load_source
from prepare_navigation_profiles import expected_runtime_parameters, value_at


SOURCE_COMMIT = 'd17fcecb51178eb566d4572f92644098652bd5dd'
SOURCE_PATH = 'src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml'
BUNDLE_PATH = 'config/auto_tuning/plant_candidate'
SOURCES = {
    SOURCE_PATH: '722169c640ac8a37c10ca1097ff212366bac7ae58ac2f596608bfc7a9454eef7',
    'src/gps_truck_nav/truck_bringup/urdf/arocs_truck.urdf': '863a35defbc79b75debb4e2650a874730c1bbd6dfd64c4b2a10f763778b7e3a1',
    'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf': '5a389dd2b5e9703c0f44c657d8b06662865209a9d40870ce06f76b063f1c7da4',
    'config/auto_tuning/models/arocs_truck_tf_sensors/model.config': '2b08eaa807415bd1e813535f4a99a76db8253a2f1219a773dc7a315fdfb3a51d',
    'config/auto_tuning/worlds/ctvi_tf_sensors.world': '9094bd4df7d7f7472ebc0ba3a2dd8910af4a9bcf3e5fa4a5bca225f7df089dab',
    'config/auto_tuning/tf_sensor_model_variant.json': '6982f4314eece195de3cc318be07a2293ea249f2eace79a5ad96749eb12a53ba',
}
PROFILES = {'candidate20kmh': 20, 'candidate25kmh': 25}
MODEL_PATH = 'models/arocs_truck_candidate/model.sdf'
URDF_PATH = 'urdf/arocs_truck_candidate.urdf'
WORLD_PATH = 'worlds/ctvi_plant_candidate.world'
ACTUATOR_TOPIC = '/actuation/steering_command'
CONTROL_FRAME = 'rear_axle_nav'


class PlantCandidateError(ValueError):
    pass


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()


def replace_once(raw, before, after):
    if raw.count(before) != 1:
        raise PlantCandidateError('Expected exactly one source token: ' + before.decode())
    return raw.replace(before, after, 1)


def source_paths(repo=None, *, installed_share=None, config_root=None):
    if repo is not None and installed_share is None and config_root is None:
        return {name: Path(repo) / name for name in SOURCES}
    if repo is not None or installed_share is None or config_root is None:
        raise PlantCandidateError('Choose either repository or installed-share plus config roots')
    result = {}
    for name in SOURCES:
        if name.startswith('src/gps_truck_nav/truck_bringup/'):
            result[name] = Path(installed_share) / name.removeprefix('src/gps_truck_nav/truck_bringup/')
        elif name.startswith('config/auto_tuning/'):
            result[name] = Path(config_root) / name.removeprefix('config/auto_tuning/')
        else:
            raise PlantCandidateError('Unmapped source path')
    return result


def read_sources(paths):
    result = {}
    for name, expected in SOURCES.items():
        path = paths[name]
        if path.is_symlink() or not path.is_file():
            raise PlantCandidateError('Missing or symlinked reviewed source: ' + name)
        raw = path.read_bytes()
        if sha256(raw) != expected:
            raise PlantCandidateError('Reviewed source SHA-256 differs: ' + name)
        result[name] = raw
    return result


def build_bundle(repo=None, *, installed_share=None, config_root=None):
    """Return deterministic files and manifest from the exact reviewed inputs."""
    paths = source_paths(repo, installed_share=installed_share, config_root=config_root)
    sources = read_sources(paths)
    source_model = 'config/auto_tuning/models/arocs_truck_tf_sensors/model.sdf'
    geometry = audit_model(paths[source_model])
    if geometry['source']['sha256'] != SOURCES[source_model]:
        raise PlantCandidateError('Model changed between source hashing and geometry audit')
    base_offset = geometry['base_origin_minus_rear_axle_model_m']
    # This candidate only supports the audited, collinear planar reference.
    if base_offset != [3.502, 0.0, -0.536]:
        raise PlantCandidateError('Rear axle reference differs from the reviewed model')
    length = geometry['wheelbase_euclidean_m']
    minimum = geometry['conservative_ideal_rear_axle_minimum_radius_m']
    radius = float(Decimal(str(minimum)).quantize(Decimal('0.01'), rounding=ROUND_CEILING))
    # repr round-trips this double exactly. Rounding steering downward would
    # incorrectly reject the Nav2 radius boundary while adding no useful margin:
    # the upward-rounded radius already sits strictly inside the joint bound.
    steer = math.atan(length / radius)
    for sign in (-1, 1):
        targets = steering_targets(sign * steer, length, geometry['front_track_euclidean_m'])
        for side, angle in targets.items():
            limit = geometry['steering_joint_limits'][side]
            if not limit['lower_rad'] <= angle <= limit['upper_rad']:
                raise PlantCandidateError('Candidate steering target exceeds an unchanged joint limit')
    outputs = {}
    sdf = replace_once(sources[source_model], b'<max_steer>0.6458</max_steer>',
                       ('<max_steer>' + repr(steer) + '</max_steer>').encode())
    sdf = replace_once(sdf, b"<parameter name='cmd_vel_topic' value='cmd_vel'/>",
                       ("<parameter name='cmd_vel_topic' value='cmd_vel'/>\n"
                        "        <remapping>cmd_vel:=" + ACTUATOR_TOPIC + "</remapping>").encode())
    outputs[MODEL_PATH] = sdf
    outputs['models/arocs_truck_candidate/model.config'] = sources['config/auto_tuning/models/arocs_truck_tf_sensors/model.config']
    outputs[WORLD_PATH] = replace_once(sources['config/auto_tuning/worlds/ctvi_tf_sensors.world'],
                                      b'model://arocs_truck_tf_sensors', b'model://arocs_truck_candidate')
    urdf = sources['src/gps_truck_nav/truck_bringup/urdf/arocs_truck.urdf']
    frame = ('  <!-- Prospective planar rear-axle reference; no collision/inertia or existing joint change. -->\n'
             '  <link name="rear_axle_nav"/>\n'
             '  <joint name="rear_axle_nav_fixed" type="fixed">\n'
             '    <parent link="base_footprint"/>\n'
             '    <child link="rear_axle_nav"/>\n'
             '    <origin xyz="-3.502 0 0" rpy="0 0 0"/>\n'
             '  </joint>\n')
    outputs[URDF_PATH] = replace_once(urdf, b'</robot>', frame.encode() + b'</robot>')
    baseline = load_source(sources[SOURCE_PATH])
    profiles = {}
    for profile_id, kmh in PROFILES.items():
        config = copy.deepcopy(baseline)
        for node in ('bt_navigator', 'behavior_server'):
            config[node]['ros__parameters']['robot_base_frame'] = CONTROL_FRAME
        for node in ('local_costmap', 'global_costmap'):
            params = config[node][node]['ros__parameters']
            polygon = json.loads(params['footprint'])
            params['robot_base_frame'] = CONTROL_FRAME
            params['footprint'] = json.dumps([[float(Decimal(str(x)) + Decimal('3.502')), y]
                                               for x, y in polygon])
        planner = config['planner_server']['ros__parameters']['GridBased']
        # The old nested GridBased.costmap footprint is not assumed consumed;
        # translating it avoids contradictory documentary geometry in this copy.
        polygon = json.loads(planner['costmap']['footprint'])
        planner['costmap']['footprint'] = json.dumps([[float(Decimal(str(x)) + Decimal('3.502')), y]
                                                      for x, y in polygon])
        planner['minimum_turning_radius'] = radius
        follow = config['controller_server']['ros__parameters']['FollowPath']
        controller = config['controller_server']['ros__parameters']
        # A goal for rear_axle_nav must be evaluated at that same reference.
        # Preserve the historical custom checker and all its source bytes, but
        # select the already configured conventional checker for this variant.
        controller['goal_checker_plugins'] = ['general_goal_checker']
        follow['AckermannConstraints']['min_turning_r'] = radius
        cap = kmh / 3.6
        follow['vx_max'], follow['vx_min'] = cap, 0.0
        smoother = config['velocity_smoother']['ros__parameters']
        smoother['max_velocity'] = [cap, 0.0, follow['wz_max']]
        smoother['min_velocity'] = [0.0, 0.0, -follow['wz_max']]
        changed = differences(baseline, config)
        expected = expected_runtime_parameters(config)
        expected['/controller_server'].update({
            'goal_checker_plugins': controller['goal_checker_plugins'],
            **{'general_goal_checker.' + key: value for key, value in controller['general_goal_checker'].items()},
        })
        for node in ('bt_navigator', 'behavior_server'):
            expected.setdefault('/' + node, {})['robot_base_frame'] = CONTROL_FRAME
        for node in ('local_costmap', 'global_costmap'):
            p = config[node][node]['ros__parameters']
            expected['/' + node + '/' + node] = {'robot_base_frame': CONTROL_FRAME, 'footprint': p['footprint']}
        name = 'navigation/' + profile_id + '.yaml'
        outputs[name] = ('# Prospective Classic candidate; static contract only, no motion authorization.\n'
                         '# Rear-axle frame and translated footprint retain the same physical body polygon.\n'
                         '# Radius derives from unchanged joint limits; speed ceilings are experimental.\n'
                         + yaml.safe_dump(config, sort_keys=False, allow_unicode=True)).encode()
        if differences(config, yaml.safe_load(outputs[name])):
            raise PlantCandidateError('Candidate YAML serialization changed values or types')
        policy_name = 'policies/' + profile_id + '_shadow.json'
        outputs[policy_name] = json_bytes({
            'schema_version': 1, 'mode': 'SHADOW_ONLY', 'profile_id': profile_id,
            'status': 'SHADOW_DIAGNOSTIC_POLICY',
            'policy': {
                'forward_speed_mps': cap, 'reverse_speed_mps': 0.0,
                'yaw_rate_rps': follow['wz_max'], 'stop_band_mps': 0.01,
                'resume_speed_mps': 0.02, 'command_timeout_s': 0.25,
                'observation_timeout_s': 0.5, 'clock_stall_timeout_s': 1.0,
                'speed_policy': 'REJECT',
            },
            'rationale': {
                'speed': 'Requested 20/25 km/h experimental ceilings; no demonstrated cruise or safety rating.',
                'reverse': 'Forward-only first navigation profile; reverse converter remains an offline contract pending plant tests.',
                'stop_resume': 'Explicit numerical diagnostic thresholds, not measured standstill accuracy or stopping capability.',
                'command_timeout': 'Five nominal 20 Hz smoother periods; wall-clock instrumentation proposal, not a validated destination watchdog.',
                'observation_and_clock': 'Wall-clock diagnostic deadlines; must be evaluated against observed load, never interpreted as physical braking proof.',
            },
            'runtime_authorized': False, 'physical_stop_verified': False,
        })
        profiles[profile_id] = {
            'file': name, 'sha256': sha256(outputs[name]), 'policy_file': policy_name,
            'policy_sha256': sha256(outputs[policy_name]), 'requested_test_speed_kmh': kmh,
            'forward_speed_cap_m_s': cap, 'reverse_speed_cap_m_s': 0.0,
            'yaw_rate_cap_rad_s': follow['wz_max'],
            'radius_at_speed_and_yaw_caps_m': cap / follow['wz_max'],
            'changes': [{'yaml_path': list(path), 'before': value_at(baseline, path),
                         'after': value_at(config, path)} for path in changed],
            'expected_parameters': expected, 'runtime_values_verified': False,
            'actuation_authorized': False,
        }
    # Reuse the confirmed authority/sensor contract by value, not by mutating it.
    old_manifest = json.loads(sources['config/auto_tuning/tf_sensor_model_variant.json'])
    rear_left = geometry['wheels']['rear_left_joint']['collision_center_model_m']
    rear_right = geometry['wheels']['rear_right_joint']['collision_center_model_m']
    rear_track = rear_left[1] - rear_right[1]
    manifest = {
        'schema_version': 1, 'profile_id': 'arocs_classic_rear_axle_candidate_v1',
        'source_commit': SOURCE_COMMIT, 'status': 'STATIC_CANDIDATE_PREPARED',
        'scope': 'PROSPECTIVE_PLANT_DIAGNOSTIC_ONLY', 'profiles': profiles,
        'source_sha256': dict(SOURCES),
        'canonical_sha256': old_manifest['canonical_sha256'],
        'variant_sha256': {'plant_candidate/' + name: sha256(raw) for name, raw in outputs.items()},
        'model_name': old_manifest['model_name'], 'mesh_uri_prefix': old_manifest['mesh_uri_prefix'],
        'expected_wheel_edges': old_manifest['expected_wheel_edges'],
        'joint_names': old_manifest['joint_names'], 'sensor_lever_arms': old_manifest['sensor_lever_arms'],
        'model_file': MODEL_PATH, 'world_file': WORLD_PATH, 'urdf_file': URDF_PATH,
        'actuator_topic': ACTUATOR_TOPIC, 'expected_actuator_publisher_count': 0,
        'control_frame': CONTROL_FRAME,
        'navigation_reference': {
            'frame': CONTROL_FRAME, 'parent': 'base_footprint',
            'translation_m': [-3.502, 0.0, 0.0], 'rotation_xyzw': [0.0, 0.0, 0.0, 1.0],
        },
        'expected_control_frame_edge': {
            'parent': 'base_footprint', 'child': CONTROL_FRAME,
            'translation_m': [-3.502, 0.0, 0.0], 'rpy_rad': [0.0, 0.0, 0.0],
            'publisher': 'robot_state_publisher',
            'semantics': 'Ground-plane projection of selected rear axle; existing TF chain unchanged.',
        },
        'geometry': {
            'wheelbase_plugin_collision_m': length,
            'front_track_plugin_collision_m': geometry['front_track_euclidean_m'],
            'rear_left_collision_center_model_m': rear_left,
            'rear_right_collision_center_model_m': rear_right,
            'rear_track_planar_m': rear_track,
            'rear_right_lateral_offset_from_axle_m': rear_right[1] - geometry['rear_axle_center_model_m'][1],
            'unchanged_joint_limits': geometry['steering_joint_limits'],
            'static_virtual_steering_bound_rad': geometry['conservative_symmetric_virtual_bound_rad'],
            'static_ideal_rear_axle_radius_bound_m': minimum,
            'candidate_radius_m': radius, 'candidate_plugin_max_steer_rad': steer,
            'radius_policy': 'Round static ideal bound upward to next 0.01 m; a reproducible numerical envelope, not measured grip/dynamic margin.',
            'base_to_control_frame_x_m': -3.502,
            'right_rear_feedback_note': 'Classic longitudinal PID measures rear-right wheel speed. Ideal rolling relation v_right=v_axle-omega*y_right requires validation; same drive torque is applied to both rear wheels.',
        },
        'preserved': [
            'Historical files and confirmed TF/sensor variant bytes',
            'All model collision/visual geometry, masses, inertias, joint limits, PID gains and plugin max_speed=20',
            'Existing odometry, joint_states, TF wheel authority and sensor frame corrections',
            'Datum, spawn pose, contacts, physics, noise, covariance and all sensor lever arms',
            'Same world-space footprint polygon under the translated rear-axle reference',
            'DUBIN planner, Ackermann controller and all objective weights',
        ],
        'known_limits': [
            'Static source equations do not prove physical vehicle fidelity, contact dynamics or loaded plugin geometry.',
            'The right front wheel offset and fixed intermediate wheel collisions remain unchanged and require observable functional assessment.',
            'Old GridBased.costmap.footprint is documentary; runtime global/local costmap footprints are authoritative.',
            'Candidate goals refer to rear_axle_nav and select existing general_goal_checker; the historical front-cabin offset checker remains unchanged but inactive.',
            'A source-level adapter and its own timer cannot stop a plant if the adapter process dies; independently supervised simulator termination remains required before motion.',
            'Covariances, absolute localization and obstacle observability are not solved by changing the actuation reference.',
        ],
        'runtime_verified': False, 'runtime_geometry_validated': False,
        'motion_authorized': False, 'mission_ready': False,
        'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED',
    }
    if read_sources(paths) != sources:
        raise PlantCandidateError('Reviewed inputs changed during candidate construction')
    outputs['manifest.json'] = json_bytes(manifest)
    return outputs, manifest


def prepare_bundle(repo, output_dir):
    output = Path(output_dir)
    if output.exists() or output.is_symlink():
        raise PlantCandidateError('Output exists; preserve previous candidates/evidence')
    outputs, manifest = build_bundle(repo)
    output.mkdir()
    for name, raw in outputs.items():
        path = output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            stream.write(raw)
    return manifest


def verify_bundle(repo=None, directory=None, *, installed_share=None, config_root=None):
    source_paths(repo, installed_share=installed_share, config_root=config_root)
    if directory is None:
        directory = Path(repo) / BUNDLE_PATH if repo is not None else Path(config_root) / 'plant_candidate'
    directory = Path(directory)
    if directory.is_symlink() or not directory.is_dir():
        raise PlantCandidateError('Candidate must be a real existing directory')
    outputs, manifest = build_bundle(repo, installed_share=installed_share, config_root=config_root)
    actual = {str(p.relative_to(directory)) for p in directory.rglob('*') if p.is_file() or p.is_symlink()}
    if actual != set(outputs):
        raise PlantCandidateError('Candidate contains missing, extra or symlinked paths')
    for name, raw in outputs.items():
        path = directory / name
        if path.is_symlink() or any(parent.is_symlink() for parent in path.parents if parent != directory.parent):
            raise PlantCandidateError('Candidate symlinks are forbidden: ' + name)
        if path.read_bytes() != raw:
            raise PlantCandidateError('Candidate differs from the reproducible contract: ' + name)
    return manifest


def load_selected_profile(source, directory, profile_id):
    """Same readback interface as the earlier diagnostic speed profiles."""
    if profile_id not in PROFILES:
        raise PlantCandidateError('Unknown plant candidate profile: ' + str(profile_id))
    source, directory = Path(source), Path(directory)
    repo = source
    for _ in Path(SOURCE_PATH).parts:
        repo = repo.parent
    if source == repo / SOURCE_PATH:
        manifest = verify_bundle(repo, directory)
    elif source.name == 'truck_nav2_params.yaml' and source.parent.name == 'params':
        manifest = verify_bundle(directory=directory, installed_share=source.parent.parent,
                                 config_root=directory.parent)
    else:
        raise PlantCandidateError('Unexpected canonical source layout')
    entry = manifest['profiles'][profile_id]
    return yaml.safe_load((Path(directory) / entry['file']).read_bytes()), entry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('generate', 'verify'))
    location = parser.add_mutually_exclusive_group(required=True)
    location.add_argument('--repo', type=Path)
    location.add_argument('--installed-share', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == 'generate':
            if args.output_dir is None or args.repo is None or args.config is not None:
                raise PlantCandidateError('Generation requires --repo and a new --output-dir')
            manifest = prepare_bundle(args.repo, args.output_dir)
        else:
            manifest = verify_bundle(args.repo, args.output_dir, installed_share=args.installed_share,
                                     config_root=args.config)
    except (OSError, ValueError, ET.ParseError, yaml.YAMLError) as error:
        print('Plant candidate failed: ' + str(error), file=sys.stderr)
        return 2
    print(json.dumps({'status': manifest['status'], 'profiles': list(manifest['profiles']),
                      'motion_authorized': False, 'gate0': manifest['gate0'], 'gate1': manifest['gate1']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
