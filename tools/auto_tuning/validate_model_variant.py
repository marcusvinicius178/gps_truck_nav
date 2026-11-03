#!/usr/bin/env python3
"""Validate the explicit three-field SDF variant without loading ROS or Gazebo."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


SOURCE_COMMIT = '978ab2328c6cb378d6eea016027192a6cf1dbadf'
CANONICAL_SHA256 = {
    'models/arocs_truck/model.sdf': 'b8f8fc8b20a129694eed58bfd439c770e112a70fcf0e6da37bd9595f7e7ca8f2',
    'models/arocs_truck/model.config': '2b08eaa807415bd1e813535f4a99a76db8253a2f1219a773dc7a315fdfb3a51d',
    'worlds/ctvi.world': 'f33f9c7ad7be1ec785edbff5e6229a22515ac9fdd84c019fd5c8c8661a86e61f',
    'urdf/arocs_truck.urdf': '863a35defbc79b75debb4e2650a874730c1bbd6dfd64c4b2a10f763778b7e3a1',
}
VARIANT_FILES = ('models/arocs_truck_tf_sensors/model.sdf',
                 'models/arocs_truck_tf_sensors/model.config', 'worlds/ctvi_tf_sensors.world')
REPLACEMENTS = (
    ('<publish_wheel_tf>1</publish_wheel_tf>', '<publish_wheel_tf>0</publish_wheel_tf>'),
    ("<plugin name='gps_plugin' filename='libgazebo_ros_gps_sensor.so'>",
     "<plugin name='gps_plugin' filename='libgazebo_ros_gps_sensor.so'>\n          <frame_name>gps_link</frame_name>"),
    ("<plugin name='imu_plugin' filename='libgazebo_ros_imu_sensor.so'>",
     "<plugin name='imu_plugin' filename='libgazebo_ros_imu_sensor.so'>\n          <frame_name>imu_link</frame_name>"),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def exact_replacements(source, replacements):
    for before, after in replacements:
        require(source.count(before) == 1, 'Expected exactly one source occurrence: ' + before)
        source = source.replace(before, after)
    return source


def fixed_sensor_pose(urdf, frame):
    """Compose this pinned model's fixed, co-oriented base-to-sensor chain."""
    by_child = {joint.find('child').get('link'): joint for joint in urdf.findall('./joint')}
    translation = [0.0, 0.0, 0.0]
    visited = set()
    while frame != 'base_footprint':
        require(frame not in visited, 'Cycle in sensor URDF chain')
        visited.add(frame)
        require(frame in by_child, 'Missing sensor URDF joint: ' + frame)
        joint = by_child[frame]
        require(joint.get('type') == 'fixed', 'Sensor chain is no longer fixed')
        origin = joint.find('origin')
        xyz = [float(value) for value in origin.get('xyz').split()]
        rpy = [float(value) for value in origin.get('rpy', '0 0 0').split()]
        require(len(xyz) == 3 and rpy == [0.0, 0.0, 0.0], 'Sensor chain orientation changed')
        translation = [left + right for left, right in zip(translation, xyz)]
        frame = joint.find('parent').get('link')
    return translation


def validate(share, config):
    share, config = Path(share), Path(config)
    manifest_path = config / 'tf_sensor_model_variant.json'
    manifest = json.loads(manifest_path.read_text())
    require(manifest.get('schema_version') == 1 and
            manifest.get('profile_id') == 'arocs_tf_sensor_frames_v1' and
            manifest.get('source_commit') == SOURCE_COMMIT,
            'Unexpected model variant identity')
    require(manifest.get('canonical_sha256') == CANONICAL_SHA256, 'Canonical source pins changed')
    require(set(manifest.get('variant_sha256', {})) == set(VARIANT_FILES), 'Variant file set changed')
    for base, expected in ((share, CANONICAL_SHA256), (config, manifest['variant_sha256'])):
        for relative, expected_hash in expected.items():
            require(digest(base / relative) == expected_hash, 'Hash mismatch: ' + relative)
    require(manifest.get('runtime_verified') is False and manifest.get('mission_ready') is False and
            manifest.get('gate0') == 'NOT_YET_VERIFIED' and manifest.get('gate1') == 'NOT_YET_VERIFIED',
            'Static validation cannot approve runtime or gates')

    canonical_text = (share / 'models/arocs_truck/model.sdf').read_text()
    variant_text = (config / VARIANT_FILES[0]).read_text()
    require(variant_text == exact_replacements(canonical_text, REPLACEMENTS),
            'SDF differs beyond the three approved metadata/TF changes')
    require((config / VARIANT_FILES[1]).read_bytes() == (share / 'models/arocs_truck/model.config').read_bytes(),
            'Model metadata changed')
    world = (share / 'worlds/ctvi.world').read_text()
    expected_world = exact_replacements(world, (('<uri>model://arocs_truck</uri>',
                                                 '<uri>model://arocs_truck_tf_sensors</uri>'),))
    require((config / VARIANT_FILES[2]).read_text() == expected_world,
            'World changed beyond the explicit model URI')
    require(manifest.get('allowed_world_change') == {
        'before': 'model://arocs_truck', 'after': 'model://arocs_truck_tf_sensors'},
        'Manifest world change does not match variant')
    ET.fromstring(expected_world)
    model = ET.fromstring(variant_text).find('./model')
    require(model is not None and model.get('name') == manifest.get('model_name') == 'arocs_truck',
            'Runtime model name changed')
    drive = model.find("./plugin[@name='ackermann_drive']")
    for field, expected in (('publish_wheel_tf', '0'), ('publish_odom', '1'), ('publish_odom_tf', '1')):
        require(drive.findtext(field) == expected, 'Unexpected drive setting: ' + field)
    urdf = ET.parse(share / 'urdf/arocs_truck.urdf').getroot()
    sensors = {}
    for sensor_name, plugin_name, frame in (('arocs_gps', 'gps_plugin', 'gps_link'),
                                            ('imu_sensor', 'imu_plugin', 'imu_link')):
        sensor = model.find("./link[@name='base_footprint']/sensor[@name='" + sensor_name + "']")
        plugin = sensor.find("./plugin[@name='" + plugin_name + "']")
        require(plugin.findtext('frame_name') == frame, 'Wrong sensor frame: ' + sensor_name)
        pose = [float(value) for value in sensor.findtext('pose').split()]
        require(len(pose) == 6 and pose[3:] == [0.0, 0.0, 0.0], 'Sensor orientation changed')
        from_urdf = fixed_sensor_pose(urdf, frame)
        require(all(abs(a - b) < 1e-12 for a, b in zip(pose[:3], from_urdf)),
                'SDF/URDF lever arms differ for ' + frame)
        sensors[frame] = {'sensor': sensor_name, 'parent': 'base_footprint',
                          'translation_m': pose[:3], 'rpy_rad': pose[3:]}
    require(sensors == manifest.get('sensor_lever_arms'), 'Manifest sensor lever arms do not match SDF/URDF')
    expected_changes = [{'element': "model/plugin[@name='ackermann_drive']/publish_wheel_tf", 'before': '1', 'after': '0'}]
    for sensor_name, plugin_name, frame in (('arocs_gps', 'gps_plugin', 'gps_link'),
                                            ('imu_sensor', 'imu_plugin', 'imu_link')):
        expected_changes.append({'element': "model/link[@name='base_footprint']/sensor[@name='" + sensor_name +
                                 "']/plugin[@name='" + plugin_name + "']/frame_name", 'before': None, 'after': frame})
    require(manifest.get('allowed_sdf_changes') == expected_changes, 'Manifest allowed changes differ')

    joint_names = [value.text for value in model.findall("./plugin[@name='truck_joint_state']/joint_name")]
    require(len(joint_names) == len(set(joint_names)) == 6 and joint_names == manifest.get('joint_names'),
            'Joint-state contract changed')
    wheel_edges = []
    for name in joint_names:
        joint = urdf.find("./joint[@name='" + name + "']")
        wheel_edges.append({'joint_name': name, 'parent': joint.find('parent').get('link'),
                            'child': joint.find('child').get('link'), 'expected_publisher': 'robot_state_publisher'})
    require(wheel_edges == manifest.get('expected_wheel_edges'), 'Wheel TF contract differs from URDF')
    mesh_uris = [value.text for value in model.findall('.//mesh/uri')]
    require(mesh_uris and manifest.get('mesh_uri_prefix') == 'model://arocs_truck/meshes/', 'Mesh contract missing')
    for uri in mesh_uris:
        require(uri.startswith('model://arocs_truck/meshes/'), 'Unexpected mesh URI: ' + uri)
        path = share / 'models' / uri.removeprefix('model://')
        require(path.is_file(), 'Mesh URI does not resolve to canonical installed asset: ' + uri)
        with path.open('rb') as stream:
            require(not stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1'),
                    'Unhydrated mesh asset: ' + uri)
    return {'profile_id': manifest['profile_id'], 'manifest_sha256': digest(manifest_path),
            'canonical_sha256': CANONICAL_SHA256, 'variant_sha256': manifest['variant_sha256'],
            'model_name': model.get('name'), 'sensor_lever_arms': sensors,
            'expected_wheel_edges': wheel_edges, 'joint_names': joint_names,
            'canonical_mesh_references_checked': len(mesh_uris)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    location = parser.add_mutually_exclusive_group(required=True)
    location.add_argument('--repo', type=Path)
    location.add_argument('--installed-share', type=Path)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args(argv)
    if args.installed_share is not None and args.config is None:
        parser.error('--installed-share requires --config')
    if args.repo is not None and args.config is not None:
        parser.error('--config is only used with --installed-share')
    share = args.installed_share if args.installed_share is not None else args.repo / 'src/gps_truck_nav/truck_bringup'
    config = args.config if args.config is not None else args.repo / 'config/auto_tuning'
    result = {'schema_version': 1, 'status': 'MODEL_VARIANT_FAILED',
              'scope': 'STATIC_ASSET_AND_METADATA_CONTRACT_ONLY', 'runtime_verified': False,
              'mission_ready': False, 'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}
    try:
        result['observations'] = validate(share, config)
        result['status'] = 'MODEL_VARIANT_VALIDATED'
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['status'] == 'MODEL_VARIANT_VALIDATED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
