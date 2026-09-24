#!/usr/bin/env python3
"""Read-only structural audit. Requires PyYAML; does not launch or evaluate ROS."""

from collections import Counter
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'src/gps_truck_nav'


class UniqueKeyLoader(yaml.SafeLoader):
    """Reject silent YAML key overwrites, including nested mappings."""


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'duplicate YAML key at line {key_node.start_mark.line + 1}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def main():
    listed = subprocess.check_output(
        ['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=ROOT)
    paths = sorted({ROOT / p.decode() for p in listed.split(b'\0') if p})
    counts = Counter()
    errors = []
    for path in paths:
        if path.is_symlink() and not path.exists():
            errors.append((path, 'broken symlink'))
        if not path.is_file():
            continue  # staged/worktree removals are expected during cleanup
        try:
            if path.suffix in {'.yaml', '.yml', '.repos', '.mvc', '.rviz'}:
                yaml.load(path.read_text(), Loader=UniqueKeyLoader)
                counts['YAML (unique keys)'] += 1
            if path.suffix in {'.xml', '.sdf', '.world', '.urdf', '.xacro', '.model', '.config'}:
                ET.parse(path)
                counts['XML syntax'] += 1
            if path.suffix == '.py':
                compile(path.read_bytes(), str(path), 'exec')
                counts['Python/launch syntax'] += 1
        except (ValueError, SyntaxError, ET.ParseError, yaml.YAMLError) as error:
            # Never print source lines: a parser error might include a credential.
            errors.append((path, type(error).__name__))

    package_names = []
    for manifest in SOURCE.glob('*/package.xml'):
        package_names.append(ET.parse(manifest).getroot().findtext('name'))
    if len(package_names) != len(set(package_names)) or len(package_names) != 3:
        errors.append((SOURCE, 'expected three unique canonical packages'))
    counts['unique canonical packages'] = len(package_names)

    for cmake in SOURCE.glob('*/CMakeLists.txt'):
        contents = cmake.read_text()
        for block in re.findall(r'install\(PROGRAMS\s+(.*?)DESTINATION', contents, re.S):
            for relative in block.split():
                script = cmake.parent / relative
                if not script.is_file() or not script.read_bytes().startswith(b'#!/usr/bin/env python3'):
                    errors.append((script, 'missing installed script or executable Python shebang'))
                else:
                    counts['installed Python entry points'] += 1

    truck = SOURCE / 'truck_bringup'
    for world, waypoints in [('ctvi.world', 'empty_world_gps_wps.yaml')]:
        origin = ET.parse(truck / 'worlds' / world).find('.//spherical_coordinates')
        lat = float(origin.findtext('latitude_deg'))
        lon = float(origin.findtext('longitude_deg'))
        wps = yaml.safe_load((truck / 'params' / waypoints).read_text())['waypoints']
        if not all(abs(wp['latitude'] - lat) < .01 and abs(wp['longitude'] - lon) < .01 for wp in wps):
            errors.append((truck / 'params' / waypoints, 'waypoints belong to a different geographic site'))
        else:
            counts['world/waypoint site consistency (not alignment or runtime)'] += 1
    for mesh in ET.parse(truck / 'models/arocs_truck/model.sdf').findall('.//mesh/uri'):
        uri = mesh.text
        if uri and uri.startswith('model://arocs_truck/'):
            local = truck / 'models' / uri.removeprefix('model://')
            if not local.is_file():
                errors.append((local, 'missing required mesh'))
            else:
                with local.open('rb') as stream:
                    if stream.read(100).startswith(b'version https://git-lfs.github.com/spec/v1'):
                        errors.append((local, 'unhydrated Git LFS pointer; run git lfs pull'))
                counts['required mesh references'] += 1

    for kind, count in counts.items():
        print(f'CHECK {kind}: {count}')
    for path, reason in errors:
        print(f'FAIL {path.relative_to(ROOT)}: {reason}')
    print('PASS: structural audit' if not errors else f'FAIL: {len(errors)} issues')
    return int(bool(errors))


if __name__ == '__main__':
    sys.exit(main())
