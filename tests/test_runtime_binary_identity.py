"""Loaded-file provenance regressions using a fake /proc; no ROS or processes."""

import hashlib
import importlib.util
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('runtime_binary_identity',
    ROOT / 'tools/auto_tuning/runtime_binary_identity.py')
identity = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(identity)


class RuntimeBinaryIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.proc = self.root / 'proc'
        self.proc.mkdir()
        self.bin = self.root / 'bin'
        self.bin.mkdir()

    def binary(self, name):
        path = self.bin / name
        path.write_bytes(('fixture binary ' + name).encode())
        return path

    def mapping(self, library, *, inode=None, device=None, deleted=False, permissions='r-xp', address='1000-2000'):
        value = library.stat()
        if device is None:
            device = f'{os.major(value.st_dev):02x}:{os.minor(value.st_dev):02x}'
        if inode is None:
            inode = value.st_ino
        return f'{address} {permissions} 00000000 {device} {inode} {library}' + (' (deleted)' if deleted else '') + '\n'

    def process(self, pid, executable, maps=''):
        directory = self.proc / str(pid)
        directory.mkdir()
        path = self.bin / executable
        if not path.exists():
            path = self.binary(executable)
        (directory / 'exe').symlink_to(path)
        (directory / 'maps').write_text(maps)
        return directory

    def inspect(self):
        return identity.inspect_runtime_binaries(self.proc)

    def test_selected_processes_and_library_hashes_are_separate_from_consumption(self):
        ackermann = self.binary('libgazebo_ros_ackermann_drive.so')
        self.process(10, 'gzserver', self.mapping(ackermann))
        for pid, name in enumerate(identity.EXECUTABLES[1:], 20):
            self.process(pid, name)
        result = self.inspect()
        self.assertEqual(result['collection_status'], 'COMPLETE')
        self.assertFalse(result['upstream_equivalence_verified'])
        self.assertFalse(result['plugin_consumption_verified'])
        target = result['targets']['ackermann_drive']
        self.assertEqual(target['status'], 'MAPPED_IDENTIFIED')
        self.assertEqual(target['records'][0]['sha256'], hashlib.sha256(ackermann.read_bytes()).hexdigest())
        navsat = next(p for p in result['processes'] if p['executable_name'] == 'navsat_transform_node')
        self.assertEqual(navsat['executable']['status'], 'IDENTIFIED')
        self.assertEqual(result['targets']['robot_localization']['status'], 'NOT_OBSERVED')

    def test_repeated_mapping_segments_are_deduplicated(self):
        library = self.binary('libmppi_controller.so')
        maps = self.mapping(library, permissions='r--p') + self.mapping(library, address='2000-3000')
        self.process(1, 'component_container_isolated', maps)
        records = self.inspect()['targets']['mppi_controller']['records']
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['segment_count'], 2)
        self.assertTrue(records[0]['executable_mapping'])

    def test_mapped_file_inode_mismatch_does_not_hash_replacement(self):
        library = self.binary('libgazebo_ros_ackermann_drive.so')
        self.process(1, 'gzserver', self.mapping(library, inode=library.stat().st_ino + 1))
        target = self.inspect()['targets']['ackermann_drive']
        self.assertEqual(target['status'], 'MAPPED_UNVERIFIED')
        self.assertEqual(target['records'][0]['status'], 'MAPPING_IDENTITY_MISMATCH')
        self.assertIsNone(target['records'][0]['sha256'])

    def test_mapped_device_mismatch_is_not_identified(self):
        library = self.binary('libnav2_smac_planner.so')
        self.process(1, 'component_container_isolated', self.mapping(library, device='ff:ff'))
        self.assertEqual(self.inspect()['targets']['smac_hybrid']['records'][0]['status'], 'MAPPING_IDENTITY_MISMATCH')

    def test_deleted_mapping_preserves_evidence_and_does_not_identify_existing_path(self):
        library = self.binary('libgazebo_ros_ackermann_drive.so')
        self.process(1, 'gzserver', self.mapping(library, deleted=True))
        record = self.inspect()['targets']['ackermann_drive']['records'][0]
        self.assertEqual(record['status'], 'DELETED_MAPPING')
        self.assertTrue(record['deleted'])
        self.assertIsNone(record['sha256'])

    def test_unrelated_process_and_library_are_ignored_without_reading_their_maps(self):
        unrelated = self.process(1, 'unrelated_application', 'not a valid maps file')
        library = self.binary('libirrelevant.so')
        self.process(2, 'gzserver', self.mapping(library))
        original = Path.read_text
        def guarded_read(path, *args, **kwargs):
            if path == unrelated / 'maps':
                raise AssertionError('Unrelated maps must not be read')
            return original(path, *args, **kwargs)
        with mock.patch.object(Path, 'read_text', guarded_read):
            result = self.inspect()
        self.assertEqual([p['pid'] for p in result['processes']], [2])
        self.assertEqual(result['processes'][0]['libraries'], [])

    def test_maps_permission_error_is_explicit(self):
        process = self.process(1, 'gzserver')
        original = Path.read_text
        def guarded_read(path, *args, **kwargs):
            if path == process / 'maps':
                raise PermissionError(13, 'denied', str(path))
            return original(path, *args, **kwargs)
        with mock.patch.object(Path, 'read_text', guarded_read):
            result = self.inspect()
        self.assertEqual(result['collection_status'], 'PARTIAL')
        self.assertEqual(result['processes'][0]['maps_status'], 'READ_ERROR')
        self.assertEqual(result['targets']['ackermann_drive']['status'], 'DISCOVERY_INCOMPLETE')
        self.assertEqual(result['processes'][0]['errors'][0]['error']['errno'], 13)

    def test_library_file_permission_failure_keeps_mapping_but_no_hash(self):
        library = self.binary('libgazebo_ros_ackermann_drive.so')
        self.process(1, 'gzserver', self.mapping(library))
        original = os.open
        def guarded_open(path, *args, **kwargs):
            if Path(path) == library:
                raise PermissionError(13, 'denied', str(path))
            return original(path, *args, **kwargs)
        with mock.patch.object(identity.os, 'open', guarded_open):
            target = self.inspect()['targets']['ackermann_drive']
        self.assertEqual(target['status'], 'MAPPED_UNVERIFIED')
        self.assertEqual(target['records'][0]['status'], 'FILE_READ_ERROR')
        self.assertIsNone(target['records'][0]['sha256'])

    def test_file_changed_during_hash_is_not_identified(self):
        library = self.binary('libmppi_controller.so')
        before = library.stat()
        after = SimpleNamespace(**{key: getattr(before, key) for key in
            ('st_mode', 'st_dev', 'st_ino', 'st_size', 'st_mtime_ns', 'st_ctime_ns')})
        after.st_mtime_ns += 1
        with mock.patch.object(identity.os, 'fstat', side_effect=[before, after]):
            result = identity._identify_file(library,
                (os.major(before.st_dev), os.minor(before.st_dev)), before.st_ino)
        self.assertEqual(result['status'], 'FILE_CHANGED_DURING_READ')
        self.assertIsNone(result['sha256'])

    def test_missing_proc_root_returns_explicit_unavailability(self):
        result = identity.inspect_runtime_binaries(self.root / 'missing_proc')
        self.assertEqual(result['collection_status'], 'UNAVAILABLE')
        self.assertEqual(result['errors'][0]['operation'], 'enumerate_processes')
        self.assertEqual(result['targets']['smac_hybrid']['status'], 'DISCOVERY_INCOMPLETE')

    def test_executable_permission_error_cannot_be_assumed_irrelevant(self):
        self.process(1, 'gzserver')
        with mock.patch.object(identity.os, 'readlink', side_effect=PermissionError(13, 'denied')):
            result = self.inspect()
        self.assertEqual(result['collection_status'], 'UNAVAILABLE')
        self.assertEqual(result['targets']['ackermann_drive']['status'], 'DISCOVERY_INCOMPLETE')
        self.assertEqual(result['errors'][0]['operation'], 'read_executable')

    def test_no_matching_processes_reports_absence(self):
        result = self.inspect()
        self.assertEqual(result['collection_status'], 'UNAVAILABLE')
        self.assertTrue(all(p['status'] == 'NOT_OBSERVED' for p in result['required_processes'].values()))
        self.assertEqual(result['targets']['ackermann_drive']['status'], 'NOT_OBSERVED')

    def test_selected_process_disappearing_during_snapshot_is_not_identified(self):
        library = self.binary('libgazebo_ros_ackermann_drive.so')
        process = self.process(1, 'gzserver', self.mapping(library))
        first = str(self.bin / 'gzserver')
        with mock.patch.object(identity.os, 'readlink', side_effect=[first, FileNotFoundError(2, 'gone')]):
            result = self.inspect()
        self.assertEqual(result['processes'][0]['process_status'], 'EXITED_OR_INACCESSIBLE')
        self.assertEqual(result['targets']['ackermann_drive']['status'], 'MAPPED_UNVERIFIED')

    def test_no_executable_mapping_is_not_a_loaded_plugin_claim(self):
        library = self.binary('libnav2_smac_planner.so')
        self.process(1, 'component_container_isolated', self.mapping(library, permissions='r--p'))
        target = self.inspect()['targets']['smac_hybrid']
        self.assertEqual(target['status'], 'MAPPED_UNVERIFIED')
        self.assertEqual(target['records'][0]['status'], 'NO_EXECUTABLE_MAPPING')

    def test_relevant_malformed_mapping_is_an_error(self):
        library = self.binary('libgazebo_ros_ackermann_drive.so')
        self.process(1, 'gzserver', self.mapping(library, device='invalid'))
        result = self.inspect()
        self.assertEqual(result['processes'][0]['maps_status'], 'PARTIAL')
        self.assertEqual(result['processes'][0]['errors'][0]['status'], 'MALFORMED_RELEVANT_MAPPING')

    def test_versioned_library_and_path_spaces_are_supported(self):
        directory = self.bin / 'directory with spaces'
        directory.mkdir()
        library = directory / 'libmppi_controller.so.1.2'
        library.write_bytes(b'versioned mppi fixture')
        self.process(1, 'component_container_isolated', self.mapping(library))
        target = self.inspect()['targets']['mppi_controller']
        self.assertEqual(target['status'], 'MAPPED_IDENTIFIED')
        self.assertEqual(target['records'][0]['path'], str(library))


if __name__ == '__main__':
    unittest.main()
