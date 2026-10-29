"""Offline contract tests; these do not constitute Gate 0 or Gate 1 evidence."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "tools/auto_tuning/prepare_candidates.py"
SOURCE = REPO / "src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml"
SPEC = importlib.util.spec_from_file_location("prepare_candidates", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source = self.root / "source.yaml"
        self.source.write_bytes(SOURCE.read_bytes())
        self.out = self.root / "candidates"

    def tearDown(self):
        self.temp.cleanup()

    def rewrite(self, mutate):
        data = yaml.safe_load(self.source.read_text())
        mutate(data)
        self.source.write_text(yaml.safe_dump(data, sort_keys=False))

    def assert_rejected(self):
        with self.assertRaises(MODULE.CandidateError):
            MODULE.prepare_candidates(self.source, self.out)
        self.assertFalse(self.out.exists())

    def test_exactly_one_leaf_diff_and_fixed_physical_parameters(self):
        raw_before = self.source.read_bytes()
        original = yaml.safe_load(raw_before)
        MODULE.prepare_candidates(self.source, self.out)
        a = yaml.safe_load((self.out / "candidate_a.yaml").read_bytes())
        b = yaml.safe_load((self.out / "candidate_b.yaml").read_bytes())
        self.assertEqual(MODULE.differences(original, a), [])
        self.assertEqual(MODULE.differences(a, b), [MODULE.PARAMETER_PATH])
        self.assertEqual(self.source.read_bytes(), raw_before)
        self.assertEqual((self.out / "candidate_a.yaml").read_bytes(), raw_before)
        self.assertEqual(b["controller_server"], original["controller_server"])
        self.assertEqual(b["local_costmap"], original["local_costmap"])
        self.assertEqual(b["global_costmap"], original["global_costmap"])
        self.assertEqual(b["planner_server"]["ros__parameters"]["GridBased"]["minimum_turning_radius"], 11.6)

    def test_manifest_records_hashes_without_runtime_claims(self):
        MODULE.prepare_candidates(self.source, self.out)
        manifest = json.loads((self.out / "manifest.json").read_text())
        self.assertEqual(manifest["status"], "proposed_not_runtime_tested")
        self.assertEqual(manifest["gate1"], "NOT_YET_VERIFIED")
        self.assertIsNone(manifest["seeds"]["mppi"])
        self.assertEqual(manifest["source"]["sha256"], hashlib.sha256(self.source.read_bytes()).hexdigest())
        for candidate in manifest["candidates"]:
            self.assertEqual(candidate["sha256"], hashlib.sha256((self.out / candidate["file"]).read_bytes()).hexdigest())
            self.assertFalse(candidate["effective_value_verified"])

    def test_existing_output_directory_is_untouched(self):
        self.out.mkdir()
        sentinel = self.out / "keep.txt"
        sentinel.write_text("owned by another run")
        with self.assertRaises(MODULE.CandidateError):
            MODULE.prepare_candidates(self.source, self.out)
        self.assertEqual(sentinel.read_text(), "owned by another run")
        self.assertEqual(list(self.out.iterdir()), [sentinel])

    def test_existing_file_and_dangling_symlink_are_not_replaced(self):
        self.out.write_text("existing")
        with self.assertRaises(MODULE.CandidateError):
            MODULE.prepare_candidates(self.source, self.out)
        self.assertEqual(self.out.read_text(), "existing")
        self.out.unlink()
        self.out.symlink_to(self.root / "missing")
        with self.assertRaises(MODULE.CandidateError):
            MODULE.prepare_candidates(self.source, self.out)
        self.assertTrue(self.out.is_symlink())

    def test_bad_penalty_values_rejected(self):
        for value in [True, 2, "2.5", 2.4, float("nan"), float("inf")]:
            with self.subTest(value=value):
                self.source.write_bytes(SOURCE.read_bytes())
                self.rewrite(lambda d: d["planner_server"]["ros__parameters"]["GridBased"].update(non_straight_penalty=value))
                self.assert_rejected()

    def test_physical_radius_changes_rejected_in_both_plugins(self):
        paths = [
            ("planner_server", "ros__parameters", "GridBased", "minimum_turning_radius"),
            ("controller_server", "ros__parameters", "FollowPath", "AckermannConstraints", "min_turning_r"),
        ]
        for path in paths:
            with self.subTest(path=path):
                self.source.write_bytes(SOURCE.read_bytes())
                def mutate(data):
                    for key in path[:-1]:
                        data = data[key]
                    data[path[-1]] = 9.0
                self.rewrite(mutate)
                self.assert_rejected()

    def test_missing_or_wrong_plugin_schema_rejected(self):
        mutations = [
            lambda d: d.pop("planner_server"),
            lambda d: d["planner_server"].update(ros__parameters=[]),
            lambda d: d["planner_server"]["ros__parameters"]["GridBased"].update(plugin="nav2_navfn_planner/NavfnPlanner"),
            lambda d: d["controller_server"]["ros__parameters"]["FollowPath"].update(motion_model="DiffDrive"),
        ]
        for mutate in mutations:
            self.source.write_bytes(SOURCE.read_bytes())
            self.rewrite(mutate)
            self.assert_rejected()

    def test_duplicate_keys_nonfinite_and_unsafe_yaml_rejected(self):
        for raw in [b"x: 1\nx: 2\n", b"!!python/object/apply:os.system ['touch forbidden']",
                    b"[]", b"root: &a\n  child: *a\n", b"1: number-key\n"]:
            with self.subTest(raw=raw):
                self.source.write_bytes(raw)
                self.assert_rejected()
        self.source.write_bytes(SOURCE.read_bytes())
        self.rewrite(lambda d: d["controller_server"]["ros__parameters"]["FollowPath"].update(model_dt=float("nan")))
        self.assert_rejected()

    def test_repeat_outputs_are_deterministic(self):
        other = self.root / "second"
        MODULE.prepare_candidates(self.source, self.out)
        MODULE.prepare_candidates(self.source, other)
        for name in ("candidate_a.yaml", "candidate_b.yaml", "manifest.json"):
            self.assertEqual((self.out / name).read_bytes(), (other / name).read_bytes())

    def test_cli_succeeds_and_refuses_reuse(self):
        command = [sys.executable, str(SCRIPT), "--source", str(self.source), "--output-dir", str(self.out)]
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["gate1"], "NOT_YET_VERIFIED")
        again = subprocess.run(command, capture_output=True, text=True, check=False)
        self.assertEqual(again.returncode, 2)
        self.assertIn("already exists", again.stderr)


if __name__ == "__main__":
    unittest.main()
