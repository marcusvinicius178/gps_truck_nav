"""Contract regressions only; these tests never execute ROS or historical bags."""

import csv
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools/auto_tuning/evaluate_trial.py"
SPEC = importlib.util.spec_from_file_location("evaluate_trial", MODULE_PATH)
adapter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adapter)


class EvaluationContractTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / "prospective_run_1"
        self.bag.mkdir()
        (self.bag / "metadata.yaml").write_text("test fixture only\n")
        (self.bag / "run_0.mcap").write_bytes(b"synthetic placeholder; never read as MCAP")
        self.reference = self.root / "reference.csv"
        self.reference.write_text("x_m,y_m,t_ns\n0,0,0\n10,0,1\n")
        self.route = self.root / "route.yaml"
        self.route.write_text("synthetic route identity only\n")
        self.association = {
            "schema_version": 2, "trial_id": "candidate_a", "attempt_id": "attempt_1",
            "dataset_kind": "prospective_auto_tuning", "scenario_id": 1, "speed_kmh": 20,
            "reference_sha256": adapter.sha256(self.reference),
            "mission_route_sha256": adapter.sha256(self.route),
            "bag_metadata_sha256": adapter.sha256(self.bag / "metadata.yaml"),
            "bag_mcap_sha256": adapter.sha256(self.bag / "run_0.mcap"),
            "association_evidence": "Artificial contract fixture, never runtime evidence",
        }

    def validate_association(self, manifest):
        return adapter.validate_association(manifest, self.reference, self.route, 1, 20,
                                            self.bag, self.bag / "run_0.mcap",
                                            "candidate_a", "attempt_1")

    def write_row(self, **changes):
        row = {
            "bag_dir": str(self.bag), "notes": "", "eval_cutoff_reason": "first_PR_ge_0.90",
            "success_geo": "1", "valid_for_mission": "1", "valid_for_tracking": "1",
            "has_plan": "1", "progress_ratio": "0.95", "mission_time_s": "15",
            "plan_tracking_samples": "19", "plan_message_count": "1",
            "raw_run_duration_s": "17", "plan_tracking_coverage": "0.95",
            "rmse_y_primary_m": "0.4", "p95_y_primary_m": "0.5",
            "rmse_psi_primary_rad": "0.05", "rms_jx_mps3": "",
        }
        row.update(changes)
        path = self.root / "per_run_metrics.csv"
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
        return path

    def test_finalized_single_bag_accepted(self):
        bag, mcap = adapter.validate_bag(self.bag)
        self.assertEqual(bag, self.bag)
        self.assertEqual(mcap.name, "run_0.mcap")

    def test_split_bag_rejected_before_evaluation(self):
        (self.bag / "run_1.mcap").write_bytes(b"second split")
        with self.assertRaisesRegex(adapter.EvaluationError, "split bags"):
            adapter.validate_bag(self.bag)

    def test_nested_bags_rejected(self):
        nested = self.bag / "nested"
        nested.mkdir()
        (nested / "metadata.yaml").write_text("fixture")
        with self.assertRaisesRegex(adapter.EvaluationError, "nested"):
            adapter.validate_bag(self.bag)

    def test_unfinalized_bag_rejected(self):
        (self.bag / "metadata.yaml").unlink()
        with self.assertRaisesRegex(adapter.EvaluationError, "finalized"):
            adapter.validate_bag(self.bag)

    def test_association_digest_and_dataset_required(self):
        manifest = self.root / "association.json"
        manifest.write_text(json.dumps(self.association))
        self.validate_association(manifest)
        self.reference.write_text("x_m,y_m,t_ns\n0,0,0\n11,0,1\n")
        with self.assertRaisesRegex(adapter.EvaluationError, "reference_sha256"):
            self.validate_association(manifest)
        self.association["dataset_kind"] = "historical"
        manifest.write_text(json.dumps(self.association))
        with self.assertRaisesRegex(adapter.EvaluationError, "dataset_kind"):
            self.validate_association(manifest)

    def test_association_evidence_required(self):
        self.association["association_evidence"] = " "
        manifest = self.root / "association.json"
        manifest.write_text(json.dumps(self.association))
        with self.assertRaisesRegex(adapter.EvaluationError, "association_evidence"):
            self.validate_association(manifest)

    def test_degenerate_or_nonfinite_reference_rejected(self):
        self.reference.write_text("x_m,y_m,t_ns\n0,0,0\n0,0,1\n")
        with self.assertRaisesRegex(adapter.EvaluationError, "distinct"):
            adapter.validate_reference(self.reference)
        self.reference.write_text("x_m,y_m,t_ns\n0,0,0\nNaN,1,1\n")
        with self.assertRaisesRegex(adapter.EvaluationError, "nonfinite"):
            adapter.validate_reference(self.reference)

    def test_bag_read_failed_row_is_not_navigation_failure_score(self):
        row = self.write_row(notes="BAG_READ_FAILED: no odometry")
        with self.assertRaisesRegex(adapter.EvaluationError, "BAG_READ_FAILED"):
            adapter.read_single_result(row, self.bag)

    def test_nonfinite_progress_rejected(self):
        row = self.write_row(progress_ratio="nan")
        with self.assertRaisesRegex(adapter.EvaluationError, "progress_ratio"):
            adapter.read_single_result(row, self.bag)

    def test_success_threshold_inconsistency_rejected(self):
        row = self.write_row(progress_ratio="0.85")
        with self.assertRaisesRegex(adapter.EvaluationError, "success"):
            adapter.read_single_result(row, self.bag)

    def test_missing_plan_preserves_ineligibility_and_nulls(self):
        row = self.write_row(has_plan="0", valid_for_tracking="0", plan_tracking_coverage="0",
                             plan_tracking_samples="0", plan_message_count="0",
                             rmse_y_primary_m="", p95_y_primary_m="", rmse_psi_primary_rad="")
        result = adapter.read_single_result(row, self.bag)
        self.assertEqual(result["success_geo"], 1)
        self.assertEqual(result["valid_for_tracking"], 0)
        self.assertIsNone(result["rmse_y_primary_m"])
        self.assertIsNone(result["rms_jx_mps3"])

    def test_valid_incomplete_attempt_keeps_progress(self):
        row = self.write_row(progress_ratio="0.4", success_geo="0", valid_for_mission="0",
                             valid_for_tracking="0", rmse_y_primary_m="", p95_y_primary_m="",
                             rmse_psi_primary_rad="")
        result = adapter.read_single_result(row, self.bag)
        self.assertEqual(result["progress_ratio"], 0.4)
        self.assertEqual(result["success_geo"], 0)

    def test_wrong_bag_or_multiple_rows_rejected(self):
        row = self.write_row(bag_dir=str(self.root / "different_bag"))
        with self.assertRaisesRegex(adapter.EvaluationError, "requested bag"):
            adapter.read_single_result(row, self.bag)
        row = self.write_row()
        with row.open("a") as stream:
            stream.write(row.read_text().splitlines()[-1] + "\n")
        with self.assertRaisesRegex(adapter.EvaluationError, "exactly one"):
            adapter.read_single_result(row, self.bag)

    def test_command_keeps_one_explicit_reference_and_frozen_thresholds(self):
        command = adapter.build_command(Path("eval.py"), self.bag, self.reference,
                                        self.root / "output", 1, 20, 6.804)
        self.assertEqual(command[command.index("--runs-dir") + 1], str(self.bag))
        self.assertEqual(command[command.index("--gt-csv") + 1], str(self.reference))
        self.assertEqual(command[command.index("--success-pr-threshold") + 1], "0.90")
        self.assertNotIn("--gt-gps-csv-glob", command)

    def test_association_requires_object_and_bound_trial_attempt(self):
        manifest = self.root / "association.json"
        for value in ([], None, 1, "manifest"):
            with self.subTest(value=value):
                manifest.write_text(json.dumps(value))
                with self.assertRaisesRegex(adapter.EvaluationError, "JSON object"):
                    self.validate_association(manifest)
        for key in ("trial_id", "attempt_id"):
            for value in (None, "", "another_id"):
                with self.subTest(key=key, value=value):
                    manifest.write_text(json.dumps({**self.association, key: value}))
                    with self.assertRaisesRegex(adapter.EvaluationError, key):
                        self.validate_association(manifest)

    def test_association_rejects_metadata_or_mcap_substitution(self):
        manifest = self.root / "association.json"
        manifest.write_text(json.dumps(self.association))
        for filename, key in (("metadata.yaml", "bag_metadata_sha256"),
                              ("run_0.mcap", "bag_mcap_sha256")):
            with self.subTest(filename=filename):
                path = self.bag / filename
                original = path.read_bytes()
                path.write_bytes(original + b"changed")
                with self.assertRaisesRegex(adapter.EvaluationError, key):
                    self.validate_association(manifest)
                path.write_bytes(original)

    def test_malformed_reference_rows_fail_cleanly(self):
        for content in ("x_m,y_m,t_ns\n0,0\n", "x_m,y_m,t_ns\n0,0,0,extra\n"):
            with self.subTest(content=content):
                self.reference.write_text(content)
                with self.assertRaisesRegex(adapter.EvaluationError, "short or overlong"):
                    adapter.validate_reference(self.reference)

    def test_malformed_result_rows_fail_cleanly(self):
        for suffix in ("", ",extra"):
            row = self.write_row()
            lines = row.read_text().splitlines()
            fields = next(csv.reader([lines[1]]))
            if not suffix:
                fields.pop()
            else:
                fields.append("extra")
            with row.open("w", newline="") as stream:
                stream.write(lines[0] + "\n")
                csv.writer(stream).writerow(fields)
            with self.assertRaisesRegex(adapter.EvaluationError, "short or overlong"):
                adapter.read_single_result(row, self.bag)

    def test_impossible_progress_or_useful_duration_rejected(self):
        for changes, reason in (({"progress_ratio": "1.01"}, "Progress ratio"),
                                ({"mission_time_s": "18"}, "raw run duration")):
            with self.subTest(changes=changes):
                with self.assertRaisesRegex(adapter.EvaluationError, reason):
                    adapter.read_single_result(self.write_row(**changes), self.bag)
        # Only a rounding-sized excursion from finite-segment projection is allowed.
        result = adapter.read_single_result(self.write_row(progress_ratio="1.0000000000005"), self.bag)
        self.assertGreater(result["progress_ratio"], 1)

    def test_negative_error_and_rms_metrics_rejected(self):
        for key in ("rmse_y_primary_m", "p95_y_primary_m", "rmse_psi_primary_rad",
                    "rms_jx_mps3", "rms_dotdelta_radps", "rmse_v_mps", "rms_cmd_w_radps",
                    "rmse_y_finite_segment_diagnostic_m", "final_dist_to_goal_m"):
            with self.subTest(key=key):
                with self.assertRaisesRegex(adapter.EvaluationError, "cannot be negative"):
                    adapter.read_single_result(self.write_row(**{key: "-0.1"}), self.bag)

    def test_tracking_exclusion_uses_sample_count_not_only_has_plan(self):
        changes = {"valid_for_tracking": "0", "plan_tracking_samples": "1",
                   "plan_tracking_coverage": "1", "rmse_y_primary_m": "",
                   "p95_y_primary_m": "", "rmse_psi_primary_rad": ""}
        result = adapter.read_single_result(self.write_row(**changes), self.bag)
        self.assertEqual(result["valid_for_tracking"], 0)
        changes["plan_tracking_samples"] = "2"
        with self.assertRaisesRegex(adapter.EvaluationError, "Tracking exclusion"):
            adapter.read_single_result(self.write_row(**changes), self.bag)

    def test_main_rejects_malformed_inputs_without_traceback_or_output(self):
        manifest = self.root / "association.json"
        fake_evaluator = self.root / "eval.py"
        fake_evaluator.write_text("# never executed\n")
        output = self.root / "new-output"
        args = ["--analysis-repo", str(self.root), "--bag-dir", str(self.bag),
                "--reference-csv", str(self.reference), "--mission-route", str(self.route),
                "--association-manifest", str(manifest), "--trial-id", "candidate_a",
                "--attempt-id", "attempt_1", "--out-dir", str(output), "--scenario-id", "1",
                "--speed-kmh", "20", "--wheelbase-m", "6.804"]
        for malformed_json, malformed_reference in (("[]", False), ("null", False), (None, True)):
            with self.subTest(malformed_json=malformed_json, malformed_reference=malformed_reference):
                self.reference.write_text("x_m,y_m,t_ns\n0,0,0\n10,0,1\n")
                manifest.write_text(malformed_json or json.dumps(self.association))
                if malformed_reference:
                    self.reference.write_text("x_m,y_m,t_ns\n0,0\n")
                stderr = io.StringIO()
                with mock.patch.object(adapter, "validate_analysis_repo", return_value=(fake_evaluator, "fixture")):
                    with contextlib.redirect_stderr(stderr):
                        code = adapter.main(args)
                self.assertEqual(code, 2)
                self.assertEqual(json.loads(stderr.getvalue())["status"], "PREFLIGHT_REJECTED")
                self.assertNotIn("Traceback", stderr.getvalue())
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
