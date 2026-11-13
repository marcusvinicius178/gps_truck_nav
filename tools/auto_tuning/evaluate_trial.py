#!/usr/bin/env python3
"""Evaluate one NEW prospective MCAP bag with the unchanged frozen CEA evaluator.

Run inside a sourced ROS 2 environment, with rosbag2_py, message packages,
numpy, and the MCAP storage plugin. This adapter does not run a simulation or
prove either engineering gate. An association manifest records a human-reviewed
route/reference association; hashes alone do not prove geometric equivalence.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime, timezone


ANALYSIS_COMMIT = "36ba76823cc7a124540412cf3ce09c73e6cd1d05"
EVALUATOR_PATH = "scripts/simulation/eval_nav2_one_combination.py"
PLAN_TOPICS = "/plan,/received_global_plan,/transformed_global_plan,/plan_smoothed"
FLAG_FIELDS = ("success_geo", "valid_for_mission", "valid_for_tracking", "has_plan")
COUNT_FIELDS = ("plan_tracking_samples", "plan_message_count")
IDENTIFIER_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
# Finite-segment mission projection clamps tau to [0, 1]. Permit only tiny
# floating-point roundoff at the upper bound; never clip an impossible score.
PROGRESS_ROUNDOFF = 1e-12
NUMBER_FIELDS = (
    "progress_ratio", "mission_time_s", "raw_run_duration_s",
    "plan_tracking_coverage", "rmse_y_primary_m", "p95_y_primary_m",
    "rmse_psi_primary_rad", "rms_jx_mps3", "rms_dotdelta_radps",
    "rmse_v_mps", "rms_cmd_w_radps", "final_dist_to_goal_m",
    "final_yaw_err_to_goal_rad",
    "rmse_y_finite_segment_diagnostic_m", "p95_y_finite_segment_diagnostic_m",
)


class EvaluationError(ValueError):
    """Input, provenance or extraction failure; never a navigation score."""


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_bag(bag):
    bag = Path(bag).resolve(strict=True)
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise EvaluationError("A finalized bag directory with metadata.yaml is required")
    metadata = list(bag.rglob("metadata.yaml"))
    if len(metadata) != 1:
        raise EvaluationError("Exactly one bag is allowed; nested bags are rejected")
    mcap_files = sorted(bag.glob("*.mcap"))
    if len(mcap_files) != 1 or not mcap_files[0].is_file():
        raise EvaluationError("Exactly one MCAP file is required; split bags are unsupported by the frozen evaluator")
    if mcap_files[0].stat().st_size == 0:
        raise EvaluationError("The MCAP file is empty")
    return bag, mcap_files[0]


def validate_reference(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not {"x_m", "y_m", "t_ns"}.issubset(reader.fieldnames or []):
            raise EvaluationError("Reference CSV requires x_m, y_m, t_ns columns")
        if len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise EvaluationError("Reference CSV contains duplicate column names")
        points = set()
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise EvaluationError("Reference CSV contains a short or overlong row")
            values = [float(row[k]) for k in ("x_m", "y_m", "t_ns")]
            if not all(math.isfinite(v) for v in values):
                raise EvaluationError("Reference CSV contains nonfinite coordinates or time")
            points.add(tuple(values[:2]))
    if len(points) < 2:
        raise EvaluationError("Reference CSV requires at least two distinct XY points")


def validate_association(manifest, reference, route, scenario_id, speed_kmh,
                         bag, mcap, trial_id, attempt_id):
    value = json.loads(Path(manifest).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise EvaluationError("Association manifest must be a JSON object")
    for key, identifier in (("trial_id", trial_id), ("attempt_id", attempt_id)):
        if not isinstance(identifier, str) or not IDENTIFIER_RE.fullmatch(identifier):
            raise EvaluationError(f"{key} must be a nonempty identifier using letters, digits, dot, dash or underscore")
    required = {
        "schema_version": 2,
        "dataset_kind": "prospective_auto_tuning",
        "trial_id": trial_id,
        "attempt_id": attempt_id,
        "scenario_id": scenario_id,
        "speed_kmh": speed_kmh,
        "reference_sha256": sha256(reference),
        "mission_route_sha256": sha256(route),
        "bag_metadata_sha256": sha256(Path(bag) / "metadata.yaml"),
        "bag_mcap_sha256": sha256(mcap),
    }
    for key, expected in required.items():
        if type(value.get(key)) is not type(expected) or value.get(key) != expected:
            raise EvaluationError(f"Association manifest mismatch: {key}")
    if not isinstance(value.get("association_evidence"), str) or not value["association_evidence"].strip():
        raise EvaluationError("association_evidence must explain the reviewed route/reference relationship")
    return value


def validate_analysis_repo(repo):
    repo = Path(repo).resolve(strict=True)
    commit = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    if commit != ANALYSIS_COMMIT:
        raise EvaluationError(f"Expected analysis commit {ANALYSIS_COMMIT}; found {commit}")
    original = subprocess.check_output(
        ["git", "-C", str(repo), "show", f"{ANALYSIS_COMMIT}:{EVALUATOR_PATH}"])
    evaluator = repo / EVALUATOR_PATH
    expected_hash = hashlib.sha256(original).hexdigest()
    if sha256(evaluator) != expected_hash:
        raise EvaluationError("Evaluator worktree differs from the frozen commit")
    return evaluator, expected_hash


def read_single_result(path, expected_bag):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise EvaluationError("Evaluator CSV has missing or duplicate column names")
        rows = list(reader)
    if len(rows) != 1:
        raise EvaluationError(f"Expected exactly one evaluator row; found {len(rows)}")
    row = rows[0]
    if None in row or any(value is None for value in row.values()):
        raise EvaluationError("Evaluator CSV contains a short or overlong row")
    if not row.get("bag_dir") or Path(row["bag_dir"]).resolve() != Path(expected_bag).resolve():
        raise EvaluationError("Evaluator row does not belong to the requested bag")
    if row.get("eval_cutoff_reason") == "bag_read_failed" or "BAG_READ_FAILED" in row.get("notes", ""):
        raise EvaluationError("Evaluator reported BAG_READ_FAILED despite its process return code")
    result = dict(row)
    for key in FLAG_FIELDS:
        if row.get(key) not in ("0", "1"):
            raise EvaluationError(f"Missing or invalid evaluator flag: {key}")
        result[key] = int(row[key])
    for key in COUNT_FIELDS:
        raw = row.get(key, "")
        if not re.fullmatch(r"[0-9]+", raw):
            raise EvaluationError(f"Missing or invalid evaluator count: {key}")
        result[key] = int(raw)
    for key in NUMBER_FIELDS:
        raw = row.get(key, "")
        number = float(raw) if raw else None
        result[key] = number if number is not None and math.isfinite(number) else None
        if result[key] is not None and result[key] < 0:
            raise EvaluationError(f"Metric cannot be negative: {key}")
    for key in ("progress_ratio", "mission_time_s", "raw_run_duration_s", "plan_tracking_coverage"):
        if result[key] is None or result[key] < 0:
            raise EvaluationError(f"Required metric is missing, nonfinite or negative: {key}")
    if result["progress_ratio"] > 1.0 + PROGRESS_ROUNDOFF:
        raise EvaluationError("Progress ratio exceeds 1 beyond floating-point roundoff")
    if result["mission_time_s"] > result["raw_run_duration_s"] + 1e-9:
        raise EvaluationError("Useful mission time exceeds raw run duration")
    if not 0 <= result["plan_tracking_coverage"] <= 1:
        raise EvaluationError("Plan coverage lies outside [0, 1]")
    if result["success_geo"] != int(result["progress_ratio"] >= 0.90):
        raise EvaluationError("Geometric success contradicts the frozen 0.90 progress threshold")
    if result["valid_for_mission"] != int(result["progress_ratio"] >= 0.70):
        raise EvaluationError("Mission eligibility contradicts the frozen 0.70 progress threshold")
    if result["valid_for_tracking"]:
        if (not result["valid_for_mission"] or not result["has_plan"]
                or result["plan_message_count"] < 1 or result["plan_tracking_samples"] < 2
                or result["plan_tracking_coverage"] < 0.80):
            raise EvaluationError("Tracking eligibility contradicts mission/plan/coverage requirements")
        if any(result[k] is None for k in ("rmse_y_primary_m", "p95_y_primary_m", "rmse_psi_primary_rad")):
            raise EvaluationError("Eligible tracking has unavailable primary metrics")
    else:
        # The frozen evaluator's availability test is fully represented by
        # these exported counts/coverage. has_plan alone is insufficient.
        if (result["valid_for_mission"] and result["plan_tracking_samples"] >= 2
                and result["plan_tracking_coverage"] >= 0.80):
            raise EvaluationError("Tracking exclusion contradicts exported sample count and coverage")
        if any(result[k] is not None for k in ("rmse_y_primary_m", "p95_y_primary_m", "rmse_psi_primary_rad")):
            raise EvaluationError("Ineligible tracking must preserve unavailable primary metrics")
    return result


def build_command(evaluator, bag, reference, output, scenario, speed, wheelbase):
    return [sys.executable, str(evaluator), "--runs-dir", str(bag),
            "--scenario-id", str(scenario), "--speed-kmh", str(speed),
            "--planner-id", "SMAC", "--controller-id", "MPPI",
            "--gt-csv", str(reference), "--out-dir", str(output / "evaluator"),
            "--wheelbase-m", str(wheelbase), "--steering-min-speed-mps", "0.50",
            "--executed-topic", "/odometry/global", "--plan-topics", PLAN_TOPICS,
            "--cmd-topics", "/cmd_vel_nav,/cmd_vel", "--success-pr-threshold", "0.90",
            "--tracking-pr-threshold", "0.70", "--min-plan-coverage", "0.80"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-repo", required=True, type=Path)
    parser.add_argument("--bag-dir", required=True, type=Path)
    parser.add_argument("--reference-csv", required=True, type=Path)
    parser.add_argument("--mission-route", required=True, type=Path)
    parser.add_argument("--association-manifest", required=True, type=Path)
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--scenario-id", required=True, type=int)
    parser.add_argument("--speed-kmh", required=True, type=int)
    parser.add_argument("--wheelbase-m", required=True, type=float)
    parser.add_argument("--timeout-seconds", type=float, default=300.0)
    parser.add_argument("--preflight-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.scenario_id not in (1, 2, 3) or args.speed_kmh <= 0:
            raise EvaluationError("Frozen evaluator supports reviewed scenario IDs 1, 2 or 3 and positive nominal speed")
        if not math.isfinite(args.wheelbase_m) or args.wheelbase_m <= 0:
            raise EvaluationError("Explicit validated positive finite equivalent wheelbase is required")
        if not math.isfinite(args.timeout_seconds) or args.timeout_seconds <= 0:
            raise EvaluationError("Timeout must be positive and finite")
        evaluator, evaluator_hash = validate_analysis_repo(args.analysis_repo)
        bag, mcap = validate_bag(args.bag_dir)
        reference = args.reference_csv.resolve(strict=True)
        route = args.mission_route.resolve(strict=True)
        validate_reference(reference)
        association_manifest_hash = sha256(args.association_manifest)
        association = validate_association(args.association_manifest, reference, route,
                                           args.scenario_id, args.speed_kmh, bag, mcap,
                                           args.trial_id, args.attempt_id)
        if sha256(args.association_manifest) != association_manifest_hash:
            raise EvaluationError("Association manifest changed during preflight")
        output = args.out_dir.resolve()
        if output.exists():
            raise EvaluationError("Output directory already exists; use a new path for every extraction attempt")
        if output.is_relative_to(bag) or output.is_relative_to(args.analysis_repo.resolve()):
            raise EvaluationError("Output must be outside the input bag and frozen analysis repository")
        command = build_command(evaluator, bag, reference, output, args.scenario_id,
                                args.speed_kmh, args.wheelbase_m)
        provenance = {
            "schema_version": 2, "dataset_kind": "prospective_auto_tuning",
            "trial_id": args.trial_id, "attempt_id": args.attempt_id,
            "analysis_commit": ANALYSIS_COMMIT, "evaluator_sha256": evaluator_hash,
            "reference_sha256": association["reference_sha256"],
            "mission_route_sha256": association["mission_route_sha256"],
            "bag_metadata_sha256": association["bag_metadata_sha256"],
            "bag_mcap_sha256": association["bag_mcap_sha256"], "association": association,
            "association_manifest_sha256": association_manifest_hash,
            "association_verification": "DECLARED_IN_MANIFEST_NOT_GEOMETRICALLY_PROVEN_BY_ADAPTER",
            "command": command, "wheelbase_m": args.wheelbase_m,
            "action_status": None, "engineering_gate_status": "NOT_ASSESSED",
            "human_likeness": "DISABLED_RAW_REFERENCE_INPUTS_NOT_PROVIDED",
        }
        if args.preflight_only:
            print(json.dumps({"status": "STATIC_PREFLIGHT_ONLY", **provenance}, indent=2))
            return 0
        output.mkdir(parents=True, exist_ok=False)
        provenance["started_at_utc"] = datetime.now(timezone.utc).isoformat()
        (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        outcome = {"schema_version": 2, "status": "MEASUREMENT_FAILURE", "metrics": None,
                   "trial_id": args.trial_id, "attempt_id": args.attempt_id,
                   "engineering_gate_status": "NOT_ASSESSED", "action_status": None}
        try:
            with (output / "evaluator.log").open("w", encoding="utf-8") as log:
                completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                           timeout=args.timeout_seconds, check=False)
            if completed.returncode != 0:
                raise EvaluationError(f"Evaluator process exited {completed.returncode}; inspect evaluator.log")
            result = read_single_result(output / "evaluator" / "per_run_metrics.csv", bag)
            if sha256(mcap) != provenance["bag_mcap_sha256"] or sha256(bag / "metadata.yaml") != provenance["bag_metadata_sha256"]:
                raise EvaluationError("Bag changed during extraction; finalize recording before evaluation")
            for path, expected in ((reference, provenance["reference_sha256"]),
                                   (route, provenance["mission_route_sha256"]),
                                   (evaluator, evaluator_hash),
                                   (args.association_manifest, provenance["association_manifest_sha256"])):
                if sha256(path) != expected:
                    raise EvaluationError("An evaluator or association input changed during extraction")
            outcome.update(status="METRICS_EXTRACTED", metrics=result)
        except KeyboardInterrupt:
            outcome.update(status="INTERRUPTED", reason="Extraction interrupted; retain this attempt and use a new output directory")
        except (EvaluationError, OSError, ValueError, subprocess.SubprocessError) as error:
            outcome["reason"] = str(error)
        outcome["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        (output / "metrics.json").write_text(json.dumps(outcome, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"status": outcome["status"], "output": str(output)}))
        return 0 if outcome["status"] == "METRICS_EXTRACTED" else (130 if outcome["status"] == "INTERRUPTED" else 3)
    except (EvaluationError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "PREFLIGHT_REJECTED", "reason": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
