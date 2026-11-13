#!/usr/bin/env python3
"""Render two prospective Smac candidates without starting ROS or a simulator.

Both configurations preserve every parsed source value except candidate B's
GridBased.non_straight_penalty (2.5 -> 2.6). These values are proposals for the
Gate 1 micro-PoC, not validated safe limits or an optimization result.

Run from any directory with Python 3 and PyYAML installed:
  python3 prepare_candidates.py --source /absolute/truck_nav2_params.yaml \
      --output-dir /absolute/new_candidate_directory

The output directory must not exist. This program never modifies the source.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import yaml


PARAMETER_PATH = (
    "planner_server", "ros__parameters", "GridBased", "non_straight_penalty"
)
BASELINE_VALUE = 2.5
CANDIDATE_VALUE = 2.6
TURNING_RADIUS_M = 11.6


class CandidateError(ValueError):
    """A source or output contract is invalid."""


class UniqueSafeLoader(yaml.SafeLoader):
    """Safe YAML loader which also rejects ambiguous duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise CandidateError("All YAML mapping keys must be strings")
        if key in result:
            raise CandidateError("Duplicate YAML key: " + key)
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueSafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _validate_tree(value, path=(), ancestors=None):
    """Reject non-ROS scalar types, nonfinite numbers and cyclic YAML aliases."""
    ancestors = set() if ancestors is None else ancestors
    if isinstance(value, (dict, list)):
        if id(value) in ancestors:
            raise CandidateError("Cyclic YAML alias at " + ".".join(path))
        ancestors = ancestors | {id(value)}
        entries = value.items() if isinstance(value, dict) else enumerate(value)
        for key, item in entries:
            _validate_tree(item, path + (str(key),), ancestors)
    elif isinstance(value, float) and not math.isfinite(value):
        raise CandidateError("Nonfinite value at " + ".".join(path))
    elif value is not None and type(value) not in (str, bool, int, float):
        raise CandidateError("Unsupported YAML value at " + ".".join(path))


def _get(tree, path):
    cursor = tree
    for key in path:
        if not isinstance(cursor, dict) or key not in cursor:
            raise CandidateError("Missing mapping/key: " + ".".join(path))
        cursor = cursor[key]
    return cursor


def _require_float(tree, path, expected):
    value = _get(tree, path)
    # ROS double parameters must remain doubles, not booleans or integers.
    if type(value) is not float or not math.isfinite(value) or value != expected:
        raise CandidateError(
            ".".join(path) + " must be the finite double " + str(expected)
        )


def load_source(raw):
    """Parse the public Smac/MPPI source and enforce fixed physical constraints."""
    try:
        tree = yaml.load(raw.decode("utf-8"), Loader=UniqueSafeLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as error:
        raise CandidateError("Invalid UTF-8 YAML: " + str(error)) from error
    if not isinstance(tree, dict):
        raise CandidateError("The source must be a ROS parameter mapping")
    _validate_tree(tree)
    _require_float(tree, PARAMETER_PATH, BASELINE_VALUE)
    _require_float(tree, (
        "planner_server", "ros__parameters", "GridBased", "minimum_turning_radius"
    ), TURNING_RADIUS_M)
    _require_float(tree, (
        "controller_server", "ros__parameters", "FollowPath", "AckermannConstraints",
        "min_turning_r"
    ), TURNING_RADIUS_M)
    if _get(tree, ("planner_server", "ros__parameters", "planner_plugins")) != ["GridBased"]:
        raise CandidateError("Expected the single active GridBased planner")
    if _get(tree, ("planner_server", "ros__parameters", "GridBased", "plugin")) != \
            "nav2_smac_planner/SmacPlannerHybrid":
        raise CandidateError("Expected the public Iron SmacPlannerHybrid plugin")
    if _get(tree, ("controller_server", "ros__parameters", "controller_plugins")) != ["FollowPath"]:
        raise CandidateError("Expected the single active FollowPath controller")
    if _get(tree, ("controller_server", "ros__parameters", "FollowPath", "plugin")) != \
            "nav2_mppi_controller::MPPIController":
        raise CandidateError("Expected the public MPPIController plugin")
    if _get(tree, ("controller_server", "ros__parameters", "FollowPath", "motion_model")) != "Ackermann":
        raise CandidateError("Expected the fixed Ackermann motion model")
    return tree


def differences(left, right, path=()):
    """Return paths with differences, including scalar-type differences."""
    if type(left) is not type(right):
        return [path]
    if isinstance(left, dict):
        if left.keys() != right.keys():
            return [path]
        return [item for key in left for item in differences(left[key], right[key], path + (key,))]
    if isinstance(left, list):
        if len(left) != len(right):
            return [path]
        return [item for index, (a, b) in enumerate(zip(left, right))
                for item in differences(a, b, path + (str(index),))]
    return [] if left == right else [path]


def _sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def prepare_candidates(source, output_dir):
    """Write complete YAMLs and a provenance manifest to a new directory."""
    source = Path(source).resolve(strict=True)
    output_dir = Path(output_dir)
    # Reject even a dangling symlink instead of following or replacing it.
    if output_dir.exists() or output_dir.is_symlink():
        raise CandidateError("Output directory already exists: " + str(output_dir))
    source_raw = source.read_bytes()
    baseline = load_source(source_raw)
    variant = copy.deepcopy(baseline)
    variant["planner_server"]["ros__parameters"]["GridBased"]["non_straight_penalty"] = CANDIDATE_VALUE
    if differences(baseline, variant) != [PARAMETER_PATH]:
        raise CandidateError("Candidate B must differ in exactly one parameter")
    variant_raw = yaml.safe_dump(variant, sort_keys=False, allow_unicode=True).encode("utf-8")
    # A preserves exact source bytes; B is full YAML with normalized formatting.
    variant_round_trip = yaml.load(variant_raw.decode("utf-8"), Loader=UniqueSafeLoader)
    if differences(variant, variant_round_trip):
        raise CandidateError("YAML serialization changed candidate values or types")
    outputs = {"candidate_a.yaml": source_raw, "candidate_b.yaml": variant_raw}
    manifest = {
        "schema_version": 1,
        "status": "proposed_not_runtime_tested",
        "gate0": "NOT_YET_VERIFIED",
        "gate1": "NOT_YET_VERIFIED",
        "purpose": "Two-candidate micro-PoC preparation only; no execution evidence",
        "source": {"path": str(source), "sha256": _sha256(source_raw)},
        "parameter": {"yaml_path": list(PARAMETER_PATH), "ros_node": "/planner_server",
                      "ros_parameter": "GridBased.non_straight_penalty", "type": "double"},
        "seeds": {"gazebo": None, "mppi": None, "status": "unavailable_not_controlled"},
        "candidates": [
            {"id": "a", "file": "candidate_a.yaml", "intended_value": BASELINE_VALUE,
             "sha256": _sha256(source_raw), "effective_value_verified": False},
            {"id": "b", "file": "candidate_b.yaml", "intended_value": CANDIDATE_VALUE,
             "sha256": _sha256(variant_raw), "effective_value_verified": False},
        ],
        "physical_constraints": {"minimum_turning_radius_m": TURNING_RADIUS_M,
                                 "mppi_min_turning_r_m": TURNING_RADIUS_M},
        "next_required_evidence": [
            "Confirm exact installed Nav2 version and parameter support",
            "Verify effective values and all fixed parameters in each live trial",
            "Validate TF, localization, sensors, motion and mission readiness",
            "Record two missions, measured metrics, complete teardown and clean reset",
        ],
    }
    outputs["manifest.json"] = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    # Parent must exist: avoid creating a tree in an unintended working directory.
    output_dir.mkdir(exist_ok=False)
    try:
        for name, raw in outputs.items():
            with (output_dir / name).open("xb") as stream:
                stream.write(raw)
        if source.read_bytes() != source_raw:
            raise CandidateError("Source changed during preparation; output discarded")
    except BaseException:
        # This directory was exclusively created by this invocation.
        shutil.rmtree(output_dir)
        raise
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Frozen full public Nav2 YAML")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory; parent must exist")
    args = parser.parse_args(argv)
    try:
        manifest = prepare_candidates(args.source, args.output_dir)
    except (OSError, CandidateError, yaml.YAMLError) as error:
        print("Candidate preparation failed: " + str(error), file=sys.stderr)
        return 2
    print(json.dumps({"output_dir": str(args.output_dir.resolve()),
                      "status": manifest["status"], "gate1": manifest["gate1"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
