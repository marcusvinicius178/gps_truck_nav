"""Offline contract regressions using artificial Docker/Git observations only."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "tools/auto_tuning/local_phase_a.py"
SPEC = importlib.util.spec_from_file_location("phase_a", SCRIPT)
phase = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(phase)
IMAGE = "sha256:" + "a" * 64
CID = "b" * 64
SOURCE = {"commit": "c" * 40, "tree": "d" * 40,
          "input_sha256": {"fixture": "e" * 64}}


class FakeRunner:
    """Records commands; never contacts Docker or starts ROS."""
    def __init__(self, output, fail=None):
        self.output = output
        self.calls = []
        self.running = False
        self.run_id = ""
        self.fail = fail
        self.stop_count = 0
        self.readiness = {'schema_version': 1, 'collection_status': 'COMPLETE',
                          'diagnostic_status': 'BLOCKED', 'mission_ready': False,
                          'gate0': 'NOT_YET_VERIFIED', 'gate1': 'NOT_YET_VERIFIED'}

    def inspect(self):
        return [{"Id": CID, "Config": {"Labels": {"project": "nav2-phase-a", phase.LABEL: self.run_id}},
                 "State": {"Running": self.running, "Pid": 123 if self.running else 0}}]

    def run(self, name, argv, **kwargs):
        self.calls.append((name, argv, kwargs))
        if self.fail == name:
            raise subprocess.TimeoutExpired(argv, kwargs.get("timeout", 1))
        if name == "docker_info":
            return json.dumps({"os_type": "linux", "architecture": "x86_64", "docker_root_dir": str(self.output)})
        if name == "base_inspect":
            return json.dumps([{"RepoDigests": ["osrf/ros@" + IMAGE]}])
        if name == "build":
            Path(argv[argv.index("--iidfile") + 1]).write_text(IMAGE)
        if name == "image_inspect":
            return json.dumps([{"Id": IMAGE, "Architecture": "amd64", "Os": "linux"}])
        if name.endswith("_create"):
            Path(argv[argv.index("--cidfile") + 1]).write_text(CID)
            self.run_id = next(item.split("=", 1)[1] for item in argv if item.startswith(phase.LABEL + "="))
        if name == "headless_start":
            self.running = True
        if name == 'runtime_probe' and self.readiness is not None:
            path = self.output / 'runtime_probe'
            path.mkdir()
            (path / 'readiness.json').write_text(json.dumps(self.readiness))
        if name == "image_validation_run":
            return json.dumps({"status": "IMAGE_SMOKE_VALIDATED"})
        if name == "package_inventory":
            Path(argv[-1]).write_text("fixture-package\t1\n")
        if name in ("headless_stop", "image_validation_stop"):
            self.running = False
            self.stop_count += 1
        if name.endswith(("_before_stop", "_after_stop")) or name.startswith("warmup_") or name == "headless_after_probe":
            return json.dumps(self.inspect())
        return ""


class LocalPhaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        self.out = self.root / "output"
        self.out.mkdir()
        self.build_dir = self.root / "build"
        self.build_dir.mkdir()
        (self.build_dir / "status.json").write_text(json.dumps({
            "stage": "build", "status": "IMAGE_VALIDATED", "image_id": IMAGE, "source": SOURCE}))
        (self.build_dir / "image_id.txt").write_text(IMAGE)
        self.state = {"run_id": "0123456789abcdef", "stage": "probe",
                      "gate0": "NOT_YET_VERIFIED", "gate1": "NOT_YET_VERIFIED"}
        self.runner = FakeRunner(self.out)
        self.docker = ["docker", "--host", phase.ENDPOINT]

    def tearDown(self):
        self.temp.cleanup()

    def probe(self, runner=None):
        with mock.patch.object(phase, "source_snapshot", return_value=SOURCE):
            phase.probe(self.repo, self.build_dir, self.out, runner or self.runner,
                        self.docker, self.state, warmup_seconds=1, sleep=lambda _: None)

    def test_probe_is_isolated_bounded_and_never_sends_goal(self):
        self.probe()
        calls = self.runner.calls
        create = next(argv for name, argv, _ in calls if name == "headless_create")
        for option, value in (("--network", "none"), ("--cpus", "8"), ("--memory", "16g"), ("--memory-swap", "16g")):
            self.assertEqual(create[create.index(option) + 1], value)
        for forbidden in ("--privileged", "--gpus", "--device", "--pid", "--ipc", "--volume", "--publish"):
            self.assertNotIn(forbidden, create)
        self.assertEqual(create.count("--mount"), 1)
        self.assertIn(f"type=bind,src={self.out},dst=/evidence", create)
        self.assertIn("xvfb-run", create)
        self.assertIn("publish_legacy_cloud_tf:=false", create)
        self.assertIn("use_ackermann_bt:=true", create)
        joined = " ".join(" ".join(argv) for _, argv, _ in calls)
        for forbidden in ("send_goal", "logged_waypoint_follower.py", "system prune", " rm ", " rmi ", "use_rviz:=true", "--network host"):
            self.assertNotIn(forbidden, joined)
        timeout = next(kw["timeout"] for name, _, kw in calls if name == "runtime_probe")
        self.assertEqual(timeout, 900)
        self.assertEqual(self.runner.stop_count, 1)
        self.assertEqual(self.state["status"], "OBSERVATIONS_COLLECTED")
        self.assertEqual(self.state["gate1"], "NOT_YET_VERIFIED")
        self.assertEqual(self.state['diagnostic_status'], 'BLOCKED')
        self.assertFalse(self.state['mission_ready'])
        self.assertEqual(len(self.state['readiness_sha256']), 64)

    def test_missing_readiness_report_cannot_be_claimed_as_complete_probe(self):
        self.runner.readiness = None
        with self.assertRaises(FileNotFoundError):
            self.probe()
        self.assertEqual(self.runner.stop_count, 1)
        self.assertNotEqual(self.state.get('status'), 'OBSERVATIONS_COLLECTED')

    def test_readiness_cannot_approve_a_mission_or_gate(self):
        self.runner.readiness['mission_ready'] = True
        with self.assertRaisesRegex(phase.PhaseError, 'readiness contract'):
            self.probe()
        self.assertEqual(self.runner.stop_count, 1)

    def test_failed_readiness_capture_preserves_failure_and_stops_owned_container(self):
        self.runner.readiness.update(collection_status='FAILED', diagnostic_status='INCOMPLETE')
        with self.assertRaisesRegex(phase.PhaseError, 'capture failed'):
            self.probe()
        self.assertEqual(self.runner.stop_count, 1)
        self.assertEqual(self.state['readiness_collection_status'], 'FAILED')
        self.assertEqual(self.state['diagnostic_status'], 'INCOMPLETE')
        self.assertNotEqual(self.state.get('status'), 'OBSERVATIONS_COLLECTED')

    def test_probe_timeout_still_stops_only_owned_container(self):
        self.runner.fail = "runtime_probe"
        with self.assertRaises(subprocess.TimeoutExpired):
            self.probe()
        stops = [argv for name, argv, _ in self.runner.calls if name in ("headless_stop", "image_validation_stop")]
        self.assertEqual(stops, [self.docker + ["stop", "--time", "20", CID]])
        self.assertFalse(self.runner.running)
        self.assertFalse(self.state["cleanup"]["gate_reset_proven"])

    def test_log_failure_does_not_prevent_cleanup_or_claim_complete_evidence(self):
        self.runner.fail = "bringup"
        with self.assertRaisesRegex(phase.PhaseError, "logs could not"):
            self.probe()
        self.assertEqual(self.runner.stop_count, 1)
        self.assertIsNotNone(self.state["cleanup"]["log_error"])

    def test_nonzero_docker_logs_marks_evidence_incomplete_after_stop(self):
        real_runner = phase.Runner(self.out)
        original_run = self.runner.run
        def run(name, argv, **kwargs):
            if name == "bringup":
                self.assertFalse(self.runner.running)
                return real_runner.run(name, [sys.executable, "-c", "raise SystemExit(1)"], **kwargs)
            return original_run(name, argv, **kwargs)
        self.runner.run = run
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(phase.PhaseError, "logs could not"):
            self.probe()
        self.assertEqual(self.runner.stop_count, 1)
        self.assertIn("exited 1", self.state["cleanup"]["log_error"])

    def test_structural_output_overflow_fails_closed_but_build_logs_are_retained(self):
        runner = phase.Runner(self.out)
        command = [sys.executable, "-c", "import sys; sys.stdout.write('x' * 2_000_001)"]
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(phase.PhaseError, "structural parsing limit"):
                runner.run("source_status", command)
            self.assertEqual(runner.run("build", command, capture=False), "")
        self.assertEqual((self.out / "source_status.log").stat().st_size, 2_000_001)
        self.assertEqual((self.out / "build.log").stat().st_size, 2_000_001)

    def test_source_mismatch_rejected_before_container_creation(self):
        with mock.patch.object(phase, "source_snapshot", return_value={**SOURCE, "commit": "f" * 40}):
            with self.assertRaisesRegex(phase.PhaseError, "source differs"):
                phase.probe(self.repo, self.build_dir, self.out, self.runner, self.docker, self.state)
        self.assertFalse(any(name.endswith("_create") for name, _, _ in self.runner.calls))

    def test_build_records_must_be_successful_and_immutable(self):
        (self.build_dir / "image_id.txt").write_text("sha256:" + "f" * 64)
        with self.assertRaisesRegex(phase.PhaseError, "disagree"):
            self.probe()
        with self.assertRaises(phase.PhaseError):
            phase.image_inspection(self.runner, self.docker, "mutable:latest", "image_inspect")

    def test_ownership_mismatch_never_stops_somebody_elses_container(self):
        (self.out / "container.id").write_text(CID)
        self.runner.run_id = "unrelated-project"
        with self.assertRaisesRegex(phase.PhaseError, "ownership"):
            phase.cleanup_container(self.runner, self.docker, self.out / "container.id", "ours", "headless")
        self.assertEqual(self.runner.stop_count, 0)

    def test_build_smoke_timeout_cleans_up_validator_container(self):
        self.runner.fail = "image_validation_run"
        with mock.patch.object(phase, "source_snapshot", return_value=SOURCE):
            with self.assertRaises(subprocess.TimeoutExpired):
                phase.build(self.repo, self.out, self.runner, self.docker, self.state)
        self.assertEqual(self.runner.stop_count, 1)
        self.assertEqual(self.state["validation_cleanup"]["status"], "OWNED_CONTAINER_STOPPED")

    def test_build_uses_digest_default_builder_and_preserves_gates(self):
        with mock.patch.object(phase, "source_snapshot", return_value=SOURCE):
            phase.build(self.repo, self.out, self.runner, self.docker, self.state)
        cmd = next(argv for name, argv, _ in self.runner.calls if name == "build")
        self.assertEqual(cmd[cmd.index("--builder") + 1], "default")
        self.assertIn("ROS_BASE=osrf/ros@" + IMAGE, cmd)
        self.assertNotIn("--tag", cmd)
        self.assertEqual(self.state["status"], "IMAGE_VALIDATED")
        self.assertEqual(self.state["gate0"], "NOT_YET_VERIFIED")
        self.assertIn("docker_root", self.state["disk"])
        self.assertIn("containerd_default_candidate", self.state["disk"])

    def test_docker_child_environment_drops_remote_and_ambient_overrides(self):
        values = {key: "untrusted" for key in ("DOCKER_HOST", "DOCKER_CONTEXT", "DOCKER_TLS_VERIFY",
                  "DOCKER_CERT_PATH", "DOCKER_DEFAULT_PLATFORM", "BUILDX_BUILDER", "BUILDKIT_HOST", "DOCKER_BUILDKIT")}
        with mock.patch.dict(os.environ, values):
            env = phase.docker_environment()
        self.assertFalse(set(values).intersection(env))

    def test_existing_output_is_never_overwritten(self):
        marker = self.out / "keep.txt"
        marker.write_text("owned by earlier attempt")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            phase.main(["build", "--repo", str(self.repo), "--output-dir", str(self.out)])
        self.assertEqual(marker.read_text(), "owned by earlier attempt")

    def test_failure_still_writes_status_without_fake_gate_pass(self):
        output = self.root / "new_failure"
        with mock.patch.object(phase, "docker_prefix", side_effect=phase.PhaseError("fixture unavailable")), contextlib.redirect_stdout(io.StringIO()):
            code = phase.main(["build", "--repo", str(self.repo), "--output-dir", str(output)])
        value = json.loads((output / "status.json").read_text())
        self.assertEqual(code, 2)
        self.assertEqual(value["status"], "FAILED")
        self.assertFalse(value["mission_sent"])
        self.assertEqual(value["gate1"], "NOT_YET_VERIFIED")

    def test_source_must_be_clean_and_mesh_must_be_hydrated(self):
        class GitRunner:
            dirty = False
            def run(inner, name, argv, **kwargs):
                if name.endswith("_root"):
                    return str(self.repo)
                if name.endswith("_status"):
                    return " M file" if inner.dirty else ""
                if name.endswith("_ignored"):
                    return ""
                if name.endswith("_files"):
                    return "src/gps_truck_nav/truck_bringup/meshes/a.dae\0"
                return "a" * 40
        runner = GitRunner()
        runner.dirty = True
        with self.assertRaisesRegex(phase.PhaseError, "not clean"):
            phase.source_snapshot(self.repo, runner)
        runner.dirty = False
        mesh = self.repo / "src/gps_truck_nav/truck_bringup/meshes/a.dae"
        mesh.parent.mkdir(parents=True)
        mesh.write_text("version https://git-lfs.github.com/spec/v1\noid sha256:fixture\n")
        with self.assertRaisesRegex(phase.PhaseError, "Unhydrated"):
            phase.source_snapshot(self.repo, runner)

    def test_ignored_inputs_cannot_bypass_clean_source_freeze(self):
        class GitRunner:
            def run(inner, name, argv, **kwargs):
                if name.endswith("_root"):
                    return str(self.repo)
                if name.endswith("_status"):
                    return ""
                if name.endswith("_ignored"):
                    return "src/gps_truck_nav/truck_bringup/extra.yaml\0"
                return "a" * 40
        with self.assertRaisesRegex(phase.PhaseError, "Ignored untracked"):
            phase.source_snapshot(self.repo, GitRunner())


if __name__ == "__main__":
    unittest.main()
