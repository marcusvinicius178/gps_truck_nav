#!/usr/bin/env python3
"""Build or probe the isolated Iron image; never send goals or approve gates.

Run with /usr/bin/python3 -I. The host needs Git/LFS and a working local Docker
daemon, but no host ROS/Python package installation. Every invocation creates a
new evidence directory. Docker containers are retained stopped for inspection.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid


ENDPOINT = "unix:///var/run/docker.sock"
BASE_TAG = "osrf/ros:iron-desktop-full"
IMAGE_RE = re.compile(r"sha256:[0-9a-f]{64}\Z")
CID_RE = re.compile(r"[0-9a-f]{64}\Z")
DIGEST_RE = re.compile(r"osrf/ros@sha256:[0-9a-f]{64}\Z")
LABEL = "nav2.phase_a.run_id"


class PhaseError(RuntimeError):
    pass


def now():
    return datetime.now(timezone.utc).isoformat()


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def docker_environment():
    env = os.environ.copy()
    for key in ("DOCKER_HOST", "DOCKER_CONTEXT", "DOCKER_TLS", "DOCKER_TLS_VERIFY",
                "DOCKER_CERT_PATH", "DOCKER_API_VERSION", "DOCKER_DEFAULT_PLATFORM",
                "BUILDX_BUILDER", "BUILDKIT_HOST", "DOCKER_BUILDKIT"):
        env.pop(key, None)
    return env


class Runner:
    def __init__(self, output):
        self.output = output
        self.env = docker_environment()

    def run(self, name, argv, *, cwd=None, timeout=60, check=True, capture=True):
        log_path = self.output / (name + ".log")
        record = {"name": name, "argv": list(argv), "started_at_utc": now(),
                  "timeout_seconds": timeout, "cwd": str(cwd) if cwd else None}
        print(f"[{name}] Log: {log_path}", flush=True)
        try:
            with log_path.open("xb") as log:
                result = subprocess.run(argv, cwd=cwd, env=self.env, stdout=log,
                                        stderr=subprocess.STDOUT, timeout=timeout, check=False)
            record["returncode"] = result.returncode
        except BaseException as error:
            record.update(returncode=None, error=type(error).__name__)
            raise
        finally:
            record["finished_at_utc"] = now()
            with (self.output / "commands.jsonl").open("a") as stream:
                stream.write(json.dumps(record) + "\n")
        if check and result.returncode:
            raise PhaseError(f"{name} exited {result.returncode}; inspect {log_path.name}")
        if not capture:
            return ""
        if log_path.stat().st_size >= 2_000_000:
            raise PhaseError(f"{name} output exceeds structural parsing limit; inspect {log_path.name}")
        return log_path.read_text(errors="replace")


def source_snapshot(repo, runner, prefix="source"):
    root = Path(runner.run(prefix + "_root", ["git", "-C", str(repo), "rev-parse", "--show-toplevel"]).strip())
    if root.resolve() != repo:
        raise PhaseError("--repo must be the Git repository root")
    dirty = runner.run(prefix + "_status", ["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=all"])
    if dirty.strip():
        raise PhaseError("Source checkout is not clean; preserve changes and use a new checkout")
    commit = runner.run(prefix + "_commit", ["git", "-C", str(repo), "rev-parse", "HEAD"]).strip()
    tree = runner.run(prefix + "_tree", ["git", "-C", str(repo), "rev-parse", "HEAD^{tree}"]).strip()
    ignored = runner.run(prefix + "_ignored", ["git", "-C", str(repo), "ls-files", "--others", "--ignored", "--exclude-standard", "-z"])
    for relative in filter(None, ignored.split("\0")):
        if (relative.startswith(("src/gps_truck_nav/truck_bringup/", "docker/auto_tuning/",
                                "config/auto_tuning/", "tools/auto_tuning/"))
                and "__pycache__" not in Path(relative).parts and not relative.endswith(".pyc")):
            raise PhaseError(f"Ignored untracked build input could bypass source freeze: {relative}")
    paths = runner.run(prefix + "_files", ["git", "-C", str(repo), "ls-files", "-z"]).split("\0")
    hashes = {}
    for relative in filter(None, paths):
        if (relative == "entrypoint.sh" or relative.startswith(("src/gps_truck_nav/truck_bringup/",
                "docker/auto_tuning/", "config/auto_tuning/", "tools/auto_tuning/"))):
            path = repo / relative
            if path.is_symlink() or not path.is_file():
                raise PhaseError(f"Build input must be a regular tracked file: {relative}")
            if path.suffix == ".dae":
                with path.open("rb") as stream:
                    if stream.read(100).startswith(b"version https://git-lfs.github.com/spec/v1"):
                        raise PhaseError(f"Unhydrated Git LFS mesh: {relative}; run git lfs pull")
            hashes[relative] = file_hash(path)
    for required in ("docker/auto_tuning/Dockerfile", "tools/auto_tuning/validate_image.sh",
                     "tools/auto_tuning/collect_runtime_evidence.sh"):
        if required not in hashes:
            raise PhaseError(f"Missing tracked build input: {required}")
    return {"commit": commit, "tree": tree, "input_sha256": hashes}


def docker_prefix():
    executable = "/usr/bin/docker" if Path("/usr/bin/docker").is_file() else shutil.which("docker")
    if not executable:
        raise PhaseError("Docker CLI unavailable; this helper does not install software")
    return [executable, "--host", ENDPOINT]


def one_json(text, label):
    try:
        data = json.loads(text)
    except (ValueError, TypeError) as error:
        raise PhaseError(f"Invalid JSON from {label}") from error
    return data


def inspect_container(runner, docker, cid, name):
    data = one_json(runner.run(name, docker + ["inspect", cid]), name)
    if not isinstance(data, list) or len(data) != 1 or not isinstance(data[0], dict):
        raise PhaseError("Unexpected container inspection result")
    return data[0]


def owned_id(cid_file):
    if not cid_file.is_file():
        return None
    cid = cid_file.read_text().strip()
    if not CID_RE.fullmatch(cid):
        raise PhaseError("Invalid CID evidence; refuse cleanup by an ambiguous identity")
    return cid


def cleanup_container(runner, docker, cid_file, run_id, prefix):
    cid = owned_id(cid_file)
    if cid is None:
        return {"status": "NO_CONTAINER_ID_CREATED"}
    before = inspect_container(runner, docker, cid, prefix + "_before_stop")
    save_json(runner.output / "container_before_stop.json", before)
    labels = before.get("Config", {}).get("Labels") or {}
    if before.get("Id") != cid or labels.get(LABEL) != run_id or labels.get("project") != "nav2-phase-a":
        raise PhaseError("Container ownership mismatch; refusing to stop it")
    stop_error = None
    try:
        runner.run(prefix + "_stop", docker + ["stop", "--time", "20", cid], timeout=40, capture=False)
    except (OSError, PhaseError, subprocess.SubprocessError) as error:
        stop_error = str(error)
    after = inspect_container(runner, docker, cid, prefix + "_after_stop")
    save_json(runner.output / "container_after_stop.json", after)
    state = after.get("State", {})
    if state.get("Running") is not False or state.get("Pid") != 0:
        raise PhaseError("Owned container did not reach Running=false, Pid=0")
    log_error = None
    try:
        runner.run("bringup" if prefix == "headless" else "image_validation_stdout",
                   docker + ["logs", cid], capture=False)
    except (OSError, PhaseError, subprocess.SubprocessError) as error:
        log_error = str(error)
    return {"status": "OWNED_CONTAINER_STOPPED", "container_id": cid,
            "running": False, "pid": 0, "gate_reset_proven": False,
            "log_error": log_error, "stop_command_error": stop_error}


def container_options(output, run_id, role, *, readonly=False):
    options = ["create", "--cidfile", str(output / "container.id"),
               "--name", f"nav2-phase-a-{role}-{run_id[:12]}", "--init", "--network", "none",
               "--cpus", "8", "--memory", "16g", "--memory-swap", "16g", "--pids-limit", "2048",
               "--shm-size", "1g", "--label", "project=nav2-phase-a", "--label", LABEL + "=" + run_id,
               "--env", "ROS_LOCALHOST_ONLY=1", "--env", "ROS_DOMAIN_ID=71",
               "--env", "LIBGL_ALWAYS_SOFTWARE=1"]
    if readonly:
        options += ["--read-only", "--tmpfs", "/tmp:rw,nosuid,nodev,size=512m"]
    else:
        options += ["--mount", f"type=bind,src={output},dst=/evidence"]
    return options


def image_inspection(runner, docker, image_id, name):
    if not IMAGE_RE.fullmatch(image_id):
        raise PhaseError("Expected immutable sha256 image ID")
    entries = one_json(runner.run(name, docker + ["image", "inspect", image_id]), name)
    if (not isinstance(entries, list) or len(entries) != 1 or not isinstance(entries[0], dict) or entries[0].get("Id") != image_id
            or entries[0].get("Architecture") != "amd64" or entries[0].get("Os") != "linux"):
        raise PhaseError("Image identity/platform differs from the frozen linux/amd64 image")
    return entries[0]


def readiness_result(output):
    path = output / 'runtime_probe/readiness.json'
    result = one_json(path.read_text(), 'runtime readiness report')
    if (not isinstance(result, dict) or result.get('schema_version') != 1
            or result.get('diagnostic_status') not in ('BLOCKED', 'INCOMPLETE', 'REVIEW_REQUIRED')
            or result.get('collection_status') not in ('COMPLETE', 'PARTIAL', 'FAILED')
            or result.get('mission_ready') is not False
            or result.get('gate0') != 'NOT_YET_VERIFIED'
            or result.get('gate1') != 'NOT_YET_VERIFIED'):
        raise PhaseError('Invalid readiness contract; no health or mission approval may be inferred')
    return result, file_hash(path)


def build(repo, output, runner, docker, status):
    source = source_snapshot(repo, runner)
    status["source"] = source
    save_json(output / "source_manifest.json", source)
    runner.run("docker_version", docker + ["version"], capture=False)
    runner.run("builder_inspect", docker + ["buildx", "inspect", "default"], capture=False)
    template = '{"server_version":{{json .ServerVersion}},"os_type":{{json .OSType}},"architecture":{{json .Architecture}},"docker_root_dir":{{json .DockerRootDir}},"storage_driver":{{json .Driver}},"driver_status":{{json .DriverStatus}},"cpus":{{json .NCPU}},"memory_bytes":{{json .MemTotal}}}'
    info = one_json(runner.run("docker_info", docker + ["info", "--format", template]), "docker_info")
    if not isinstance(info, dict) or info.get("os_type") != "linux" or info.get("architecture") not in ("x86_64", "amd64"):
        raise PhaseError("Expected the approved local Linux amd64 Docker daemon")
    status["docker_info"] = info
    status["disk"] = {}
    for label, path in (("evidence", output), ("docker_root", Path(info["docker_root_dir"]))):
        try:
            usage = shutil.disk_usage(path)
            status["disk"][label] = {"path": str(path), "free_bytes": usage.free, "total_bytes": usage.total}
        except OSError as error:
            status["disk"][label] = {"path": str(path), "error": str(error)}
            raise PhaseError(f"Cannot measure {label} filesystem; verify its free space before build") from error
    # Docker 29's containerd image store can use a filesystem outside DockerRoot.
    # This is a default-location observation, not discovery of a custom active root.
    candidate = Path("/var/lib/containerd")
    try:
        usage = shutil.disk_usage(candidate)
        status["disk"]["containerd_default_candidate"] = {
            "path": str(candidate), "free_bytes": usage.free, "total_bytes": usage.total,
            "active_root_verified": False}
    except OSError:
        status["disk"]["containerd_default_candidate"] = {
            "path": str(candidate), "status": "UNKNOWN_OR_UNAVAILABLE", "active_root_verified": False}
    save_json(output / "status.json", status)
    runner.run("lfs_check", ["git", "-C", str(repo), "lfs", "fsck"], timeout=300, capture=False)
    runner.run("base_pull", docker + ["pull", "--platform", "linux/amd64", BASE_TAG], timeout=1800, capture=False)
    base = one_json(runner.run("base_inspect", docker + ["image", "inspect", BASE_TAG]), "base_inspect")
    digests = base[0].get("RepoDigests", []) if isinstance(base, list) and len(base) == 1 else []
    digest = next((entry for entry in digests if DIGEST_RE.fullmatch(entry)), None)
    if digest is None:
        raise PhaseError("No valid osrf/ros RepoDigest resolved")
    status["base_digest"] = digest
    runner.run("build", docker + ["build", "--builder", "default", "--platform", "linux/amd64", "--progress=plain",
               "--build-arg", "ROS_BASE=" + digest, "--iidfile", str(output / "image_id.txt"),
               "-f", "docker/auto_tuning/Dockerfile", "."], cwd=repo, timeout=7200, capture=False)
    image_id = (output / "image_id.txt").read_text().strip()
    status["image_id"] = image_id
    save_json(output / "image_inspect.json", image_inspection(runner, docker, image_id, "image_inspect"))
    if source_snapshot(repo, runner, "postbuild_source") != source:
        raise PhaseError("Source changed during build; do not use this image")
    role = "image_validation"
    cid_file = output / "container.id"
    run_id = status["run_id"]
    try:
        runner.run(role + "_create", docker + container_options(output, run_id, role, readonly=True)
                   + [image_id, "bash", "/opt/auto_tuning/tools/validate_image.sh"], capture=False)
        cid = owned_id(cid_file)
        if cid is None:
            raise PhaseError("Validation container ID was not recorded")
        validator_output = runner.run(role + "_run", docker + ["start", "--attach", cid], timeout=300)
        result = one_json(validator_output.strip().splitlines()[-1] if validator_output.strip() else "", role)
        if not isinstance(result, dict) or result.get("status") != "IMAGE_SMOKE_VALIDATED":
            raise PhaseError("Image validator did not report IMAGE_SMOKE_VALIDATED")
        status["image_validation"] = result
        runner.run("package_inventory", docker + ["cp", cid + ":/ws/installed-packages.tsv",
                   str(output / "installed-packages.tsv")], capture=False)
    finally:
        status["validation_cleanup"] = cleanup_container(runner, docker, cid_file, run_id, role)
    if status["validation_cleanup"].get("log_error"):
        raise PhaseError("Validator logs could not be fully preserved; container was stopped, evidence is incomplete")
    status["status"] = "IMAGE_VALIDATED"
    status["limitation"] = "Image prerequisites validated only; no mission or ROS/Gazebo bringup was performed by build stage. APT rebuild is not pinned by base digest alone."


def probe(repo, build_dir, output, runner, docker, status, *, warmup_seconds=10, sleep=time.sleep):
    previous = json.loads((build_dir / "status.json").read_text())
    if not isinstance(previous, dict) or previous.get("stage") != "build" or previous.get("status") != "IMAGE_VALIDATED":
        raise PhaseError("--build-dir must contain an IMAGE_VALIDATED build record")
    status["source"] = source_snapshot(repo, runner)
    save_json(output / "source_manifest.json", status["source"])
    if status["source"] != previous.get("source"):
        raise PhaseError("Current source differs from the image build; create a new build")
    image_id = previous.get("image_id", "")
    if (build_dir / "image_id.txt").read_text().strip() != image_id:
        raise PhaseError("Build image ID files disagree")
    save_json(output / "image_inspect.json", image_inspection(runner, docker, image_id, "image_inspect"))
    status.update(image_id=image_id, build_directory=str(build_dir), build_status_sha256=file_hash(build_dir / "status.json"))
    cid_file = output / "container.id"
    run_id = status["run_id"]
    command = [image_id, "xvfb-run", "-a", "-e", "/evidence/xvfb.log", "-s", "-screen 0 1280x720x24 -nolisten tcp",
               "ros2", "launch", "truck_bringup", "gps_waypoint_follower.launch.py",
               "use_gzclient:=false", "use_rviz:=false", "use_mapviz:=false", "use_gpu:=false",
               "use_xcb:=false", "publish_legacy_cloud_tf:=false", "use_ackermann_bt:=true",
               "localization_params_file:=/opt/auto_tuning/config/dual_ekf_navsat_gate0.yaml"]
    try:
        runner.run("headless_create", docker + container_options(output, run_id, "headless") + command, capture=False)
        cid = owned_id(cid_file)
        if cid is None:
            raise PhaseError("Headless container ID was not recorded")
        runner.run("headless_start", docker + ["start", cid], capture=False)
        for index in range(warmup_seconds):
            sleep(1)
            state = inspect_container(runner, docker, cid, f"warmup_{index + 1:02d}")
            if state.get("State", {}).get("Running") is not True:
                raise PhaseError("Bringup container exited during diagnostic warmup")
        runner.run("runtime_probe", docker + ["exec", cid, "/entrypoint.sh", "bash",
                   "/opt/auto_tuning/tools/collect_runtime_evidence.sh", "/evidence/runtime_probe"], timeout=900, capture=False)
        state = inspect_container(runner, docker, cid, "headless_after_probe")
        if state.get("State", {}).get("Running") is not True:
            raise PhaseError("Bringup container exited during evidence collection")
        readiness, digest = readiness_result(output)
        status.update(diagnostic_status=readiness['diagnostic_status'],
                      readiness_collection_status=readiness['collection_status'],
                      mission_ready=False, readiness_report='runtime_probe/readiness.json',
                      readiness_sha256=digest)
        if readiness['collection_status'] == 'FAILED':
            raise PhaseError('Readiness capture failed; inspect preserved readiness report')
        status["status"] = "OBSERVATIONS_COLLECTED"
        status["limitation"] = "Collection status is separate from diagnostic_status. Read readiness.json checks, effective parameters, lifecycle, TF authorities and sensor evidence. Mission remains blocked by localization/actuation review; no gates are approved."
    finally:
        status["cleanup"] = cleanup_container(runner, docker, cid_file, run_id, "headless")
    if status["cleanup"].get("log_error"):
        raise PhaseError("Bringup logs could not be fully preserved; container was stopped, evidence is incomplete")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("build", "probe"))
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path)
    args = parser.parse_args(argv)
    if (args.stage == "probe") != bool(args.build_dir):
        parser.error("--build-dir is required only for probe")
    repo = args.repo.resolve()
    output = args.output_dir.absolute()
    if output.exists() or output.is_symlink():
        parser.error("Output already exists; use a new directory")
    if output.resolve().is_relative_to(repo):
        parser.error("Output must be outside the source checkout")
    if any(character in str(output) for character in (",", "\n", "\r")):
        parser.error("Output path cannot contain commas or line breaks (Docker bind syntax)")
    output.mkdir(parents=False, exist_ok=False)
    status = {"schema_version": 1, "stage": args.stage, "status": "RUNNING", "run_id": uuid.uuid4().hex,
              "started_at_utc": now(), "docker_endpoint": ENDPOINT,
              "gate0": "NOT_YET_VERIFIED", "gate1": "NOT_YET_VERIFIED", "mission_sent": False,
              "mission_ready": False}
    save_json(output / "status.json", status)
    code = 0
    try:
        runner = Runner(output)
        docker = docker_prefix()
        if args.stage == "build":
            build(repo, output, runner, docker, status)
        else:
            probe(repo, args.build_dir.resolve(), output, runner, docker, status)
    except KeyboardInterrupt:
        status.update(status="INTERRUPTED", reason="Interrupted; inspect cleanup evidence before continuing")
        code = 130
    except (OSError, ValueError, KeyError, TypeError, IndexError, PhaseError, subprocess.SubprocessError) as error:
        status.update(status="FAILED", reason=str(error))
        code = 2
    finally:
        status["finished_at_utc"] = now()
        save_json(output / "status.json", status)
    print(json.dumps({"status": status["status"], "evidence": str(output),
                      "diagnostic_status": status.get('diagnostic_status', 'NOT_ASSESSED'),
                      "readiness_collection_status": status.get('readiness_collection_status', 'NOT_ASSESSED'),
                      "mission_ready": False, "gate0": status["gate0"], "gate1": status["gate1"]}))
    return code


def interrupted(_signal, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupted)
    raise SystemExit(main())
