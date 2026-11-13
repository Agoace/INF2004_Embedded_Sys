#!/usr/bin/env python3
"""Generate and load one offline Webots world with timeout and PID-group cleanup.

The installed Webots wrapper/compatibility libraries are never changed. No UART
or Pico is opened. Successful loading is not real-time or physical acceptance.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE))
from pil.course_builder import Course
from pil.robot_profile import RobotProfile
from pil.world import write_project


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--webots", default="webots")
    parser.add_argument("--timeout-seconds", type=int, default=40)
    args = parser.parse_args()
    if not 5 <= args.timeout_seconds <= 120:
        parser.error("load smoke timeout must be5..120seconds")
    course, profile = Course.load(args.course), RobotProfile.load(args.robot)
    output = write_project(course, profile, args.output)
    relative = "controllers/stage0_smoke/stage0_smoke.py"
    target = output / relative
    target.parent.mkdir(parents=True)
    shutil.copyfile(SOURCE / relative, target)
    world_name = "worlds/" + course.name + ".wbt"
    world = output / world_name
    original = world.read_bytes()
    text = original.decode()
    if text.count("RoboPico {\n") != 1:
        raise RuntimeError("expected exactly one generated RoboPico instance")
    world.write_text(text.replace("RoboPico {\n", 'RoboPico {\n controller "stage0_smoke"\n'))
    manifest_path = output / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["load_smoke_overlay"] = {
        "original_preview_world_sha256": hashlib.sha256(original).hexdigest(),
        "purpose": "bounded initial-pose load test; not firmware control",
    }
    for name in (world_name, relative):
        manifest["artifacts_sha256"][name] = hashlib.sha256((output / name).read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    env = os.environ.copy()
    env["XDG_CONFIG_HOME"] = str(output / "private-config")
    # Keep Unix-domain socket paths short; Webots creates PID-specific children.
    env["WEBOTS_TMPDIR"] = str(SOURCE.parents[1] / "build/wb")
    Path(env["XDG_CONFIG_HOME"]).mkdir()
    Path(env["WEBOTS_TMPDIR"]).mkdir(exist_ok=True)
    command = ["timeout", "--kill-after=3s", f"{args.timeout_seconds}s", args.webots,
               "--batch", "--mode=fast", "--no-rendering", "--minimize", "--stdout",
               "--stderr", str(world)]
    log = output / "load-smoke.log"
    with log.open("w") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        pid = process.pid
        try:
            status = process.wait(timeout=args.timeout_seconds + 8)
        except subprocess.TimeoutExpired:
            os.killpg(pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(pid, signal.SIGKILL)
                process.wait(timeout=3)
            status = 124
        finally:
            # Only OUR explicitly created process group. Never pkill by a name.
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    contents = log.read_text(errors="replace")
    proof = output / "smoke-result.json"
    passed = (status == 0 and proof.is_file() and
              json.loads(proof.read_text()).get("load_smoke_pass") is True and
              "STAGE0_WORLD_SMOKE_PASS" in contents and
              "ERROR:" not in contents and "WARNING:" not in contents)
    summary = {"course": course.name, "exit_status": status, "pid_group": pid,
               "load_smoke_pass": passed, "Pico_or_Probe_used": False,
               "scope": "load/initial-pose smoke only", "command": command}
    manifest["webots_executed"] = True
    manifest["load_smoke_pass"] = passed
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    (output / "load-smoke-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))
    print(f"Retained log/proof: {output}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
