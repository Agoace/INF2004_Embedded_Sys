"""Run the native firmware in a bounded process and save a deterministic trace."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile

from scenario import load
from protocol import normalise


def source_hashes(repo):
    return {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted((repo / "src").iterdir())
            if path.suffix in (".c", ".h") and path.name != "wifi_config.h"}


def simulator_hashes(repo):
    sim = repo / "sim"
    return {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(sim.rglob("*")) if path.is_file()
            and not set(path.relative_to(sim).parts) & {"build", "output", "__pycache__"}}


def cpu_limit():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    resource.setrlimit(resource.RLIMIT_CPU, (5, 6))


def text_lines(events):
    """Timestamp a displayed line at the fragment that completes it."""
    pending = ""
    for event in events:
        if event["type"] == "text":
            pending += event["text"]
            while "\n" in pending:
                line, pending = pending.split("\n", 1)
                yield event["t_us"], line
    if pending:
        yield events[-1]["t_us"], pending


def write_trace(path, trace):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False,
                                         prefix=".trace-", suffix=".tmp") as stream:
            temporary = Path(stream.name)
            json.dump(trace, stream, ensure_ascii=True, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(repo_path, build_path):
    parser = argparse.ArgumentParser(description="Milestone 2 motion/IR firmware simulator")
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="Output directory for trace.json")
    args = parser.parse_args()
    repo, build = Path(repo_path), Path(build_path)
    try:
        raw, scenario, assumptions = load(args.scenario)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f"scenario error: {error}\n")
    before = source_hashes(repo)
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    firmware_revision = subprocess.check_output(
        ["git", "-C", str(repo), "log", "-1", "--format=%H", "--", "src"], text=True).strip()
    config = json.loads((build / "target.json").read_text())
    config["compile_commands"] = json.loads((build / "compile_commands.json").read_text())
    header = {
        "schema_version": 1, "milestone": 2, "source_commit": firmware_revision,
        "repository_commit": revision, "source_hashes": before,
        "simulator_hashes": simulator_hashes(repo),
        "compiler": (build / "compiler.txt").read_text().strip(),
        "build": config, "scenario": raw, "assumed_values": assumptions,
        "clock_quantum_us": scenario["clock_quantum_us"],
        "units": {"time": "us", "distance": "mm", "angle": "degrees"},
        "event_order": "advance previous commands; buttons in input order; service devices; observation",
        "device_model": "motion, encoders, coast, tile IR; latched safety contact stop; timed NO_RETURN ultrasound",
        "watchdog": {"cpu_soft_seconds": 5, "cpu_hard_seconds": 6, "wall_seconds": 10},
    }
    normalised = normalise(scenario)
    error = None
    output = ""
    try:
        result = subprocess.run([str(build / "firmware_sim")], input=normalised,
                                text=True, capture_output=True, timeout=10, preexec_fn=cpu_limit)
        output = result.stdout
        if result.returncode:
            error = f"native exit {result.returncode}: {result.stderr.strip()}"
        elif result.stderr:
            error = "unexpected native stderr: " + result.stderr.strip()
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        error = "host wall watchdog expired"
    except OSError as exc:
        error = str(exc)
    events = []
    try:
        for line in output.splitlines():
            event = json.loads(line)
            if event.get("type") == "contact":
                event["obstacle_id"] = scenario["obstacles"][event["obstacle_index"]]["id"]
            events.append(event)
    except ValueError:
        error = "incomplete or invalid native event stream"
    complete = (not error and bool(events) and events[-1].get("type") == "end"
                and events[-1].get("complete") is True
                and events[-1].get("t_us") == scenario["end_us"])
    if source_hashes(repo) != before:
        error, complete = "src/ changed during run", False
    if not complete and error is None:
        error = "missing scenario end record"
    trace = {"header": header, "complete": complete, "events": events}
    if error is not None:
        trace["error"] = error
    write_trace(args.output / "trace.json", trace)
    for time_us, line in text_lines(events):
        if line:
            print(f"{time_us / 1000000:.6f}s {line}")
    if not complete:
        parser.exit(1, f"simulation failed: {error}; incomplete trace saved\n")
    print(f'Complete: scenario_end at {scenario["end_us"]} us')


if __name__ == "__main__":
    raise SystemExit("Use the generated sim/build/robot_sim launcher")
