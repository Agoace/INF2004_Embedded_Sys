"""Deterministic offline display only; never loads or runs firmware."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "core"))
from scenario import unwrap

WORLD_FORMAT = ["t_us", "pose_mm_deg", "speed_mm_s", "fractional_counts", "encoder_counts",
                "duty_pct", "ir_raw", "ir_points_mm", "servo_deg", "contact"]


def display_data(trace):
    """Thin only unused hook/packet data. Preserve every world sample and command."""
    events = trace["events"]
    header = trace["header"]
    scenario = unwrap(header["scenario"], "$", [])
    worlds, commands, dt, console, triggers, results, contacts = [], [], [], [], [], [], []
    origin = None
    for event in events:
        kind = event["type"]
        t = event["t_us"]
        if kind == "world":
            worlds.append([t, [event["x_mm"], event["y_mm"], event["heading_deg"]],
                           event["speed_mm_s"], event["counts"], event.get("encoder_counts"),
                           event["duty"], event["ir_raw"], event["ir_points_mm"],
                           event.get("servo_deg"), event["contact"]])
        elif kind == "duty":
            commands.append([t, "duty", event["wheel"], event["pct"]])
        elif kind == "servo":
            commands.append([t, "servo", event["pulse_us"]])
        elif kind == "text":
            console.append([t, event["text"]])
        elif kind == "ultrasonic_trigger":
            triggers.append(event)
        elif kind == "ultrasonic_no_echo" and triggers and triggers[-1]["t_us"] == t:
            triggers[-1] = {**triggers[-1], "no_echo": event}
        elif kind == "ultrasonic_result":
            results.append(event)
        elif kind == "contact":
            contacts.append(event)
        elif kind == "hook" and event["call"] in ("dt_init", "dt_update", "dt_reading"):
            if event["call"] == "dt_init":
                origin = event.get("world_pose")
            ghost = None
            side = event.get("side")
            if origin is not None and side in (-1, 1):
                # S-frame y/heading point toward the chosen side. Do not estimate
                # ongoing travel or turn: public module values are start-of-step.
                angle = math.radians(origin[2])
                x, y = event["x_mm"], side * event["y_mm"]
                ghost = [origin[0] + x * math.cos(angle) - y * math.sin(angle),
                         origin[1] + x * math.sin(angle) + y * math.cos(angle),
                         origin[2] + side * event["heading"]]
            dt.append({key: event[key] for key in ("t_us", "call", "step", "fail", "heading", "x_mm", "y_mm")}
                      | {"step_name": event.get("step_name", str(event["step"])),
                         "fail_name": event.get("fail_name", str(event["fail"])),
                         "world_estimate": ghost, "side": side, "word": event.get("word")})
    observation = trace.get("firmware_observation", {})
    return {"display_schema_version": 1, "world_format": WORLD_FORMAT,
            "meta": {"name": scenario["name"], "source_commit": header["source_commit"],
                     "complete": trace["complete"], "error": trace.get("error"),
                     "end_us": scenario["end_us"],
                     "recorded_end_us": max((e["t_us"] for e in events), default=0),
                     "assumptions": header["assumed_values"]},
            "scenario": scenario, "world": worlds, "commands": commands, "dt": dt,
            "console": console, "triggers": triggers, "results": results, "contacts": contacts,
            "phases": observation.get("phase_timeline", []),
            "steps": observation.get("detour_timeline", []),
            "crosscheck": observation.get("step_crosscheck", {"status": "unavailable"}),
            "omitted": "Other hook calls, packets and device-completion events remain in trace.json; no world samples, dt observations, duty/servo commands or text fragments are thinned."}


def write_html(trace, destination):
    base = Path(__file__).parent
    data = json.dumps(display_data(trace), separators=(",", ":"), ensure_ascii=True, allow_nan=False, sort_keys=True)
    # HTML tokenization happens before JSON parsing, even in application/json.
    data = data.replace("&", "\\u0026").replace("<", "\\u003c").replace(">", "\\u003e")
    html = (base / "template.html").read_text().replace("/* VIEWER_CSS */", (base / "viewer.css").read_text())
    html = html.replace("/* VIEWER_JS */", (base / "viewer.js").read_text()).replace("__VIEWER_DATA__", data)
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".viewer-", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(html)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description="Regenerate a self-contained viewer from a saved trace; no simulation")
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path, help="Default: run.html beside the trace")
    args = parser.parse_args()
    destination = args.output or args.trace.with_name("run.html")
    if destination.resolve() == args.trace.resolve():
        parser.error("HTML output must not overwrite the input trace")
    write_html(json.loads(args.trace.read_text()), destination)
    print(destination)


if __name__ == "__main__":
    main()
