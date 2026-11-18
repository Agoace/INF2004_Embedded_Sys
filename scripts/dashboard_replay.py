#!/usr/bin/env python3
"""Make offline, public-fixture dashboard screenshots. Never connects to MQTT.

The C fixture emitter supplies decoded layouts. Explicit scene edits illustrate
LEFT, retry and paused states; these are not new hardware measurements.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from dashboard_model import DashboardModel
from dashboard_page import PAGE
import robot_wire


def samples(binary):
    result = {}
    for line in subprocess.check_output([str(binary)], text=True).splitlines():
        name, encoded = line.split()
        if name.startswith("dash/"):
            result[name[5:]] = bytes.fromhex(encoded)
    return result


def scene(fixtures, name):
    """Use a fixed clock and explicit fixture edits, with no private configuration."""
    model = DashboardModel(clock=lambda: 100.0, token_configured=False)
    model.set_broker_connected(True)
    scope = model.log.sessions.current
    state = robot_wire.decode("robot/state", fixtures["state_running"])
    state.update(state=1, module=3, active=1, kr1_servo_deg=15,
                 telemetry_idle=0, boot_session=scope, duty_left_permille=180,
                 duty_right_permille=180, heading_cdeg=0, turn_rate_ddps=0)
    model._store("robot/state", state, 100000)
    model._store("robot/status", {"robot_online":1, "boot_session":scope}, 100000)
    heartbeat = robot_wire.decode("robot/heartbeat", fixtures["heartbeat"])
    heartbeat.update(telemetry_idle=0, boot_session=scope)
    model._store("robot/heartbeat", heartbeat, 100000)
    sonar = robot_wire.decode("robot/sonar", fixtures["sonar_valid"])
    # Explicit illustration: blocked right, free left. Not an inferred box.
    for i, angle in enumerate(range(-30, 31, 5)):
        if name == "reattempt" and angle in (-10, 20):
            continue  # two absent observations, not two invented ranges
        mm = 235 if angle <= 0 or name == "reattempt" else 430
        model._store_sonar(dict(sonar, angle=angle, echo_mm=mm, status=1,
                                valid=1, echo_status=3 if angle == 30 and name != "reattempt" else 0,
                                sequence=100+i), 99000+i*60)
    obstacle = robot_wire.decode("robot/obstacle", fixtures["obstacle"])
    obstacle.update(decision=2, closest_valid=1, closest_mm=235,
                    closest_angle_deg=-15, profile_word=235,
                    scan_attempt=0, boot_session=scope)
    model._store_obstacle(obstacle, 99900)
    # Live-path illustration, using the same numeric row fields as the log decoder.
    for i in range(140):
        module = 1 if i < 50 else 3 if i < 65 else 4 if i < 115 else 2
        if hasattr(model, "_path_row"):  # the path commit is optional
            model._path_row({"generation":1, "index":i, "kr1_time_ms":i*100,
                             "kr1_state":1, "kr1_module":module,
                             "kr1_measured_l":0 if module == 3 else 100,
                             "kr1_measured_r":0 if module == 3 else 115 if module == 4 else 100})
    summary = robot_wire.decode("robot/summary", fixtures["summary_normal"])
    summary.update(boot_session=scope, stop_reason=2)
    model._store("robot/summary", summary, 98000)
    if name == "reattempt":
        model._store_event({"kind":8, "sequence":200, "uptime_ms":9000,
                            "word":1, "detail":1}, 100000)
        model.completed_scan["fields"]["decision"] = 0
    if name == "idle":
        model._store("robot/state", dict(state, state=0, module=0, active=0,
                                         telemetry_idle=1), 100000)
        model._store("robot/heartbeat", dict(heartbeat, telemetry_idle=1), 100000)
    model.last_message_ms = 100000
    snapshot = model.snapshot()
    snapshot["server_instance"] = "public-replay-instance"
    return snapshot


def replay_page(snapshot):
    page = PAGE.replace("__CSRF__", "public-replay-not-a-token").replace(
        "__INSTANCE__", "public-replay-instance").replace("__NONCE__", "public-replay-nonce")
    page = page.replace("<strong>Robot dashboard</strong>",
                        "<strong>REPLAY · public fixtures</strong>")
    # Replace only the start invocation, not production startup/security functions.
    # No HTTP, SSE, broker or command endpoint is used in this offline document.
    encoded = json.dumps(snapshot).replace("<", "\\u003c")
    return page.replace("\nstart();\n", "\nrender(" + encoded + ");\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path, help="host test_mqtt_wire fixture emitter")
    parser.add_argument("--output", type=Path, default=Path("/tmp/dash-shots"))
    parser.add_argument("--html-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fixtures = samples(args.binary.resolve())
    firefox = shutil.which("firefox")
    if not firefox and not args.html_only:
        parser.error("firefox is needed for screenshots (or use --html-only)")
    for name, size in (("scan-left", "1440,1200"), ("reattempt", "1440,1200"),
                       ("idle", "1440,1200"), ("phone", "390,2400"),
                       ("health", "1440,1800")):
        html = args.output / (name + ".html")
        page = replay_page(scene(fixtures, name))
        if name == "health":
            page = page.replace('<details id="health"', '<details open id="health"')
        html.write_text(page, encoding="utf-8")
        if not args.html_only:
            with tempfile.TemporaryDirectory(prefix="dashboard-firefox-") as profile:
                subprocess.run([firefox, "--headless", "--no-remote", "--profile", profile,
                                "--window-size", size, "--screenshot",
                                str((args.output / (name + ".png")).resolve()),
                                html.resolve().as_uri()], check=True, timeout=120,
                               stdout=subprocess.DEVNULL)
        print(html)


if __name__ == "__main__":
    main()
