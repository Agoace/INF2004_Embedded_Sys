#!/usr/bin/env python3
"""Check the documented core-0 task priority order from the source constants.

Lower numbers run first. The USB console drains RAM output, so it must stay
below every driving task, the radio and the TEST runner, but above the GP20
dump producer, health and the always-ready CPU meter.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def number(path, pattern):
    text = (ROOT / path).read_text()
    found = re.findall(pattern, text, re.M)
    if len(found) != 1:
        raise SystemExit(f"FAIL {path}: expected one match for {pattern!r}, got {found}")
    return int(found[0])


p = {
    "motion": number("app_program/subsystem2/motion.c", r"^#define MOTION_TASK_PRIORITY\s+(\d+)"),
    "line": number("app_program/subsystem3/line_service.c", r"^#define LINE_TASK_PRIORITY\s+(\d+)"),
    "sonar": number("app_program/subsystem5/sonar_service.c", r"^#define SONAR_TASK_PRIORITY\s+(\d+)"),
    "vehicle": number("app_program/vehicle/vehicle_task.c", r"^#define VEHICLE_TASK_PRIORITY\s+(\d+)"),
    "bench": number("app_program/subsystem2/motion_bench.c", r"app_start_task\(\(FP\)bench_task, (\d+),"),
    "wifi": number("lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c", r"^#define CYW43_TASK_PRIORITY\s+(\d+)"),
    "test_runner": number("app_program/app_main.c", r"^#define TEST_RUNNER_PRIORITY\s+(\d+)"),
    "usb": number("lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c", r"^#define USB_TASK_PRIORITY\s+(\d+)"),
    "telemetry": number("app_program/subsystem1/telemetry.c", r"app_start_task\(\(FP\)telemetry_task, (\d+),"),
    "health": number("app_program/subsystem1/robot_health.c", r"^#define HEALTH_TASK_PRIORITY\s+(\d+)"),
    "idle_meter": number("app_program/platform/idle_meter.c", r"^#define IDLE_METER_PRIORITY\s+(\d+)"),
    "max": number("config/config.h", r"^#define\s+CNF_MAX_TSKPRI\s+(\d+)"),
}
driving = ("motion", "line", "sonar", "vehicle", "bench")
checks = [
    (all(p[name] < p["usb"] for name in driving), "USB below every driving task"),
    (p["wifi"] < p["usb"] and p["test_runner"] < p["usb"], "USB below Wi-Fi and TEST runner"),
    (p["usb"] < p["telemetry"] < p["health"] < p["idle_meter"], "USB above dump producer, health and CPU meter"),
    (p["idle_meter"] <= p["max"], "CPU meter within configured priority range"),
    (p["motion"] < p["line"] < p["sonar"] < p["vehicle"] < p["wifi"], "driving order unchanged"),
]
failed = [label for ok, label in checks if not ok]
print(" ".join(f"{k}={v}" for k, v in p.items()))
if failed:
    for label in failed:
        print("FAIL", label)
    sys.exit(1)
print(f"PASS: {len(checks)} priority-order checks")
