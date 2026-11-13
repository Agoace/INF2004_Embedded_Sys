#!/usr/bin/env python3
"""Evaluate the real Gate1 option guard without invoking tools or devices."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
mk = (root / "build_make/pico_rp2040.mk").read_text()
assert "PIL_GATE1 ?= 0" in mk
region = "PIL_GATE1 ?= 0" + mk.split("PIL_GATE1 ?= 0", 1)[1].split(
    "# RT-0075", 1)[0]
base = dict(SIM="1", WIFI="none", NET="none", MQTT="0", TEST="0", BENCH="0",
            DIAG="0", IMU_STREAM="0", BARCODE_CAPTURE="0",
            BARCODE_LIVE_ENABLE="0", JUNCTION_TEST_CMD="NONE")
with tempfile.TemporaryDirectory() as tmp:
    program = ".DEFAULT_GOAL:=print\n" + region + "print:\n\t@echo OK\n"
    def check(values, expected):
        result = subprocess.run(["make", "-f", "-", "--no-print-directory",
            *[f"{k}={v}" for k, v in values.items()]], input=program,
            cwd=tmp, text=True, capture_output=True, check=False)
        assert (result.returncode == 0) == expected, result.stderr
    check({**base, "PIL_GATE1": "1"}, True)
    for key, value in (("SIM", "0"), ("WIFI", "cyw43"), ("NET", "lwip"),
                       ("MQTT", "1"), ("TEST", "1"), ("BENCH", "1"),
                       ("JUNCTION_TEST_CMD", "A"), ("DIAG", "1")):
        check({**base, "PIL_GATE1": "1", key: value}, False)
    for value in ("", "2", "0 1"):
        check({**base, "PIL_GATE1": value}, False)
    check({"PIL_GATE1": "0"}, True)
print("Gate1 guard: positive/negative cases PASS")
