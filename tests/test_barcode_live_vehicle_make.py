#!/usr/bin/env python3
"""V1/V3 live vehicle policy: seams only in BARCODE_LIVE_ENABLE=1 builds."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
UP = ("SMP=0", "WIFI=cyw43", "NET=lwip", "MQTT=1", "KERNEL_RAM=1")
LIVE = ("BARCODE_LIVE_ENABLE=1", "BARCODE_MEASURED_CLEAR_MM=100",
        "BARCODE_MAP=F=LEFT", "JUNCTION_TEST_CMD=NONE")
SEAMS = ("barcode_live_vehicle/policy.c",
         "-include ../app_program/barcode_live_vehicle/vehicle_impl.h",
         "-include ../app_program/barcode_live_vehicle/print_impl.h")


def make(*extra):
    return subprocess.run(["make", "-n", "-p", *UP, *extra],
                          cwd=root / "build_make", capture_output=True,
                          text=True, check=False)


for extra in (LIVE, LIVE + ("BARCODE_FAIL_CAPTURE=1",)):
    result = make(*extra)
    assert result.returncode == 0, result.stderr[-500:]
    assert "barcode_live_vehicle/action.c" in result.stdout
    assert "vehicle/junction_action.c" not in result.stdout
    for seam in SEAMS:
        assert seam in result.stdout, (extra, seam)
for extra in ((), ("BARCODE_FAIL_CAPTURE=1",),
              ("BARCODE_MEASURED_CLEAR_MM=100",)):
    result = make(*extra)
    assert result.returncode == 0, result.stderr[-300:]
    for seam in SEAMS:
        assert seam not in result.stdout, (extra, seam)
    assert "barcode_live_vehicle/policy.o" not in result.stdout, extra
print("barcode live vehicle make seams PASS")
