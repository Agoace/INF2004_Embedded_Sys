#!/usr/bin/env python3
"""Only an explicit, non-destructive UP MQTT TEST profile may replay IMU."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
def make(*extra):
    return subprocess.run(["make", "-n", "-p", "SMP=0", "WIFI=cyw43",
                           "NET=lwip", "MQTT=1", "KERNEL_RAM=1", *extra],
                          cwd=root / "build_make", capture_output=True,
                          text=True, check=False)
for extra in (("IMU_SAFETY_BENCH=1",),
              ("TEST=1", "IMU_SAFETY_BENCH=2"),
              ("TEST=1", "IMU_SAFETY_BENCH="),
              ("TEST=1", "IMU_SAFETY_BENCH=1", "BENCH=1"),
              ("TEST=1", "IMU_SAFETY_BENCH=1", "SMP=1"),
              ("TEST=1", "IMU_SAFETY_BENCH=1", "IMU_STREAM=1"),
              ("TEST=1", "IMU_SAFETY_BENCH=1", "BARCODE_CAPTURE=1")):
    result = make(*extra)
    assert result.returncode != 0, f"accepted unsafe replay profile {extra}"
    assert any(key in result.stderr for key in
               ("IMU_SAFETY_BENCH", "KERNEL_RAM", "BARCODE_CAPTURE")), \
        result.stderr[-300:]
result = make("TEST=1", "IMU_SAFETY_BENCH=1")
assert result.returncode == 0, result.stderr[-500:]
assert "_imu_safety_bench" in result.stdout
assert "service_impl.h" in result.stdout and "main_impl.h" in result.stdout
assert "app_program/imu_safety_bench/replay.c" in result.stdout
for args in ((), ("IMU_SAFETY_BENCH=0",)):
    result = make(*args)
    assert result.returncode == 0, result.stderr[-300:]
    assert " -DIMU_SAFETY_BENCH=1" not in result.stdout
    assert " -include ../app_program/imu_safety_bench/" not in result.stdout
print("IMU safety bench make guards PASS")
