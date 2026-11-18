#!/usr/bin/env python3
"""IMU_SAFETY_CAPTURE is an opt-in ROBOT profile; OFF adds nothing."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
UP = ("SMP=0", "WIFI=cyw43", "NET=lwip", "MQTT=1", "KERNEL_RAM=1")


def make(*extra):
    return subprocess.run(["make", "-n", "-p", *UP, *extra],
                          cwd=root / "build_make", capture_output=True,
                          text=True, check=False)


CAP = "IMU_SAFETY_CAPTURE=1"
for extra in (("IMU_SAFETY_CAPTURE=2",),
              ("IMU_SAFETY_CAPTURE=",),
              ("IMU_SAFETY_CAPTURE=1 1",),
              (CAP, "TEST=1"),
              (CAP, "BENCH=1"),
              (CAP, "IMU_STREAM=1", "TEST=1"),
              (CAP, "IMU_SAFETY_BENCH=robot_synthetic")):
    result = make(*extra)
    assert result.returncode != 0, f"accepted unsafe capture profile {extra}"
    assert "IMU_SAFETY_CAPTURE" in result.stderr or \
        "IMU_SAFETY_BENCH" in result.stderr, result.stderr[-300:]
for extra in ((CAP,), (CAP, "BARCODE_FAIL_CAPTURE=1")):
    result = make(*extra)
    assert result.returncode == 0, result.stderr[-500:]
    out = result.stdout
    assert "_imucap" in out and "-imucap" in out
    assert "app_program/imu_safety_capture/capture.c" in out
    assert "app_program/imu_safety_capture/hook.c" in out
    assert "-include ../app_program/imu_safety_capture/service_impl.h" in out
    assert "-include ../app_program/imu_safety_capture/telemetry_impl.h" \
        in out
    assert "-include ../app_program/imu_safety_capture/vehicle_impl.h" \
        in out
    assert "kernel_ram/robot.list" in out
    if "BARCODE_FAIL_CAPTURE=1" in extra:
        assert "_bcfail2_imucap" in out
for args in ((), ("IMU_SAFETY_CAPTURE=0",), ("BARCODE_FAIL_CAPTURE=1",)):
    result = make(*args)
    assert result.returncode == 0, result.stderr[-300:]
    out = result.stdout
    for name in ("capture.", "hook.", "service_impl.h", "telemetry_impl.h",
                 "vehicle_impl.h"):
        assert f"imu_safety_capture/{name}" not in out, (args, name)
    assert "_imucap" not in out and "-imucap" not in out
    assert "-DIMU_SAFETY_CAPTURE" not in out
print("IMU safety capture make guards PASS")
