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
# ROBOT synthetic profile: one explicit long token, full robot app,
# never TEST/BENCH, never a capture or stream profile.
ROBOT = "IMU_SAFETY_BENCH=robot_synthetic"
for extra in (("IMU_SAFETY_BENCH=robot",),
              ("IMU_SAFETY_BENCH=ROBOT_SYNTHETIC",),
              ("IMU_SAFETY_BENCH=robot_synthetic 1",),
              (ROBOT, "TEST=1"),
              (ROBOT, "BENCH=1"),
              (ROBOT, "SMP=1"),
              (ROBOT, "MQTT=0"),
              (ROBOT, "KERNEL_RAM=0"),
              (ROBOT, "IMU_STREAM=1"),
              (ROBOT, "BARCODE_CAPTURE=1"),
              (ROBOT, "BARCODE_FAIL_CAPTURE=1")):
    result = make(*extra)
    assert result.returncode != 0, f"accepted unsafe robot replay {extra}"
    assert any(key in result.stderr for key in
               ("IMU_SAFETY_BENCH", "KERNEL_RAM", "BARCODE_CAPTURE",
                "MQTT", "BENCH", "TEST")), result.stderr[-300:]
result = make(ROBOT)
assert result.returncode == 0, result.stderr[-500:]
assert "_imu_synth_robot" in result.stdout
assert "_imu_safety_bench" not in result.stdout
assert "service_impl.h" in result.stdout and "robot_impl.h" in result.stdout
assert "main_impl.h" not in result.stdout
assert "-DIMU_SAFETY_BENCH_ROBOT=1" in result.stdout
assert "app_program/imu_safety_bench/robot.c" in result.stdout
# The full robot keeps its ordinary KERNEL_RAM lists.
assert "kernel_ram/robot.list" in result.stdout
assert "kernel_ram/imu_safety_bench.list" not in result.stdout
result = make("TEST=1", "IMU_SAFETY_BENCH=1")
assert "-DIMU_SAFETY_BENCH_ROBOT=1" not in result.stdout
assert "robot_impl.h" not in result.stdout
print("IMU safety bench make guards PASS")
