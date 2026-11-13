"""Bounded host load/initial-pose smoke. No UART, firmware or navigation."""

import json
import math
from pathlib import Path

from controller import Supervisor

project = Path(__file__).resolve().parents[2]
manifest = json.loads((project / "manifest.json").read_text())
expected_axle_m = 0.01 + manifest["robot_parameters"]["wheel_diameter_mm"]["value"] / 2000
robot = Supervisor()
for name in ("left_motor", "right_motor"):
    motor = robot.getDevice(name)
    assert motor is not None, name
    motor.setTorque(0.0)
for name in ("left_encoder", "right_encoder", "servo_angle",
             "imu_accelerometer", "imu_compass_direction_only"):
    sensor = robot.getDevice(name)
    assert sensor is not None, name
    sensor.enable(1)
robot.getDevice("sonar_servo").setPosition(0.0)
for _ in range(200):
    assert robot.step(1) != -1
point = robot.getSelf().getPosition()
accel = robot.getDevice("imu_accelerometer").getValues()
compass = robot.getDevice("imu_compass_direction_only").getValues()
joints = {name: robot.getDevice(name).getValue()
          for name in ("left_encoder", "right_encoder", "servo_angle")}
assert all(math.isfinite(x) for x in point + accel + compass + list(joints.values()))
assert expected_axle_m - 0.0075 <= point[2] <= expected_axle_m + 0.0125, point
assert 4.9 <= math.sqrt(sum(x * x for x in accel)) <= 14.8, accel
result = {
    "load_smoke_pass": True, "sim_s": robot.getTime(), "axle_m": point,
    "accel_sensor_m_s2": accel,
    "left_encoder_rad": joints["left_encoder"],
    "right_encoder_rad": joints["right_encoder"],
    "servo_angle_rad": joints["servo_angle"],
    "compass_direction_only": compass,
    "scope": "initial-pose/load smoke only; no driving, realtime or hardware proof",
}
(project / "smoke-result.json").write_text(json.dumps(result, indent=2) + "\n")
print("STAGE0_WORLD_SMOKE_PASS " + json.dumps(result), flush=True)
# Allow controller output to drain before Webots terminates its child process.
robot.step(1)
robot.step(1)
robot.simulationQuit(0)
