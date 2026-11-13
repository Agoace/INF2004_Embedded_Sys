"""Geometry preview only: passive zero torque, no UART and no navigation."""

from controller import Robot

robot = Robot()
for name in ("left_motor", "right_motor"):
    robot.getDevice(name).setTorque(0.0)
robot.getDevice("sonar_servo").setPosition(0.0)
print("STAGE0 PREVIEW ONLY: no Pico link, no firmware verdict, no timing qualification")
while robot.step(1) != -1:
    pass
