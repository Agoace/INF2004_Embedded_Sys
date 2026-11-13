import copy
import json
import math
from pathlib import Path
import unittest

from pil.motor_plant import (EncoderAccumulator, MotorPlant, ServoPlant,
                             round_away, signed32, steady_speed_mm_s)
from pil.robot_profile import RobotProfile

PROFILE = Path(__file__).resolve().parents[1] / "calibration/robot_assumed.json"


class ProfileTests(unittest.TestCase):
    def test_dimensions_sources_and_rigid_mounts(self):
        profile = RobotProfile.load(PROFILE)
        self.assertEqual(profile["track_mm"], 111)
        self.assertEqual(profile["counts_per_revolution"], 2535)
        self.assertEqual(profile["firmware_counts_per_m"], 12414)
        self.assertEqual(profile.mount_mm("barcode"), (80, 77, -27))
        self.assertEqual(profile.mount_mm("line_l"), (119, 14.5, -23.5))
        self.assertTrue(profile.assumption_manifest()["total_mass_kg"]["assumed"])
        self.assertTrue(profile.assumption_manifest()["imu_axle_height_mm"]["assumed"])
        self.assertFalse(profile.assumption_manifest()["track_mm"]["assumed"])

    def test_missing_unknown_nonfinite_unsourced_rejected(self):
        original = json.loads(PROFILE.read_text())
        changes = (
            lambda d: d["parameters"].pop("total_mass_kg"),
            lambda d: d["parameters"].update(unknown={}),
            lambda d: d["parameters"]["track_mm"].update(value=math.nan),
            lambda d: d["parameters"]["track_mm"].update(value=True),
            lambda d: d["parameters"]["track_mm"].update(source=""),
            lambda d: d["parameters"]["total_mass_kg"].update(value=-1),
            lambda d: d["parameters"]["total_mass_kg"].update(value=0.05),
            lambda d: d["parameters"]["counts_per_revolution"].update(value=2535.5),
            lambda d: d.update(schema=2),
        )
        for change in changes:
            data = copy.deepcopy(original)
            change(data)
            with self.subTest(change=change), self.assertRaises(ValueError):
                RobotProfile.from_dict(data)


class PlantTests(unittest.TestCase):
    def setUp(self):
        self.profile = RobotProfile.load(PROFILE)

    def test_measured_duty_map_per_direction(self):
        self.assertEqual(steady_speed_mm_s(0, 0.12), 60)
        self.assertEqual(steady_speed_mm_s(1, 0.12), 59)
        self.assertEqual(steady_speed_mm_s(0, -0.12), -65)
        self.assertEqual(steady_speed_mm_s(1, -0.12), -63)
        self.assertAlmostEqual(steady_speed_mm_s(0, 0.22), 144.6)
        self.assertEqual(steady_speed_mm_s(0, 0.06), 0)
        with self.assertRaises(ValueError):
            steady_speed_mm_s(0, 0.8)
        self.assertEqual(steady_speed_mm_s(0, 0.8, assumed_limit=0.8), 658)

    def test_torque_direction_static_deadband_and_clamp(self):
        plant = MotorPlant(self.profile, 0)
        self.assertEqual(plant.torque_nm(500, 0), 0)
        self.assertGreater(plant.torque_nm(750, 0), 0)
        self.assertLess(plant.torque_nm(-750, 0), 0)
        self.assertLess(plant.torque_nm(0, 3), 0)
        self.assertGreater(plant.torque_nm(0, -3), 0)
        self.assertEqual(plant.torque_nm(0, 0), 0)
        self.assertLessEqual(abs(plant.torque_nm(5000, -100)),
                             self.profile["stall_torque_nm"])
        for compare in (6251, -6251, True, 6000):
            with self.subTest(compare=compare), self.assertRaises(ValueError):
                plant.torque_nm(compare, 0)

    def test_open_loop_response_and_passive_coast(self):
        plant = MotorPlant(self.profile, 0)
        omega = 0.0
        for _ in range(1000):
            omega += plant.torque_nm(1875, omega) / plant.reflected_inertia * 0.001
        self.assertAlmostEqual(omega * plant.radius_m * 1000, 211, places=4)
        previous = omega
        for _ in range(1000):
            omega += plant.torque_nm(0, omega) / plant.reflected_inertia * 0.001
            self.assertGreaterEqual(omega, 0)
            self.assertLessEqual(omega, previous)
            previous = omega
        self.assertLess(omega, 1e-6)

    def test_encoder_fraction_retained_and_reverse(self):
        encoder = EncoderAccumulator()
        encoder.reset((0, 0))
        for i in range(1, 1001):
            count = encoder.sample((i * 2 * math.pi / 1000, -i * 2 * math.pi / 1000))
        self.assertEqual(count, (2535, -2535))
        self.assertEqual(round_away(0.5), 1)
        self.assertEqual(round_away(-0.5), -1)
        self.assertEqual(signed32(1 << 31), -(1 << 31))
        self.assertEqual(signed32((1 << 32) + 5), 5)

    def test_encoder_large_jump_rejected_without_state_change(self):
        encoder = EncoderAccumulator()
        with self.assertRaises(ValueError):
            encoder.sample((0, 0))
        encoder.reset((1, 1))
        with self.assertRaises(ValueError):
            encoder.sample((2, 2))
        self.assertEqual(encoder.sample((1, 1)), (0, 0))

    def test_servo_rigid_slew_direction_and_clamp(self):
        servo = ServoPlant(self.profile["servo_slew_deg_s"])
        servo.set_pulse(1900)
        self.assertGreater(servo.step(0.001), 0)
        self.assertLess(servo.angle_deg, 36)
        for _ in range(200):
            servo.step(0.001)
        self.assertEqual(servo.angle_deg, 36)
        servo.set_pulse(1100)
        for _ in range(300):
            servo.step(0.001)
        self.assertEqual(servo.angle_deg, -36)
        for value in (1000, 2000, 0):
            with self.assertRaises(ValueError):
                servo.set_pulse(value)
        with self.assertRaises(ValueError):
            servo.step(0.01)


if __name__ == "__main__":
    unittest.main()
