"""Independent exact geometry checks; never rewrite firmware encoders."""
import math
import unittest
from sim.webots.pil.s3_kinematics import (
    Pose2, advance, point_world, point_velocity_body, stripe_progress)


class KinematicsTests(unittest.TestCase):
    def test_straight_forward_and_reverse(self):
        for distance in (100.0, -50.0):
            p = advance(Pose2(), distance, distance)
            self.assertAlmostEqual(p.x_mm, distance)
            self.assertAlmostEqual(p.y_mm, 0)
            self.assertAlmostEqual(stripe_progress(
                point_world(p, 80, 77), point_world(Pose2(), 80, 77),
                (1, 0)), distance)

    def test_turn_circle_and_split_step_are_exact(self):
        yaw = math.pi / 2
        p = advance(Pose2(), 0, 111*yaw)
        self.assertAlmostEqual(p.x_mm, 55.5)
        self.assertAlmostEqual(p.y_mm, 55.5)
        self.assertAlmostEqual(p.yaw_rad, yaw)
        q = Pose2()
        for _ in range(100):
            q = advance(q, 0, 111*yaw/100)
        self.assertAlmostEqual(p.x_mm, q.x_mm)
        self.assertAlmostEqual(p.y_mm, q.y_mm)
        self.assertAlmostEqual(p.yaw_rad, q.yaw_rad)

    def test_pure_yaw_moves_sensor_not_axle(self):
        p = advance(Pose2(), -111*math.pi/4, 111*math.pi/4)
        self.assertAlmostEqual(p.x_mm, 0)
        self.assertAlmostEqual(p.y_mm, 0)
        x, y = point_world(p, 80, 77)
        self.assertAlmostEqual(x, -77)
        self.assertAlmostEqual(y, 80)
        self.assertAlmostEqual(math.hypot(x, y), math.hypot(80, 77))
        self.assertNotEqual(stripe_progress((x, y), (80, 77), (1, 0)), 0)

    def test_offset_outside_half_track_can_reverse_forward_motion(self):
        vx, vy = point_velocity_body(40, 217, 80, 77)
        self.assertAlmostEqual(vx, 5.716216216216225)
        self.assertAlmostEqual(vy, 127.56756756756756)
        self.assertLess(vx, (40+217)/2)
        vx, _ = point_velocity_body(0, 217, 80, 77)
        self.assertLess(vx, 0)

    def test_yaw_derivative_matches_point_velocity(self):
        for left, right in ((40, 217), (217, 40), (146, 146), (-50, 50)):
            epsilon = 1e-6
            p = advance(Pose2(), left*epsilon, right*epsilon)
            x, y = point_world(p, 80, 77)
            vx, vy = point_velocity_body(left, right, 80, 77)
            self.assertAlmostEqual((x-80)/epsilon, vx, delta=.001)
            self.assertAlmostEqual((y-77)/epsilon, vy, delta=.001)

    def test_stripe_normal_is_not_arclength_and_sign_rotates(self):
        self.assertEqual(stripe_progress((10, 25), (0, 0), (1, 0)), 10)
        self.assertEqual(stripe_progress((10, 25), (0, 0), (0, -2)), -25)
        with self.assertRaises(ValueError):
            stripe_progress((0, 0), (0, 0), (0, 0))
        for track in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                advance(Pose2(), 1, 1, track)


if __name__ == '__main__':
    unittest.main()
