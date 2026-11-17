import math
import random
import unittest

from pil.course_builder import build_course
from pil.s3_assumed_line import load_boot5209_line_profile
from pil.s3_optics import sample_sensor
from pil.sil_course8 import course8_course
from pil.sil_course_path import CoursePath
from pil.sil_ray_np import NumpySceneSampler


def same(a, b):
    return (a.adc, a.valid, a.reason, a.mean_adc, a.standoff_mm,
            dict(a.material_fractions), a.assumptions, a.clipped, a.rays) == (
            b.adc, b.valid, b.reason, b.mean_adc, b.standoff_mm,
            dict(b.material_fractions), b.assumptions, b.clipped, b.rays)


def rot(roll, pitch, yaw):
    """Body axes (x forward, z up) in world, as Webots getOrientation."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    r = [[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
         [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
         [-sp, cp*sr, cp*cr]]
    return (r[0][0], r[1][0], r[2][0]), (r[0][2], r[1][2], r[2][2])


class NumpySamplerIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        course = course8_course()
        cls.built = build_course(course)
        cls.fast = NumpySceneSampler(cls.built)
        cls.path = CoursePath.from_course(course)
        cls.profiles = [load_boot5209_line_profile(n) for n in ('GP26', 'GP27')]

    def check(self, poses):
        counts = {'valid': 0, 'invalid': 0}
        for i, (origin, d, u, extra) in enumerate(poses):
            p = self.profiles[i % 2]
            ref = sample_sensor(self.built.scene, p, origin, d, u,
                                sample_us=i * 1000, allow_extrapolation=extra)
            new = self.fast.sample(p, origin, d, u, sample_us=i * 1000,
                                   allow_extrapolation=extra)
            self.assertTrue(same(ref, new), (i, origin, d, ref, new))
            counts['valid' if ref.valid else 'invalid'] += 1
        return counts

    def test_flat_poses_on_and_near_the_track(self):
        rng = random.Random(8)
        poses = []
        for _ in range(400):
            s = rng.uniform(0, self.path.length_mm)
            x, y = self.path.point(s)
            x += rng.uniform(-25, 25)
            y += rng.uniform(-25, 25)
            yaw = rng.uniform(-math.pi, math.pi)
            h = self.profiles[len(poses) % 2].reference_height_mm
            poses.append(((x, y, 10 + h), (0, 0, -1),
                          (math.cos(yaw), math.sin(yaw), 0), False))
        counts = self.check(poses)
        self.assertGreater(counts['valid'], 350)

    def test_tilted_poses_with_extrapolation(self):
        rng = random.Random(9)
        poses = []
        for _ in range(150):
            s = rng.uniform(0, self.path.length_mm)
            x, y = self.path.point(s)
            roll = math.radians(rng.uniform(-6, 6))
            pitch = math.radians(rng.uniform(-3, 3))
            yaw = rng.uniform(-math.pi, math.pi)
            fwd, up = rot(roll, pitch, yaw)
            h = 8.0 + rng.uniform(-3, 3)
            poses.append(((x + rng.uniform(-20, 20), y + rng.uniform(-20, 20),
                           10 + h), tuple(-c for c in up), fwd, True))
        counts = self.check(poses)
        self.assertGreater(counts['valid'], 140)

    def test_tilted_without_extrapolation_is_invalid_same_reason(self):
        fwd, up = rot(math.radians(5), 0, 0.3)
        x, y = self.path.point(1000)
        poses = [((x, y, 18), tuple(-c for c in up), fwd, False)]
        self.check(poses)

    def test_over_ramp_and_off_course(self):
        hump = self.built.course.data['humps'][0]
        hx, hy, _ = hump['centre_mm']
        poses = []
        for dx in range(-110, 111, 10):
            for h in (8.0, 14.0):
                poses.append(((hx + dx, hy, 10 + h), (0, 0, -1), (1, 0, 0), h != 8.0))
                poses.append(((hx + dx, hy + 30, 10 + h), (0, 0, -1), (0, 1, 0), True))
        poses.append(((-500.0, -500.0, 18.0), (0, 0, -1), (1, 0, 0), False))
        counts = self.check(poses)
        self.assertGreater(counts['invalid'], 0)


if __name__ == '__main__':
    unittest.main()
