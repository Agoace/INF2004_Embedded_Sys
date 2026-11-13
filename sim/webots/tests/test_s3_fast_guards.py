"""F1 invalidity, revision, noise and coverage tests; no runtime integration."""
import math
import random
import unittest
from unittest.mock import patch
from sim.webots.pil import s3_geometry as g
from sim.webots.pil.course_builder import Course, build_course
from sim.webots.pil.s3_fast import FastScene
from sim.webots.pil.s3_kinematics import Pose2, advance, point_world
from sim.webots.pil.s3_profiles import load_profile
from sim.webots.pil.s3_referee import fine_sample
from sim.webots.pil.scene import SceneSnapshot
from pathlib import Path


class GuardTests(unittest.TestCase):
    def test_no_ray_or_cache_build_occurs_at_runtime_and_seed_is_unchanged(self):
        p = load_profile()
        fast = FastScene.from_patches(g.tile_geometry('CROSS').patches)
        for t in (0, 1000, 999999):
            a = fast.sample(p, (0, 0, 15.5), sample_us=t, seed=42,
                            expected_revision=fast.revision)
            noise = random.Random(f'42:{p.name}:{t}').gauss(0, p.noise_sigma_adc)
            self.assertEqual(a.adc, max(0, min(4095, round(a.mean_adc+noise))))
            self.assertEqual(a.rays, 0)
            self.assertEqual(a, fast.sample(p, (0, 0, 15.5), sample_us=t, seed=42))
            with self.assertRaises(TypeError):
                a.material_fractions[g.MAT_WHITE] = 1
        with self.assertRaises(TypeError):
            fast.cells[(0, 0)] = ()

    def test_zero_noise_synthetic_path_does_not_allocate_a_private_rng(self):
        from sim.webots.pil.s3_optics import SensorProfile
        fast = FastScene.from_patches(g.tile_geometry('BLANK').patches)
        profile = SensorProfile('SYNTHETIC_ZERO_NOISE', 5.5, 2, 2,
                                dict(load_profile().levels_adc),
                                assumptions=('ASSUMED_TEST_ONLY',))
        with patch('sim.webots.pil.s3_fast.random.Random',
                   side_effect=AssertionError('zero-noise RNG must not be built')):
            sample = fast.sample(profile, (0, 0, 15.5))
        self.assertEqual(sample.adc, round(sample.mean_adc))

    def test_hole_raised_paper_and_occluder_are_never_valid_white(self):
        p = load_profile()
        base = g.tile_geometry('BLANK').patches
        raised = g.barcode_geometry('F', bar_height_mm=20,
                                    paper_size_mm=(285, 30), paper_thickness_mm=.1)
        for patches, bounds in ((base+raised.patches, ()),
                (base, ((-5, -5, 10, 5, 5, 30, 'UNSUPPORTED_OCCLUDER'),)),
                (base, ((-5, -5, 10, 5, 5, 10, 'UNSUPPORTED_CURVE'),))):
            a = FastScene.from_patches(patches, bounds).sample(p, (0, 0, 15.5))
            self.assertFalse(a.valid)
            self.assertIsNone(a.adc)
        a = FastScene.from_patches(base).sample(p, (155, 0, 15.5))
        self.assertEqual(a.reason, 'UNSUPPORTED_ENDCAP_OR_GAP')

    def test_curve_course_is_explicitly_unsupported_not_a_full_course_pass(self):
        path = Path(__file__).resolve().parents[1]/'courses/home_mat_1_assumed.json'
        built = build_course(Course.load(path))
        fast = FastScene.from_course(built)
        for _, (x, y, _), _ in built.tile_geometries:
            self.assertEqual(fast.sample(load_profile(), (x, y, 15.5)).reason,
                             'UNSUPPORTED_CURVE')

    def test_rigid_point_yaw_can_exit_and_reenter_a_finite_bar(self):
        floor = g.PolygonPatch('floor', g.PAPER_WHITE,
            ((-500, -500, 10), (500, -500, 10), (500, 500, 10), (-500, 500, 10)), 20)
        ink = g.PolygonPatch('ink', g.PAPER_INK,
            ((77, 73, 10), (83, 73, 10), (83, 81, 10), (77, 81, 10)), 30)
        fast = FastScene.from_patches((floor, ink))
        values = []
        for yaw in (0, math.pi/2, 0):
            pose = advance(Pose2(), -111*yaw/2, 111*yaw/2)
            x, y = point_world(pose)
            sample = fast.sample(load_profile(), (x, y, 15.5),
                                  axis_u=(math.cos(yaw), math.sin(yaw), 0))
            self.assertTrue(sample.valid)
            values.append(sample.mean_adc)
        self.assertGreater(values[0], values[1]+20)
        self.assertAlmostEqual(values[0], values[2])

    def test_revision_changes_with_material_layer_height_and_guard(self):
        base = g.tile_geometry('BLANK').patches
        hashes = {FastScene.from_patches(base).revision}
        for material, height, layer in ((g.PAPER_WHITE, 10, 0),
                                       (g.MAT_WHITE, 10.1, 0),
                                       (g.MAT_WHITE, 10, 1)):
            p = g.PolygonPatch('tile:base', material,
                               tuple((x, y, height) for x, y, _ in base[0].vertices_mm), layer)
            hashes.add(FastScene.from_patches((p,)).revision)
        hashes.add(FastScene.from_patches(base, ((0, 0, 10, 1, 1, 10, 'UNSUPPORTED'),)).revision)
        self.assertEqual(len(hashes), 5)
        with self.assertRaises(ValueError):
            FastScene.from_patches(base+base)

    def test_referee_and_fast_agree_on_invalid_known_domains(self):
        base = g.tile_geometry('BLANK').patches
        for material, layer in (('UNKNOWN', 20), (g.PAPER_INK, 0)):
            patches = base+(g.PolygonPatch('overlay', material, base[0].vertices_mm, layer),)
            fast = FastScene.from_patches(patches).sample(load_profile(), (0, 0, 15.5))
            ref = fine_sample(SceneSnapshot.from_patches(patches), load_profile(), (0, 0, 15.5))
            self.assertFalse(fast.valid)
            self.assertFalse(ref.valid)
            self.assertEqual(fast.reason, ref.reason)


if __name__ == '__main__':
    unittest.main()
