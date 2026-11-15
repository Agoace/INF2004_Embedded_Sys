"""F1 runtime contracts. Physical capture failures remain in test_s3_replay."""
import math
import unittest
from sim.webots.pil import s3_geometry as g
from sim.webots.pil import s3_optics as o
from sim.webots.pil.s3_fast import FastScene
from sim.webots.pil.s3_profiles import load_profile
from sim.webots.pil.s3_referee import fine_sample
from sim.webots.pil.scene import SceneSnapshot


def sheet(letter='F', **kwargs):
    return g.barcode_geometry(letter, bar_height_mm=20,
                              paper_size_mm=(285, 30), **kwargs)


def assumed_line(name):
    return o.SensorProfile(name, 5.5, 2, 2,
                           {g.MAT_WHITE: 200, g.MAT_LINE: 3700,
                            g.PAPER_WHITE: 190, g.PAPER_INK: 1600},
                           assumptions=('SYNTHETIC_TIMING_ONLY_NOT_CALIBRATION',))


class FastTests(unittest.TestCase):
    def test_uniform_and_straight_analytic_line(self):
        patches = g.tile_geometry('STRAIGHT').patches
        fast = FastScene.from_patches(patches)
        profile = assumed_line('SYNTHETIC')
        for x in (-30, -9, -3, 0, 3, 9, 30):
            sample = fast.sample(profile, (x, 0, 15.5))
            self.assertTrue(sample.valid, sample.reason)
            fraction = o.stripe_coverage(x, ((-9, 9),), 2)
            self.assertAlmostEqual(sample.mean_adc, 200+3500*fraction, delta=1)

    def test_cross_t_union_does_not_double_count(self):
        profile = assumed_line('SYNTHETIC')
        for kind in ('CROSS', 'T'):
            patches = g.tile_geometry(kind).patches
            fast, scene = FastScene.from_patches(patches), SceneSnapshot.from_patches(patches)
            for xy in ((0, 0), (9, 9), (12, 12), (0, -20)):
                actual = fast.sample(profile, (*xy, 15.5))
                expected = fine_sample(scene, profile, (*xy, 15.5))
                self.assertTrue(actual.valid)
                self.assertTrue(expected.valid)
                self.assertLess(abs(actual.mean_adc-expected.mean_adc), 2)
                self.assertAlmostEqual(sum(actual.material_fractions.values()), 1)

    def test_barcode_finite_ends_paper_over_line_and_arbitrary_yaw(self):
        patches = g.tile_geometry('CROSS').patches + sheet().patches
        fast, scene = FastScene.from_patches(patches), SceneSnapshot.from_patches(patches)
        profile = load_profile()
        for x, y in ((-133.85, 0), (-133.85, 10), (0, 14), (140, 0), (0, 23)):
            for degrees in (0, 17, 45, 89, 180, 271):
                angle = math.radians(degrees)
                axis = (math.cos(angle), math.sin(angle), 0)
                a = fast.sample(profile, (x, y, 15.5), axis_u=axis)
                b = fine_sample(scene, profile, (x, y, 15.5), axis_u=axis)
                self.assertTrue(a.valid, a.reason)
                self.assertTrue(b.valid, b.reason)
                self.assertLessEqual(abs(a.mean_adc-b.mean_adc), 8)

    def test_referee_converges_independently_and_matches_exact_strip(self):
        scene = SceneSnapshot.from_patches(g.tile_geometry('STRAIGHT').patches)
        profile = assumed_line('SYNTHETIC')
        for x in (-10.1, -8.9, 0, 8.9, 10.1):
            a = fine_sample(scene, profile, (x, 0, 15.5), tolerance=1e-6)
            b = fine_sample(scene, profile, (x, 0, 15.5), tolerance=1e-8)
            self.assertLessEqual(abs(a.mean_adc-b.mean_adc), 1)
            self.assertAlmostEqual(b.mean_adc, 200+3500*o.stripe_coverage(x, ((-9, 9),), 2), delta=1)

    def test_curves_nonrectangles_and_stale_revision_fail_explicitly(self):
        curve = g.tile_geometry('CURVE')
        fast = FastScene.from_patches(curve.patches)
        sample = fast.sample(load_profile(), (46.13, 46.13, 15.5))
        self.assertFalse(sample.valid)
        self.assertIn('UNSUPPORTED', sample.reason)
        fast = FastScene.from_patches(g.tile_geometry('STRAIGHT').patches)
        sample = fast.sample(load_profile(), (0, 0, 15.5), expected_revision='old')
        self.assertFalse(sample.valid)
        self.assertEqual(sample.reason, 'STALE_SCENE_REVISION')

    def test_missing_unknown_ambiguous_height_tilt_never_white(self):
        base = g.tile_geometry('BLANK').patches
        p = load_profile()
        for patches, origin, options in (
                ((), (0, 0, 15.5), {}),
                (base, (0, 0, 16), {}),
                (base, (0, 0, 15.5), {'direction': (0, .1, -1)}),
                (base+(g.PolygonPatch('unknown', 'UNKNOWN', base[0].vertices_mm, 20),),
                 (0, 0, 15.5), {}),
                (base+(g.PolygonPatch('ambiguous', g.PAPER_INK, base[0].vertices_mm, 0),),
                 (0, 0, 15.5), {})):
            sample = FastScene.from_patches(patches).sample(p, origin, **options)
            self.assertFalse(sample.valid)
            self.assertIsNone(sample.adc)

    def test_anisotropic_and_null_channels_are_not_silently_substituted(self):
        fast = FastScene.from_patches(g.tile_geometry('STRAIGHT').patches)
        p = o.SensorProfile('ASSUMED_ANISOTROPIC', 5.5, 2, 3,
                            dict(load_profile().levels_adc))
        self.assertEqual(fast.sample(p, (0, 0, 15.5)).reason, 'UNSUPPORTED_ANISOTROPIC')
        for name in ('GP26', 'GP27'):
            with self.assertRaises(ValueError):
                load_profile(name)


if __name__ == '__main__':
    unittest.main()
