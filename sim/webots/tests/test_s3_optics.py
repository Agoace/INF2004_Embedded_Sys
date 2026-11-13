"""Structural first_hit contract doubles; no Webots or GPU needed."""
import math
from types import SimpleNamespace
import unittest
from sim.webots.pil import s3_optics as o
from sim.webots.pil import s3_geometry as g


class Plane:
    def __init__(self, material=g.MAT_WHITE, edge=False, z=10):
        self.material, self.edge, self.z = material, edge, z
        self.calls = 0

    def first_hit(self, origin_mm, direction, max_distance_mm):
        self.calls += 1
        length = math.sqrt(sum(x*x for x in direction))
        d = tuple(x/length for x in direction)
        if d[2] >= 0:
            return SimpleNamespace(valid=False, reason='miss')
        distance = (self.z-origin_mm[2])/d[2]
        if distance < 0 or distance > max_distance_mm:
            return SimpleNamespace(valid=False, reason='miss')
        point = tuple(a+distance*b for a, b in zip(origin_mm, d))
        material = g.MAT_LINE if self.edge and point[0] >= 0 else self.material
        return SimpleNamespace(valid=True, reason='', point_mm=point,
                               normal=(0, 0, 1), material_id=material,
                               distance_mm=distance, local_uv_mm=point[:2],
                               surface_id='plane')


def profile(**kwargs):
    return o.SensorProfile('GP28', 5.5, 3.0, 3.0,
                           {g.MAT_WHITE: 167., g.MAT_LINE: 2673.,
                            g.PAPER_WHITE: 162., g.PAPER_INK: 1600.},
                           **kwargs)


class OpticsTests(unittest.TestCase):
    def test_flat_material_anchors(self):
        p = profile()
        for material, level in p.levels_adc.items():
            result = o.sample_sensor(Plane(material), p, (0, 0, 15.5))
            self.assertTrue(result.valid)
            self.assertEqual(result.adc, round(level))
            self.assertAlmostEqual(sum(result.material_fractions.values()), 1)

    def test_blurred_edge_is_symmetric_and_narrow_peak_is_not_plateau(self):
        result = o.sample_sensor(Plane(edge=True), profile(), (0, 0, 15.5),
                                 quadrature=32)
        self.assertAlmostEqual(result.mean_adc, (167+2673)/2, places=6)
        bars = ((0., 5.8),)
        middle = o.stripe_coverage(2.9, bars, 3.)
        self.assertGreater(middle, .6)
        self.assertLess(middle, .7)
        self.assertAlmostEqual(o.stripe_coverage(-2, bars, 3),
                               o.stripe_coverage(7.8, bars, 3))
        self.assertAlmostEqual(o.stripe_coverage(2.9, bars, 0), 1)

    def test_response_power_preserves_anchors_without_averaging_adc_again(self):
        p = profile(paper_response_power=6)
        self.assertEqual(o.mean_adc(p, {g.PAPER_INK: 1}), 1600)
        self.assertEqual(o.mean_adc(p, {g.PAPER_WHITE: 1}), 162)
        self.assertAlmostEqual(o.mean_adc(p, {g.PAPER_WHITE: .5, g.PAPER_INK: .5}),
                               162+(1600-162)*.5**6)
        with self.assertRaises(ValueError):
            o.mean_adc(p, {g.PAPER_INK: .5})
        with self.assertRaises(ValueError):
            o.mean_adc(p, {'RGB_BLACK': 1})

    def test_no_hit_and_unknown_material_never_become_white(self):
        for material, origin, reason in [
                ('UNKNOWN', (0, 0, 15.5), 'unknown_material'),
                (g.MAT_WHITE, (0, 0, 150), 'miss')]:
            result = o.sample_sensor(Plane(material), profile(), origin)
            self.assertFalse(result.valid)
            self.assertIsNone(result.adc)
            self.assertEqual(result.reason, reason)

    def test_height_extrapolation_is_not_silent(self):
        result = o.sample_sensor(Plane(), profile(), (0, 0, 18))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, 'uncalibrated_height_or_tilt')
        result = o.sample_sensor(Plane(), profile(), (0, 0, 18),
                                 allow_extrapolation=True)
        self.assertTrue(result.valid)
        self.assertIn('uncalibrated_height_or_tilt', result.assumptions)

    def test_seed_is_time_keyed_and_adc_is_quantized_saturated(self):
        p = profile(noise_sigma_adc=4.45)
        a = o.sample_sensor(Plane(), p, (0, 0, 15.5),
                            sample_us=10, seed=42, quadrature=1)
        o.sample_sensor(Plane(), p, (0, 0, 15.5),
                        sample_us=20, seed=42, quadrature=1)
        b = o.sample_sensor(Plane(), p, (0, 0, 15.5),
                            sample_us=10, seed=42, quadrature=1)
        self.assertEqual(a.adc, b.adc)
        for bias, expected in ((10000, 4095), (-10000, 0)):
            r = o.sample_sensor(Plane(), profile(), (0, 0, 15.5),
                                bias_adc=bias, quadrature=1)
            self.assertEqual(r.adc, expected)
            self.assertTrue(r.clipped)

    def test_invalid_basis_profile_and_resolution_fail(self):
        for sigma in (-1, math.nan):
            with self.assertRaises(ValueError):
                o.SensorProfile('GP28', 5.5, sigma, 3, {})
        with self.assertRaises(ValueError):
            o.sample_sensor(Plane(), profile(), (0, 0, 15.5), quadrature=0)
        with self.assertRaises(ValueError):
            o.sample_sensor(Plane(), profile(), (0, 0, 15.5),
                             direction=(0, 0, 0))
        with self.assertRaises(ValueError):
            o.sample_sensor(Plane(), profile(), (0, 0, 15.5), axis_u=(0, 0, -1))


if __name__ == '__main__':
    unittest.main()
