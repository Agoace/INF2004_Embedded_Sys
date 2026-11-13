"""Real frozen SW5 scene integration, NOT a replacement scene implementation.

After merge, imports sim.webots.pil.scene. Before merge, set S3_SCENE_MODULE
to the exact committed scene file extracted under build/. A missing optional
fixture gives an explicit skip; a requested but broken fixture fails.
"""
import importlib
import importlib.util
import math
import os
import sys
import unittest
from sim.webots.pil import s3_geometry as g
from sim.webots.pil import s3_optics as o
from sim.webots.pil.s3_kinematics import Pose2, advance, point_world


class SceneIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = os.environ.get('S3_SCENE_MODULE')
        if path:
            spec = importlib.util.spec_from_file_location('s3_frozen_scene_fixture', path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            cls.scene = module
        else:
            try:
                cls.scene = importlib.import_module('sim.webots.pil.scene')
            except ModuleNotFoundError as error:
                if error.name != 'sim.webots.pil.scene':
                    raise
                raise unittest.SkipTest('SW5 scene not merged; set S3_SCENE_MODULE for integration')

    def make_profile(self, **kwargs):
        return o.SensorProfile('SYNTHETIC_ASSUMED', 5.5, 2, 2,
                               {g.MAT_WHITE: 167, g.MAT_LINE: 2673,
                                g.PAPER_WHITE: 164, g.PAPER_INK: 1500},
                               assumptions=('SYNTHETIC_ASSUMED',), **kwargs)

    def test_all_generated_tiles_are_accepted_convex_patches(self):
        for kind in ('STRAIGHT', 'CURVE', 'CROSS', 'T', 'BLANK'):
            for angle in (0, 90, 180, 270):
                tile = g.tile_geometry(kind, rotation_deg=angle)
                scene = self.scene.SceneSnapshot.from_patches(tile.patches)
                for port in tile.ports:
                    hit = scene.first_hit((port.point_mm[0], port.point_mm[1], 15.5),
                                          (0, 0, -20), 10)
                    self.assertTrue(hit.valid)
                    self.assertEqual(hit.material_id, g.MAT_LINE)
                    self.assertAlmostEqual(hit.distance_mm, 5.5)

    def test_every_letter_and_rotated_bar_material(self):
        for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
            for angle in (0, 90):
                bar = g.barcode_geometry(letter, bar_height_mm=30,
                                          paper_size_mm=(285, 40), rotation_deg=angle)
                scene = self.scene.SceneSnapshot.from_patches(bar.patches)
                for start, end in bar.bar_intervals_mm:
                    x = (start+end)/2
                    origin = (x, 0, 15.5) if angle == 0 else (0, x, 15.5)
                    hit = scene.first_hit(origin, (0, 0, -1), 10)
                    self.assertTrue(hit.valid)
                    self.assertEqual(hit.material_id, g.PAPER_INK)

    def test_nearer_occluder_wins_without_becoming_white(self):
        tile = g.tile_geometry('STRAIGHT')
        cover = g.PolygonPatch('cover', 'UNKNOWN_OBSTACLE',
                               ((-20, -20, 12), (20, -20, 12),
                                (20, 20, 12), (-20, 20, 12)), 0)
        scene = self.scene.SceneSnapshot.from_patches(tile.patches+(cover,))
        self.assertEqual(scene.first_hit((0, 0, 15.5), (0, 0, -1), 100).surface_id, 'cover')
        sample = o.sample_sensor(scene, self.make_profile(), (0, 0, 15.5),
                                  allow_extrapolation=True)
        self.assertFalse(sample.valid)
        self.assertIsNone(sample.adc)
        self.assertEqual(sample.reason, 'unknown_material')

    def test_miss_conflicting_layers_and_invalid_ray(self):
        a = g.PolygonPatch('a', g.MAT_WHITE,
                           ((-30, -30, 10), (30, -30, 10),
                            (30, 30, 10), (-30, 30, 10)), 0)
        b = g.PolygonPatch('b', g.PAPER_INK, a.vertices_mm, 0)
        scene = self.scene.SceneSnapshot.from_patches((a, b))
        sample = o.sample_sensor(scene, self.make_profile(), (0, 0, 15.5))
        self.assertFalse(sample.valid)
        self.assertEqual(sample.reason, 'ambiguous_material')
        self.assertIsNone(sample.adc)
        self.assertEqual(scene.first_hit((0, 0, 15.5), (0, 0, 0), 100).reason, 'invalid_ray')
        self.assertEqual(scene.first_hit((500, 0, 15.5), (0, 0, -1), 100).reason, 'miss')

    def test_rigid_sensor_pure_yaw_exits_and_reenters_finite_bar(self):
        # Synthetic 1000mm floor; single 6x8mm bar placed at GP28 initial pose.
        floor = g.PolygonPatch('floor', g.PAPER_WHITE,
                               ((-500, -500, 10), (500, -500, 10),
                                (500, 500, 10), (-500, 500, 10)), 20)
        ink = g.PolygonPatch('ink', g.PAPER_INK,
                             ((77, 73, 10), (83, 73, 10),
                              (83, 81, 10), (77, 81, 10)), 30)
        scene = self.scene.SceneSnapshot.from_patches((floor, ink))
        profile = self.make_profile()
        values = []
        for yaw in (0, math.pi/2, 0):
            pose = advance(Pose2(), -111*yaw/2, 111*yaw/2)
            x, y = point_world(pose, 80, 77)
            values.append(o.sample_sensor(scene, profile, (x, y, 15.5),
                                           quadrature=32).adc)
        self.assertGreater(values[0], 800)
        self.assertEqual(values[1], 164)
        self.assertEqual(values[0], values[2])
        # Axle mean distance is zero for every pose; signal is not held still.

    def test_ray_resolution_error_is_not_hidden_in_measured_noise(self):
        floor = g.PolygonPatch('floor', g.PAPER_WHITE,
                               ((-500, -500, 10), (500, -500, 10),
                                (500, 500, 10), (-500, 500, 10)), 20)
        ink = g.PolygonPatch('ink', g.PAPER_INK,
                             ((0, -100, 10), (5.8, -100, 10),
                              (5.8, 100, 10), (0, 100, 10)), 30)
        scene = self.scene.SceneSnapshot.from_patches((floor, ink))
        profile = self.make_profile()
        for x in (-1, 1, 2.9, 5, 7):
            exact = 164 + (1500-164)*o.stripe_coverage(x, ((0, 5.8),), 2)
            sample = o.sample_sensor(scene, profile, (x, 0, 15.5), quadrature=128)
            self.assertLess(abs(sample.mean_adc-exact), 40)
            self.assertEqual(profile.noise_sigma_adc, 0)


if __name__ == '__main__':
    unittest.main()
