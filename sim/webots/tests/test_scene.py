import dataclasses
import math
import random
import unittest

from pil.scene import PolygonSurface, SceneSnapshot, box_surfaces, rectangle


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.floor = rectangle("floor", "MAT_WHITE", (0, 0, 10), (315, 315))

    def test_normalized_distance_uv(self):
        hit = SceneSnapshot((self.floor,)).first_hit((0, 0, 15.5), (0, 0, -5), 20)
        self.assertTrue(hit.valid)
        self.assertEqual(hit.reason, "")
        self.assertEqual(hit.normal, (0, 0, 1))
        self.assertEqual(hit.point_mm, (0, 0, 10))
        self.assertEqual(hit.distance_mm, 5.5)
        self.assertEqual(hit.local_uv_mm, (157.5, 157.5))

    def test_miss_not_white(self):
        hit = SceneSnapshot((self.floor,)).first_hit((400, 0, 15), (0, 0, -1), 20)
        self.assertFalse(hit.valid)
        self.assertEqual(hit.reason, "miss")
        self.assertIsNone(hit.material_id)

    def test_range_parallel_and_invalid_rays(self):
        scene = SceneSnapshot((self.floor,))
        for origin, direction, limit, reason in (
            ((0, 0, 20), (0, 0, -1), 9, "miss"),
            ((0, 0, 20), (1, 0, 0), 20, "miss"),
            ((0, 0, 20), (0, 0, 0), 20, "invalid_ray"),
            ((math.nan, 0, 20), (0, 0, -1), 20, "invalid_ray"),
            ((0, 0, 20), (0, math.inf, -1), 20, "invalid_ray"),
            ((0, 0, 20), (0, 0, -1), 0, "invalid_ray"),
            ((0, 0, 20), (0, 0, -1), math.inf, "invalid_ray"),
        ):
            with self.subTest(origin=origin, direction=direction, limit=limit):
                self.assertEqual(scene.first_hit(origin, direction, limit).reason, reason)

    def test_layers_never_beat_nearer_surface(self):
        line = rectangle("line", "MAT_LINE", (0, 0, 10), (18, 315), layer=10)
        paper = rectangle("paper", "PAPER_WHITE", (0, 0, 10.1), (80, 80), layer=20)
        box = box_surfaces("box", "BOX", (0, 0, 50), (20, 20, 40))
        scene = SceneSnapshot((self.floor, line, paper) + box)
        self.assertEqual(scene.first_hit((0, 0, 90), (0, 0, -1), 100).material_id, "BOX")
        self.assertEqual(scene.first_hit((20, 0, 90), (0, 0, -1), 100).material_id,
                         "PAPER_WHITE")
        self.assertEqual(scene.first_hit((0, 100, 90), (0, 0, -1), 100).material_id,
                         "MAT_LINE")
        self.assertEqual(scene.first_hit((60, 100, 90), (0, 0, -1), 100).material_id,
                         "MAT_WHITE")

    def test_coplanar_priority_and_ambiguity(self):
        a = rectangle("a", "MAT_LINE", (0, 0, 10), (18, 315), layer=10)
        b = rectangle("b", "MAT_LINE", (0, 0, 10), (315, 18), layer=10)
        scene = SceneSnapshot((b, self.floor, a))
        self.assertEqual(scene.first_hit((0, 0, 20), (0, 0, -1), 20).surface_id, "a")
        conflict = rectangle("c", "PAPER_WHITE", (0, 0, 10), (80, 80), layer=10)
        self.assertEqual(SceneSnapshot((a, conflict)).first_hit(
            (0, 0, 20), (0, 0, -1), 20).reason, "ambiguous_material")

    def test_tilted_triangle(self):
        ramp = PolygonSurface("ramp", "MAT_WHITE", ((0, 0, 10), (20, 0, 20), (0, 20, 10)))
        hit = SceneSnapshot((ramp,)).first_hit((5, 5, 40), (0, 0, -1), 50)
        self.assertAlmostEqual(hit.point_mm[2], 12.5)
        self.assertLess(hit.normal[0], 0)
        self.assertGreater(hit.normal[2], 0)

    def test_rotated_box_occludes_horizontal_ray(self):
        scene = SceneSnapshot(box_surfaces("box", "BOX", (0, 0, 30), (40, 20, 20), 90))
        hit = scene.first_hit((-100, 0, 30), (1, 0, 0), 200)
        self.assertAlmostEqual(hit.distance_mm, 90)
        self.assertAlmostEqual(hit.normal[0], -1)

    def test_polygon_validation_and_snapshot_immutability(self):
        for vertices in (
            ((0, 0, 0), (0, 0, 0), (1, 1, 0)),
            ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 1)),
            ((0, 0, 0), (2, 0, 0), (0.5, 0.5, 0), (2, 2, 0), (0, 2, 0)),
        ):
            with self.subTest(vertices=vertices), self.assertRaises(ValueError):
                PolygonSurface("bad", "MAT_WHITE", vertices)
        with self.assertRaises(ValueError):
            SceneSnapshot((self.floor, self.floor))
        with self.assertRaises(ValueError):
            SceneSnapshot((object(),))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            self.floor.layer = 10
        with self.assertRaises(TypeError):
            SceneSnapshot((self.floor,))._cells[(0, 0)] = ()

    def test_index_matches_brute_force(self):
        random.seed(202611)
        surfaces = tuple(rectangle(str(i), "MAT_WHITE", (i * 80 - 800, i * 15, 10),
                                   (200, 200), (i % 4) * 90) for i in range(20))
        surfaces += box_surfaces("box", "BOX", (50, 30, 50), (80, 80, 80), 33)
        scene = SceneSnapshot(surfaces)
        for _ in range(500):
            origin = (random.uniform(-1200, 1200), random.uniform(-300, 700), 150)
            direction = (random.uniform(-0.7, 0.7), random.uniform(-0.7, 0.7), -1)
            norm = math.sqrt(sum(x * x for x in direction))
            ray = tuple(x / norm for x in direction)
            raw = [s.intersect(origin, ray, 2000) for s in surfaces]
            raw = [h for h in raw if h is not None]
            hit = scene.first_hit(origin, direction, 2000)
            self.assertEqual(hit.valid, bool(raw))
            if raw:
                self.assertAlmostEqual(hit.distance_mm, min(h.distance_mm for h in raw))

    def test_grid_boundaries_and_negative_coordinates(self):
        surfaces = tuple(rectangle(f"tile{i}", "MAT_WHITE", (i * 315, 0, 10),
                                   (315, 315)) for i in range(-2, 3))
        scene = SceneSnapshot(surfaces)
        for x in (-787.5, -472.5, -315, -157.5, 0, 157.5, 472.5, 787.5):
            self.assertTrue(scene.first_hit((x, 0, 20), (0, 0, -1), 20).valid)

    def test_structural_patch_copy(self):
        @dataclasses.dataclass(frozen=True)
        class Patch:
            surface_id: str
            material_id: str
            vertices_mm: tuple
            layer: int
        patch = Patch("paper", "PAPER_INK", ((0, 0, 10), (10, 0, 10), (0, 10, 10)), 30)
        scene = SceneSnapshot.from_patches((patch,))
        self.assertEqual(scene.first_hit((1, 1, 15), (0, 0, -1), 10).material_id,
                         "PAPER_INK")


if __name__ == "__main__":
    unittest.main()
