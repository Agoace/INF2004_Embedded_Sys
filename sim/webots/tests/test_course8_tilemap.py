"""Checks for tools/course8_tilemap.py (figure-8 demo course).

Run: python3 -m unittest sim/webots/tests/test_course8_tilemap.py
"""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "tools"))
import course8_tilemap as t  # noqa: E402

MAP = (HERE / "courses/course_figure8_tilemap.txt").read_text(encoding="utf-8")


class TileMap(unittest.TestCase):
    def setUp(self):
        self.cells = t.parse(MAP)

    def test_track_shape(self):
        facts = t.check(self.cells)
        self.assertEqual(facts["crossing"], (3, 3))
        self.assertEqual(facts["loops_tiles"], [12, 18])
        self.assertEqual(len(facts["loose_arcs"]), 11)
        self.assertEqual(facts["tiles"], 57)

    def test_broken_track_is_rejected(self):
        broken = MAP.replace("r0     .    SE   -    SW", "r0     .    SE   B    SW")
        with self.assertRaises(ValueError):
            t.check(t.parse(broken))
        joined = MAP.replace("SW*  B    .    .    .\nr1", "SW   B    .    .    .\nr1")
        with self.assertRaises(ValueError):
            t.check(t.parse(joined))

    def test_loops_are_closed_and_continuous(self):
        for name in ("L1", "L1r", "L2", "L2r"):
            total = t.loop_length(self.cells, name)
            x0, y0, _ = t.pose(self.cells, name, 0)
            x1, y1, _ = t.pose(self.cells, name, total - 1e-6)
            self.assertLess(math.hypot(x1 - x0, y1 - y0), 1e-3, name)
            prev = None
            step = 5.0
            d = 0.0
            while d < total:
                x, y, _ = t.pose(self.cells, name, d)
                if prev is not None:
                    gap = math.hypot(x - prev[0], y - prev[1])
                    self.assertLessEqual(gap, step + 1e-6, (name, d))
                    self.assertGreater(gap, step * 0.85, (name, d))
                prev = (x, y)
                d += step
        self.assertAlmostEqual(t.loop_length(self.cells, "L1"),
                               7 * 315 + 5 * math.pi * 157.5 / 2,  # 6 straights + crossing, 5 arcs
                               places=6)

    def test_pose_on_line_tiles(self):
        # Every 10 mm the line point lies inside a track tile.
        rows = 8
        track = {c for c, v in self.cells.items() if not v[1] and v[0] != "B"}
        for name in ("L1", "L2"):
            d = 0.0
            while d < t.loop_length(self.cells, name):
                x, y, _ = t.pose(self.cells, name, d)
                col = int(x // 315)
                row = rows - 1 - int(y // 315)
                self.assertIn((col, row), track, (name, d))
                d += 10.0

    def test_default_has_no_objects(self):
        data = t.course(self.cells)
        self.assertEqual((data["boxes"], data["humps"], data["barcodes"]),
                         ([], [], []))

    def test_committed_json_matches_map(self):
        data = json.loads((HERE / "courses/course_figure8_photo.json")
                          .read_text(encoding="utf-8"))
        self.assertEqual(data, json.loads(json.dumps(t.course(self.cells))))

    def test_barcode_is_left_of_drive_direction(self):
        for name, dist in (("L2", 1700.0), ("L2r", 1700.0), ("L1", 400.0)):
            data = t.course(self.cells, barcodes=[(name, dist, "centre")])
            x, y, h = t.pose(self.cells, name, dist)
            bx, by, _ = data["barcodes"][0]["centre_mm"]
            a = math.radians(h)
            left = -math.sin(a) * (bx - x) + math.cos(a) * (by - y)
            self.assertAlmostEqual(left, 77.0, places=6)

    def test_placement_parse(self):
        self.assertEqual(t.placement("L2r:900:left"), ("L2r", 900.0, "left"))
        for bad in ("L3:10", "L1", "L1:10:up"):
            with self.assertRaises(ValueError):
                t.placement(bad)
        with self.assertRaises(ValueError):
            t.pose(self.cells, "L1", 1e6)


if __name__ == "__main__":
    unittest.main()
