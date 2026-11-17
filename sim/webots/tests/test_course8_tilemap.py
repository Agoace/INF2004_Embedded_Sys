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
            data = t.course(self.cells,
                            barcodes=[t.placement(name + ":%g" % dist,
                                                  "barcode")])
            x, y, h = t.pose(self.cells, name, dist)
            bx, by, _ = data["barcodes"][0]["centre_mm"]
            a = math.radians(h)
            left = -math.sin(a) * (bx - x) + math.cos(a) * (by - y)
            self.assertAlmostEqual(left, 77.0, places=6)

    def test_placement_parse(self):
        self.assertEqual(t.placement("L2r:900"), ("L2r", 900.0, "C", 0.0))
        self.assertEqual(t.placement("L2r:900", "ramp"),
                         ("L2r", 900.0, "R", 34.0))
        self.assertEqual(t.placement("L2:600:L:80", "ramp"),
                         ("L2", 600.0, "L", 80.0))
        self.assertEqual(t.placement("L1:600", "barcode"),
                         ("L1", 600.0, "L", 77.0))
        for bad, kind in (("L3:10", "obstacle"), ("L1", "obstacle"),
                          ("L1:10:up", "obstacle"), ("L1:10:R:-1", "ramp"),
                          ("L1:10:C:5", "ramp"), ("L1:10:R", "barcode")):
            with self.assertRaises(ValueError):
                t.placement(bad, kind)
        with self.assertRaises(ValueError):
            t.pose(self.cells, "L1", 1e6)


    def test_ramp_beside_line_on_the_right(self):
        for name, dist in (("L2", 600.0), ("L2r", 4000.0), ("L1", 400.0)):
            data = t.course(self.cells,
                            ramps=[t.placement(name + ":%g" % dist, "ramp")])
            x, y, h = t.pose(self.cells, name, dist)
            hump = data["humps"][0]
            hx, hy, _ = hump["centre_mm"]
            a = math.radians(h)
            left = -math.sin(a) * (hx - x) + math.cos(a) * (hy - y)
            along = math.cos(a) * (hx - x) + math.sin(a) * (hy - y)
            # Near edge 34 mm right of the line centre; long axis along it.
            self.assertAlmostEqual(-left - hump["width_mm"] / 2, 34.0)
            self.assertEqual((hump["width_mm"], hump["length_mm"],
                              hump["height_mm"]), (50.0, 195.0, 10.0))
            self.assertAlmostEqual(along, 0.0)
            self.assertAlmostEqual(hump["rotation_deg"], h)

    def test_right_wheel_overlap(self):
        # Tyre 47.75..63.75 mm. Default gap 34 (25 from the line edge) and
        # 25 from the centre: the full tyre (16 mm) is on the 50 mm ramp.
        self.assertEqual(t.wheel_overlap_mm(34.0, 50.0), 16.0)
        self.assertEqual(t.wheel_overlap_mm(25.0, 50.0), 16.0)
        # Photo estimate 75 mm: no overlap when the robot is on the line;
        # touch from 11.25 mm right shift.
        self.assertEqual(t.wheel_overlap_mm(75.0), 0.0)
        self.assertEqual(t.wheel_overlap_mm(75.0, shift_mm=11.25), 0.0)
        self.assertAlmostEqual(t.wheel_overlap_mm(75.0, shift_mm=19.25), 8.0)
        self.assertAlmostEqual(t.wheel_overlap_mm(75.0, shift_mm=27.25), 16.0)
        self.assertAlmostEqual(t.wheel_overlap_mm(40.0), 16.0)
        self.assertAlmostEqual(t.wheel_overlap_mm(55.0), 8.75)


if __name__ == "__main__":
    unittest.main()
