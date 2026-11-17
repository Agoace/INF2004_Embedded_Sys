"""Checks for tools/course8_tilemap.py (figure-8 demo course).

Run: python3 -m unittest sim/webots/tests/test_course8_tilemap.py
"""
import contextlib
import io
import json
import math
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "tools"))
import course8_tilemap as t  # noqa: E402
import course8_world_patch as wp  # noqa: E402

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


    WORLD = ("#VRML_SIM R2025a utf8\n"
             "Viewpoint { position 1.2 1.3 4.5 orientation 0 0 1 0 }\n"
             'Solid { name "ramp_0" contactMaterial "course" children '
             "[ Shape { appearance PBRAppearance { baseColor 0.93 0.93 "
             "0.93 roughness 1 } } ] }\n"
             'Solid { name "r3c3" children [ Shape { appearance '
             "PBRAppearance { baseColor 0.93 0.93 0.93 } } ] }")

    def test_world_patch(self):
        text, views, ramps, solids = wp.patch(self.WORLD)
        self.assertEqual((views, ramps, solids), (1, 1, 1))
        self.assertIn("position 1.2 1.3 3.6 orientation -0.57735", text)
        self.assertIn('"ramp_0" contactMaterial "course" children [ Shape { '
                      "appearance PBRAppearance { baseColor 0.85 0.1 0.55",
                      text)
        self.assertIn('"r3c3" children [ Shape { appearance PBRAppearance '
                      "{ baseColor 0.93 0.93 0.93", text)
        self.assertEqual(text.split("\n")[1], wp.PREVIEW)
        again = wp.patch(text)
        self.assertEqual((again[0], again[1:]), (text, (1, 1, 1)))

    def test_view_height_fits_course_16_9(self):
        cells = t.parse(MAP)
        cx, cy, w, h = wp.extents(t.course(cells))
        self.assertAlmostEqual(w, 9 * 0.315)
        self.assertAlmostEqual(h, 8 * 0.315)
        z = wp.view_height(w, h)
        vfov = 2 * math.atan(math.tan(math.pi / 8) * 9 / 16)
        # Height is the limit in 16:9: 2.52 m * 1.1 fits the vertical FOV.
        self.assertAlmostEqual(2 * z * math.tan(vfov / 2), h * 1.1)
        self.assertGreaterEqual(2 * z * math.tan(math.pi / 8), w * 1.1)
        text = wp.patch(self.WORLD, (cx, cy, w, h))[0]
        self.assertIn("position %.4f %.4f %.3f orientation" % (cx, cy, z), text)

    def test_world_patch_main_atomic_and_no_viewpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            world = pathlib.Path(tmp) / "w.wbt"
            bad = self.WORLD.replace("Viewpoint", "Viewxoint")
            world.write_text(bad, encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(wp.main([str(world)]), 1)
            self.assertEqual(world.read_text(encoding="utf-8"), bad)
            world.write_text(self.WORLD, encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(wp.main([str(world)]), 0)
            self.assertIn("baseColor 0.85 0.1 0.55", world.read_text(encoding="utf-8"))
            self.assertEqual(sorted(p.name for p in pathlib.Path(tmp).iterdir()), ["w.wbt"])

    def test_tilemap_out_creates_parent(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp) / "a" / "b" / "example.json"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(t.main(["--out", str(out)]), 0)
            self.assertEqual(json.loads(out.read_text())["name"], "course_figure8_photo")

if __name__ == "__main__":
    unittest.main()
