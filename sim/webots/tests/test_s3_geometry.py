"""Host-only contracts for the shared S3 scene/texture geometry."""
import math
from pathlib import Path
import re
import unittest

from sim.webots.pil import s3_geometry as g

ROOT = Path(__file__).resolve().parents[3]


class GeometryTests(unittest.TestCase):
    def test_code39_matches_firmware_table(self):
        source = (ROOT / 'app_program/subsystem3/barcode.c').read_text()
        alphabet = re.search(r'alphabet\[\] = "([^"]+)"', source)[1]
        values = re.search(r'patterns\[\] = \{([^}]+)', source)[1]
        masks = [int(x, 16) for x in re.findall(r'0x([0-9a-f]+)u', values)]
        expected = dict(zip(alphabet, masks))
        for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ*':
            self.assertEqual(g.CODE39[letter], expected[letter])

    def test_all_letters_have_same_extent_and_fifteen_bars(self):
        for letter in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
            barcode = g.barcode_geometry(letter, bar_height_mm=40,
                                         paper_size_mm=(285, 50))
            self.assertEqual(len(barcode.elements_mm), 29)
            self.assertEqual(barcode.elements_mm.count(5.8), 20)
            self.assertEqual(barcode.elements_mm.count(17.5), 9)
            self.assertAlmostEqual(barcode.body_length_mm, 273.5)
            self.assertAlmostEqual(barcode.quiet_length_mm, 389.5)
            self.assertEqual(sum(p.material_id == g.PAPER_INK
                                 for p in barcode.patches), 15)
            self.assertTrue(barcode.assumptions)

    def test_reverse_is_exact_mirror(self):
        a = g.barcode_geometry('A', bar_height_mm=40)
        b = g.barcode_geometry('A', bar_height_mm=40, reverse=True)
        self.assertEqual(a.elements_mm[::-1], b.elements_mm)
        for x in range(-140, 141):
            self.assertEqual(g.material_at(a.patches, x, 0),
                             g.material_at(b.patches, -x, 0))

    def test_paper_is_not_full_quiet_envelope(self):
        b = g.barcode_geometry('F', bar_height_mm=40,
                               paper_size_mm=(285, 50))
        self.assertEqual(g.material_at(b.patches, -180, 0), None)
        self.assertEqual(g.material_at(b.patches, -140, 0), g.PAPER_WHITE)
        self.assertEqual(g.material_at(b.patches, 0, 26), None)

    def test_tile_ports_rotations_and_surface_height(self):
        expected = {'STRAIGHT': {'N', 'S'}, 'CURVE': {'N', 'E'},
                    'CROSS': set('NESW'), 'T': set('NEW'), 'BLANK': set()}
        for name, ports in expected.items():
            tile = g.tile_geometry(name, centre_mm=(10, 20, 10))
            self.assertEqual({p.name for p in tile.ports}, ports)
            self.assertTrue(all(v[2] == 10 for p in tile.patches
                                for v in p.vertices_mm))
            rotated = g.tile_geometry(name, rotation_deg=90)
            self.assertEqual({p.name for p in rotated.ports},
                             {dict(N='W', W='S', S='E', E='N')[p]
                              for p in ports})
        with self.assertRaises(ValueError):
            g.tile_geometry('STRAIGHT', rotation_deg=45)

    def test_curve_is_corner_centred_not_tile_centred(self):
        tile = g.tile_geometry('CURVE')
        self.assertEqual(g.material_at(tile.patches, 0, 157.5), g.MAT_LINE)
        self.assertEqual(g.material_at(tile.patches, 157.5, 0), g.MAT_LINE)
        self.assertEqual(g.material_at(tile.patches, 0, 0), g.MAT_WHITE)
        q = 157.5 - 157.5 / math.sqrt(2)
        self.assertEqual(g.material_at(tile.patches, q, q), g.MAT_LINE)
        for patch in tile.patches[1:]:
            for x, y, _ in patch.vertices_mm:
                radius = math.hypot(x - 157.5, y - 157.5)
                self.assertLess(min(abs(radius - 148.5),
                                    abs(radius - 166.5)), 1e-8)
        self.assertLessEqual(tile.max_chord_error_mm, .01)

    def test_cross_and_t_have_correct_half_strokes(self):
        cross = g.tile_geometry('CROSS')
        tee = g.tile_geometry('T')
        for x, y in [(0, 100), (100, 0), (-100, 0), (0, 0)]:
            self.assertEqual(g.material_at(tee.patches, x, y), g.MAT_LINE)
        self.assertEqual(g.material_at(tee.patches, 0, -100), g.MAT_WHITE)
        self.assertEqual(g.material_at(cross.patches, 0, -100), g.MAT_LINE)

    def test_station_is_explicit_and_invalid_inputs_fail(self):
        with self.assertRaises(ValueError):
            g.tile_geometry('STATION')
        station = g.tile_geometry('STATION', station_ports='NS',
                                  station_marker_mm=((-20, -10), (20, -10),
                                                     (20, 10), (-20, 10)))
        self.assertIn('station_marker_assumed', station.assumptions)
        for letter in ['a', '*', 'AB', '1']:
            with self.assertRaises(ValueError):
                g.barcode_geometry(letter, bar_height_mm=40)
        for size in [(250, 50), (285, 20), (math.nan, 50)]:
            with self.assertRaises(ValueError):
                g.barcode_geometry('F', bar_height_mm=40, paper_size_mm=size)
        with self.assertRaises(ValueError):
            g.barcode_geometry('F', bar_height_mm=0)

    def test_svg_uses_same_polygons_and_escapes_ids(self):
        tile = g.tile_geometry('CROSS', tile_id='a<&')
        svg = g.geometry_svg(tile.patches, (-157.5, -157.5, 157.5, 157.5))
        self.assertEqual(svg.count('<polygon '), len(tile.patches))
        self.assertIn('a&lt;&amp;', svg)
        self.assertNotIn('a<&', svg)
        self.assertIn('315mm', svg)


if __name__ == '__main__':
    unittest.main()
