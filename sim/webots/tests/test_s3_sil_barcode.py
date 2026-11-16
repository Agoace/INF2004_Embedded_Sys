"""Approved A4 geometry and explicit unlocated MQTT mat proxies."""
import csv
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from sim.webots.pil.s3_sil_barcode import (
    ASSUMPTIONS_PATH, barcode_sheet, load_sil_barcode_profile, mqtt_mat_proxy)
from sim.webots.pil.s3_geometry import (
    MAT_WHITE, MAT_LINE, PAPER_WHITE, PAPER_INK, PolygonPatch, material_at)
from sim.webots.pil.s3_gp28_nonlinear import board_axis_u
from sim.webots.pil.s3_optics import sample_sensor
from sim.webots.pil.s3_referee import fine_sample
from sim.webots.pil.scene import SceneSnapshot
from sim.webots.pil.s3_profiles import DEFAULT_PATH


class SILBarcodeTests(unittest.TestCase):
    def test_a4_large_a_body_quiet_and_left_placement(self):
        sheet = barcode_sheet(sheet_start_mm=500, line_centre_y_mm=157.5,
                              surface_z_mm=10)
        self.assertEqual(sheet.geometry.body_length_mm, 273.5)
        self.assertEqual(len(sheet.geometry.elements_mm), 29)
        self.assertEqual(sheet.sheet_centre_mm, (648.5, 287.5, 10))
        self.assertEqual(sheet.sheet_bounds_mm, (500, 182.5, 797, 392.5))
        self.assertEqual(sheet.ink_bounds_mm, (511.75, 207.5, 785.25, 367.5))
        self.assertEqual(sheet.quiet_bounds_mm, (453.75, 843.25))
        self.assertEqual(sheet.junction_centre_x_mm, 947)
        self.assertEqual(material_at(sheet.geometry.patches, 501, 227.25), PAPER_WHITE)
        self.assertEqual(material_at(sheet.geometry.patches, 512, 227.25), PAPER_INK)
        # GP28 centre is axle150.25+77, not line157.5+77.
        self.assertTrue(sheet.ink_bounds_mm[1] < 227.25 < sheet.ink_bounds_mm[3])
        self.assertGreater(sheet.sheet_bounds_mm[1], 157.5+7.25+4*2.8563079833984375)
        self.assertIn('PRINT_CENTRED_ASSUMED', sheet.assumptions)

    def test_mat_proxy_profile_explicit_family_and_no_frozen_change(self):
        for along in (1, 4, 8):
            p = load_sil_barcode_profile(sigma_along_mm=along)
            self.assertEqual(p.levels_adc[MAT_WHITE], 178)
            self.assertEqual(p.levels_adc[MAT_LINE], 2321)
            self.assertEqual(p.reference_height_mm, 5.5)
            self.assertEqual(p.sigma_x_mm, along)
            self.assertEqual(p.mat_response_power, 1)
            self.assertIn('MAT_RESPONSE_POWER_1_ASSUMED_NOT_FITTED', p.assumptions)
            self.assertIn('MEASURED_MQTT_2026_11_11_UNLOCATED', p.assumptions)
            self.assertNotIn('MAT_LEVELS_PLACEHOLDERS_UNSUPPORTED', p.assumptions)
        with self.assertRaises(ValueError):
            load_sil_barcode_profile(sigma_along_mm=2)
        self.assertEqual(hashlib.sha256(DEFAULT_PATH.read_bytes()).hexdigest(),
                         '481ae444ab4095372793224501f68e1fd1ec317a8e7946340194a16208c36f65')

    def test_mat_paper_edge_is_optical_not_fake_white_or_invalid(self):
        # Infinite white mat below half-plane paper, same plane, layer20 wins.
        patches = (
            PolygonPatch('mat', MAT_WHITE,
                ((-500, -500, 10), (500, -500, 10), (500, 500, 10), (-500, 500, 10)), 0),
            PolygonPatch('paper', PAPER_WHITE,
                ((0, -500, 10), (500, -500, 10), (500, 500, 10), (0, 500, 10)), 20))
        scene = SceneSnapshot.from_patches(patches)
        p = load_sil_barcode_profile(sigma_along_mm=1)
        values = []
        for x in (-30, 0, 30):
            sample = sample_sensor(scene, p, (x, 0, 15.5), axis_u=board_axis_u(0),
                                   quadrature=128, sample_us=0)
            ref = fine_sample(scene, p, (x, 0, 15.5), axis_u=board_axis_u(0))
            self.assertTrue(sample.valid, sample.reason)
            self.assertTrue(ref.valid, ref.reason)
            self.assertAlmostEqual(sample.mean_adc, ref.mean_adc, delta=.1)
            values.append(sample.mean_adc)
        self.assertAlmostEqual(values[0], 178, places=6)
        self.assertAlmostEqual(values[1], 171, places=6)
        self.assertAlmostEqual(values[2], 164, places=6)
        dark_patches = (PolygonPatch('dark-mat', MAT_LINE,
                        patches[0].vertices_mm, 0), patches[1])
        dark_scene = SceneSnapshot.from_patches(dark_patches)
        edge = fine_sample(dark_scene, p, (0, 0, 15.5), axis_u=board_axis_u(0))
        self.assertTrue(edge.valid, edge.reason)
        self.assertAlmostEqual(edge.mean_adc, (2321+164)/2, places=6)
        far = fine_sample(dark_scene, p, (-30, 0, 15.5), axis_u=board_axis_u(0))
        self.assertAlmostEqual(far.mean_adc, 2321, places=6)

    def test_mqtt_method_filters_boot_and_is_robust(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mqtt.csv'
            with path.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('topic', 'boot_id', 'raw_bc'))
                for value in (174, 178, 178, 182, 2310, 2321, 2330):
                    writer.writerow(('robot/state', '3162310911', value))
                writer.writerow(('robot/state', 'other', 4092))
                writer.writerow(('robot/state', '3162310911', ''))
            result = mqtt_mat_proxy(path)
            self.assertEqual(result['low']['median_adc'], 178)
            self.assertEqual(result['high']['median_adc'], 2321)
            self.assertEqual(result['samples'], 7)
            self.assertEqual((result['low']['samples'], result['high']['samples']), (4, 3))
        record = json.loads(ASSUMPTIONS_PATH.read_text())
        self.assertEqual(record['mat_proxy']['samples'], 3233)
        self.assertEqual(record['mat_proxy']['low']['samples'], 2134)
        self.assertEqual(record['mat_proxy']['high']['samples'], 1099)


if __name__ == '__main__':
    unittest.main()
