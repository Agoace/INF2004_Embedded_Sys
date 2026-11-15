"""Explicit SIL assumptions must never replace the measured-profile loader."""
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from sim.webots.pil.s3_assumed_line import (
    ASSUMED_LINE_PATH, load_assumed_line_profile, load_assumed_line_mount)
from sim.webots.pil.s3_profiles import DEFAULT_PATH, load_profile
from sim.webots.pil.s3_fast import FastScene
from sim.webots.pil import s3_geometry as g


class AssumedLineTests(unittest.TestCase):
    def test_profile_is_labelled_and_matches_approved_numbers(self):
        for channel in ('GP26', 'GP27'):
            p = load_assumed_line_profile(channel)
            self.assertEqual(p.name, channel)
            self.assertEqual(p.evidence_status, 'ASSUMED_SIL_STRAIGHT_ONLY')
            self.assertEqual(p.reference_height_mm, 9)
            self.assertAlmostEqual(p.sigma_x_mm, 15.924122728751232)
            self.assertEqual(p.sigma_y_mm, p.sigma_x_mm)
            self.assertIn('FITTED_TO_BOOT_5209_SPIN_RECORDING', p.assumptions)
            self.assertEqual(p.noise_sigma_adc, 0)
            self.assertEqual(p.levels_adc[g.MAT_WHITE], 200)
            self.assertEqual(p.levels_adc[g.MAT_LINE], 3700)
            self.assertEqual(p.mat_response_power, 1)
            self.assertIn('PAPER_LEVELS_SCHEMA_PLACEHOLDERS_NOT_SUPPORTED', p.assumptions)
            with self.assertRaises(TypeError):
                p.levels_adc[g.MAT_WHITE] = 0

    def test_fitted_sigma_reproduces_boot_5209_target_without_width_change(self):
        p = load_assumed_line_profile('GP26')
        sigma = p.sigma_x_mm
        coverage = .5*(math.erf((9-7.25)/(math.sqrt(2)*sigma)) -
                       math.erf((-9-7.25)/(math.sqrt(2)*sigma)))
        self.assertAlmostEqual(coverage, .39, places=12)
        record = json.loads(ASSUMED_LINE_PATH.read_text())['footprint_fit']
        self.assertEqual(record['record'], 'boot-5209 spin recording')
        self.assertEqual((record['line_width_mm'], record['relative_spacing_mm']), (18, 14.5))

    def test_mounts_preserve_14_point_5_spacing_not_29(self):
        c, l = (load_assumed_line_mount(ch) for ch in ('GP26', 'GP27'))
        self.assertEqual((c.forward_mm, l.forward_mm), (119, 119))
        self.assertEqual((c.left_mm, l.left_mm), (0, 14.5))
        self.assertEqual((c.flat_clearance_mm, l.flat_clearance_mm), (9, 9))
        self.assertTrue(c.forward_assumed)
        self.assertTrue(l.height_assumed)
        self.assertEqual(l.spacing_evidence, 'MEASURED_RELATIVE_GP26')

    def test_straight_sampling_deterministic_raw_and_pair_midpoint(self):
        # Line parallel to body x; line centre is body y=+7.25 at target.
        patches = (
            g.PolygonPatch('floor', g.MAT_WHITE,
                ((-500, -500, 10), (500, -500, 10), (500, 500, 10), (-500, 500, 10)), 0),
            g.PolygonPatch('line', g.MAT_LINE,
                ((-500, -1.75, 10), (500, -1.75, 10), (500, 16.25, 10), (-500, 16.25, 10)), 10))
        fast = FastScene.from_patches(patches)
        values = []
        for channel in ('GP26', 'GP27'):
            p, m = load_assumed_line_profile(channel), load_assumed_line_mount(channel)
            a = fast.sample(p, (m.forward_mm, m.left_mm, 19), sample_us=0)
            b = fast.sample(p, (m.forward_mm, m.left_mm, 19), sample_us=999999)
            self.assertTrue(a.valid, a.reason)
            self.assertEqual(a.adc, b.adc)
            values.append(a.adc)
        self.assertEqual(values[0], values[1])
        for adc in values:
            self.assertTrue(.38 <= (adc-200)/3500 <= .40)
        # Across-strip plain-line sum must stay within the measured upper bound.
        p = load_assumed_line_profile('GP26')
        sums = []
        for shift in range(-100, 101):
            adc = [fast.sample(p, (0, y+shift, 19)).adc for y in (0, 14.5)]
            sums.append(sum((a-200)/3500 for a in adc))
        self.assertLessEqual(max(sums), 1.02)
        self.assertAlmostEqual(max(sums), .78, delta=.001)

    def test_frozen_file_and_null_loaders_unchanged(self):
        self.assertNotEqual(ASSUMED_LINE_PATH, DEFAULT_PATH)
        self.assertEqual(hashlib.sha256(DEFAULT_PATH.read_bytes()).hexdigest(),
                         '481ae444ab4095372793224501f68e1fd1ec317a8e7946340194a16208c36f65')
        for channel in ('GP26', 'GP27'):
            with self.assertRaisesRegex(ValueError, 'UNMEASURED'):
                load_profile(channel)

    def test_wrong_channel_or_missing_assumption_label_rejected(self):
        for channel in ('GP28', 'C', 'L'):
            with self.assertRaises(ValueError):
                load_assumed_line_profile(channel)
        data = json.loads(ASSUMED_LINE_PATH.read_text())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'bad.json'
            data['classification'] = 'CALIBRATED'
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_assumed_line_profile('GP26', path)


if __name__ == '__main__':
    unittest.main()
