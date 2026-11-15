"""Joint endpoint/spin fit, not physical calibration acceptance."""
import csv
import hashlib
import json
import math
from pathlib import Path
import unittest

from sim.webots.pil.s3_assumed_line import (
    ASSUMED_LINE_PATH, BOOT_5209_LINE_PATH, load_assumed_line_profile,
    load_boot5209_line_profile)
from sim.webots.pil.s3_fast import FastScene
from sim.webots.pil.s3_spin_fit import ENDPOINTS, coverage, normalized, parameters
from sim.webots.pil import s3_geometry as g

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT/'tests/fixtures/spin_raw_boot5209.csv'


class Boot5209FitTests(unittest.TestCase):
    def test_joint_fit_midpoint_peaks_floor_and_pair_bound(self):
        scene = FastScene.from_patches((
            g.PolygonPatch('floor', g.MAT_WHITE,
                ((-500, -500, 0), (500, -500, 0), (500, 500, 0), (-500, 500, 0)), 0),
            g.PolygonPatch('line', g.MAT_LINE,
                ((-500, -9, 0), (500, -9, 0), (500, 9, 0), (-500, 9, 0)), 10)))
        for channel, (floor, peak) in ENDPOINTS.items():
            p = load_boot5209_line_profile(channel)
            self.assertIn('FITTED_TO_BOOT_5209_SPIN_PLUS_ENDPOINTS', p.assumptions)
            self.assertEqual(p.reference_height_mm, 8)
            values = [scene.sample(p, (0, y, 8)) for y in (0, 7.25, 100)]
            self.assertTrue(all(v.valid for v in values))
            self.assertAlmostEqual(values[0].mean_adc, peak, delta=1)
            self.assertAlmostEqual(values[2].mean_adc, floor, delta=.01)
            self.assertAlmostEqual((values[1].mean_adc-floor)/(peak-floor), .39,
                                   delta=.001)
            # Sweep every0.1mm with real runtime evaluation and ADC rounding.
            sums = []
            for i in range(-1000, 1001):
                samples = [scene.sample(p, (0, i/10+y, 8)) for y in (0, 14.5)]
                self.assertTrue(all(s.valid for s in samples))
                sums.append(sum((s.adc-floor)/(peak-floor) for s in samples))
            self.assertLessEqual(max(sums), 1.02)

    def test_all_rows_residuals_reproduce_record_and_source_hash(self):
        data = json.loads(BOOT_5209_LINE_PATH.read_text())['joint_fit']
        self.assertEqual(hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                         data['source_sha256'])
        with SOURCE.open() as source:
            rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 1634)
        sigma = data['sigma_mm']
        c0, power = parameters(sigma)
        self.assertAlmostEqual(power, data['power'])
        residuals = {'GP27': [], 'GP26': []}
        low, high = data['sweep_end_indices']
        for i, row in enumerate(rows):
            sweep = 0 if i <= low else 1 if i <= high else 2
            k, zero = data['pose_parameters'][2*sweep:2*sweep+2]
            angle = k*(int(row['count'])-zero)
            dc = 119*math.sin(angle)
            for ch, offset, col in [('GP27', dc-14.5*math.cos(angle), 'L'),
                                    ('GP26', dc, 'C')]:
                floor, peak = ENDPOINTS[ch]
                predicted = floor+(peak-floor)*normalized(offset, sigma, c0, power)
                residuals[ch].append(predicted-int(row[col]))
        for ch, errors in residuals.items():
            expected = data['residuals'][ch]
            self.assertAlmostEqual(math.sqrt(sum(e*e for e in errors)/len(errors)),
                                   expected['rmse_adc'], places=6)
            self.assertAlmostEqual(max(abs(e) for e in errors),
                                   expected['max_abs_adc'], places=6)
            # Retain large residuals: a good midpoint is not a full-spin pass.
            self.assertGreater(expected['max_abs_adc'], 1000)

    def test_old_profile_remains_separate_and_unchanged(self):
        self.assertNotEqual(ASSUMED_LINE_PATH, BOOT_5209_LINE_PATH)
        old = load_assumed_line_profile('GP26')
        self.assertEqual(old.mat_response_power, 1)
        self.assertEqual(old.sigma_x_mm, 15.924122728751232)
        self.assertEqual(old.levels_adc[g.MAT_LINE], 3700)


if __name__ == '__main__':
    unittest.main()
