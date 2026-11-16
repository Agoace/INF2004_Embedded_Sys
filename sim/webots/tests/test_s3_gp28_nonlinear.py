"""Offline GP28 model; no invented along-bar calibration or F1 support."""
import hashlib
import json
import math
from pathlib import Path
import unittest

from sim.webots.pil.s3_gp28_nonlinear import (
    PROFILE_PATH, load_gp28_profile, board_axis_u, projected_sigma,
    capture_rows, predict_rows, evaluate_rows)
from sim.webots.pil.s3_profiles import DEFAULT_PATH, load_profile
from sim.webots.pil.s3_replay import read_capture

ROOT = Path(__file__).resolve().parents[3]


class GP28NonlinearTests(unittest.TestCase):
    def test_no_silent_along_bar_assumption(self):
        with self.assertRaisesRegex(ValueError, 'along-bar'):
            load_gp28_profile()
        p = load_gp28_profile(sigma_along_mm=4)
        self.assertEqual(p.reference_height_mm, 5.5)
        self.assertIn('ALONG_BAR_SIGMA_ASSUMED_NOT_IDENTIFIED', p.assumptions)
        self.assertGreater(p.paper_response_power, 1)
        self.assertEqual(p.sigma_x_mm, 4)
        with self.assertRaises(ValueError):
            load_gp28_profile(sigma_along_mm=3)

    def test_board_rotation_is_90_degrees_and_projection_swaps_axes(self):
        self.assertEqual(board_axis_u(0), (0., 1., 0.))
        self.assertAlmostEqual(projected_sigma(0, 8, 2), 2)
        self.assertAlmostEqual(projected_sigma(math.pi/2, 8, 2), 8)
        u = board_axis_u(math.pi/2)
        self.assertAlmostEqual(u[0], -1)
        self.assertAlmostEqual(u[1], 0, places=12)

    def test_train_holdout_metrics_and_source_hashes_reproduce(self):
        data = json.loads(PROFILE_PATH.read_text())
        for axis, report in data['variants'].items():
            for key, result in report['families'].items():
                for split, file in [('train', 'barcode_430_large_fwd.csv'),
                                    ('holdout', 'barcode_large_f_forward.csv')]:
                    capture = read_capture(ROOT/'tests/fixtures'/file)
                    self.assertEqual(capture.sha256, report[split+'_source_sha256'])
                    rows, meta = capture_rows(capture, axis)
                    metrics = evaluate_rows(rows, result['fit'], float(key),
                                            result[split]['registration_offset_mm'])
                    self.assertAlmostEqual(metrics['rmse_adc'],
                                           result[split]['rmse_adc'], places=6)
                    self.assertEqual(metrics['check'], result[split]['check'])
                    self.assertEqual(meta['input_adc'], len(capture.adc))
                    self.assertEqual(len(predict_rows(rows, result['fit'], float(key))),
                                     len(rows))
        self.assertTrue(all(r['holdout']['check'] == 'FAIL'
                            for v in data['variants'].values()
                            for r in v['families'].values()))

    def test_photo_sensitivity_is_separate_not_a_width_refit(self):
        data = json.loads(PROFILE_PATH.read_text())
        self.assertEqual(data['geometry']['narrow_mm'], 5.8)
        self.assertEqual(data['geometry']['wide_mm'], 17.5)
        for variant in data['variants'].values():
            for sensitivity in variant['photo_dimension_sensitivity']:
                self.assertTrue(272 <= sensitivity['body_mm'] <= 284)
                self.assertEqual(sensitivity['method'],
                                 'fixed nominal optical parameters, translation only')
                self.assertTrue(all(r['check'] == 'FAIL'
                                    for r in sensitivity['results'].values()))

    def test_fast_anisotropic_runtime_remains_unsupported(self):
        from sim.webots.pil.s3_fast import FastScene
        from sim.webots.pil.s3_geometry import PolygonPatch, PAPER_WHITE
        scene = FastScene.from_patches((PolygonPatch('paper', PAPER_WHITE,
            ((-500, -500, 0), (500, -500, 0), (500, 500, 0), (-500, 500, 0)), 0),))
        sample = scene.sample(load_gp28_profile(sigma_along_mm=4), (0, 0, 5.5),
                              axis_u=board_axis_u(0))
        self.assertFalse(sample.valid)
        self.assertEqual(sample.reason, 'UNSUPPORTED_ANISOTROPIC')

    def test_frozen_json_and_normal_loader_unchanged(self):
        self.assertEqual(hashlib.sha256(DEFAULT_PATH.read_bytes()).hexdigest(),
                         '481ae444ab4095372793224501f68e1fd1ec317a8e7946340194a16208c36f65')
        self.assertNotEqual(PROFILE_PATH, DEFAULT_PATH)
        self.assertEqual(load_profile('GP28').sigma_x_mm, 2.125)


if __name__ == '__main__':
    unittest.main()
