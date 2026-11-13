"""Frozen tracked-capture regression; known physical-model failures stay visible."""
import dataclasses
import json
from pathlib import Path
import tempfile
import unittest
from sim.webots.pil import s3_replay as r
from sim.webots.pil.s3_profiles import DEFAULT_PATH, load_profile

ROOT = Path(__file__).resolve().parents[3]


class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = json.loads(DEFAULT_PATH.read_text())
        cls.actual = {}
        cls.data = {}
        for axis, variant in cls.bundle['replay_variants'].items():
            fit = r.Fit(**variant['fit'])
            cls.data[axis] = {}
            cls.actual[axis] = {}
            for kind in ('training', 'holdout'):
                entry = variant[kind]
                capture = r.read_capture(ROOT/entry['path'])
                rows, meta = r.observations(capture, axis)
                cls.data[axis][kind] = rows
                offset = entry['metrics']['registration_offset_mm']
                cls.actual[axis][kind] = r.evaluate(rows, fit, offset)

    def test_frozen_numbers_hashes_and_coverage_are_reproducible(self):
        for axis, variant in self.bundle['replay_variants'].items():
            self.assertNotEqual(variant['training']['sha256'], variant['holdout']['sha256'])
            for kind in ('training', 'holdout'):
                entry = variant[kind]
                self.assertFalse(Path(entry['path']).is_absolute())
                capture = r.read_capture(ROOT/entry['path'])
                self.assertEqual(capture.sha256, entry['sha256'])
                _, meta = r.observations(capture, axis)
                for key, value in meta.items():
                    self.assertEqual(value, entry['coverage'][key])
                for key, expected in entry['metrics'].items():
                    actual = self.actual[axis][kind][key]
                    if isinstance(expected, float):
                        self.assertAlmostEqual(actual, expected, places=6, msg=key)
                    else:
                        self.assertEqual(actual, expected, key)
        self.assertTrue(self.bundle['replay_variants']['mean_wheel']['fit']['search_bound_hit'])

    def test_noise_uses_only_tracked_training_white(self):
        noise = self.bundle['noise_fit']
        self.assertEqual(noise['sample_count'], 201)
        self.assertEqual(noise['median_adc'], 164)
        self.assertEqual(noise['MAD_adc'], 3)
        self.assertAlmostEqual(noise['robust_sigma_adc'], 4.4478)
        self.assertEqual(noise['sha256'], self.bundle['replay_variants']['sensor_x']['training']['sha256'])
        self.assertEqual(load_profile().noise_sigma_adc, 4.4478)

    def test_unmeasured_channels_fail_instead_of_reusing_gp28(self):
        for channel in ('GP26', 'GP27'):
            with self.assertRaisesRegex(ValueError, 'UNMEASURED'):
                load_profile(channel)
        self.assertEqual(load_profile().evidence_status, 'EXPLORATORY_EXPECTED_FAIL')
        with self.assertRaises(ValueError):
            load_profile('GP99')

    def test_holdout_registration_does_not_change_response_or_geometry(self):
        v = self.bundle['replay_variants']['sensor_x']
        fit = r.Fit(**v['fit'])
        before = dataclasses.asdict(fit)
        offset = r.register_only(self.data['sensor_x']['holdout'], fit)
        self.assertAlmostEqual(offset, v['holdout']['metrics']['registration_offset_mm'])
        self.assertEqual(before, dataclasses.asdict(fit))
        self.assertEqual(self.bundle['fixed_geometry_mm'],
                         {'narrow': 5.8, 'wide': 17.5, 'body': 273.5})

    @unittest.expectedFailure
    def test_training_sensor_x_edge_check_known_failure(self):
        """FAIL: nominal pose/no-slip optical model misses the fixed1.5mm check."""
        self.assertLessEqual(self.actual['sensor_x']['training']['edge_p95_error_mm'], 1.5)

    @unittest.expectedFailure
    def test_holdout_sensor_x_edge_check_known_failure(self):
        """FAIL:1.87mm>1.5mm; no surveyed sheet, mounting or height evidence."""
        self.assertLessEqual(self.actual['sensor_x']['holdout']['edge_p95_error_mm'], 1.5)

    @unittest.expectedFailure
    def test_holdout_mean_wheel_check_known_failure(self):
        """FAIL: alternate mean-wheel fit does not generalise to held-out trace."""
        self.assertEqual(self.actual['mean_wheel']['holdout']['check'], 'PASS')

    def test_parser_wraps_count_and_time_but_never_extrapolates(self):
        text = ('BC_ADC,index,us,raw,flags,delta_us\n'
                'BC_ADC,0,10,160,0,0\nBC_ADC,1,20,161,0,10\n'
                'BC_ADC,2,40,160,0,20\n'
                'BC_MOTOR,index,us,left,right\n'
                'BC_MOTOR,0,4294967290,2147483646,2147483646\n'
                'BC_MOTOR,1,30,-2147483646,-2147483646\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'wrap.csv'
            path.write_text(text)
            capture = r.read_capture(path)
            rows, meta = r.observations(capture)
            self.assertEqual(meta['unbracketed_adc'], 1)
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(rows[1][0], 4*(10/36)/r.COUNTS_PER_MM, places=6)
            self.assertEqual(capture.motor[1][1]-capture.motor[0][1], 4)
            path.write_text(text.replace('BC_ADC,1,20', 'BC_ADC,1,9'))
            with self.assertRaisesRegex(ValueError, 'time'):
                r.read_capture(path)

    def test_optimizer_recovers_a_synthetic_waveform_without_scale_fit(self):
        truth = r.Fit('F', 164, 1500, 2.125, 6, 65, False)
        positions = [(i*1.25, 0, i*1000) for i in range(345)]
        synthetic = r.predict(positions, truth)
        fitted = r.fit_capture(synthetic)
        result = r.evaluate(synthetic, fitted)
        self.assertLess(result['rmse_adc'], 15)
        self.assertLess(abs(fitted.offset_mm-truth.offset_mm), .1)
        self.assertEqual(result['model_edge_count'], 30)

    def test_invalid_data_and_axis_fail_explicitly(self):
        with self.assertRaises(ValueError):
            r.fit_capture([(i, 1000, i) for i in range(200)])
        with self.assertRaises(ValueError):
            r.observations(r.Capture('fake', '', (), ()), 'hidden_corrected_distance')
        with self.assertRaises(ValueError):
            r.profile_bundle({}, {}, [])


if __name__ == '__main__':
    unittest.main()
