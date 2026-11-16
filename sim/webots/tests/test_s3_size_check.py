"""Host-only reader/optical SIL. No Webots process or device access."""
import tempfile
import unittest
from pathlib import Path

from sim.webots.pil.s3_size_check import Reader, check_case, print_bars


class SizeCheckTests(unittest.TestCase):
    def test_geometry_is_scaled_not_relabelled(self):
        for body in (274, 210, 141):
            bars = print_bars('F', body, False)
            self.assertEqual(len(bars), 15)
            self.assertAlmostEqual(bars[-1][1] - bars[0][0], body)
            expected = tuple((80 + body - (b - 80),
                              80 + body - (a - 80))
                             for a, b in reversed(bars))
            for actual, wanted in zip(print_bars('F', body, True), expected):
                self.assertAlmostEqual(actual[0], wanted[0])
                self.assertAlmostEqual(actual[1], wanted[1])

    def test_real_c_reader_size_and_negatives(self):
        with tempfile.TemporaryDirectory() as directory:
            reader = Reader(Path(directory))
            for body, accepted in ((274, True), (210, True), (141, False)):
                result = check_case(reader, body_mm=body, sigma_mm=0)
                self.assertEqual(result['accepted'], accepted)
                if accepted:
                    self.assertEqual(result['letter'], 'F')
            # The run must reach the final white proof even at low speed.
            self.assertTrue(check_case(reader, body_mm=274, sigma_mm=1,
                                       speed_mm_s=90, noise_adc=10)['accepted'])
            for kind in ('line', 'junction', 'white'):
                self.assertFalse(check_case(reader, kind=kind)['accepted'])
            for sigma in (1, 2.15, 4):
                self.assertFalse(check_case(reader, body_mm=141,
                                            sigma_mm=sigma)['accepted'])

    def test_invalid_geometry_fails_closed(self):
        for value in (-1, 0, float('nan')):
            with self.assertRaises(ValueError):
                print_bars('F', value, False)


if __name__ == '__main__':
    unittest.main()
