import math
import tempfile
import unittest
from pathlib import Path

from pil.course_builder import Course
from pil.s3_assumed_line import load_boot5209_line_profile, load_assumed_line_mount
import pil.sil_course8 as s8
from pil.sil_course8 import build_course8_library, host_drive
from pil.sil_search import SearchController
from dataclasses import asdict


def cross_course():
    """E-W line over five tiles; the middle tile is a 4-way crossing."""
    tiles = [dict(id=f'c{c}', column=c, row=1, kind='CROSS' if c == 2 else 'STRAIGHT',
                  rotation_deg=0 if c == 2 else 90) for c in range(5)]
    tiles += [dict(id=f'n{r}', column=2, row=r, kind='STRAIGHT', rotation_deg=0)
              for r in (0, 2)]
    return Course.from_dict(dict(
        schema=1, name='course8_cross_check', provenance=dict(assumed=True, source='test'),
        origin_mm=[0, 0], start=dict(position_mm=[60.0, 472.5], yaw_deg=0),
        allow_open_ports=True, texture_mm_per_pixel=2, tiles=tiles, boxes=[],
        humps=[], barcodes=[]))


class Course8BuildTests(unittest.TestCase):
    def test_cases_demo_default_and_old_values(self):
        # Staging 7017580: demo default (7f78002) + IMU port; school image.
        self.assertEqual(s8.FIRMWARE_REVISION, '70175801256d3d5e23257c872884cf6a48bb5952')
        self.assertEqual(s8.DEFAULT_CASE, 'demo')
        self.assertEqual(s8.CASES, {'demo': (120, 30, True),
                                    'p7': (146, 85, False),
                                    'kd30st': (146, 30, True)})

    def test_case_builds(self):
        want = {'demo': [], 'p7': ['-DLINE_KD=85', 'line_kd.h', '-DLINE_BASE=146',
                                   'line_p_near.h'],
                'kd30st': ['-DLINE_BASE=146', 'line_p_near.h']}
        for case, (base, kd, same) in s8.CASES.items():
            with tempfile.TemporaryDirectory() as tmp:
                record = build_course8_library(tmp, case=case)
                self.assertEqual(record['configuration']['steer'],
                                 [base, 85, kd, 62, 378, 50, 340])
                self.assertEqual((record['case'], record['base'], record['kd'],
                                  record['same_tick_apply']), (case, base, kd, same))
                adapter = ' '.join(record['commands'][0])
                for flag in want[case]:
                    self.assertIn(flag, adapter)
                for flag in {'line_kd.h', 'line_p_near.h', '-DLINE_'} - set(want[case]):
                    if not any(flag in w for w in want[case]):
                        self.assertNotIn(flag, adapter)
                # Only the adapter TU gets the override flags.
                for cmd in record['commands'][1:]:
                    self.assertNotIn('line_kd.h', ' '.join(cmd))
                ctl = SearchController(Path(tmp) / 'libsil_line.so')
                ctl.calibrate([[172, 158, 0]] * 16 + [[2498, 2623, 0]] * 4)
                for ms in range(1, 11):
                    ctl.update(2498, 2623, ms, 0, 0)
                self.assertEqual(ctl.timing()[4], 1 if same else 0)

    def test_default_build_is_demo(self):
        with tempfile.TemporaryDirectory() as tmp:
            record = build_course8_library(tmp)
            self.assertEqual(record['case'], 'demo')
            self.assertEqual(record['configuration']['steer'][:3], [120, 85, 30])

    def test_unapproved_cases_rejected(self):
        for bad in ('default', 'kd40', ''):
            with tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(ValueError):
                    build_course8_library(tmp, case=bad)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(TypeError):
                build_course8_library(tmp, kd=30)

    def test_host_crossing_straight_through_no_barcode(self):
        profiles = {n: load_boot5209_line_profile(n) for n in ('GP26', 'GP27')}
        mounts = {n: asdict(load_assumed_line_mount(n)) for n in ('GP26', 'GP27')}
        # Same distance for every case: 8 s at base 146 (the old check),
        # scaled for slower bases (9733 ms at 120). The course ends after
        # about 10 s at 146.
        for case, (base, _, _) in s8.CASES.items():
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                build_course8_library(tmp, case=case)
                rows = host_drive(cross_course(), Path(tmp) / 'libsil_line.so',
                                  ms=8000 * 146 // base, profiles=profiles,
                                  mounts=mounts)
                # Starts at x 60 heading east; crossing centre x 787.5.
                self.assertGreater(rows[-1][1], 787.5 + 250)
                self.assertTrue(all(r[4] == 1 for r in rows), 'left LINE (SEARCH?)')
                self.assertEqual({r[5] for r in rows}, {0})
                self.assertEqual({r[6] for r in rows}, {0})      # JA_IDLE
                self.assertLess(max(abs(r[2] - 472.5) for r in rows), 15)
                self.assertLess(max(abs(math.degrees(r[3])) for r in rows), 10)

if __name__ == '__main__':
    unittest.main()
