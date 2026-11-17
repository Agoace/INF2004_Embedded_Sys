import math
import tempfile
import unittest
from pathlib import Path

from pil.course_builder import Course
from pil.s3_assumed_line import load_boot5209_line_profile, load_assumed_line_mount
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
    def test_kd_and_same_tick_builds(self):
        for kd, same in ((85, False), (30, True)):
            with tempfile.TemporaryDirectory() as tmp:
                record = build_course8_library(tmp, kd=kd, same_tick=same)
                self.assertEqual(record['configuration']['steer'],
                                 [146, 85, kd, 62, 378, 50, 340])
                self.assertEqual(record['same_tick_apply'], same)
                self.assertIn('-DLINE_KD=30' if kd == 30 else '-O2',
                              record['commands'][0])
                self.assertNotIn('app_program/vehicle/line_kd.h' if kd == 85 else '-DX',
                                 ' '.join(record['commands'][0]))
                ctl = SearchController(Path(tmp) / 'libsil_line.so')
                ctl.calibrate([[172, 158, 0]] * 16 + [[2498, 2623, 0]] * 4)
                for ms in range(1, 11):
                    ctl.update(2498, 2623, ms, 0, 0)
                self.assertEqual(ctl.timing()[4], 1 if same else 0)

    def test_unapproved_cases_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                build_course8_library(tmp, kd=40)

    def test_host_crossing_straight_through_no_barcode(self):
        profiles = {n: load_boot5209_line_profile(n) for n in ('GP26', 'GP27')}
        mounts = {n: asdict(load_assumed_line_mount(n)) for n in ('GP26', 'GP27')}
        with tempfile.TemporaryDirectory() as tmp:
            build_course8_library(tmp)
            rows = host_drive(cross_course(), Path(tmp) / 'libsil_line.so', ms=8000,
                              profiles=profiles, mounts=mounts)
        # Starts at x 60 heading east; crossing centre x 787.5.
        self.assertGreater(rows[-1][1], 787.5 + 250)
        self.assertTrue(all(r[4] == 1 for r in rows), 'left LINE (SEARCH?)')
        self.assertEqual({r[5] for r in rows}, {0})
        self.assertEqual({r[6] for r in rows}, {0})      # JA_IDLE
        self.assertLess(max(abs(r[2] - 472.5) for r in rows), 15)
        self.assertLess(max(abs(math.degrees(r[3])) for r in rows), 10)


if __name__ == '__main__':
    unittest.main()
