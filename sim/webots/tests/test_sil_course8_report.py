import math
import unittest

from pil.sil_course8_report import report, straights


def row(ms, **kw):
    base = dict(sim_ms=ms, yaw_rad=0.0, module=1, searches=0, ja_phase=0,
                error_mm=0.0, tile='t1', under_tile='t1', progress_mm=ms * 0.146,
                desired_left_mm_s=146, desired_right_mm_s=146, tilted=0,
                roll_deg=0.0, right_wheel_lift_mm=0.0, left_wheel_lift_mm=0.0,
                track_dist_mm=0.0)
    base.update(kw)
    return base


class Course8ReportTests(unittest.TestCase):
    def test_search_loose_ramp_crossing(self):
        rows = [row(ms) for ms in range(1, 4001)]
        for ms in range(1000, 1100):
            rows[ms - 1].update(module=2, searches=1, tile='r5c3')
        for ms in range(2000, 2050):
            rows[ms - 1].update(under_tile='r4c0_loose', track_dist_mm=70.0)
        for ms in range(3000, 3200):
            rows[ms - 1].update(tilted=1, roll_deg=-4.8, right_wheel_lift_mm=9.6)
        for ms in range(3500, 3600):
            rows[ms - 1].update(tile='X')
        rep = report(rows, loop_mm=400.0, crossing_tile='X', loose_ids={'r4c0_loose'},
                     wall_seconds=12.0)
        self.assertEqual(rep['laps_completed'], 1)
        self.assertEqual([e['tile'] for e in rep['search_entries']], ['r5c3'])
        self.assertEqual(rep['search_entries'][0]['lap'], 1)
        self.assertEqual(rep['loose_arc']['tiles'], ['r4c0_loose'])
        self.assertEqual(rep['loose_arc']['line_ms_off_track'], 50)
        self.assertTrue(rep['ramp']['right_tyre_on_ramp'])
        self.assertEqual(rep['ramp']['roll_peak_deg'], 4.8)
        self.assertEqual(rep['ramp']['right_wheel_lift_peak_mm'], 9.6)
        self.assertEqual(len(rep['crossing_passes']), 1)
        self.assertTrue(rep['crossing_passes'][0]['straight'])
        self.assertEqual((rep['sim_seconds'], rep['wall_seconds']), (4.0, 12.0))

    def test_straights_feel_definition(self):
        # 2 deg/s wobble at 2 Hz on a straight: deviation about the mean.
        rows = [row(ms, yaw_rad=math.radians(0.5 * math.sin(2 * math.pi * 2 * ms / 1000)))
                for ms in range(1, 10001)]
        s = straights(rows)
        self.assertGreater(s['straight_seconds'], 9)
        self.assertLess(s['deviation_p99'], 7.0)
        self.assertGreater(s['deviation_p99'], 5.0)


if __name__ == '__main__':
    unittest.main()
