import unittest
from pil.sil_recovery import analyze_recovery


def sample(ms,error,norm=390,event=0,fault=0):
    row=[0.]*22;row[0]=ms;row[5]=1;row[9]=norm;row[10]=norm;row[4]=event;row[11]=fault;row[20]=error
    return row


class RecoveryTests(unittest.TestCase):
    def test_settle_hold_overshoot_and_lost_events(self):
        rows=[sample(ms,5 if ms<100 else -2)for ms in range(1,1001)]
        result=analyze_recovery(rows)
        self.assertEqual(result['settle_ms'],100)
        self.assertEqual(result['opposite_side_overshoot_mm'],2)
        self.assertTrue(result['line_acquired']);self.assertEqual(result['lost_events'],0)
        self.assertTrue(result['final_in_band'])

    def test_short_transient_is_not_settling_and_initial_flag_is_not_acquisition(self):
        rows=[sample(ms,5,norm=100,event=5 if ms==301 else 0,fault=ms>=301)for ms in range(1,1001)]
        for row in rows[99:300]:row[20]=1
        result=analyze_recovery(rows)
        self.assertIsNone(result['settle_ms']);self.assertFalse(result['line_acquired'])
        self.assertFalse(result['initial_in_sensor_reach']);self.assertEqual(result['lost_events'],1)
        self.assertTrue(result['stop_latched'])

    def test_long_transient_then_departure_is_not_terminal_settling(self):
        rows=[sample(ms,0 if ms<700 else 5)for ms in range(1,1001)]
        result=analyze_recovery(rows)
        self.assertIsNone(result['settle_ms']);self.assertEqual(result['first_500ms_hold_ms'],1)

    def test_initial_small_error_requires_actual_hold(self):
        result=analyze_recovery([sample(ms,0)for ms in range(1,501)])
        self.assertEqual(result['settle_ms'],1)
        self.assertEqual(result['opposite_side_overshoot_mm'],0)
