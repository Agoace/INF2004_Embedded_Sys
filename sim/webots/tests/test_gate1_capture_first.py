"""C2: capture at the original target before unrelated work; RX can cancel TX."""
import csv
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak, neutral_payload
from test_gate1 import FakeClock, FakePair


class CaptureFirstTests(unittest.TestCase):
    def test_capture_precedes_read_in_same_loop(self):
        clock = FakeClock()
        pair = FakePair(clock)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            result = run_soak(pair, out, clock=clock, duration_us=40000,
                              session=123, measure_io=True)
            self.assertEqual(result['verdict'], 'INCOMPLETE')
            rows = list(csv.DictReader((out/'attribution.csv').read_text().splitlines()))
            loops = {}
            for row in rows:
                loops.setdefault(row['loop'], []).append(row['kind'])
            captured = [v for v in loops.values() if 'capture_enter' in v]
            self.assertTrue(captured)
            for names in captured:
                self.assertLess(names.index('tick_exit'), names.index('capture_enter'))
                self.assertLess(names.index('capture_enter'), names.index('read_enter'))

    def test_rx_fault_cancels_already_captured_sensor_before_transmit(self):
        clock = FakeClock()
        pair = FakePair(clock)
        original_read = pair.read
        capture = []
        def payload(source):
            capture.append(source)
            return neutral_payload(source)
        def read(maximum):
            if capture:
                return b'\x02x\0'
            return original_read(maximum)
        with tempfile.TemporaryDirectory() as tmp:
            with patch('pil.gate1.neutral_payload', side_effect=payload), patch.object(pair, 'read', side_effect=read):
                result = run_soak(pair, Path(tmp)/'run', clock=clock,
                                  duration_us=40000, session=123)
            self.assertEqual(result['verdict'], 'FAIL')
            self.assertTrue(capture)
            self.assertFalse(pair.sensors)
            self.assertTrue(pair.closed)


if __name__ == '__main__':
    unittest.main()
