"""Combined D-TRIAL+C2: order, no-early rule, cancellation and second check."""
import csv
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak, neutral_payload
from test_gate1 import FakeClock, FakePair


class TrialCaptureFirstTests(unittest.TestCase):
    def test_trial_capture_first_and_no_early_midpoint(self):
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'run'
            result=run_soak(pair,out,clock=clock,duration_us=40000,session=123,
                            transport_only=True,measure_io=True,trial_clock_400_sync20=True)
            self.assertNotEqual(result['verdict'],'FAIL')
            rows=list(csv.DictReader((out/'attribution.csv').read_text().splitlines()))
            loops={}
            for row in rows:loops.setdefault(row['loop'],[]).append(row['kind'])
            captured=[names for names in loops.values()if'capture_enter'in names]
            self.assertTrue(captured)
            for names in captured:
                self.assertLess(names.index('tick_exit'),names.index('capture_enter'))
                self.assertLess(names.index('capture_enter'),names.index('read_enter'))
                self.assertGreaterEqual(names.count('tick_exit'),2)
            import json
            manifest=json.loads((out/'trial-manifest.json').read_text())
            self.assertTrue(manifest['capture_first'])
            self.assertFalse(manifest['queued_logger'])
            self.assertEqual(result['pc_capture_order'],'capture-first')
            self.assertEqual(result['pc_logging'],'synchronous')
            for row in csv.DictReader((out/'events.csv').read_text().splitlines()):
                if row['event']=='source_capture':
                    d=json.loads(row['details_json'])
                    self.assertTrue(d['no_early_midpoint'])
                    self.assertGreaterEqual(d['start_midpoint_error_us'],0)
                    self.assertLessEqual(d['phase_us'],250)

    def run_fault(self,delay):
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        read=pair.read;captured=[]
        def payload(source):
            captured.append(source)
            return neutral_payload(source)
        def rx(maximum):
            if captured:
                if delay:
                    clock.now+=6000
                    return b''
                return b'\x02x\0'
            return read(maximum)
        with tempfile.TemporaryDirectory() as tmp:
            with patch('pil.gate1.neutral_payload',side_effect=payload),patch.object(pair,'read',side_effect=rx):
                summary=run_soak(pair,Path(tmp)/'run',clock=clock,duration_us=40000,
                                 session=123,transport_only=True,trial_clock_400_sync20=True)
        self.assertTrue(captured)
        self.assertEqual(summary['verdict'],'FAIL')
        self.assertFalse(pair.sensors)
        self.assertTrue(pair.closed)
        return summary

    def test_trial_rx_fault_cancels_capture_before_tx(self):
        self.run_fault(False)

    def test_trial_second_check_cancels_source_after_rx_delay(self):
        summary=self.run_fault(True)
        self.assertTrue('lookahead deadline' in summary['failure'] or
                        'lease expired' in summary['failure'])


if __name__=='__main__':
    unittest.main()
