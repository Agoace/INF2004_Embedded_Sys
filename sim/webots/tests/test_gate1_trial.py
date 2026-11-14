"""Explicit operator D-TRIAL; never changes the default Gate1 qualification."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pil.clock import ClockBounds, ClockError, Timeline
from pil.gate1 import run_soak, neutral_payload
from pil.gate1_diagnostics import load_pc_evidence, evaluate, decode_dump
from test_gate1 import FakeClock, FakePair
import test_gate1_attribution as attribution


class TrialTests(unittest.TestCase):
    def test_clock_default250_and_explicit_trial400_are_separate(self):
        normal=ClockBounds()
        trial=ClockBounds(trial_clock_400_sync20=True)
        self.assertEqual(normal.MAX_HALFWIDTH_US,250)
        self.assertEqual(ClockBounds.MAX_HALFWIDTH_US,250)
        self.assertEqual(trial.MAX_HALFWIDTH_US,400)
        self.assertEqual(trial.DRIFT_PPM,100)

    def test_every_trial_capture_caller_must_reject_early_midpoint(self):
        host=attribution.AttributionTests().phase_host(995999,995999)
        host.clock.trial_clock_400_sync20=True
        host.clock.MAX_HALFWIDTH_US=400
        with self.assertRaisesRegex(ClockError,'TRIAL.*midpoint'):
            host.timeline.validate_capture(0,10000,host.clock)
        from pil.transport import TransportFault
        with self.assertRaisesRegex(TransportFault,'TRIAL.*midpoint'):
            host.sensor(neutral_payload(0),0,10000)
        self.assertFalse(host.tx)
        self.assertEqual(host.next_sensor,0)

    def test_trial_artifacts_and_actual_sync_cap_and_loader(self):
        clock=FakeClock(); pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'run'
            summary=run_soak(pair,out,clock=clock,duration_us=80000,session=123,
                             transport_only=True,measure_io=True,trial_clock_400_sync20=True)
            self.assertNotEqual(summary['verdict'],'FAIL')
            self.assertNotEqual(summary['verdict'],'PASS')
            self.assertEqual(summary['candidate_bounds_us']['clock_halfwidth'],400)
            self.assertEqual(summary['candidate_bounds_us']['source_phase'],250)
            self.assertEqual(summary['sync_max_period_us'],20000)
            for name in ('summary.json','events.csv','attribution.csv','attribution.json','trial-manifest.json'):
                text=(out/name).read_text()
                self.assertIn('TRIAL',text,name)
                self.assertIn('NOT GATE 1 PASS',text,name)
            from pil.wire import Type
            sent=[int.from_bytes(f.payload,'little') for f in pair.sent_frames if f.type==Type.SYNC_REQ]
            self.assertGreater(len(sent),10)
            self.assertLessEqual(max(b-a for a,b in zip(sent,sent[1:])),20250)
            pc=load_pc_evidence(out)
            self.assertTrue(pc['source_limits_verified'])
            fixture=Path(__file__).parent/'fixtures/pil_gate1_rx1_clean.hex.txt'
            report=evaluate(decode_dump(bytes.fromhex(fixture.read_text())),pc)
            self.assertTrue(report['trial_clock_400_sync20'])
            self.assertIn('NOT GATE 1 PASS',report['qualification'])
            self.assertNotEqual(report['verdict'],'PASS')
            self.assertFalse(report['transport_subset_qualified'])
            from pil.gate1_measurement import correlate
            timing=correlate(out)
            self.assertEqual(timing['qualification'],'TRIAL / NOT GATE 1 PASS')
            self.assertEqual(timing['clock_halfwidth_limit_us'],400)
            self.assertFalse(timing['physical_gate1_pass'])

    def test_aged300_rejected_by_default_but_allowed_only_in_trial(self):
        host=attribution.AttributionTests().phase_host(995700,996300)
        with self.assertRaises(ClockError):
            host.clock.pico_range(10000)
        host.clock.trial_clock_400_sync20=True
        host.clock.MAX_HALFWIDTH_US=400
        phase,half=host.timeline.validate_capture(0,10000,host.clock)
        self.assertEqual(half,300)
        self.assertLessEqual(phase,250)

    def test_trial_qualified_report_never_qualifies_standard_subset(self):
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'run'
            run_soak(pair,out,clock=clock,duration_us=1000,session=123,
                     transport_only=True,trial_clock_400_sync20=True)
            pc=load_pc_evidence(out)
            self.assertTrue(pc['source_limits_verified'])
            rows=list(csv.DictReader((out/'events.csv').read_text().splitlines()))
            for row in rows:
                if row['event']=='source_capture':
                    d=json.loads(row['details_json']);d['start_midpoint_error_us']=-1
                    row['details_json']=json.dumps(d);break
            with (out/'events.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,('event','pc_us','details_json'));writer.writeheader();writer.writerows(rows)
            self.assertFalse(load_pc_evidence(out)['source_limits_verified'])

    def test_completed_trial_has_no_pass_verdict(self):
        from pil.gate1 import Recorder
        from pil.transport import HostTransport
        from pil.gate1_trial import labels
        with tempfile.TemporaryDirectory() as tmp:
            recorder=Recorder(Path(tmp)/'run')
            recorder.trial_clock_400_sync20=True
            recorder.stopped_status_observed=True
            recorder.record('run_metadata',0,**labels(True))
            summary=recorder.finish(HostTransport(123),None,600_000_000)
            self.assertEqual(summary['verdict'],'TRIAL_COMPLETE')
            self.assertFalse(summary['physical_gate1_pass'])
            self.assertEqual(summary['qualification'],'TRIAL / NOT GATE 1 PASS')

    def test_trial_rejection_has_standalone_trial_label(self):
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        def late(source):
            clock.now+=251
            return neutral_payload(source)
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'run'
            with patch('pil.gate1.neutral_payload',side_effect=late):
                summary=run_soak(pair,out,clock=clock,duration_us=1000,session=123,
                                transport_only=True,trial_clock_400_sync20=True)
            self.assertEqual(summary['verdict'],'FAIL')
            self.assertIn('TRIAL',(out/'capture-rejected.json').read_text())
            self.assertIn('NOT GATE 1 PASS',(out/'capture-rejected.json').read_text())


if __name__=='__main__':
    unittest.main()
