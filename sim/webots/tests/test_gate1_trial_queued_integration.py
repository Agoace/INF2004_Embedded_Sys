"""C3 integration: trial labels, closure, caller slack and C2b checks."""
import csv
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak
from pil.wire import Type
from test_gate1 import FakeClock, FakePair


class TrialQueuedIntegration(unittest.TestCase):
    def test_full_trial_queue_disarms_and_preserves_labels_and_failure(self):
        from pil.gate1_log import QueuedEventWriter
        class Paused(QueuedEventWriter):
            def __init__(self,path):super().__init__(path,capacity=1,start=False)
            def start(self):
                if self.stopping:super().start()
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            with patch('pil.gate1_log.QueuedEventWriter',Paused):
                s=run_soak(pair,out,clock=clock,session=123,duration_us=1000,
                    transport_only=True,trial_clock_400_sync20=True)
            self.assertEqual(s['verdict'],'FAIL')
            self.assertEqual(s['failure'],'evidence queue full')
            self.assertTrue(pair.closed)
            self.assertTrue(any(f.type==Type.DISARM for f in pair.sent_frames))
            self.assertTrue(s['evidence_logger']['close_ack'])
            self.assertTrue(s['evidence_logger']['failed_submission_retained'])
            self.assertEqual(s['qualification'],'TRIAL / NOT GATE 1 PASS')
            self.assertFalse(s['physical_gate1_pass'])
            status=json.loads((out/'logger-status.json').read_text())
            self.assertEqual(status['qualification'],s['qualification'])

    def test_failed_initial_metadata_submission_still_disarms_and_closes(self):
        from pil.gate1_log import QueuedEventWriter
        class Failed(QueuedEventWriter):
            def __init__(self,path):
                super().__init__(path,start=False)
                self.worker_error='test initial writer failure'
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp:
            with patch('pil.gate1_log.QueuedEventWriter',Failed):
                s=run_soak(pair,Path(tmp)/'run',clock=clock,session=123,duration_us=1000,
                    transport_only=True,trial_clock_400_sync20=True)
            self.assertEqual(s['verdict'],'FAIL')
            self.assertTrue(pair.closed)
            self.assertTrue(any(f.type==Type.DISARM for f in pair.sent_frames))
            self.assertTrue(s['evidence_logger']['failed_submission_retained'])

    def test_trial_writer_closes_and_caller_slack_is_read_before_worker(self):
        caller=threading.get_ident();calls=[]
        def read_slack():
            calls.append(threading.get_ident())
            return {'status':'KNOWN','value_ns':50000,'source':'prctl(PR_GET_TIMERSLACK)'}
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            with patch('pil.gate1.read_timer_slack',side_effect=read_slack):
                s=run_soak(pair,out,clock=clock,session=123,duration_us=40000,
                    transport_only=True,measure_io=True,trial_clock_400_sync20=True)
            self.assertEqual(calls,[caller])
            self.assertNotEqual(s['verdict'],'FAIL',s['failure'])
            self.assertEqual(s['pc_capture_revision'],'C1+C2+C2b+C3')
            self.assertEqual(s['pc_logging'],'bounded-queue')
            self.assertTrue(s['evidence_logger']['close_ack'])
            rows=list(csv.DictReader((out/'events.csv').read_text().splitlines()))
            meta=json.loads(next(r for r in rows if r['event']=='run_metadata')['details_json'])
            self.assertEqual(meta['timer_slack']['value_ns'],50000)
            self.assertEqual(s['timer_slack'],meta['timer_slack'])
            manifest=json.loads((out/'trial-manifest.json').read_text())
            self.assertTrue(manifest['queued_logger'])
            self.assertEqual(manifest['capture_checks'],['loop-entry','post-RX/pre-SYNC','post-SYNC'])
            self.assertTrue((out/'attribution.csv').exists())


if __name__=='__main__':unittest.main()
