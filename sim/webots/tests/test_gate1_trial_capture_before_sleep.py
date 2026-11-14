"""C2c: post-TX/flush capture and fresh check before normal transmit."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak
from pil.transport import HostTransport
from pil.wire import Type
from test_gate1 import FakeClock, FakePair
import test_gate1_trial_capture_opportunities as opportunities


class TrialCaptureBeforeSleep(unittest.TestCase):
    def test_target_crossed_during_sync_tx_captured_after_flush_before_sleep(self):
        opportunities.TrialCaptureOpportunities().exercise('tx')

    def test_next_loop_rx_fault_cancels_pre_sleep_capture(self):
        opportunities.TrialCaptureOpportunities().exercise('txfault')

    def test_metadata_reads_switch_interval_and_timer_slack_without_setting(self):
        import csv,json
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            with patch('pil.gate1.sys.getswitchinterval',return_value=.0032)as get,patch('pil.gate1.sys.setswitchinterval')as setter,patch('pil.gate1.read_timer_slack',return_value={'status':'KNOWN','value_ns':21000}):
                s=run_soak(pair,out,clock=clock,session=123,duration_us=1000,
                    transport_only=True,trial_clock_400_sync20=True)
            get.assert_called_once_with();setter.assert_not_called()
            meta=json.loads(next(r for r in csv.DictReader((out/'events.csv').read_text().splitlines())if r['event']=='run_metadata')['details_json'])
            self.assertEqual(s['interpreter_switch_interval_s'],.0032)
            self.assertEqual(meta['interpreter_switch_interval_s'],.0032)
            self.assertEqual(meta['timer_slack']['value_ns'],21000)
            self.assertEqual(s['timer_slack'],meta['timer_slack'])

    def test_work_before_normal_tx_cannot_transmit_expired_sensor(self):
        clock=FakeClock();pair=FakePair(clock,capabilities=4)
        original=HostTransport.peek_tx;delayed=[]
        def peek(h,*a,**kw):
            data=original(h,*a,**kw)
            if h.tx and h.tx[0][0].type==Type.SENSOR and not delayed:
                delayed.append(h.tx[0][0].sequence)
                clock.now+=6000
            return data
        with tempfile.TemporaryDirectory()as tmp:
            with patch.object(HostTransport,'peek_tx',peek):
                s=run_soak(pair,Path(tmp)/'run',clock=clock,session=123,duration_us=40000,
                    transport_only=True,trial_clock_400_sync20=True)
            self.assertTrue(delayed)
            self.assertEqual(s['verdict'],'FAIL')
            self.assertNotIn(delayed[0],pair.sensors)
            self.assertTrue(pair.closed)
            self.assertTrue(any(f.type==Type.DISARM for f in pair.sent_frames))


if __name__=='__main__':unittest.main()
