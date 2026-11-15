import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from pil.clock import Interval,ClockError,Timeline
from pil.transport import HostTransport,TransportFault
from pil.gate1 import neutral_payload,run_soak,Recorder
from test_gate1 import FakeClock,FakePair

class LateToleranceTests(unittest.TestCase):
    def host(self,enabled=True):
        h=HostTransport(123,transport_only=True,trial_clock_400_sync20=True,trial_tolerate_late=enabled)
        class Clock:
            trial_clock_400_sync20=True
            MAX_HALFWIDTH_US=400
            updated_pc_us=10000
            def pico_range(self,t):return Interval(t+986000-400,t+986000+400)
        h.clock=Clock();h.timeline=Timeline(1000000);h.state='ARMED'
        return h

    def sensor(self,h,slot,phase=1):
        h.sensor(neutral_payload(slot*1000),slot*1000,10000+slot*1000+phase-1)
        h.tx.clear();h.drain_events()

    def test_three_allowed_and_oldest_expires_at_exactly1000(self):
        h=self.host()
        for n in range(1001):self.sensor(h,n,600 if n in(0,2,4,1000)else 1)
        self.assertEqual(len(h.tolerated_captures),4)
        self.assertEqual(h.tolerated_captures[0]['phase_us'],600)
        self.assertEqual(h.tolerated_captures[0]['end_upper_pico_us']+1,997000)

    def test_fourth_in_window_and_consecutive_are_fatal(self):
        for prior,last in(((0,2,4),999),((998,1000,1002),1004),((0,),1),((0,2),3)):
            h=self.host()
            for n in range(last):self.sensor(h,n,300 if n in prior else 1)
            with self.assertRaises(TransportFault):self.sensor(h,last,300)
            self.assertEqual(h.state,'FAULT');self.assertFalse(h.tx)

    def test_default_cap600_and_noearly_remain_fatal(self):
        for enabled,phase in((False,251),(True,601),(True,0)):
            h=self.host(enabled)
            with self.assertRaises(TransportFault):self.sensor(h,0,phase)
            self.assertFalse(h.tx)

    def test_half_microsecond_cap_and_expired_end_clock(self):
        h=self.host();original=h.clock.pico_range
        h.clock.pico_range=lambda t:Interval(original(t).lower+1,original(t).upper)
        with self.assertRaises(TransportFault):self.sensor(h,0,600)
        h=self.host();original=h.clock.pico_range
        def expired(t):
            if t>10000:raise ClockError('clock headroom expired')
            return original(t)
        h.clock.pico_range=expired
        with self.assertRaisesRegex(TransportFault,'headroom'):
            h.sensor(neutral_payload(0),0,10000,capture_end_pc_us=10001)
        self.assertFalse(h.tolerated_captures)

    def test_nominal_boundary_and_expiry_faults_are_unchanged(self):
        h=self.host();self.sensor(h,0,250)
        self.assertFalse(h.tolerated_captures)
        for now,reason in((13001,'lookahead'),(15000,'lease')):
            h=self.host();h.arm_pc_us=10000
            h.sensor(neutral_payload(0),0,10299)
            if reason=='lease':h.tx.clear()
            with self.assertRaisesRegex(TransportFault,reason):h.tick(now)
            self.assertFalse(h.tx)

    def test_tolerance_never_waives_d1_negative_lower_age(self):
        from test_gate1_diagnostics import fixture
        from pil.gate1_diagnostics import decode_dump,evaluate
        from pil.gate1_trial import labels
        raw,pc,_=fixture(stage_age=999)
        pc.update(labels(True,True),tolerated_late_count=1)
        epoch=pc['identity']['epoch_pico_us']
        for r in pc['sources']:
            r['lower_pico_us']=epoch+r['source_us']-4400
        pc['sources'][-1]['upper_pico_us']=epoch+pc['sources'][-1]['source_us']-3000
        result=evaluate(decode_dump(raw),pc)
        self.assertEqual(result['verdict'],'FAIL')
        self.assertLess(result['availability_age_lower_us'],0)
        self.assertFalse(result['transport_subset_qualified'])

    def test_explicit_trial_only(self):
        with self.assertRaises(ValueError):HostTransport(123,trial_tolerate_late=True)
        with tempfile.TemporaryDirectory()as tmp:
            with self.assertRaises(ValueError):run_soak(FakePair(FakeClock()),Path(tmp)/'x',trial_tolerate_late=True)

    def test_full_records_counters_loader_and_never_pass(self):
        from pil.gate1_diagnostics import load_pc_evidence
        c=FakeClock();io=FakePair(c,capabilities=4)
        def late(source):
            if source==0:c.now+=300
            return neutral_payload(source)
        with tempfile.TemporaryDirectory()as tmp,patch('pil.gate1.neutral_payload',side_effect=late):
            out=Path(tmp)/'run'
            s=run_soak(io,out,clock=c,session=123,duration_us=20000,transport_only=True,
                       trial_clock_400_sync20=True,trial_tolerate_late=True)
            self.assertNotEqual(s['verdict'],'FAIL')
            self.assertEqual(s['tolerated_late_count'],1)
            self.assertEqual(s['late_tolerance_policy']['max_late_in_window'],3)
            r=json.loads((out/'tolerated-late-captures.json').read_text())['records'][0]
            for field in('capture_start_pc_us','capture_end_pc_us','start_lower_pico_us',
                'end_upper_pico_us','last_sync_pc_us','sequence','excess_us'):
                self.assertIn(field,r)
            pc=load_pc_evidence(out);self.assertTrue(pc['source_limits_verified'])
            from pil.gate1_measurement import correlate
            timing=correlate(out)
            self.assertTrue(timing['trial_tolerate_late']);self.assertEqual(timing['tolerated_late_count'],1)
            self.assertFalse(timing['physical_gate1_pass'])
            (out/'tolerated-late-captures.json').unlink()
            self.assertFalse(load_pc_evidence(out)['source_limits_verified'])

    def test_completed_tolerance_is_distinct_not_pass(self):
        h=self.host();self.sensor(h,0,300)
        with tempfile.TemporaryDirectory()as tmp:
            r=Recorder(Path(tmp)/'run');r.trial_clock_400_sync20=True;r.trial_tolerate_late=True
            r.stopped_status_observed=True
            s=r.finish(h,None,600000000)
            self.assertEqual(s['verdict'],'TRIAL_COMPLETE_WITH_TOLERATED_LATE')
            self.assertFalse(s['physical_gate1_pass'])
