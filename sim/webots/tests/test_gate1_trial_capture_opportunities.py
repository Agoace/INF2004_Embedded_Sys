"""C2b: fresh post-RX/pre-SYNC and post-SYNC capture opportunities."""
import csv
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak, neutral_payload
from pil.transport import HostTransport
from pil.wire import StreamDecoder, Type
from test_gate1 import FakeClock, FakePair


class TrialCaptureOpportunities(unittest.TestCase):
    def exercise(self,where):
        clock=FakeClock()
        # Synthetic wake latency avoids an accidental50us/1ms grid alignment.
        clock.sleep_us=lambda delay:setattr(clock,'now',clock.now+delay+7)
        pair=FakePair(clock,capabilities=4)
        original_read=pair.read;original_sync=HostTransport.sync_request
        hosts=[];injected=[];last_payload=[];delayed_source=[]
        def factory(*a,**kw):
            h=HostTransport(*a,**kw)
            # Unit-test request cadence: arrange a real request in the crossing
            # iteration, without needing to depend on20ms grid coincidence.
            h.clock.refresh_deadline_pc_us=lambda:clock.now+100
            hosts.append(h)
            return h
        def gap(h):
            if h.state not in ('ARM_PENDING','ARMED'):return None
            return h.timeline.source_target(h.next_sensor*1000)-h.clock.pico_range(clock.now).midpoint
        def read(n):
            h=hosts[0];data=original_read(n);g=gap(h)
            # Return a completed SYNC reply: RX clears pending so the overdue
            # real next request is generated in this same crossing iteration.
            reply=any(f.type==Type.SYNC_REPLY for f in StreamDecoder().feed(data))
            if where=='rx'and not injected and reply and g is not None and 0<g<=100:
                clock.now+=math.ceil(g)+10;injected.append(clock.now)
            return data
        def sync(h,pc):
            result=original_sync(h,pc)
            g=gap(h)
            if where=='sync'and not injected and g is not None and 0<g<=100:
                clock.now+=math.ceil(g)+10;injected.append(clock.now)
            if where in ('one','expire')and not injected and last_payload and last_payload[-1]==clock.now:
                delayed_source.append(h.next_sensor-1)
                clock.now+=6000 if where=='expire' else 1100
                injected.append(clock.now)
            return result
        def payload(source):
            clock.now+=1  # Actual capture brackets are not always zero length.
            last_payload.append(clock.now)
            return neutral_payload(source)
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            with patch('pil.gate1.HostTransport',side_effect=factory),patch.object(pair,'read',side_effect=read),patch.object(HostTransport,'sync_request',sync),patch('pil.gate1.neutral_payload',side_effect=payload):
                summary=run_soak(pair,out,clock=clock,duration_us=40000,session=123,
                    transport_only=True,measure_io=True,trial_clock_400_sync20=True)
            self.assertTrue(injected,summary['failure'] or 'test must actually cross a target')
            if where=='expire':
                self.assertEqual(summary['verdict'],'FAIL')
                self.assertTrue('lease expired' in summary['failure'] or 'lookahead deadline' in summary['failure'])
                self.assertNotIn(delayed_source[0],pair.sensors)
                return
            self.assertNotEqual(summary['verdict'],'FAIL',summary['failure'])
            self.assertEqual(summary['pc_capture_revision'],'C1+C2+C2b+C3')
            import json
            manifest=json.loads((out/'trial-manifest.json').read_text())
            self.assertEqual(manifest['capture_checks'],['loop-entry','post-RX/pre-SYNC','post-SYNC'])
            self.assertEqual(manifest['max_captures_per_iteration'],1)
            source_rows=[json.loads(x['details_json']) for x in csv.DictReader(
                (out/'events.csv').read_text().splitlines()) if x['event']=='source_capture']
            self.assertEqual([x['source_us'] for x in source_rows],list(range(0,len(source_rows)*1000,1000)))
            for source in source_rows:
                self.assertTrue(source['no_early_midpoint'])
                self.assertGreaterEqual(source['start_midpoint_error_us'],0)
                self.assertLessEqual(source['phase_us'],250)
            rows=[dict(x,pc_us=int(x['pc_us']))for x in csv.DictReader((out/'attribution.csv').read_text().splitlines())]
            loops={}
            for r in rows:loops.setdefault(r['loop'],[]).append(r)
            for rs in loops.values():self.assertLessEqual(sum(r['kind']=='capture_enter'for r in rs),1)
            if where!='one':
                kind='read_exit'if where=='rx'else'sync_exit'
                crossing=next(rs for rs in loops.values()if any(r['kind']==kind and r['pc_us']>=injected[0]for r in rs))
                names=[r['kind']for r in crossing]
                self.assertIn('capture_enter',names,'capture must occur before next sleep')
                if where=='rx':
                    self.assertLess(names.index('drain_exit'),names.index('capture_enter'))
                    self.assertLess(names.index('capture_enter'),names.index('sync_exit'))
                    # Real SYNC queued in this same iteration, not a mock no-op.
                    queued=[x for x in csv.DictReader((out/'events.csv').read_text().splitlines())if x['event']=='tx_queued'and json.loads(x['details_json']).get('type')==7]
                    self.assertTrue(any(crossing[0]['pc_us']<=int(x['pc_us'])<=crossing[-1]['pc_us']for x in queued))
                else:self.assertLess(names.index('sync_exit'),names.index('capture_enter'))
                ticks=[i for i,n in enumerate(names)if n=='tick_exit']
                cap=names.index('capture_enter')
                self.assertTrue(any(names.index('receive_exit')<i<cap for i in ticks))

    def test_slot_due_during_rx_captured_before_same_loop_sync(self):self.exercise('rx')
    def test_slot_due_during_sync_captured_before_sleep(self):self.exercise('sync')
    def test_only_one_capture_even_if_next_slot_due_after_sync(self):self.exercise('one')
    def test_fresh_post_sync_expiry_cancels_before_tx(self):self.exercise('expire')


if __name__=='__main__':unittest.main()
