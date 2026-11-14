import json
from pathlib import Path
import struct
import tempfile
import unittest
from test_gate1 import FakeClock,FakePair
from pil.gate1 import run_soak
from pil.wire import Type,SCHEMA_ID

class FailureTailTests(unittest.TestCase):
    def test_failure_tail_uses_same_reader_no_tx_and_retains_failure_and_new_boot(self):
        c=FakeClock()
        class Peer(FakePair):
            def write(self,data):
                count=super().write(data)
                if self.sent_frames and self.sent_frames[-1].type==Type.DISARM:
                    self.queue(Type.BOOT,struct.pack('<4I',42,4,921600,SCHEMA_ID))
                    self.pending[-1][0]=c.now+100000
                return count
        io=Peer(c,capabilities=4,fault='session')
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            s=run_soak(io,out,clock=c,session=123,duration_us=200000,transport_only=True,
                trial_clock_400_sync20=True,post_fail_rx_us=500000)
            self.assertEqual(s['verdict'],'FAIL')
            self.assertTrue(io.closed)
            stop=next(i for i,f in enumerate(io.sent_frames)if f.type==Type.DISARM)
            self.assertEqual(len(io.sent_frames),stop+1)
            t=json.loads((out/'post-fail-rx.json').read_text())
            self.assertEqual(t['original_failure'],s['failure'])
            self.assertEqual(t['requested_us'],500000)
            self.assertIn(42,[x['nonce']for x in t['boots']])
            self.assertTrue((out/'post-fail-rx.bin').exists())

    def test_tail_read_error_cannot_replace_original_failure_or_prevent_close(self):
        c=FakeClock()
        class Peer(FakePair):
            def read(self,n):
                if any(f.type==Type.DISARM for f in self.sent_frames):raise OSError('tail disconnected')
                return super().read(n)
        io=Peer(c,capabilities=4,fault='session')
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            s=run_soak(io,out,clock=c,session=123,duration_us=200000,transport_only=True,
                trial_clock_400_sync20=True,post_fail_rx_us=500000)
            t=json.loads((out/'post-fail-rx.json').read_text())
            self.assertEqual(t['error'],'tail disconnected')
            self.assertEqual(t['original_failure'],s['failure'])
            self.assertNotEqual(s['failure'],'tail disconnected')
            self.assertTrue(io.closed)

    def test_cdc_reopen_retains_segments_and_completes_only_full_new_record(self):
        from pil.gate1_reset_capture import capture_cdc
        def row(n):return b'[PILG1RESET1 '+b' '.join(f'{v:08x}'.encode()for v in[n,0,*range(2,12)])+b']\n'
        opened=[]
        class Port:
            def __init__(self,chunks):self.chunks=list(chunks);self.closed=False
            def read(self,n):
                if not self.chunks:raise OSError('disconnect')
                return self.chunks.pop(0)
            def write(self,*a):raise AssertionError('CDC must never transmit')
            def close(self):self.closed=True
        ports=[Port([row(1)]),Port([row(2)[:60],row(2)[60:]])]
        def opener():
            p=ports[len(opened)];opened.append(p);return p
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'cdc'
            result=capture_cdc(opener,out,FakeClock(),duration_us=500000,labels={'qualification':'TRIAL / NOT GATE 1 PASS'})
            self.assertEqual(len(result['segments']),2)
            self.assertEqual(result['segments'][1]['reset_records'][0]['nonce'],2)
            self.assertTrue(all(p.closed for p in opened))
            self.assertEqual((out/'pico-usb-001.bin').read_bytes(),row(2))

    def test_cdc_d1_completion_keeps_reset_watch_grace(self):
        from unittest.mock import patch
        from pil.gate1_reset_capture import capture_cdc
        c=FakeClock();opened=[]
        row=b'[PILG1RESET1 '+b' '.join(f'{v:08x}'.encode()for v in[2,0,*range(2,12)])+b']\n'
        class Port:
            def __init__(self,index):self.index=index;self.sent=False
            def read(self,n):
                if not self.sent:
                    self.sent=True
                    return row if self.index else b'[PILG1] stopped; reset required; worker stack untouched fixture\n'
                if c.now<100000:raise BlockingIOError()
                raise OSError('reset')
            def close(self):pass
        def opener():
            p=Port(len(opened));opened.append(p);return p
        with tempfile.TemporaryDirectory()as tmp,patch('pil.gate1_reset_capture.decode_dump',return_value=None):
            result=capture_cdc(opener,Path(tmp)/'cdc',c,duration_us=2000000)
        self.assertEqual(len(result['segments']),2)
        self.assertEqual(result['segments'][1]['reset_records'][0]['nonce'],2)

    def test_reset_record_strict_schema(self):
        from pil.gate1_reset_capture import parse_reset_records
        row=b'[PILG1RESET1 '+b' '.join(f'{n:08x}'.encode()for n in[1,0,*range(2,12)])+b']\n'
        self.assertEqual(len(row),122)
        self.assertEqual(parse_reset_records(row)[0]['nonce'],1)
        self.assertEqual(parse_reset_records(row[:-1]),[])
        self.assertEqual(parse_reset_records(row.replace(b'0000000a',b'0000000A')),[])
        self.assertEqual(parse_reset_records(row.replace(b'00000000',b'00000001',1)),[])

if __name__=='__main__':unittest.main()
