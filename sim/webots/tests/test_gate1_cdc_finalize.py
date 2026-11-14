"""No-device regressions: native C stream, visible writes, owned signal cleanup."""
import json
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import unittest
from test_gate1 import FakeClock
from pil.gate1_reset_capture import capture_cdc

FIXTURE=Path(__file__).parent/'fixtures/pil_gate1_reset_prefixed_crlf.bin'

class CDCFinalizeTests(unittest.TestCase):
    def test_real_prefixed_c_stream_crlf_stack_and_trailing_nul_completes(self):
        raw=FIXTURE.read_bytes()
        class Port:
            offset=0
            def read(self,n):
                if self.offset<len(raw):
                    chunk=raw[self.offset:self.offset+n];self.offset+=len(chunk);return chunk
                raise BlockingIOError()
            def close(self):pass
        with tempfile.TemporaryDirectory()as tmp:
            result=capture_cdc(Port,Path(tmp)/'cdc',FakeClock(),duration_us=1200000)
            self.assertIsNotNone(result['stopped_seen_pc_us'])
            self.assertEqual(result['reason'],'stopped D1 and stack plus1s reset-watch grace')

    def test_each_chunk_is_visible_before_close(self):
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'cdc';test=self
            class Port:
                sent=False
                def read(self,n):
                    if not self.sent:self.sent=True;return b'boot partial\r\n'
                    test.assertEqual((out/'pico-usb-000.bin').read_bytes(),b'boot partial\r\n')
                    raise BlockingIOError()
                def close(self):pass
            capture_cdc(Port,out,FakeClock(),duration_us=20000)

    def test_sigterm_and_inherited_ignored_sigint_finalize(self):
        tools=Path(__file__).resolve().parents[1]/'tools'
        code='''
import signal,sys
sys.path.insert(0,sys.argv[1])
import gate1_pico_cdc_capture as tool
signal.signal(signal.SIGINT,signal.SIG_IGN)
class Port:
    sent=False
    ready=False
    def __init__(self,path):pass
    def read(self,n):
        if not self.sent:self.sent=True;return b'boot partial\\r\\n'
        if not self.ready:self.ready=True;print('TEST_RAW_WRITTEN',flush=True)
        raise BlockingIOError()
    def close(self):pass
tool.PicoCDC=Port
sys.argv=['capture','--hardware-authorized','--device','/dev/serial/by-id/usb-micro_T-Kernel_Forum_MTK3_Thread-Metric_RP2040_MTK3PICO-if00','--output',sys.argv[2],'--duration-seconds','5']
sys.exit(tool.main())
'''
        for sig in(signal.SIGTERM,signal.SIGINT):
            with self.subTest(sig=sig),tempfile.TemporaryDirectory()as tmp:
                out=Path(tmp)/'cdc'
                p=subprocess.Popen([sys.executable,'-u','-c',code,str(tools),str(out)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0)
                try:
                    for _ in range(4):
                        self.assertTrue(select.select([p.stdout],[],[],5)[0],'fake reader not ready')
                        if b'TEST_RAW_WRITTEN'in p.stdout.readline():break
                    else:self.fail('readiness marker missing')
                    p.send_signal(sig)
                    p.communicate(timeout=3)
                    self.assertEqual(p.returncode,2)
                    result=json.loads((out/'cdc-capture.json').read_text())
                    self.assertIn(f'signal {int(sig)}',result['reason'])
                    self.assertEqual((out/'pico-usb-000.bin').read_bytes(),b'boot partial\r\n')
                finally:
                    if p.poll()is None:p.kill()
                    p.communicate()
