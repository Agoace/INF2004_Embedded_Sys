"""Actual C idle watermark rows: offline only, no device access."""
from pathlib import Path
import tempfile
import unittest
from pil.gate1_measurement import correlate
from pil.gate1_diagnostics import decode_dump
from pil.gate1_rx_timing import parse_rx1
from pil.gate1_reset_capture import capture_cdc,parse_reset_records,parse_breadcrumb_records
from test_gate1 import FakeClock
from test_gate1_measurement import _mini_pc

FIXTURES=Path(__file__).parent/'fixtures'

class IdleStackCaptureTests(unittest.TestCase):
    def test_actual_c_suffix_cohorts_and_capture(self):
        for name,size,used,packets,alarms in(('runtime',5918,0,14,10),('missing',2709,0,4,3),('painted',5919,96,14,10)):
            raw=(FIXTURES/f'pil_gate1_idle_{name}.bin').read_bytes()
            self.assertEqual(len(raw),size)
            dump=decode_dump(raw);rx=parse_rx1(raw,dump)
            self.assertTrue(rx['d1_identity_verified'])
            self.assertEqual((len(rx['packets']),len(rx['alarms'])),(packets,alarms))
            self.assertEqual(parse_reset_records(raw)[0]['length'],122)
            self.assertEqual(parse_breadcrumb_records(raw)[0]['length'],281)
            with tempfile.TemporaryDirectory()as tmp:
                path=Path(tmp);_mini_pc(path);stopped=path/'stopped.bin';stopped.write_bytes(raw)
                report=correlate(path,stopped=stopped)
                self.assertEqual(report['diagnostic'],'COMPLETE',report)
                self.assertEqual(report['stopped_idle_stack_high_water_bytes'],used)
                self.assertEqual(report['stopped_worker_stack_untouched_bytes'],2048)
                self.assertFalse(report['physical_gate1_pass'])
                class Port:
                    at=0
                    def read(self,n):
                        if self.at<len(raw):
                            # Split the appended suffix across separate reads.
                            end=min(self.at+37,len(raw));data=raw[self.at:end];self.at=end;return data
                        raise BlockingIOError()
                    def close(self):pass
                    def write(self,*a):raise AssertionError('RX only')
                result=capture_cdc(Port,path/'cdc',FakeClock(),duration_us=1200000)
                self.assertIsNotNone(result['stopped_seen_pc_us'])
                self.assertEqual((path/'cdc/pico-usb-000.bin').read_bytes(),raw)

    def test_suffix_strict_ranges_legacy_and_crlf(self):
        raw=(FIXTURES/'pil_gate1_idle_painted.bin').read_bytes()
        suffix=b'; idle stack high-water 96/512 bytes'
        for changed,complete,used in((raw,True,96),(raw.replace(suffix,b''),True,None),
            (raw.replace(suffix,suffix.replace(b'96',b'512')),True,512),
            (raw.replace(suffix,suffix.replace(b'96',b'513')),False,None),
            (raw.replace(suffix,suffix.replace(b'96',b'-1')),False,None),
            (raw.replace(b'2048/2048',b'0/2048'),True,96),
            (raw.replace(b'2048/2048',b'2049/2048'),False,None),
            (raw.replace(suffix,suffix.replace(b'512',b'513')),False,None),
            (raw.replace(suffix,suffix+b'; unknown'),False,None),
            (raw[:-1],False,None),(raw[:-1]+b'\r\n',True,96)):
            with self.subTest(tail=changed[-150:]),tempfile.TemporaryDirectory()as tmp:
                path=Path(tmp);_mini_pc(path);stopped=path/'stopped.bin';stopped.write_bytes(changed)
                report=correlate(path,stopped=stopped)
                self.assertEqual(report['stopped_stack_line_complete'],complete)
                self.assertEqual(report['stopped_idle_stack_high_water_bytes'],used)
