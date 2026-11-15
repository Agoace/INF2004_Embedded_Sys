"""Reset-B companion capture tests; no serial ports or hardware."""
from pathlib import Path
import tempfile
import unittest
from pil.gate1_reset_capture import capture_cdc,parse_reset_records
from test_gate1 import FakeClock

FIXTURES=Path(__file__).parent/'fixtures'

class BreadcrumbCaptureTests(unittest.TestCase):
    def rows(self,name):
        return (FIXTURES/('pil_gate1_bc_'+name+'.txt')).read_bytes()

    def test_real_c_rows_and_partial_history(self):
        from pil.gate1_reset_capture import parse_breadcrumb_records
        for name in('empty','valid','partial','badframe','power'):
            row=self.rows(name)
            self.assertEqual(len(row),281)
            records=parse_breadcrumb_records(row,watchdog_reason=0 if name=='power' else 1)
            self.assertEqual(len(records),1,name)
            record=records[0]
            self.assertEqual(record['validity_mask'],{'empty':0,'valid':0x7f,'partial':0x2d,'badframe':0x3f,'power':0}[name])
            self.assertEqual(len(record['raw_previous_words']),28)
            if name in('empty','power'):
                self.assertEqual(record['validity_mask'],0)
                self.assertIsNone(record['previous_owner_nonce'])
                self.assertIsNone(record['motion'])
                self.assertIsNone(record['fault'])
            elif name=='valid':
                self.assertIsNotNone(record['motion'])
                self.assertTrue(record['fault']['entry_marker_observed'])
                self.assertTrue(record['fault']['frame_valid'])
            elif name=='partial':
                self.assertTrue(record['fault']['entry_marker_observed'])
                self.assertFalse(record['fault']['integrity_valid'])
                self.assertIsNone(record['fault']['time_us'])
                self.assertIsNone(record['fault']['frame'])
            elif name=='badframe':
                self.assertTrue(record['fault']['integrity_valid'])
                self.assertFalse(record['fault']['frame_valid'])
                self.assertIsNone(record['fault']['frame'])

    def test_strict_framing_and_independent_integrity_checks(self):
        from pil.gate1_reset_capture import parse_breadcrumb_records
        row=self.rows('valid')
        for bad in(row[:-1],row.replace(b'\n',b'\r\n'),row.replace(b'BC1',b'BC2'),
                    row.replace(b'50424731',b'5042473A'),row.replace(b' ',b'  ',1),
                    row.replace(b']\n',b' 00000000]\n'),row.replace(b']\n',b']\x00\n')):
            self.assertEqual(parse_breadcrumb_records(bad,watchdog_reason=1),[])
        words=row[len(b'[PILG1BC1 '):-2].split(b' ')
        for index in(1,5,9,13,17,28,29):
            damaged=words.copy();damaged[index]=f'{int(damaged[index],16)^0x80000000:08x}'.encode()
            bad=b'[PILG1BC1 '+b' '.join(damaged)+b']\n'
            self.assertEqual(parse_breadcrumb_records(bad,watchdog_reason=1),[],index)
        self.assertEqual(parse_breadcrumb_records(row,watchdog_reason=0),[])
        self.assertEqual(parse_breadcrumb_records(row),[])  # Missing RESET1 context.


    def test_actual_combined_c_streams_keep_d1_rx1_and_stop_detection(self):
        from pil.gate1_reset_capture import parse_breadcrumb_records
        from pil.gate1_diagnostics import decode_dump
        from pil.gate1_rx_timing import parse_rx1
        for name,size,stop,packets,alarms in(('runtime',5883,1,14,10),('missing',2674,256,4,3)):
            raw=(FIXTURES/('pil_gate1_bc_'+name+'.bin')).read_bytes()
            self.assertEqual(len(raw),size)
            dump=decode_dump(raw);rx=parse_rx1(raw,dump)
            self.assertEqual(dump.boot_nonce,123);self.assertEqual(dump.context[20],stop)
            self.assertTrue(rx['d1_identity_verified'])
            self.assertEqual((len(rx['packets']),len(rx['alarms'])),(packets,alarms))
            self.assertEqual(parse_breadcrumb_records(raw)[0]['validity_mask'],0)
            class Port:
                offset=0
                def read(self,n):
                    if self.offset<len(raw):
                        chunk=raw[self.offset:self.offset+n];self.offset+=len(chunk);return chunk
                    raise BlockingIOError()
                def write(self,*a):raise AssertionError('CDC RX only')
                def close(self):pass
            with tempfile.TemporaryDirectory()as tmp:
                result=capture_cdc(Port,Path(tmp)/'cdc',FakeClock(),duration_us=1200000)
                self.assertIsNotNone(result['stopped_seen_pc_us'])
                self.assertEqual(result['segments'][0]['breadcrumb_status'],'COMPLETE_ROW')
                self.assertEqual(result['segments'][0]['breadcrumb_records'][0]['nonce'],123)

    def test_delayed_split_real_companion_and_nonce_mismatch(self):
        row=self.rows('valid')
        for nonce,expected in((0x5678,'COMPLETE_ROW'),(0x5679,'UNKNOWN')):
            with self.subTest(nonce=nonce),tempfile.TemporaryDirectory()as tmp:
                reset=b'[PILG1RESET1 '+b' '.join(f'{v:08x}'.encode()for v in[nonce,0,1,256,*([0]*8)])+b']\n'
                clock=FakeClock();ports=[]
                class Port:
                    def __init__(self,index):self.index=index;self.part=0
                    def read(self,n):
                        if not self.index:
                            if not self.part:self.part=1;return b'initial boot\n'
                            raise OSError('reset')
                        if self.part==0:self.part=1;return reset
                        if clock.now<60000:raise BlockingIOError()
                        if self.part==1:self.part=2;return row[:100]
                        if self.part==2:self.part=3;return row[100:]
                        raise BlockingIOError()
                    def close(self):pass
                def opener():
                    p=Port(len(ports));ports.append(p);return p
                result=capture_cdc(opener,Path(tmp)/'cdc',clock,duration_us=2000000)
                segment=result['segments'][1]
                self.assertEqual(segment['breadcrumb_status'],expected)
                if expected=='COMPLETE_ROW':
                    self.assertLess(clock.now,1000000)
                    r=segment['breadcrumb_records'][0]
                    self.assertEqual(r['nonce'],0x5678);self.assertEqual(r['previous_owner_nonce'],0x1234)
                    self.assertEqual(r['motion']['time_us'],100)
                    self.assertEqual(r['pil']['state_name'],'BEFORE_SLEEP')
                    self.assertEqual(r['alarm0_tick']['entry_time_us'],1000)
                    self.assertGreaterEqual(r['complete_pc_us'],60000)
                    self.assertEqual((Path(tmp)/'cdc/pico-usb-001.bin').read_bytes(),reset+row)
                else:
                    self.assertGreaterEqual(clock.now,1000000)
                    self.assertEqual(segment['breadcrumb_records'],[])

    def test_reopened_reset_waits_bounded_time_for_missing_companion(self):
        # Exact original RESET1 emitted by SW6's C fixture, not a guessed B row.
        raw=(FIXTURES/'pil_gate1_reset_prefixed_crlf.bin').read_bytes()
        record=parse_reset_records(raw)[0]
        row=raw[record['offset']:record['offset']+record['length']]
        ports=[]
        class Port:
            sent=False
            def __init__(self,index):self.index=index
            def read(self,n):
                if not self.sent:
                    self.sent=True
                    return row if self.index else b'initial boot\n'
                if not self.index:raise OSError('reset disconnect')
                raise BlockingIOError()
            def close(self):pass
        def opener():
            p=Port(len(ports));ports.append(p);return p
        clock=FakeClock()
        with tempfile.TemporaryDirectory()as tmp:
            result=capture_cdc(opener,Path(tmp)/'cdc',clock,duration_us=2000000)
            self.assertGreaterEqual(clock.now,1000000)
            self.assertEqual(result['segments'][1]['breadcrumb_status'],'UNKNOWN')
            self.assertEqual(result['segments'][1]['reset_records'][0]['nonce'],123)
