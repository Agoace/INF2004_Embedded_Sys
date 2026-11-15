"""Host-only process recorder tests. Never open a tty."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
from pil.gate1_process_log import BinaryRecordCodec,ProcessLog,EvidenceFailure,SLOT_BYTES

class ProcessLogTests(unittest.TestCase):
    def test_frozen_soak_equality_and_no_parent_loop_json_or_file_writes(self):
        from unittest.mock import patch
        from pil.gate1 import run_soak
        from pil.gate1_replay import load_fixture,ReplayClock,RecordedUART
        fixture=load_fixture(Path(__file__).parent/'fixtures/pil_gate1_cap4_synthetic_replay.json')
        with tempfile.TemporaryDirectory()as tmp:
            outputs=[]
            for mode in('synchronous','process'):
                clock=ReplayClock(fixture['start_pc_us']);uart=RecordedUART(fixture,clock)
                active=[False];original_reset=uart.reset_input_buffer;original_close=uart.close
                def reset():original_reset();active[0]=True
                def close():original_close();active[0]=False
                uart.reset_input_buffer=reset;uart.close=close
                original_json=json.dumps;original_write=Path.write_text;original_csv=csv.writer
                def checked_json(*args,**kwargs):
                    if mode=='process'and active[0]:raise AssertionError('parent timing-loop JSON')
                    return original_json(*args,**kwargs)
                def checked_write(*args,**kwargs):
                    if mode=='process'and active[0]:raise AssertionError('parent timing-loop file write')
                    return original_write(*args,**kwargs)
                def checked_csv(*args,**kwargs):
                    if mode=='process'and active[0]:raise AssertionError('parent timing-loop CSV')
                    return original_csv(*args,**kwargs)
                output=Path(tmp)/mode
                with patch('json.dumps',side_effect=checked_json),patch.object(Path,'write_text',checked_write),patch('csv.writer',side_effect=checked_csv):
                    result=run_soak(uart,output,clock=clock,duration_us=fixture['duration_us'],
                        session=fixture['session'],transport_only=True,logging_mode=mode)
                self.assertEqual(result['verdict'],'INCOMPLETE');self.assertTrue(uart.complete)
                rows=list(csv.DictReader((output/'events.csv').read_text().splitlines()))
                for row in rows:
                    details=json.loads(row['details_json'])
                    if row['event']=='run_metadata':details['pc_logging']='MODE'
                    row['details_json']=json.dumps(details,sort_keys=True)
                result.pop('pc_logging');outputs.append((rows,result))
            self.assertEqual(outputs[0],outputs[1])
            self.assertTrue(json.loads((Path(tmp)/'process/process-recorder.json').read_text())['evidence_complete'])

    def test_codec_exact_json_types_bytes_nested_and_crc(self):
        codec=BinaryRecordCodec();buffer=bytearray(SLOT_BYTES)
        details=dict(integer=17,floating=17.,negative=-99,large=2**64-1,yes=True,no=False,
                     missing=None,text='µ',packet_hex=b'\x00\xff',nested={'a':[1,.5,None]})
        size=codec.encode_into(buffer,0,7,'sample',None,details)
        seq,event,pc,result=codec.decode(buffer[:size])
        self.assertEqual((seq,event,pc),(7,'sample',None));expected=dict(details,packet_hex='00ff')
        self.assertEqual(json.dumps(result,sort_keys=True),json.dumps(expected,sort_keys=True))
        buffer[size-1]^=1
        with self.assertRaises(ValueError):codec.decode(buffer[:size])
        with self.assertRaises(ValueError):codec.encode_into(buffer,0,8,'huge',1,{'x':b'x'*SLOT_BYTES})

    def test_child_death_before_metadata_still_closes_owned_io(self):
        from unittest.mock import patch
        from pil.gate1 import run_soak
        from test_gate1 import FakeClock,FakePair
        clock=FakeClock();uart=FakePair(clock)
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp)/'run'
            def factory(output):
                log=ProcessLog(output,normal_capacity=8,stop_capacity=8,paused=True)
                log.process.kill();log.process.join(2);return log
            with patch('pil.gate1.ProcessLog',side_effect=factory),self.assertRaises(EvidenceFailure):
                run_soak(uart,path,clock=clock,session=123,duration_us=1000)
            self.assertTrue(uart.closed)
            meta=json.loads((path/'process-recorder.json').read_text())
            self.assertTrue(meta['serial_closed']);self.assertFalse(meta['evidence_complete'])

    def test_overflow_in_real_soak_inhibits_and_closes_without_waiting_for_slots(self):
        from unittest.mock import patch
        from pil.gate1 import run_soak
        from test_gate1 import FakeClock,FakePair
        clock=FakeClock();uart=FakePair(clock)
        uart.session=123  # Permit this fake peer to construct a stop report before HELLO.
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp)/'run'
            factory=lambda output:ProcessLog(output,normal_capacity=8,stop_capacity=8,paused=True)
            with patch('pil.gate1.ProcessLog',side_effect=factory):
                result=run_soak(uart,path,clock=clock,session=123,duration_us=40000,measure_io=True)
            self.assertEqual(result['verdict'],'FAIL');self.assertIn('ring full',result['failure'])
            self.assertTrue(uart.closed)
            meta=json.loads((path/'process-recorder.json').read_text())
            self.assertTrue(meta['serial_closed']);self.assertTrue(meta['inhibited']);self.assertFalse(meta['evidence_complete'])
            rows=list(csv.DictReader((path/'events.csv').read_text().splitlines()))
            self.assertIn('best_effort_disarm',[row['event']for row in rows])

    def test_interrupted_and_oversized_publication_mark_incomplete(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory()as tmp:
            log=ProcessLog(Path(tmp),normal_capacity=1,stop_capacity=2,paused=True)
            with patch.object(log.codec,'encode_into',side_effect=KeyboardInterrupt):
                with self.assertRaises(KeyboardInterrupt):log.record('interrupted',1,{})
            meta=log.finish(2,serial_closed=True,inhibited=True)
            self.assertFalse(meta['evidence_complete']);self.assertEqual(meta['lost_records'],1)
        with tempfile.TemporaryDirectory()as tmp:
            log=ProcessLog(Path(tmp),normal_capacity=1,stop_capacity=2,paused=True)
            log.record('first',1,{})
            with self.assertRaises(EvidenceFailure):log.record('huge',2,{'payload_hex':bytes(SLOT_BYTES)})
            meta=log.finish(3,serial_closed=True,inhibited=True)
            self.assertFalse(meta['evidence_complete']);self.assertEqual(meta['lost_records'],1)

    def test_fixed_schemas_preserve_numeric_types_missing_and_raw_bytes(self):
        from pil.gate1_process_log import FAST_SCHEMAS
        codec=BinaryRecordCodec();buffer=bytearray(SLOT_BYTES)
        for name,fields,raw in FAST_SCHEMAS:
            for example in(0,17.,True,None):
                details={key:example for key in fields[::2]}
                if raw:details[raw]=bytes(range(256))*16
                size=codec.encode_into(buffer,0,0,name,123,details)
                decoded=codec.decode(buffer[:size])[-1]
                expected={k:v.hex()if isinstance(v,bytes)else v for k,v in details.items()}
                self.assertEqual(json.dumps(decoded,sort_keys=True),json.dumps(expected,sort_keys=True))

    def test_child_sigint_does_not_discard_parent_stop_proof(self):
        import os,signal
        with tempfile.TemporaryDirectory()as tmp:
            log=ProcessLog(Path(tmp),normal_capacity=4,stop_capacity=4)
            log.record('first',1,{})
            os.kill(log.process.pid,signal.SIGINT)
            log.begin_stop();log.record('stop',2,{})
            self.assertTrue(log.finish(3,serial_closed=True,inhibited=True)['evidence_complete'])

    def test_process_order_spool_and_csv_byte_equality(self):
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);log=ProcessLog(path,normal_capacity=8,stop_capacity=4)
            # Waits are test-side only, never producer-side.
            for n in range(40):
                log.record('raw',n,{'packet_hex':bytes([n]),'value':n+.5})
                log.wait_consumed(n+1)
            log.begin_stop();log.record('stop',40,{'closed':False})
            meta=log.finish(41,serial_closed=True,inhibited=True)
            self.assertTrue(meta['evidence_complete'],meta)
            self.assertEqual(meta['written_records'],41)
            rows=list(csv.DictReader((path/'events.csv').read_text().splitlines()))
            self.assertEqual([int(row['pc_us'])for row in rows],list(range(41)))
            self.assertEqual(json.loads(rows[3]['details_json'])['packet_hex'],'03')
            self.assertFalse(log.process.is_alive())

    def test_full_normal_preserves_failed_record_then_reserve_and_close(self):
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);log=ProcessLog(path,normal_capacity=1,stop_capacity=2,paused=True)
            log.record('normal',1,{})
            with self.assertRaisesRegex(EvidenceFailure,'normal ring full'):log.record('retained_failed',2,{})
            log.begin_stop();log.record('disarm',3,{})
            with self.assertRaisesRegex(EvidenceFailure,'stop ring full'):log.record('not_retained',4,{})
            log.resume();meta=log.finish(5,serial_closed=True,inhibited=True)
            self.assertFalse(meta['evidence_complete']);self.assertEqual(meta['lost_records'],1)
            rows=list(csv.DictReader((path/'events.csv').read_text().splitlines()))
            self.assertEqual([r['event']for r in rows],['normal','retained_failed','disarm'])
            self.assertTrue(meta['serial_closed']);self.assertTrue(meta['inhibited'])
            self.assertTrue((path/'recorder-close.bin').exists())

    def test_killed_child_never_seals_success_and_retains_pending(self):
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp);log=ProcessLog(path,normal_capacity=4,stop_capacity=4,paused=True)
            log.record('pending',1,{'payload_hex':b'abc'})
            log.process.kill();log.process.join(2)
            with self.assertRaises(EvidenceFailure):log.record('dead',2,{})
            meta=log.finish(3,serial_closed=True,inhibited=True)
            self.assertFalse(meta['evidence_complete']);self.assertTrue((path/'evidence-pending.bin').exists())
            from pil.gate1_process_log import HEADER
            raw=(path/'evidence-pending.bin').read_bytes();offset=0;recovered=[]
            while offset<len(raw):
                size=HEADER.unpack_from(raw,offset)[3]
                recovered.append(BinaryRecordCodec.decode(raw[offset:offset+size]));offset+=size
            self.assertEqual([row[0]for row in recovered],[0,1])
            self.assertEqual(recovered[0][3]['payload_hex'],'616263')

    def test_spool_write_failure_and_slow_child_timeout(self):
        for fault in('write_error','short_write','hang'):
            with self.subTest(fault=fault),tempfile.TemporaryDirectory()as tmp:
                log=ProcessLog(Path(tmp),normal_capacity=4,stop_capacity=4,test_fault=fault)
                log.record('event',1,{})
                meta=log.finish(2,serial_closed=True,inhibited=True,timeout=.5)
                if fault=='short_write':self.assertTrue(meta['evidence_complete'],meta)
                else:self.assertFalse(meta['evidence_complete'],meta)
                self.assertFalse(log.process.is_alive())
