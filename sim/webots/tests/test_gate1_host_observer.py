import csv
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from test_gate1 import FakeClock,FakePair
from pil.gate1 import run_soak

class HostObserverTests(unittest.TestCase):
    def test_bounded_spans_keep_thread_cpu_and_rusage_deltas(self):
        from pil.gate1_host_observer import HostSpans
        c=FakeClock();usage=SimpleNamespace(ru_nvcsw=1,ru_nivcsw=2,ru_minflt=3,ru_majflt=4)
        cpu=[100]
        spans=HostSpans(c,capacity=1,cpu=lambda:cpu[0],usage=lambda:usage)
        for _ in range(2):
            start=spans.begin('select',7);c.now+=600;cpu[0]+=100
            usage=SimpleNamespace(ru_nvcsw=2,ru_nivcsw=4,ru_minflt=6,ru_majflt=8)
            spans.end(start)
        with tempfile.TemporaryDirectory()as tmp:
            spans.finish(Path(tmp))
            rows=list(csv.DictReader((Path(tmp)/'host-spans.csv').read_text().splitlines()))
            self.assertEqual(len(rows),1)
            self.assertEqual(int(rows[0]['wall_us']),600)
            self.assertEqual(int(rows[0]['thread_cpu_ns']),100)
            self.assertEqual(spans.overwritten,1)

    def test_receive_span_covers_decode_and_serialization_markers(self):
        from unittest.mock import patch
        from pil.gate1 import receive_chunk
        c=FakeClock();io=FakePair(c,capabilities=4)
        def slow_receive(*args,**kwargs):
            result=receive_chunk(*args,**kwargs);c.now+=151;return result
        with tempfile.TemporaryDirectory()as tmp,patch('pil.gate1.receive_chunk',side_effect=slow_receive):
            out=Path(tmp)/'run'
            run_soak(io,out,clock=c,session=123,duration_us=1000,transport_only=True,
                     measure_io=True,trial_clock_400_sync20=True)
            spans=list(csv.DictReader((out/'host-spans.csv').read_text().splitlines()))
            spans=[s for s in spans if s['kind']=='receive']
            self.assertTrue(spans)
            self.assertTrue(any(int(s['wall_us'])>=151 for s in spans))
            marks=list(csv.DictReader((out/'attribution.csv').read_text().splitlines()))
            by_loop={}
            for r in marks:
                if r['kind']in ('read_exit','receive_exit'):by_loop.setdefault(r['loop'],[]).append(r)
            checked=0
            for s in spans:
                rows=by_loop.get(s['loop'],[])
                if len(rows)==2:
                    self.assertTrue(all(int(s['raw_start_us'])<=int(r['pc_us'])<=int(s['raw_end_us'])for r in rows))
                    checked+=1
            self.assertGreater(checked,0)

    def test_receive_span_closes_on_transport_error(self):
        from unittest.mock import patch
        from pil.gate1 import GateFailure
        c=FakeClock();io=FakePair(c,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp,patch('pil.gate1.receive_chunk',side_effect=GateFailure('receive fault test')):
            out=Path(tmp)/'run'
            summary=run_soak(io,out,clock=c,session=123,duration_us=1000,transport_only=True,
                     measure_io=True,trial_clock_400_sync20=True)
            self.assertEqual(summary['failure'],'receive fault test')
            spans=list(csv.DictReader((out/'host-spans.csv').read_text().splitlines()))
            self.assertEqual(spans[-1]['kind'],'receive')
            self.assertTrue(io.closed)

    def test_snapshot_records_per_cpu_resume_qos_without_device_access(self):
        from unittest.mock import patch
        from pil.gate1_host_observer import host_snapshot
        cpu=Path('/sys/devices/system/cpu/cpu0')
        def glob(path,pattern):return [cpu] if pattern=='cpu[0-9]*' else []
        def read(path):return {'status':'UNKNOWN','reason':'test missing file'}
        with patch('pil.gate1_host_observer.Path.glob',glob),patch('pil.gate1_host_observer._read',side_effect=read)as reads:
            snapshot=host_snapshot()
        key=str(cpu/'power/pm_qos_resume_latency_us')
        self.assertIn(key,snapshot['power_files'])
        self.assertEqual(snapshot['power_files'][key]['status'],'UNKNOWN')
        self.assertFalse(any(str(call.args[0]).startswith('/dev/')for call in reads.call_args_list))

    def test_read_return_timestamp_precedes_observer_postwork(self):
        from unittest.mock import patch
        from pil.serial_io import ProbeSerial
        c=FakeClock();io=ProbeSerial.__new__(ProbeSerial);io.fd=10
        class Observer:
            def begin(self,*a):return None
            def end(self,*a):c.now+=20
        with patch('pil.serial_io.select.select',return_value=([10],[],[])),patch('pil.serial_io.os.read',return_value=b'x'):
            data,t=io.read_timed(4096,c.now_us,observer=Observer())
        self.assertEqual(data,b'x')
        self.assertEqual(t['read_return_pc_us'],c.now-20)
        io.fd=None

    def test_measured_soak_exports_sleep_and_io_observer(self):
        c=FakeClock();io=FakePair(c,capabilities=4)
        with tempfile.TemporaryDirectory()as tmp:
            out=Path(tmp)/'run'
            run_soak(io,out,clock=c,session=123,duration_us=1000,transport_only=True,
                     measure_io=True,trial_clock_400_sync20=True)
            self.assertTrue((out/'host-spans.csv').exists())
            self.assertTrue((out/'host-before.json').exists())
            self.assertTrue((out/'host-after.json').exists())

if __name__=='__main__':unittest.main()
