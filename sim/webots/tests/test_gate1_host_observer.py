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
