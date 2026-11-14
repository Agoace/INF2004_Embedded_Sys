"""C3 bounded nonblocking producer, immutable evidence, stopped close ACK."""
import csv
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from pil.gate1 import Recorder, run_soak
from test_gate1 import FakeClock, FakePair
from pil.wire import Type


class QueuedLogTests(unittest.TestCase):
    def test_full_queue_fails_without_losing_failed_record_or_stop(self):
        from pil.gate1_log import QueuedEventWriter, EvidenceFailure
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'events.csv'
            writer = QueuedEventWriter(path, capacity=1, start=False)
            writer.record('first', 1, {'packet_hex': b'\x01'})
            with self.assertRaises(EvidenceFailure):
                writer.record('overflow', 2, {'packet_hex': b'\x02'})
            writer.begin_stop()
            writer.record('stop', 3, {'packet_hex': b'\x03'})
            status = writer.finish()
            self.assertTrue(status['close_ack'])
            self.assertIsNotNone(status['failure'])
            rows = list(csv.DictReader(path.read_text().splitlines()))
            self.assertEqual([x['event'] for x in rows], ['first', 'overflow', 'stop'])

    def test_snapshot_copies_mutable_values_and_serializes_fifo(self):
        from pil.gate1_log import QueuedEventWriter
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'events.csv'
            writer = QueuedEventWriter(path, start=False)
            values = {'packet_hex': bytearray(b'\0\xff'), 'nested': {'n': [1, 2]}}
            writer.record('one', 1, values)
            values['packet_hex'][0] = 3
            values['nested']['n'].append(9)
            writer.record('two', 2, {})
            self.assertTrue(writer.finish()['close_ack'])
            rows=list(csv.DictReader(path.read_text().splitlines()))
            self.assertEqual(json.loads(rows[0]['details_json']),
                             {'packet_hex':'00ff', 'nested':{'n':[1,2]}})
            self.assertEqual(rows[1]['event'], 'two')

    def test_recorder_record_and_flush_do_not_serialize_on_producer(self):
        main = threading.get_ident()
        original = json.dumps
        def guard(*args, **kwargs):
            if threading.get_ident() == main:
                raise AssertionError('producer serialized JSON')
            return original(*args, **kwargs)
        with tempfile.TemporaryDirectory() as tmp:
            recorder=Recorder(Path(tmp)/'run')
            try:
                with patch('json.dumps', side_effect=guard):
                    recorder.record('raw_rx', 1, packet_hex=b'\0\xff')
                    recorder.flush()
            finally:
                recorder.stop_writer()
            self.assertTrue(recorder.logger_status['close_ack'])

    def test_fault_slot_preserves_tail_order_when_regular_reserve_is_full(self):
        from pil.gate1_log import QueuedEventWriter
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'events.csv'
            writer=QueuedEventWriter(path, start=False, reserve_capacity=2)
            writer.begin_stop()
            writer.record('before',1,{})
            writer.record('source_capture_rejected',2,{'phase_us':250.5})
            writer.record('after',3,{})
            self.assertTrue(writer.finish()['close_ack'])
            rows=list(csv.DictReader(path.read_text().splitlines()))
            self.assertEqual([x['event'] for x in rows], ['before','source_capture_rejected','after'])

    def test_full_logger_inhibits_and_sends_stop_before_writer_wait(self):
        from pil.gate1_log import QueuedEventWriter
        class Paused(QueuedEventWriter):
            def __init__(self,path):
                super().__init__(path,capacity=1,start=False)
            def start(self):
                if self.stopping:
                    super().start()
        clock=FakeClock()
        pair=FakePair(clock)
        with tempfile.TemporaryDirectory() as tmp:
            with patch('pil.gate1_log.QueuedEventWriter',Paused):
                result=run_soak(pair,Path(tmp)/'run',clock=clock,session=123,duration_us=1000)
            self.assertEqual(result['failure'],'evidence queue full')
            self.assertTrue(pair.closed)
            self.assertTrue(any(f.type==Type.DISARM for f in pair.sent_frames))
            self.assertTrue(result['evidence_logger']['close_ack'])
            self.assertTrue(result['evidence_logger']['failed_submission_retained'])

    def test_slow_writer_times_out_without_false_ack(self):
        from pil.gate1_log import QueuedEventWriter
        release=threading.Event()
        with tempfile.TemporaryDirectory() as tmp:
            writer=QueuedEventWriter(Path(tmp)/'events.csv',start=False)
            original=writer._emit
            def slow(snapshot):
                release.wait()
                original(snapshot)
            writer.record('one',1,{})
            try:
                with patch.object(writer,'_emit',side_effect=slow):
                    status=writer.finish(timeout=.001)
                    self.assertFalse(status['close_ack'])
                    self.assertTrue(status['writer_alive'])
                    self.assertIn('timeout',status['failure'])
                    release.set()
                    writer.thread.join(2)
            finally:
                release.set()
                writer.thread.join(2)

    def test_writer_exception_has_no_false_close_ack(self):
        from pil.gate1_log import QueuedEventWriter
        with tempfile.TemporaryDirectory() as tmp:
            writer=QueuedEventWriter(Path(tmp)/'events.csv', start=False)
            writer.record('one', 1, {})
            with patch.object(writer, '_emit', side_effect=OSError('test write failure')):
                status=writer.finish()
            self.assertFalse(status['close_ack'])
            self.assertIn('test write failure', status['worker_error'])


if __name__ == '__main__':
    unittest.main()
