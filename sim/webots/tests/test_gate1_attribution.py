"""Offline C1 measurement tests; no live I/O or timing-limit changes."""
import csv
import gc
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pil.clock import ClockBounds, Timeline, Interval
from pil.transport import HostTransport, TransportFault
from pil.gate1 import neutral_payload, run_soak
from test_gate1 import FakeClock, FakePair


class AttributionTests(unittest.TestCase):
    def test_ring_is_numeric_bounded_tail_and_has_explicit_loss_count(self):
        from pil.gate1_attribution import AttributionRing
        clock = FakeClock()
        ring = AttributionRing(clock, capacity=3)
        for i in range(5):
            ring.mark('loop', i, i)
        self.assertEqual(len(ring.data), 3*5)
        self.assertEqual(ring.overwritten, 2)
        with tempfile.TemporaryDirectory() as tmp:
            ring.finish(Path(tmp))
            rows = list(csv.DictReader((Path(tmp)/'attribution.csv').read_text().splitlines()))
            self.assertEqual([int(x['value']) for x in rows], [2, 3, 4])
            meta = json.loads((Path(tmp)/'attribution.json').read_text())
            self.assertEqual(meta['overwritten_rows'], 2)
            self.assertIn('observer_cost', meta)

    def test_measurement_has_sleep_gc_serialization_flush_stamps_after_stop(self):
        clock = FakeClock()
        pair = FakePair(clock)
        callbacks = tuple(gc.callbacks)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            result = run_soak(pair, out, clock=clock, duration_us=40000,
                              session=123, measure_io=True)
            self.assertEqual(result['verdict'], 'INCOMPLETE')
            self.assertTrue(pair.closed)
            rows = list(csv.DictReader((out/'attribution.csv').read_text().splitlines()))
            names = {x['kind'] for x in rows}
            self.assertTrue({'sleep_enter', 'wake', 'serialize_enter',
                             'serialize_exit', 'flush_enter', 'flush_exit'} <= names)
            meta = json.loads((out/'attribution.json').read_text())
            self.assertTrue(meta['serialized_after_serial_close'])
        self.assertEqual(tuple(gc.callbacks), callbacks)

    def phase_host(self, lower=996011, upper=996488):
        host = HostTransport(123)
        host.state = 'ARMED'
        host.timeline = Timeline(1000000)
        host.clock = ClockBounds()
        host.clock.observations = 8
        host.clock.interval = Interval(lower-10000, upper-10000)
        host.clock.updated_pc_us = 10000
        host.clock._rho = 100
        host.clock._lower_intercept = (lower-10000)*1000000+100*10000
        host.clock._upper_intercept = (upper-10000)*1000000-100*10000
        return host

    def test_rejected_capture_point_survives_fault_and_is_not_transmitted(self):
        # Half238.5, midpoint error249.5, +1 makes phase250.5.
        host = self.phase_host()
        with self.assertRaisesRegex(TransportFault, 'source phase'):
            host.sensor(neutral_payload(0), 0, 10000, capture_end_pc_us=10000)
        rejection = host.capture_rejection
        self.assertEqual(rejection['phase_us'], 250.5)
        self.assertEqual(rejection['excess_us'], .5)
        self.assertEqual(rejection['uncertainty_us'], 238.5)
        self.assertEqual(host.next_sensor, 0)
        self.assertFalse(host.tx)
        self.assertEqual(host.state, 'FAULT')

    def test_exact250_is_accepted_and_early250point5_rejected(self):
        host = self.phase_host(996010, 996488)
        host.sensor(neutral_payload(0), 0, 10000)
        self.assertEqual(host.next_sensor, 1)
        host = self.phase_host(995511, 995988)
        with self.assertRaises(TransportFault):
            host.sensor(neutral_payload(0), 0, 10000)
        self.assertEqual(host.capture_rejection['start_phase_us'], 250.5)
        self.assertEqual(host.capture_rejection['end_phase_us'], 249.5)
        self.assertEqual(host.capture_rejection['excess_us'], .5)

    def test_reserved_rejection_survives_full_ordinary_event_queue(self):
        host = self.phase_host()
        host.events = [{'event': 'occupied'} for _ in range(host.EVENT_LIMIT)]
        with self.assertRaisesRegex(TransportFault, 'source phase'):
            host.sensor(neutral_payload(0), 0, 10000)
        self.assertEqual(host.capture_rejection['excess_us'], .5)
        self.assertEqual(host.failure, 'source phase error exceeds250us')
        self.assertFalse(host.tx)

    def test_unqualified_clock_does_not_fabricate_phase(self):
        host = self.phase_host()
        host.clock.observations = 7
        with self.assertRaisesRegex(TransportFault, 'not qualified'):
            host.sensor(neutral_payload(0), 0, 10000)
        self.assertIsNone(host.capture_rejection['phase_us'])

    def test_gc_callback_rows_and_detachment_on_failed_close(self):
        from pil.gate1_attribution import AttributionRing
        ring = AttributionRing(FakeClock(), capacity=10)
        ring.callback('start', {'generation': 2})
        ring.callback('stop', {'generation': 2})
        self.assertEqual([ring.KINDS[ring.data[i]] for i in (1, 6)], ['gc_start', 'gc_stop'])
        clock = FakeClock()
        pair = FakePair(clock)
        callbacks = tuple(gc.callbacks)
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(pair, 'close', side_effect=OSError('test close')):
                summary = run_soak(pair, Path(tmp)/'run', clock=clock,
                                   session=123, duration_us=1000, measure_io=True)
            self.assertIn('serial close failed', summary['failure'])
            meta = json.loads((Path(tmp)/'run/attribution.json').read_text())
            self.assertFalse(meta['serialized_after_serial_close'])
        self.assertEqual(tuple(gc.callbacks), callbacks)

    def test_rejection_is_written_only_after_best_effort_disarm_and_close(self):
        clock = FakeClock()
        pair = FakePair(clock)
        original = neutral_payload
        def late_payload(source):
            clock.now += 251
            return original(source)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'run'
            with patch('pil.gate1.neutral_payload', side_effect=late_payload):
                result = run_soak(pair, out, clock=clock, duration_us=40000,
                                  session=123, measure_io=True)
            self.assertEqual(result['failure'], 'source phase error exceeds250us')
            rows = list(csv.DictReader((out/'events.csv').read_text().splitlines()))
            names = [x['event'] for x in rows]
            self.assertLess(names.index('best_effort_disarm'), names.index('source_capture_rejected'))
            r=json.loads(rows[names.index('source_capture_rejected')]['details_json'])
            self.assertGreater(r['excess_us'], 0)
            self.assertTrue(r['serial_closed_before_log'])
            self.assertFalse(pair.sensors)


if __name__ == '__main__':
    unittest.main()
