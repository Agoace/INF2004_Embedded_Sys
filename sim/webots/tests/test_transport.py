import json
from pathlib import Path
import struct
import unittest

from pil.clock import ClockBounds, ClockError, Timeline, unwrap32
from pil.transport import HostTransport, TransportFault
from pil.wire import (CAPABILITIES, NO_ACK, SCHEMA_ID, Frame, Type, decode, due, encode)

VECTORS = json.loads((Path(__file__).parent / "fixtures/pil_wire_v1.json").read_text())["vectors"]
SESSION = 0x11223344
NONCE = 0xA1B2C3D4
OUTPUT_BASE = 99  # Valid pending snapshots cover the100ms negotiated setup lead.


def sensor_payload(source):
    payload = bytearray(bytes.fromhex(next(v["payload_hex"] for v in VECTORS
                                          if v["name"] == "sensor_zero")))
    valid = (struct.unpack_from("<H", payload, 36)[0] & ~0x60)
    valid |= (0x20 if due(source, 400) else 0) | (0x40 if due(source, 75) else 0)
    struct.pack_into("<H", payload, 36, valid)
    return bytes(payload)


def output(seq=0, timer=111000, ack=0, flags=1, compare=1250, actuator=111000,
           sequence_base=OUTPUT_BASE):
    p = bytearray(32)
    struct.pack_into("<hhHBBHBBIIIHHHH", p, 0, compare, compare, 1500, 0, flags,
                     0, 2, 0, actuator, 0, 0, 0, 0, 0, 0)
    return Frame(Type.OUTPUT, SESSION, seq + sequence_base,
                 timer & 0xFFFFFFFF, ack, bytes(p))


def qualified_clock():
    clock = ClockBounds()
    for n in range(8):
        t = n * 1000
        clock.observe(t, t + 200, t + 80, t + 100)
    return clock


def drain(host, pc_us):
    while host.peek_tx():
        host.consume_tx(len(host.peek_tx()), pc_us)


def negotiated():
    host = HostTransport(SESSION, clock=qualified_clock())
    boot = Frame(Type.BOOT, 0, 0, 10000, NO_ACK,
                 struct.pack("<IIII", NONCE, CAPABILITIES, 921600, SCHEMA_ID))
    host.receive(encode(boot), 10000)
    hello = host.hello(10100)
    drain(host, 10200)
    ack = Frame(Type.HELLO_ACK, SESSION, 0, 11000, hello.sequence,
                hello.payload + struct.pack("<I", 111000))
    host.receive(encode(ack), 11200)
    return host


def pending():
    host = negotiated()
    host.arm(11300)
    drain(host, 11400)
    return host


def streaming():
    host = pending()
    # The real TIMER/report stream continues during the100ms setup lead.
    # Pending zero outputs renew only the report lease, never ARM acceptance.
    for seq, timer in enumerate(range(12000, 111000, 1000)):
        if timer >= 107000:
            source = timer - 107000
            host.sensor(sensor_payload(source), source, timer)
            drain(host, timer + 50)
        frame = output(seq=seq, timer=timer, ack=NO_ACK, flags=20, compare=0,
                       actuator=0, sequence_base=0)
        host.receive(encode(frame), timer + 100)
    host.sensor(sensor_payload(4000), 4000, 111000)
    drain(host, 111100)
    host.receive(encode(output()), 111500)
    return host


class ClockTests(unittest.TestCase):
    def test_attempt4_real_sync_zero_minima_cannot_qualify(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/pil_gate1_attempt4_sync.json').read_text())
        clock = ClockBounds()
        for sample in fixture['samples']:
            clock.observe(*sample)
        self.assertEqual(clock.observations, 8)
        self.assertEqual(clock.bounds_at(fixture['arm_pc_us'], qualified=False).halfwidth, 583.0)
        with self.assertRaises(ClockError):
            clock.bounds_at(fixture['arm_pc_us'])

    def test_attempt4_real_sync_transport_only_uses_proven_wire_minima(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/pil_gate1_attempt4_sync.json').read_text())
        host = HostTransport(SESSION, transport_only=True)
        for seq, (tx, rx, pico_rx, pico_tx) in enumerate(fixture['samples']):
            host.sync_pending[seq] = tx
            payload = struct.pack('<QII', tx, pico_rx, 0)
            host.receive(encode(Frame(Type.SYNC_REPLY, SESSION, seq, pico_tx, seq, payload)), rx)
        self.assertEqual((host.proven_min_tx_us, host.proven_min_rx_us), (334, 444))
        self.assertEqual(host.clock.bounds_at(fixture['arm_pc_us']).halfwidth, 194.0)
        self.assertEqual(host.clock.MAX_HALFWIDTH_US, 250)
        normal = HostTransport(SESSION)
        self.assertEqual((normal.proven_min_tx_us, normal.proven_min_rx_us), (0, 0))

    def test_proven_minimum_cannot_exceed_observed_round_trip(self):
        with self.assertRaises(ClockError):
            ClockBounds().observe(10000, 10100, 20000, 20000, proven_min_tx_us=101)
        with self.assertRaises(ClockError):
            ClockBounds().observe(10000, 10100, 20000, 20000,
                                  proven_min_tx_us=60, proven_min_rx_us=41)

    def test_attempt5_all_sync_cones_round_once_not_at_every_refresh(self):
        fixture = json.loads((Path(__file__).parent / 'fixtures/pil_gate1_attempt5_sync.json').read_text())
        clock = ClockBounds()
        halves = []
        for sample in fixture['samples']:
            halves.append(clock.observe(*sample, proven_min_tx_us=334,
                                        proven_min_rx_us=444).halfwidth)
        self.assertEqual(halves, [303.5, 264, 200.5, 200.5, 200.5, 168, 168,
                                 168.5, 168.5, 178.5, 188.5, 198.5, 208.5,
                                 218.5, 228.5, 237, 241])
        self.assertEqual(clock.bounds_at(fixture['sensor658_write_pc_us']).halfwidth, 242.5)
        deadline = clock.refresh_deadline_pc_us()
        clock.bounds_at(deadline)
        with self.assertRaises(ClockError):
            clock.bounds_at(deadline + 1)

    def test_rate_can_change_after_last_sync_without_lying(self):
        clock = ClockBounds()
        for t in range(0, 8000, 1000):
            clock.observe(t, t + 878, t + 334, t + 434,
                          proven_min_tx_us=334, proven_min_rx_us=444)
        # Identical past, two legal future rates: no learned constant-rate credit.
        future = clock.bounds_at(clock.updated_pc_us + 500000)
        self.assertLessEqual(future.lower, -50)
        self.assertGreaterEqual(future.upper, 50)
        self.assertGreaterEqual(future.halfwidth, 50)

    def test_600_seconds_constant_then_changing_skew_is_covered(self):
        clock = ClockBounds()
        def offset(t):
            return (min(t, 300000000) * 20 - max(0, t - 300000000) * 80) / 1000000
        for n, t in enumerate(range(0, 600000001, 100000)):
            tx_delay, rx_delay = 400 + n % 7 * 10, 500 + n % 11 * 10
            reply_at = t + tx_delay + 100
            received = reply_at + rx_delay
            pico_rx = int(t + tx_delay + offset(t + tx_delay)) & 0xffffffff
            pico_tx = int(reply_at + offset(reply_at)) & 0xffffffff
            clock.observe(t, received, pico_rx, pico_tx,
                          proven_min_tx_us=334, proven_min_rx_us=444)
            if n >= 7:
                query = received + 50000
                bounds = clock.bounds_at(query)
                self.assertLess(bounds.halfwidth, 250)
                self.assertLessEqual(bounds.lower, offset(query))
                self.assertGreaterEqual(bounds.upper, offset(query))

    def test_timestamp_quantization_is_separate_from_drift(self):
        clock = ClockBounds()
        clock.DRIFT_PPM = 0  # Isolate quantization, never use drift as its allowance.
        bounds = clock.observe(1000, 1000, 1000, 1000)
        self.assertEqual((bounds.lower, bounds.upper), (-1, 1))
        default = ClockBounds().observe(1000, 1000, 1000, 1000)
        self.assertEqual((default.lower, default.upper), (-2, 2))

    def test_source_capture_retains_outward_bracket(self):
        host = streaming()
        frame = host.sensor(sensor_payload(5000), 5000, 111950,
                            capture_end_pc_us=112050)
        events = host.drain_events()
        capture = next(e for e in reversed(events) if e['event'] == 'source_capture')
        lower = host.clock.pico_range(111950).lower
        upper = host.clock.pico_range(112050).upper + 1
        self.assertEqual(capture['source_lower_pico_us'], lower)
        self.assertEqual(capture['source_upper_pico_us'], upper)
        self.assertEqual(frame.source_us, 5000)

    def test_eight_intersections_and_headroom(self):
        clock = qualified_clock()
        bounds = clock.bounds_at(7200)
        self.assertLessEqual(bounds.lower, 0)
        self.assertGreaterEqual(bounds.upper, 0)
        self.assertLess(bounds.halfwidth, 250)
        deadline = clock.refresh_deadline_pc_us()
        self.assertLessEqual(deadline - clock.updated_pc_us, 1_000_000)
        clock.bounds_at(deadline)
        with self.assertRaises(ClockError):
            clock.bounds_at(deadline + 10001)

    def test_full_uart_rtt_cannot_be_called_250us(self):
        clock = ClockBounds()
        for n in range(8):
            t = 1000 * n
            clock.observe(t, t + 800, t + 350, t + 450)
        self.assertGreater(clock.interval.halfwidth, 250)
        with self.assertRaises(ClockError):
            clock.pico_range(7800)

    def test_no_statistical_sqrt_reduction(self):
        clock = ClockBounds()
        for n in range(100):
            t = n * 1000
            clock.observe(t, t + 1000, t + 400, t + 600)
        self.assertGreater(clock.interval.halfwidth, 390)

    def test_clock_wrap_and_empty_intersection(self):
        clock = ClockBounds()
        near = (1 << 32) - 100
        clock.observe(0, 200, near, (near + 20) & 0xFFFFFFFF)
        self.assertEqual(clock.pico_anchor, near + 20)
        clock.observe(1000, 1200, (near + 1000) & 0xFFFFFFFF,
                      (near + 1020) & 0xFFFFFFFF)
        self.assertGreater(clock.pico_anchor, 1 << 32)
        self.assertEqual(unwrap32(10, (1 << 32) - 10), (1 << 32) + 10)
        with self.assertRaises(ClockError):
            clock.observe(2000, 2200, 200000, 200020)

    def test_capture_available_deadlines_and_fixed_epoch(self):
        clock = qualified_clock()
        timeline = Timeline(111000)
        phase, uncertainty = timeline.validate_capture(1000, 108000, clock)
        self.assertLessEqual(phase, 250)
        self.assertGreater(uncertainty, 0)
        timeline.validate_available(1000, 108000, 110000, clock)
        with self.assertRaises(ClockError):
            timeline.validate_available(1000, 108000, 111001, clock)
        with self.assertRaises(ClockError):
            timeline.validate_capture(1000, 109000, clock)
        self.assertEqual(timeline.apply(1000), 112000)
        clock.observe(108100, 108300, 108180, 108200)
        self.assertEqual(timeline.apply(1000), 112000)


class TransportTests(unittest.TestCase):
    def test_arm_pending_is_not_acceptance(self):
        host = pending()
        self.assertEqual(host.state, "ARM_PENDING")
        self.assertTrue(host.model_inhibited)
        self.assertEqual(host.next_sensor, 0)

    def test_complete_fake_flow_and_ack_only_after_physics(self):
        host = streaming()
        self.assertEqual(host.state, "ARMED")
        self.assertFalse(host.model_inhibited)
        self.assertEqual(host.applied_output, NO_ACK)
        self.assertEqual(host.next_output().sequence, OUTPUT_BASE)
        self.assertEqual(len(host.outputs), 1)  # Pending reports never queue physics output.
        host.mark_output_applied(OUTPUT_BASE, 111750)
        frame = host.sensor(sensor_payload(5000), 5000, 112000)
        self.assertEqual(frame.ack, OUTPUT_BASE)
        records = host.drain_events()
        self.assertTrue(any(r["event"] == "physics_applied" for r in records))
        self.assertTrue(all(not r["device_available"] for r in records
                            if r["event"] == "pc_write"))

    def test_partial_write_preserves_packet(self):
        host = negotiated()
        expected = encode(host.arm(11300))
        first = host.peek_tx(7)
        host.consume_tx(7, 11400)
        self.assertEqual(first + host.peek_tx(), expected)
        host.consume_tx(len(expected) - 7, 11500)
        self.assertEqual(host.peek_tx(), b"")

    def test_duplicate_output_does_not_renew_lease(self):
        host = streaming()
        host.receive(encode(output()), 115500)
        self.assertEqual(host.last_output_pc_us, 111500)
        with self.assertRaises(TransportFault):
            host.tick(116500)
        self.assertTrue(host.model_inhibited)
        self.assertIn("Pico PWM unknown", host.failure)

    def test_new_snapshot_renews_lease_not_actuator_timestamp(self):
        host = streaming()
        host.mark_output_applied(OUTPUT_BASE, 111750)
        host.receive(encode(output(seq=1, timer=115000, actuator=111000)), 115500)
        host.tick(116000)
        self.assertEqual(host.last_output_pc_us, 115500)
        self.assertFalse(host.model_inhibited)

    def test_fresh_packet_after_five_ms_gap_cannot_revive_session(self):
        host = streaming()
        with self.assertRaises(TransportFault):
            host.receive(encode(output(seq=1, timer=117000)), 117500)
        self.assertTrue(host.model_inhibited)

    def test_rx_chunk_bound_fails_closed(self):
        host = streaming()
        with self.assertRaises(TransportFault):
            host.receive(b"x" * 4097, 112000)
        self.assertTrue(host.model_inhibited)

    def test_new_sequence_same_snapshot_does_not_renew(self):
        host = streaming()
        host.receive(encode(output(seq=1, timer=111000)), 115500)
        with self.assertRaises(TransportFault):
            host.tick(116500)

    def test_progressing_but_stale_or_future_output_does_not_hide_age(self):
        # Arrive within the progress lease, so snapshot-age validation itself
        # must reject the old report (rather than the receive-gap guard).
        for timer, receive in ((111001, 116400), (120000, 112500)):
            host = streaming()
            with self.subTest(timer=timer), self.assertRaises(TransportFault):
                host.receive(encode(output(seq=1, timer=timer)), receive)
            self.assertTrue(host.model_inhibited)

    def test_ack_regression_cannot_renew_progress(self):
        host = streaming()
        host.receive(encode(output(seq=1, timer=112000, ack=1)), 112500)
        with self.assertRaises(TransportFault):
            host.receive(encode(output(seq=2, timer=113000, ack=0)), 113500)

    def test_delayed_unsent_sensor_faults_no_deadline_sliding(self):
        host = pending()
        host.sensor(sensor_payload(0), 0, 107000)
        with self.assertRaises(TransportFault):
            host.tick(110100)
        self.assertTrue(host.model_inhibited)
        self.assertEqual(host.timeline.epoch_pico_us, 111000)

    def test_missing_replayed_source_and_bad_origin_fault(self):
        for mode in ("missing", "origin", "buttons", "timing"):
            host = pending()
            p = bytearray(sensor_payload(0))
            if mode == "origin":
                struct.pack_into("<i", p, 8, 1)
            elif mode == "buttons":
                p[6] = 2
            elif mode == "timing":
                struct.pack_into("<H", p, 38, 1)
            with self.subTest(mode=mode), self.assertRaises(TransportFault):
                host.sensor(bytes(p), 1000 if mode == "missing" else 0, 107000)
            self.assertTrue(host.model_inhibited)

    def test_fault_disarm_is_delimited_and_never_rearms(self):
        host = streaming()
        packet = host.disarm_packet(2, 112000)
        self.assertEqual(packet[0], 0)
        self.assertEqual(decode(packet[1:]).type, Type.DISARM)
        self.assertTrue(host.model_inhibited)
        with self.assertRaises(TransportFault):
            host.arm(112100)

    def test_fault_flags_overflow_and_unexpected_ack(self):
        for case in ("fault", "ack", "overflow"):
            host = streaming()
            with self.subTest(case=case), self.assertRaises(TransportFault):
                if case == "fault":
                    host.receive(encode(output(seq=1, timer=112000, flags=14, compare=0)),
                                 112500)
                elif case == "ack":
                    host.receive(encode(output(seq=1, timer=112000, ack=100)), 112500)
                else:
                    for seq in range(1, 10):
                        host.receive(encode(output(seq=seq, timer=111000 + seq * 1000)),
                                     111500 + seq * 1000)
            self.assertTrue(host.model_inhibited)

    def test_reset_and_skeleton_cannot_arm(self):
        host = streaming()
        boot = Frame(Type.BOOT, 0, 0, 0, NO_ACK,
                     struct.pack("<IIII", NONCE + 1, CAPABILITIES, 921600, SCHEMA_ID))
        with self.assertRaises(TransportFault):
            host.receive(encode(boot), 112000)
        host = HostTransport(SESSION)
        boot = Frame(Type.BOOT, 0, 0, 0, NO_ACK,
                     struct.pack("<IIII", NONCE, 0, 921600, SCHEMA_ID))
        host.receive(encode(boot), 0)
        with self.assertRaises(TransportFault):
            host.hello(100)

    def test_late_setup_and_unqualified_clock_rejected(self):
        host = negotiated()
        with self.assertRaises(TransportFault):
            host.arm(100000)
        host = negotiated()
        host.clock = ClockBounds()
        with self.assertRaises(TransportFault):
            host.arm(11300)

    def test_evidence_overflow_latches_instead_of_dropping(self):
        host = negotiated()
        for i in range(host.EVENT_LIMIT):
            if len(host.events) >= host.EVENT_LIMIT:
                break
            host._record("synthetic", i=i)
        with self.assertRaises(TransportFault):
            host._record("one_more")
        self.assertTrue(host.model_inhibited)


if __name__ == "__main__":
    unittest.main()
