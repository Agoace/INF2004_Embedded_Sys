"""No devices: a fake nonblocking byte pair and scripted frozen-v1 peer."""

import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from pil.gate1 import Histogram, run_soak
from pil.serial_io import SerialError, discover
from pil.wire import CAPABILITIES, NO_ACK, SCHEMA_ID, Frame, StreamDecoder, Type, encode


class FakeClock:
    def __init__(self, start=10000):
        self.now = start

    def now_us(self):
        return self.now

    def sleep_us(self, delay):
        self.now += delay


class FakePair:
    """Partial write/read pair. Peer time runs independently of source step."""
    def __init__(self, clock, *, write_limit=130, read_limit=4096, fault=None,
                 capabilities=CAPABILITIES):
        self.clock, self.write_limit, self.read_limit = clock, write_limit, read_limit
        self.fault = fault
        self.closed, self.decoder, self.pending = False, StreamDecoder(), []
        self.session, self.epoch, self.armed, self.output_seq = 0, None, False, 0
        self.next_output, self.sensors = None, {}
        self.sequence = {}
        self.sent_frames = []
        self.capabilities, self.input_resets = capabilities, 0
        self.next_boot = clock.now + 100000
        self.queue(Type.BOOT, struct.pack('<4I', 7, capabilities, 921600, SCHEMA_ID))
        if fault == 'stale_open':
            self.pending[0][1][:] = bytes.fromhex((Path(__file__).parent /
                'fixtures/pil_gate1_attempt3_stale_rx.txt').read_text())

    def queue(self, kind, payload, ack=NO_ACK, source=None):
        seq = self.sequence.get(kind, 0)
        if kind == Type.BOOT and self.fault == 'boot_gap' and seq == 4:
            seq += 1
        self.sequence[kind] = seq + 1
        frame = Frame(kind, 0 if kind == Type.BOOT else self.session, seq,
                      (self.clock.now if source is None else source) & 0xffffffff,
                      ack, payload)
        packet = encode(frame)
        if kind == Type.BOOT and seq == 3:
            if self.fault == 'bootstrap_noise':
                packet = b'\xff\xff\xff' + packet[:8] + b'\0' + packet
            elif self.fault == 'post_boot_noise':
                packet += b'\x02x\0'
        self.pending.append([self.clock.now + 100, bytearray(packet)])

    def reset_input_buffer(self):
        self.input_resets += 1
        self.pending = [row for row in self.pending if row[0] > self.clock.now]

    def write(self, data):
        if self.fault == 'unplug' and self.clock.now > 540000:
            raise OSError('fake unplug')
        if self.fault == 'txstall' and self.clock.now > 540000:
            return 0
        count = min(len(data), self.write_limit)
        for frame in self.decoder.feed(data[:count]):
            self.sent_frames.append(frame)
            if frame.type == Type.HELLO:
                self.session = frame.session
                self.epoch = ((self.clock.now + 100000 + 999) // 1000) * 1000
                self.queue(Type.HELLO_ACK, frame.payload + struct.pack('<I', self.epoch & 0xffffffff),
                           frame.sequence)
                self.next_output = self.clock.now + 1000
            elif frame.type == Type.SYNC_REQ:
                self.queue(Type.SYNC_REPLY, frame.payload + struct.pack('<II', self.clock.now & 0xffffffff, 0),
                           frame.sequence)
            elif frame.type == Type.ARM:
                self.armed = True
            elif frame.type == Type.SENSOR:
                self.sensors[frame.sequence] = frame
            elif frame.type == Type.DISARM:
                self.next_output = None
                payload = struct.pack('<hhHBBHBBIIIHHHH', 0, 0, 1500, 0, 6,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                self.queue(Type.OUTPUT, payload)
        return count

    def read(self, maximum):
        if not self.armed and self.clock.now >= self.next_boot:
            self.queue(Type.BOOT, struct.pack('<4I', 7, self.capabilities, 921600, SCHEMA_ID))
            self.next_boot = self.clock.now + 100000
        if self.fault == 'pause' and self.clock.now > 540000:
            self.clock.now += 6000
        if self.next_output is not None and self.clock.now >= self.next_output:
            tick = self.next_output
            self.next_output += 1000
            ack = min((tick - self.epoch) // 1000, max(self.sensors, default=-1))
            active = self.armed and tick >= self.epoch and ack >= 0
            if active and self.fault == 'loss' and self.output_seq == 120:
                self.sequence[Type.OUTPUT] += 1
            flags = 1 if active else 20 if self.armed else 4
            payload = struct.pack('<hhHBBHBBIIIHHHH', 0, 0, 1500, 0, flags,
                                  0, min(8, len(self.sensors)), 0, 0, 0, 0, 0, 0, 0, 0)
            if not (self.fault == 'stall' and tick > 540000):
                if self.fault == 'reset' and tick > 540000:
                    self.queue(Type.BOOT, struct.pack('<4I', 8, CAPABILITIES, 921600, SCHEMA_ID))
                else:
                    if self.fault == 'duplicate' and tick > 540000:
                        self.sequence[Type.OUTPUT] -= 1
                    saved_session = self.session
                    if self.fault == 'session' and tick > 540000:
                        self.session += 1
                    self.queue(Type.OUTPUT, payload, ack if active else NO_ACK, tick)
                    self.session = saved_session
                if self.fault == 'crc' and active:
                    self.pending[-1][1][-3] ^= 1
            self.output_seq += 1
        if self.pending and self.pending[0][0] <= self.clock.now:
            packet = self.pending[0][1]
            size = min(maximum, self.read_limit, len(packet))
            result = bytes(packet[:size])
            del packet[:size]
            if not packet:
                self.pending.pop(0)
            return result
        return b''

    def close(self):
        self.closed = True


class Gate1Tests(unittest.TestCase):
    def run_fake(self, *, clock_start=10000, transport_only=False, **options):
        clock = FakeClock(clock_start)
        pair = FakePair(clock, **options)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'run'
            summary = run_soak(pair, path, clock=clock, duration_us=40000, session=123,
                               transport_only=transport_only)
            self.assertEqual(summary, json.loads((path / 'summary.json').read_text()))
            events = (path / 'events.csv').read_text()
            if summary['verdict'] == 'INCOMPLETE':
                from pil.gate1_diagnostics import load_pc_evidence
                retained = load_pc_evidence(path)
                self.assertTrue(retained['source_limits_verified'])
                self.assertTrue(retained['active_output_prefix_complete'])
                self.assertGreater(retained['active_output_count'], 0)
        self.assertTrue(pair.closed)
        if pair.write_limit >= 130 and pair.fault not in ('unplug', 'txstall'):
            self.assertEqual(pair.sent_frames[-1].type, Type.DISARM)
        return summary, events, pair

    def test_fake_stream_is_incomplete_never_physical_pass(self):
        summary, events, pair = self.run_fake()
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)
        self.assertFalse(summary['physical_gate1_pass'])
        self.assertGreaterEqual(summary['elapsed_us'], 40000)
        self.assertGreater(summary['metrics_us']['output_age_upper_us']['count'], 0)
        self.assertIn('output_observed_no_physics', events)
        self.assertIn('stopped_status_observed', events)
        self.assertTrue(summary['stopped_status_observed'])
        self.assertNotIn('physics_applied', events)
        self.assertTrue(all(f.ack == NO_ACK for f in pair.sent_frames if f.type == Type.SENSOR))

    def test_transport_only_capability_is_explicit(self):
        from pil.transport import HostTransport, TransportFault
        boot = Frame(Type.BOOT, 0, 0, 10000, NO_ACK,
                     struct.pack('<4I', 7, 4, 921600, SCHEMA_ID))
        normal = HostTransport(123)
        normal.receive(encode(boot), 10000)
        with self.assertRaises(TransportFault):
            normal.hello(10100)
        gate = HostTransport(123, transport_only=True)
        gate.receive(encode(boot), 10000)
        self.assertEqual(struct.unpack_from('<I', gate.hello(10100).payload, 8)[0], 4)

    def test_explicit_cap4_fake_soak(self):
        summary, _, pair = self.run_fake(transport_only=True, capabilities=4)
        self.assertEqual(summary['verdict'], 'INCOMPLETE')
        self.assertEqual(summary['required_capabilities'], 4)
        hello = next(f for f in pair.sent_frames if f.type == Type.HELLO)
        self.assertEqual(struct.unpack_from('<I', hello.payload, 8)[0], 4)

    def test_attempt3_stale_probe_bytes_cannot_select_boot_or_trigger_hello(self):
        summary, events, pair = self.run_fake(fault='stale_open')
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)
        self.assertEqual(pair.input_resets, 1)
        self.assertGreaterEqual(summary['startup_discard_us'], 300000)
        self.assertGreater(summary['startup_discarded_bytes'], 100)
        self.assertEqual(summary['bootstrap_first_valid_boot_seq'], 3)
        self.assertGreaterEqual(summary['bootstrap_consecutive_boots'], 2)
        hello = next(f for f in pair.sent_frames if f.type == Type.HELLO)
        self.assertEqual(struct.unpack_from('<I', hello.payload)[0], 7)
        self.assertIn('startup_discard', events)

    def test_boot_sequence_gap_requires_a_later_consecutive_pair(self):
        summary, _, _ = self.run_fake(fault='boot_gap')
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)
        self.assertEqual(summary['bootstrap_first_valid_boot_seq'], 3)
        self.assertEqual(summary['bootstrap_last_valid_boot_seq'], 6)
        self.assertEqual(summary['bootstrap_consecutive_boots'], 2)

    def test_startup_ff_and_partial_frame_resync_before_first_valid_boot(self):
        summary, events, _ = self.run_fake(fault='bootstrap_noise')
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)
        self.assertEqual(summary['bootstrap_discarded_bad_frames'], 1)
        self.assertEqual(summary['decoder_total_bad_frames'], 1)
        self.assertEqual(summary['decoder_bad_frames'], 0)
        self.assertIn('bootstrap_discard', events)
        self.assertTrue(summary['stopped_status_observed'])

    def test_corrupt_packet_after_boot_in_same_chunk_still_fails(self):
        summary, _, _ = self.run_fake(fault='post_boot_noise')
        self.assertEqual(summary['verdict'], 'FAIL')
        self.assertEqual(summary['bootstrap_discarded_bad_frames'], 0)
        self.assertEqual(summary['decoder_bad_frames'], 1)

    def test_fragmented_reads_and_partial_writes(self):
        summary, _, _ = self.run_fake(write_limit=17, read_limit=19)
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)

    def test_pico_timer_wrap_does_not_move_fixed_epoch(self):
        summary, _, _ = self.run_fake(clock_start=0xffffffff - 450000)
        self.assertEqual(summary['verdict'], 'INCOMPLETE', summary)
        self.assertGreater(summary['identity']['epoch_pico_us'], 0xffffffff)

    def test_measured_faults_stop_and_fail(self):
        for fault in ('unplug', 'pause', 'stall', 'crc', 'loss', 'reset',
                      'duplicate', 'session', 'txstall'):
            with self.subTest(fault=fault):
                summary, _, _ = self.run_fake(fault=fault)
                self.assertEqual(summary['verdict'], 'FAIL', summary)
                self.assertTrue(summary['failure'])
                if fault == 'crc':
                    self.assertGreater(summary['decoder_crc_errors'], 0)

    def test_nearest_rank_metrics_include_max(self):
        hist = Histogram()
        for n in range(1, 101):
            hist.add(n)
        self.assertEqual(hist.summary(), {'count': 100, 'p50': 50, 'p95': 95,
                                         'p99': 99, 'max': 100})

    def test_input_reset_flushes_input_only_without_device_access(self):
        from pil.serial_io import ProbeSerial
        import termios
        serial = ProbeSerial.__new__(ProbeSerial)
        serial.fd = 123
        with patch('pil.serial_io.termios.tcflush') as flush:
            serial.reset_input_buffer()
        flush.assert_called_once_with(123, termios.TCIFLUSH)

    def test_discovery_only_by_id_and_ambiguity_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            target = directory / 'fake-device'
            target.touch()
            first = directory / 'usb-Debug_Probe_A'
            first.symlink_to(target)
            self.assertEqual(discover(directory=directory), first)
            second = directory / 'usb-Debug_Probe_B'
            second.symlink_to(target)
            with self.assertRaises(SerialError):
                discover(directory=directory)
            self.assertEqual(discover(first, directory=directory), first)
            with self.assertRaises(SerialError):
                discover('/dev/ttyACM0', directory=directory)

    def test_cli_requires_permission_before_discovery(self):
        import runpy
        path = Path(__file__).parents[1] / 'tools/gate1_soak.py'
        with patch('sys.argv', [str(path), '--output', 'never-created']), \
                patch('pil.serial_io.discover', side_effect=AssertionError('device access')), \
                self.assertRaises(SystemExit) as error:
            runpy.run_path(str(path), run_name='__main__')
        self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
