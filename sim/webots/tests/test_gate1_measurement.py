import csv
import io
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from pil.gate1_rx_timing import parse_rx1, RXTimingError, elapsed
from pil.gate1_measurement import read_usb_pcap, realtime_to_raw, correlate
from pil.serial_io import ProbeSerial, SerialError
from pil.wire import Frame, Type, encode, crc32c
from pil.gate1_diagnostics import decode_dump
from test_gate1 import FakeClock, FakePair
from pil.gate1 import run_soak

FIXTURE = Path(__file__).parent / 'fixtures/pil_gate1_rx1_synthetic.txt'


def pcap(events, link=220, endian='<', nano=False):
    output = bytearray(struct.pack(endian+'IHHIIII', 0xa1b23c4d if nano else 0xa1b2c3d4,
                                  2, 4, 0, 0, 262144, link))
    for tag, kind, ep, sec, usec, status, length, payload in events:
        header = bytearray(64 if link == 220 else 48)
        struct.pack_into(endian+'QBBBBHBBqiiII', header, 0, tag, ord(kind), 3, ep, 7, 2, 0,
                         0 if payload else ord('>'), sec, usec, status, length, len(payload))
        packet = header + payload
        output.extend(struct.pack(endian+'IIII', sec, usec*1000 if nano else usec, len(packet), len(packet)))
        output.extend(packet)
    return bytes(output)


class MeasurementTests(unittest.TestCase):
    def test_independent_c_rx1_wrap_tail_and_crc(self):
        result = parse_rx1(FIXTURE.read_bytes())
        self.assertEqual(len(result['packets']), 80)
        self.assertEqual(len(result['alarms']), 16)
        self.assertEqual(result['header'][7], 6)
        self.assertEqual(result['packets'][0][0], 7)
        self.assertFalse(result['d1_identity_verified'])

    def test_bad_partial_duplicate_rx1_never_qualifies(self):
        data = FIXTURE.read_bytes()
        for bad in (data[:-15], data + data, data.replace(b'd9161656', b'd9161657'),
                    data.replace(b'P 00000007', b'P 00000008', 1)):
            with self.subTest(size=len(bad)), self.assertRaises(RXTimingError):
                parse_rx1(bad)

    def test_full_usb_pcap_64_and_48_byte_headers(self):
        events = [(5, 'S', 2, 100, 10, -115, 66, b'a'*66),
                  (5, 'C', 2, 100, 20, 0, 66, b''),
                  (6, 'C', 0x83, 100, 30, 0, 42, b'b'*42)]
        for link in (189, 220):
            for endian in ('<', '>'):
                for nano in (False, True):
                    rows = list(read_usb_pcap(io.BytesIO(pcap(events, link, endian, nano))))
                    self.assertEqual(len(rows), 3)
                    self.assertEqual(rows[0]['payload'], b'a'*66)
                    self.assertEqual(rows[2]['usb_realtime_us'], 100000030)
                    self.assertEqual(rows[2]['endpoint'], 0x83)

    def test_truncated_or_wrong_pcap_rejected(self):
        valid = pcap([(5, 'S', 2, 100, 10, -115, 34, b'x'*34)])
        for bad in (valid[:-1], b'pcapng', pcap([], 1)):
            with self.assertRaises(ValueError):
                list(read_usb_pcap(io.BytesIO(bad)))

    def test_clock_mapping_is_bracket_not_point_and_rejects_step(self):
        pairs = [{'realtime_us': 1000, 'raw_before_us': 100, 'raw_after_us': 102},
                 {'realtime_us': 2000, 'raw_before_us': 1099, 'raw_after_us': 1102}]
        self.assertEqual(realtime_to_raw(1500, pairs), (99, 1103))
        self.assertIsNone(realtime_to_raw(999, pairs))
        pairs[1]['realtime_us'] = 999
        self.assertIsNone(realtime_to_raw(1500, pairs))

    def test_adapter_stamps_select_and_actual_read_return(self):
        serial = ProbeSerial.__new__(ProbeSerial)
        serial.fd = 123
        ticks = iter(range(10, 20))
        with patch('pil.serial_io.select.select', return_value=([123], [], [])), \
                patch('pil.serial_io.os.read', return_value=b'abc'):
            data, timing = serial.read_timed(4096, lambda: next(ticks))
        self.assertEqual(data, b'abc')
        self.assertLessEqual(timing['select_enter_pc_us'], timing['select_return_pc_us'])
        self.assertLessEqual(timing['read_enter_pc_us'], timing['read_return_pc_us'])

    def test_measured_worker_preserves_wire_and_missing_urb_is_unknown(self):
        clock = FakeClock()
        pair = FakePair(clock, capabilities=4)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'pc'
            result = run_soak(pair, path, clock=clock, duration_us=40000,
                              session=123, transport_only=True, measure_io=True)
            self.assertEqual(result['verdict'], 'INCOMPLETE', result)
            report = correlate(path)
            self.assertEqual(report['urb_layer'], 'UNKNOWN')
            self.assertIn('not supplied', report['urb_reason'])
            self.assertGreaterEqual(len(report['sync']), 8)
            self.assertFalse(report['physical_gate1_pass'])
            events = (path/'events.csv').read_text()
            self.assertIn('io_write_timing', events)
            self.assertIn('io_read_timing', events)
            self.assertIn('transport_receive_timing', events)
            for row in report['sync']:
                self.assertGreaterEqual(row['rtt_us'], 0)
                self.assertGreaterEqual(row['pico_residence_us'], 0)

    def test_reserved_rx_type_rejected_even_with_correct_crc(self):
        with self.assertRaises(RXTimingError):
            parse_rx1(_rx1_changed({(1, 8): 9}))

    def test_pending_decode_is_not_invalid_result_ff(self):
        pending = {(1, i): 0xffffffff for i in (3, 4, 5, 6, 7, 8)}
        pending[(1, 9)] = parse_rx1(FIXTURE.read_bytes())['packets'][0][9] | 0xf0000
        record = parse_rx1(_rx1_changed(pending))['packets'][0]
        self.assertEqual(record[3], 0xffffffff)
        invalid = {(1, 6): 0xffffffff, (1, 7): 0xffffffff, (1, 8): 0xffff}
        record = parse_rx1(_rx1_changed(invalid))['packets'][0]
        self.assertNotEqual(record[3], 0xffffffff)
        self.assertEqual((record[8] >> 8) & 255, 255)

    def test_reserved_aux_counts_serial_and_alarm_order(self):
        parsed = parse_rx1(FIXTURE.read_bytes())
        aux = parsed['packets'][0][9]
        cases = ({(0, 4): 65}, {(1, 9): aux | (4 << 24)},
                 {(1, 9): (aux & ~0xf00) | 0x900}, {(1, 8): 0x101},
                 {(2, 0): parsed['packets'][0][0]}, {(81, 3): 4},
                 {(82, 0): parsed['alarms'][0][0]-1})
        for changed in cases:
            with self.subTest(changed=changed), self.assertRaises(RXTimingError):
                parse_rx1(_rx1_changed(changed))

    def test_rx1_timer_wrap_preserves_canonical_order(self):
        stamps = (0xfffffff0, 0xfffffff4, 0xfffffff8, 2, 4)
        record = parse_rx1(_rx1_changed({(1, i+1): t for i, t in enumerate(stamps)}))['packets'][0]
        self.assertEqual([elapsed(b, a) for a, b in zip(record[1:5], record[2:6])], [4, 4, 10, 2])

    def test_actual_c_dumps_and_missing_rx_preserve_underflow(self):
        for name, packets in (('underflow', 4), ('clean', 14)):
            data = bytes.fromhex((FIXTURE.parent/f'pil_gate1_rx1_{name}.hex.txt').read_text())
            d1 = decode_dump(data)
            rx1 = parse_rx1(data, d1)
            self.assertTrue(rx1['d1_identity_verified'])
            self.assertEqual(len(rx1['packets']), packets)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            _mini_pc(path)
            fixture = path/'stopped.bin'
            fixture.write_bytes(bytes.fromhex((FIXTURE.parent/'pil_gate1_rx1_underflow.hex.txt').read_text()))
            result = correlate(path, stopped=fixture)
            self.assertEqual(result['diagnostic'], 'COMPLETE', result)
            self.assertTrue(result['measured_device_fault'])
            self.assertEqual(result['stopped_d1_reason'], 0x100)
            bad = path/'partial.bin'
            bad.write_bytes(fixture.read_bytes().split(b'[PILG1RX1')[0])
            result = correlate(path, stopped=bad)
            self.assertEqual(result['diagnostic'], 'INCOMPLETE')
            self.assertEqual(result['stopped_d1_reason'], 0x100)
            self.assertEqual(result['original_pc_verdict'], 'FAIL')

    def test_full_urb_split_frames_id_reuse_and_wrong_device(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            events = _mini_pc(path)
            capture = path/'usb.pcap'
            capture.write_bytes(pcap(events))
            report = correlate(path, pcap=capture, usb_bus=2, usb_device=7, out_ep=2, in_ep=0x83)
            self.assertEqual(report['urb_layer'], 'PARTIAL', report)
            sync = report['sync'][0]
            self.assertEqual(sync['rtt_us'], 2000)
            self.assertTrue(sync['tx_urb']['byte_exact_coverage'])
            self.assertTrue(sync['rx_urb']['byte_exact_coverage'])
            self.assertEqual(len(sync['tx_urb']['parts']), 2)
            self.assertTrue(all(part['status'] == 0 for part in sync['tx_urb']['parts']))
            self.assertIsNotNone(sync['rx_urb']['complete_raw_interval_us'])
            self.assertIsNotNone(sync['tx_urb']['submit_raw_interval_us'])
            wrong = correlate(path, pcap=capture, usb_bus=2, usb_device=8, out_ep=2, in_ep=0x83)
            self.assertEqual(wrong['urb_layer'], 'UNKNOWN')

    def test_short_out_completion_does_not_prove_complete_frame(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            events = _mini_pc(path)
            row = list(events[3])
            row[6] -= 1  # success status, but one byte short
            events[3] = tuple(row)
            capture = path/'short.pcap'
            capture.write_bytes(pcap(events))
            report = correlate(path, pcap=capture, usb_bus=2, usb_device=7, out_ep=2, in_ep=0x83)
            urb = report['sync'][0]['tx_urb']
            self.assertTrue(urb['byte_exact_coverage'])  # submission bytes only
            self.assertFalse(urb['usb_completion_success'])
            self.assertIsNone(urb['complete_raw_interval_us'])

    def test_native_sync_wrap_does_not_treat_valid_ffffffff_as_unset(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            _mini_pc(path)
            with (path/'events.csv').open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                if row['event'] == 'raw_rx':
                    frame = Frame(Type.SYNC_REPLY, 456, 5, 3, 17,
                                  struct.pack('<QII', 1000000, 0xffffffff, 0))
                    row['details_json'] = json.dumps({'packet_hex': encode(frame).hex()})
            with (path/'events.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, ('event', 'pc_us', 'details_json'))
                writer.writeheader()
                writer.writerows(rows)
            self.assertEqual(correlate(path)['sync'][0]['pico_residence_us'], 4)

    def test_sensor_one_way_is_interval_and_requires_raw_d1_proof(self):
        from pil.gate1 import neutral_payload
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            events = _mini_pc(path)
            request_len = len(events[0][-1]) + len(events[2][-1])
            sensor = encode(Frame(Type.SENSOR, 456, 0, 0, 0xffffffff, neutral_payload(0)))
            with (path/'events.csv').open('a', newline='') as stream:
                writer = csv.writer(stream)
                for event, at, details in (
                        ('source_clock_bounds', 1000100, {'source_us': 0, 'capture_end_pc_us': 1000100,
                                                         'lower_pico_us': 2597000, 'upper_pico_us': 2597041}),
                        ('raw_pc_write', 1000200, {'packet_hex': sensor.hex()}),
                        ('io_write_timing', 1000200, {'call_enter_pc_us': 1000190, 'bytes': len(sensor),
                                                    'stream_start': request_len, 'stream_end': request_len+len(sensor)})):
                    writer.writerow((event, at, json.dumps(details)))
            stopped = path/'stopped.bin'
            stopped.write_bytes(bytes.fromhex((FIXTURE.parent/'pil_gate1_rx1_underflow.hex.txt').read_text()))
            report = correlate(path, stopped=stopped)
            row = next(p for p in report['packets'] if p['type'] == 1 and p['seq'] == 0)
            self.assertEqual(row['pc_write_return_to_pico_rx_interval_us'], [927, 972])
            self.assertTrue(report['cross_clock_bounds_eligible'])
            self.assertEqual(report['measurement_verdict'], 'FAIL')
            csv_path = path/'events.csv'
            csv_path.write_text(csv_path.read_text().replace('CLOCK_MONOTONIC_RAW', 'CLOCK_MONOTONIC'))
            report = correlate(path, stopped=stopped)
            self.assertFalse(report['cross_clock_bounds_eligible'])
            self.assertTrue(all(p['pc_write_return_to_pico_rx_interval_us'] is None for p in report['packets']))

    def test_absent_pcap_and_timestamp_mismatch_stay_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            events = _mini_pc(path)
            report = correlate(path, pcap=path/'absent.pcap', usb_bus=2, usb_device=7, out_ep=2, in_ep=0x83)
            self.assertEqual(report['urb_layer'], 'UNKNOWN')
            data = bytearray(pcap(events))
            struct.pack_into('<I', data, 24+4, events[0][4]+10)
            capture = path/'badclock.pcap'
            capture.write_bytes(data)
            report = correlate(path, pcap=capture, usb_bus=2, usb_device=7, out_ep=2, in_ep=0x83)
            self.assertEqual(report['urb_layer'], 'UNKNOWN')
            self.assertIn('timestamp mismatch', report['urb_reason'])
            self.assertFalse(report['sync'][0]['tx_urb']['byte_exact_coverage'])

    def test_codec_failure_retains_processing_timestamp(self):
        clock = FakeClock()
        pair = FakePair(clock, capabilities=4, fault='crc')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'pc'
            result = run_soak(pair, path, clock=clock, duration_us=40000,
                              session=123, transport_only=True, measure_io=True)
            self.assertEqual(result['verdict'], 'FAIL')
            self.assertIn('transport_receive_failed_timing', (path/'events.csv').read_text())

    def test_offline_inputs_never_open_fifo_or_device_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            _mini_pc(path)
            fifo = path/'not-a-capture.pipe'
            os.mkfifo(fifo)
            original = Path.open
            def guarded(file, *args, **kwargs):
                if file == fifo:
                    self.fail('offline tool tried to open a non-regular input')
                return original(file, *args, **kwargs)
            with patch.object(Path, 'open', guarded):
                report = correlate(path, pcap=fifo, stopped=fifo,
                                   usb_bus=2, usb_device=7, out_ep=2, in_ep=0x83)
            self.assertEqual(report['urb_layer'], 'UNKNOWN')
            self.assertEqual(report['diagnostic'], 'INCOMPLETE')
            self.assertIn('regular', report['diagnostic_error'])

    def test_rx1_order_and_missing_stack_stay_incomplete(self):
        data = bytes.fromhex((FIXTURE.parent/'pil_gate1_rx1_underflow.hex.txt').read_text())
        dump = decode_dump(data)
        begin = data.index(b'[PILG1RX1 BEGIN ')
        stack = data.index(b'[PILG1] stopped;')
        with self.assertRaises(RXTimingError):
            parse_rx1(data[begin:stack]+data[:begin]+data[stack:], dump)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            _mini_pc(path)
            stopped = path/'no-stack.bin'
            stopped.write_bytes(data[:stack])
            report = correlate(path, stopped=stopped)
            self.assertEqual(report['diagnostic'], 'INCOMPLETE')
            self.assertFalse(report['stopped_stack_line_complete'])
            self.assertEqual(report['measurement_verdict'], 'FAIL')

    def test_timed_adapter_eagain_and_disconnect_match_original(self):
        serial = ProbeSerial.__new__(ProbeSerial)
        serial.fd = 123
        with patch('pil.serial_io.select.select', return_value=([123], [], [])):
            with patch('pil.serial_io.os.read', side_effect=BlockingIOError):
                self.assertEqual(serial.read_timed(4096, lambda: 10)[0], b'')
            with patch('pil.serial_io.os.read', return_value=b''):
                with self.assertRaises(SerialError):
                    serial.read_timed(4096, lambda: 10)

    def test_instrumentation_does_not_change_wire_policy(self):
        wire = []
        for measure in (False, True):
            clock = FakeClock()
            pair = FakePair(clock, capabilities=4, write_limit=7, read_limit=9)
            with tempfile.TemporaryDirectory() as tmp:
                summary = run_soak(pair, Path(tmp)/'pc', clock=clock, duration_us=40000,
                                   session=123, transport_only=True, measure_io=measure)
                wire.append([encode(frame) for frame in pair.sent_frames])
                self.assertEqual(summary['candidate_bounds_us']['clock_halfwidth'], 250)
        self.assertEqual(wire[0], wire[1])


def _rx1_changed(changes):
    rows = []
    for line in FIXTURE.read_text().splitlines():
        fields = line[:-1].split()
        rows.append((fields[1], [int(w, 16) for w in fields[2:]]))
    for (row, column), value in changes.items():
        rows[row][1][column] = value
    words = [word for _, values in rows[:-1] for word in values]
    rows[-1][1][-1] = crc32c(struct.pack('<'+'I'*len(words), *words))
    return ''.join('[PILG1RX1 '+kind+' '+ ' '.join(f'{w:08x}' for w in values)+']\n'
                   for kind, values in rows).encode()


def _mini_pc(path):
    t1, bias = 1000000, 1000000000000
    request = encode(Frame(Type.SYNC_REQ, 456, 17, t1, 0xffffffff, struct.pack('<Q', t1)))
    reply = encode(Frame(Type.SYNC_REPLY, 456, 5, 2200123, 17, struct.pack('<QII', t1, 2200023, 0)))
    (path/'summary.json').write_text(json.dumps({'verdict': 'FAIL', 'failure': 'Pico link fault',
                                               'identity': {'host_session': 456, 'boot_nonce': 123}}))
    rows = [('run_metadata', 999980, {'pc_clock_domain': 'CLOCK_MONOTONIC_RAW'})]
    for raw in (999990, 1000025, 1001940, 1002010):
        rows.append(('measurement_clock_pair', raw+2, {'raw_before_us': raw,
                    'raw_after_us': raw+2, 'realtime_us': bias+raw+1}))
    rows.extend([('raw_pc_write', 1000020, {'packet_hex': request.hex()}),
                 ('io_write_timing', 1000020, {'call_enter_pc_us': 1000010, 'bytes': len(request),
                                              'stream_start': 0, 'stream_end': len(request)}),
                 ('io_read_timing', 1002000, {'call_enter_pc_us': 1001945, 'read_return_pc_us': 1002000,
                                             'bytes': len(reply), 'stream_start': 0, 'stream_end': len(reply)}),
                 ('raw_rx', 1002000, {'packet_hex': reply.hex()}),
                 ('transport_receive_timing', 1002010, {'received_pc_us': 1002000,
                                                        'session': 456, 'type': 8, 'seq': 5})])
    with (path/'events.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(('event', 'pc_us', 'details_json'))
        for event, at, details in rows:
            writer.writerow((event, at, json.dumps(details)))
    events = []
    for tag, kind, ep, raw, length, payload in (
            (5, 'S', 2, 1000015, 20, request[:20]),
            (5, 'C', 2, 1000025, 20, b''),
            (5, 'S', 2, 1000026, len(request)-20, request[20:]),
            (5, 'C', 2, 1000050, len(request)-20, b''),
            (7, 'C', 0x83, 1001900, 20, reply[:20]),
            (8, 'C', 0x83, 1001975, len(reply)-20, reply[20:])):
        sec, usec = divmod(bias+raw, 1000000)
        events.append((tag, kind, ep, sec, usec, -115 if kind == 'S' else 0, length, payload))
    return events


if __name__ == '__main__':
    unittest.main()
