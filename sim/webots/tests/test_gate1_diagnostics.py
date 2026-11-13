"""D1 reference producer fixtures. No device I/O or Pico-model truth verdicts."""

import copy
import csv
import io
import json
from pathlib import Path
import random
import runpy
import struct
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from pil.gate1_diagnostics import DiagnosticError, decode_dump, evaluate, load_pc_evidence
from pil.wire import CAPABILITIES, NO_ACK, crc32c

HEADER = struct.Struct('<IHHIIII')


def packet(kind, sequence, words, *, boot=7, session=123):
    payload = struct.pack('<' + 'I' * len(words), *(word & 0xffffffff for word in words))
    body = HEADER.pack(0x31444750, 1, kind, sequence, boot, session, len(payload)) + payload
    # Independent simple COBS reference; records have no >=254-byte runs.
    encoded = b''.join(bytes([len(part) + 1]) + part for part in
                       (body + struct.pack('<I', crc32c(body))).split(b'\0'))
    return b'\0' + encoded + b'\0', body


def from_rows(rows, *, boot=7, session=123):
    packets, bodies = [], []
    for seq, (kind, words) in enumerate(rows):
        encoded, body = packet(kind, seq, words, boot=boot, session=session)
        packets.append(encoded)
        bodies.append(body)
    packets.append(packet(5, len(rows), [len(rows), 12, rows[0][1][25],
                                        crc32c(b''.join(bodies))],
                          boot=boot, session=session)[0])
    return b''.join(packets)


def fixture(*, base=110000, count=4, stage_age=1000, apply_age=4000, jitter=0):
    # CONTEXT, COUNTS, twelve EXTREMUMs, no optional trace, MANIFEST.
    context = [0x50494c01, 4, base, base, 125000000, 921600, 8, 31,
               1000, 4000, 5000, 8, 0, count-1, 0, count-1, 0, count-1,
               base, base+count*1000, 3, 6, 0x43f, 0x79, 12, 0]
    counts = [count] * 6 + [2*count, 2*count] + [0] * 18
    rows = [(1, context), (2, counts)]
    for metric in range(1, 13):
        sample_count = 2*count if metric in (9, 10) else count
        reference = base - 4000 if metric in (1, 2) else base
        if metric == 1:
            value, raw = stage_age, reference + stage_age
        elif metric == 2:
            value, raw = apply_age, reference + apply_age
        elif metric == 3:
            reference = base - 1000
            value, raw = 3000-stage_age, base-4000+stage_age
        elif metric in (4, 5, 6):
            value, raw = jitter, reference+jitter
        elif metric in (7, 8, 9, 10):
            value, raw = 2, base
        else:
            value, raw = 10, base
        if metric in (9, 10, 11, 12):
            arg, reference = NO_ACK, 0
        else:
            arg = 0
        if metric in (7, 8):
            reference = 0
        hi = raw + 10 if metric in (11, 12) else raw
        rows.append((3, [metric, sample_count, value, arg, raw, hi, reference,
                         value, arg, raw, hi, reference]))
    bodies, packets = [], []
    for seq, (kind, words) in enumerate(rows):
        encoded, body = packet(kind, seq, words)
        packets.append(encoded)
        bodies.append(body)
    manifest = [len(rows), 12, 0, crc32c(b''.join(bodies))]
    packets.append(packet(5, len(rows), manifest)[0])
    pc = {'identity': {'boot_nonce': 7, 'host_session': 123, 'epoch_pico_us': base},
          'verdict': 'INCOMPLETE', 'elapsed_us': count*1000,
          'ten_minute_duration_met': False, 'active_output_count': count,
          'active_output_prefix_complete': True,
          'sources': [{'source_us': n*1000, 'lower_pico_us': base+n*1000-4010,
                       'upper_pico_us': base+n*1000-3990} for n in range(count)]}
    return b''.join(packets), pc, rows


class D1Tests(unittest.TestCase):
    def test_full_132_byte_body_and_clean_bounds_stay_incomplete(self):
        blob, pc, _ = fixture()
        dump = decode_dump(blob)
        result = evaluate(dump, pc)
        self.assertEqual(result['verdict'], 'INCOMPLETE', result)
        self.assertEqual(result['availability_age_upper_us'], 1011)
        self.assertEqual(result['apply_age_upper_us'], 4011)
        self.assertEqual(result['availability_age_lower_us'], 990)
        self.assertIsNone(result['device_availability_percentiles'])

    def test_measured_bounds_fail_never_raise_limits(self):
        for params in ({'stage_age': 2990}, {'apply_age': 4990}, {'jitter': 250}):
            blob, pc, _ = fixture(**params)
            with self.subTest(params=params):
                result = evaluate(decode_dump(blob), pc)
                self.assertEqual(result['verdict'], 'FAIL', result)

    def test_corrupt_truncated_missing_or_reordered_dump_rejected(self):
        blob, _, _ = fixture()
        broken = bytearray(blob)
        broken[30] ^= 1
        for bad in (bytes(broken), blob[:-8], blob[135:], blob + b'\0\x02x\0'):
            with self.subTest(size=len(bad)), self.assertRaises(DiagnosticError):
                decode_dump(bad)

    def test_console_prefix_and_identical_repeat_not_mixed(self):
        blob, _, _ = fixture()
        self.assertEqual(decode_dump(b'boot text\r\n' + blob + blob), decode_dump(blob))
        other, _, _ = fixture(stage_age=1100)
        with self.assertRaises(DiagnosticError):
            decode_dump(blob + other)

    def test_changed_identity_crc_and_chain_are_not_coverage(self):
        blob, _, _ = fixture()
        altered = bytearray(blob)
        altered[-8] ^= 1
        with self.assertRaises(DiagnosticError):
            decode_dump(bytes(altered))

    def test_counts_identity_and_capture_gaps_cannot_qualify(self):
        blob, pc, _ = fixture()
        for change in ('session', 'capture', 'active_count', 'pc_fail'):
            evidence = copy.deepcopy(pc)
            if change == 'session':
                evidence['identity']['host_session'] += 1
            elif change == 'capture':
                evidence['sources'].pop(1)
            elif change == 'active_count':
                evidence['active_output_count'] -= 1
            else:
                evidence['verdict'] = 'FAIL'
                evidence['identity']['host_session'] += 1
            with self.subTest(change=change):
                result = evaluate(decode_dump(blob), evidence)
                self.assertNotEqual(result['verdict'], 'PASS')
                if change == 'pc_fail':
                    self.assertEqual(result['verdict'], 'FAIL')

    def test_wrap_uses_signed_reference_not_unsigned_subtraction(self):
        blob, pc, _ = fixture(base=0xffffffff + 1000)
        result = evaluate(decode_dump(blob), pc)
        self.assertEqual(result['availability_age_upper_us'], 1011)
        self.assertEqual(result['verdict'], 'INCOMPLETE')

    def test_prehello_zero_identity_decodes_but_cannot_qualify(self):
        _, pc, rows = fixture()
        dump = decode_dump(from_rows(rows, boot=0, session=0))
        self.assertEqual(dump.session, 0)
        self.assertEqual(evaluate(dump, pc)['verdict'], 'INCOMPLETE')

    def test_fault_snapshot_is_fail_even_without_complete_cohort(self):
        _, pc, rows = fixture()
        rows[0][1][22] |= 1 << 7
        rows[0][1][20] = 0x100
        rows[1][1][2] += 1
        self.assertEqual(evaluate(decode_dump(from_rows(rows)), pc)['verdict'], 'FAIL')

    def test_raw_reference_and_coverage_corruption_cannot_qualify(self):
        for case in ('reference', 'count', 'flags', 'cohort'):
            _, pc, rows = fixture()
            if case == 'reference':
                rows[2][1][6] += 1
            elif case == 'count':
                rows[2][1][1] -= 1
            elif case == 'flags':
                rows[0][1][22] |= 1 << 31
            else:
                rows[1][1][5] -= 1
            with self.subTest(case=case):
                if case == 'flags':
                    with self.assertRaises(DiagnosticError):
                        decode_dump(from_rows(rows))
                else:
                    self.assertEqual(evaluate(decode_dump(from_rows(rows)), pc)['verdict'], 'INCOMPLETE')

    def test_all_frame_reduction_bounds_cover_brute_force_oracle(self):
        rng = random.Random(612)
        for _ in range(40):
            base = 0xffffffff - 2000
            _, pc, rows = fixture(base=base, count=8)
            observations = []
            for n, source in enumerate(pc['sources']):
                b = base + n*1000 - 4000
                source['lower_pico_us'] = b-rng.randrange(1, 50)
                source['upper_pico_us'] = b+rng.randrange(1, 50)
                alo = b+rng.randrange(200, 2500)
                ahi = alo+rng.randrange(0, 8)
                observations.append((alo, ahi, b))
            for metric, index, minimum_key, maximum_key in (
                    (1, 2, lambda a: a[0]-a[2], lambda a: a[1]-a[2]),
                    (3, 4, lambda a: a[2]+3000-a[1], lambda a: a[2]+3000-a[0])):
                mi = min(range(8), key=lambda n: minimum_key(observations[n]))
                ma = max(range(8), key=lambda n: maximum_key(observations[n]))
                lo, hi = observations[mi], observations[ma]
                rows[index] = (3, [metric, 8, minimum_key(lo), mi, lo[0], lo[1],
                                   lo[2]+(3000 if metric == 3 else 0),
                                   maximum_key(hi), ma, hi[0], hi[1],
                                   hi[2]+(3000 if metric == 3 else 0)])
            result = evaluate(decode_dump(from_rows(rows)), pc)
            brute_max = max(a[1]+1-s['lower_pico_us'] for a, s in zip(observations, pc['sources']))
            brute_min = min(a[0]-s['upper_pico_us'] for a, s in zip(observations, pc['sources']))
            self.assertGreaterEqual(result['availability_age_upper_us'], brute_max)
            self.assertLessEqual(result['availability_age_lower_us'], brute_min)
            self.assertEqual(result['verdict'], 'INCOMPLETE')

    def test_retained_source_bounds_must_match_capture_records(self):
        _, pc, _ = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'summary.json').write_text(json.dumps(pc))
            for tamper in (False, True):
                with (root / 'events.csv').open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(('event', 'pc_us', 'details_json'))
                    for n, source in enumerate(pc['sources']):
                        capture = {'source_us': n*1000, 'capture_end_pc_us': 1000+n*1000,
                                   'phase_us': 0, 'uncertainty_us': 10,
                                   'source_lower_pico_us': source['lower_pico_us'],
                                   'source_upper_pico_us': source['upper_pico_us']}
                        writer.writerow(('source_capture', 1000+n*1000, json.dumps(capture)))
                        bounds = dict(source)
                        if tamper and n == 0:
                            bounds['lower_pico_us'] += 1
                        writer.writerow(('source_clock_bounds', 1000+n*1000, json.dumps(bounds)))
                        payload = bytearray(32)
                        payload[7] = 1
                        writer.writerow(('rx_complete', 1000+n*1000,
                                         json.dumps({'type': 2, 'ack': n,
                                                     'payload_hex': payload.hex()})))
                loaded = load_pc_evidence(root)
                self.assertEqual(loaded['source_limits_verified'], not tamper)
                self.assertTrue(loaded['active_output_prefix_complete'])

    def test_offline_cli_truncation_reports_incomplete_without_device_access(self):
        blob, _, _ = fixture()
        script = Path(__file__).parents[1] / 'tools/gate1_report.py'
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            usb = root / 'stopped.bin'
            usb.write_bytes(blob[:-10])
            (root / 'events.csv').write_text('event,pc_us,details_json\n')
            for verdict in ('INCOMPLETE', 'FAIL'):
                (root / 'summary.json').write_text(json.dumps({'verdict': verdict}))
                output = root / (verdict + '-report.json')
                argv = [str(script), '--pc-run', str(root), '--usb-dump', str(usb), '--output', str(output)]
                with patch('sys.argv', argv), redirect_stdout(io.StringIO()), \
                        patch('pil.serial_io.discover', side_effect=AssertionError('device access')), \
                        self.assertRaises(SystemExit) as error:
                    runpy.run_path(str(script), run_name='__main__')
                self.assertEqual(error.exception.code, 1 if verdict == 'FAIL' else 2)
                result = json.loads(output.read_text())
                self.assertEqual(result['verdict'], verdict)
                self.assertFalse(result['physical_gate1_pass'])

    def test_uart_v1_body_limit_is_not_relaxed(self):
        from pil.wire import MAX_BODY
        self.assertEqual(MAX_BODY, 128)


if __name__ == '__main__':
    unittest.main()
