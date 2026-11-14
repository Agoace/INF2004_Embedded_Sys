"""Attempt6 regressions: short USB chunks and unchanged console CRLF.

Uses only saved bytes. Never opens a device or invokes a privileged capture.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from pil.gate1_measurement import ClockBrackets, correlate
from test_gate1_measurement import _mini_pc

FIXTURES = Path(__file__).parent/'fixtures'
REAL = FIXTURES/'pil_gate1_attempt6_usb_rx.json'
CAPTURE = Path(__file__).parents[1]/'tools/gate1_usbmon.sh'


class USBRecoveryTests(unittest.TestCase):
    def test_attempt6_all_short_fresh_chunks_recover_every_sync(self):
        from pil.gate1_measurement import USBStreamMatcher
        fixture = json.loads(REAL.read_text())
        data = bytes.fromhex(fixture['pc_rx_hex'])
        matcher, matched = USBStreamMatcher(data), []
        for index, chunk in enumerate(fixture['usb_in_chunks']):
            matched.extend(matcher.feed(bytes.fromhex(chunk['payload_hex']),
                           {'fixture_index': index, 'complete_realtime_us': chunk['complete_realtime_us']}))
        matcher.finish()
        self.assertEqual(matcher.unmatched, 4)  # 2 stale-prefix and 2 unread-tail URBs.
        self.assertEqual(sum(x['stream_end']-x['stream_start'] for x in matched), len(data))
        cursor = 0
        for part in matched:
            self.assertEqual(part['stream_start'], cursor)
            payload = bytes.fromhex(fixture['usb_in_chunks'][part['fixture_index']]['payload_hex'])
            self.assertEqual(payload, data[part['stream_start']:part['stream_end']])
            cursor = part['stream_end']
        clock = ClockBrackets(fixture['clock_pairs'])
        for reply in fixture['sync_reply_ends']:
            part = next(p for p in matched if p['stream_start'] < reply['pc_stream_end'] <= p['stream_end'])
            self.assertEqual(part['complete_realtime_us'], reply['complete_realtime_us'])
            self.assertIsNotNone(clock.map(part['complete_realtime_us']))
        self.assertEqual(len(fixture['sync_reply_ends']), 13)

    def test_unanchored_memory_bounded_and_ambiguous_bytes_unknown(self):
        from pil.gate1_measurement import USBStreamMatcher
        matcher = USBStreamMatcher(b'not the captured stream')
        for index in range(200):
            self.assertEqual(matcher.feed(b'x', {'index': index}), [])
            self.assertLessEqual(matcher.pending_records, 64)
            self.assertLessEqual(matcher.pending_bytes, 4096)
        matcher.finish()
        self.assertEqual(matcher.unmatched, 200)
        ambiguous = USBStreamMatcher(b'x'*200)
        self.assertEqual(ambiguous.feed(b'x'*16, {}), [])
        ambiguous.finish()
        self.assertEqual(ambiguous.unmatched, 1)

    def test_short_out_parts_keep_completion_metadata_when_backfilled(self):
        from pil.gate1_measurement import USBStreamMatcher
        data = bytes(range(1, 60))
        matcher = USBStreamMatcher(data)
        first, second, third = {}, {}, {}
        self.assertEqual(matcher.feed(data[:5], first), [])
        first['complete_realtime_us'] = 123
        self.assertEqual(matcher.feed(data[5:12], second), [])
        second['complete_realtime_us'] = 234
        matched = matcher.feed(data[12:28], third)
        self.assertEqual([p['stream_start'] for p in matched], [0, 5, 12])
        self.assertIs(matched[0], first)
        self.assertEqual(matched[0]['complete_realtime_us'], 123)
        self.assertEqual(matched[1]['complete_realtime_us'], 234)

    def test_known_missing_usb_bytes_are_not_fabricated(self):
        from pil.gate1_measurement import USBStreamMatcher
        data = bytes(range(1, 100))
        matcher = USBStreamMatcher(data)
        matcher.feed(data[:20], {})
        matcher.finish()  # Simulated missing/short URB: alignment is lost.
        self.assertEqual(matcher.feed(data[40:47], {}), [])
        recovered = matcher.feed(data[47:60], {})
        self.assertEqual(recovered[0]['stream_start'], 40)
        self.assertEqual(recovered[-1]['stream_end'], 60)

    def test_attempt6_real_crlf_stack_line_is_complete(self):
        fixture = json.loads(REAL.read_text())
        data = bytes.fromhex((FIXTURES/'pil_gate1_rx1_underflow.hex.txt').read_text())
        real_stack = bytes.fromhex(fixture['stack_line_hex'])
        self.assertTrue(real_stack.endswith(b'\r\n'))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            _mini_pc(path)
            stopped = path/'stopped.bin'
            stopped.write_bytes(data[:data.index(b'[PILG1] stopped;')]+real_stack)
            report = correlate(path, stopped=stopped)
            self.assertTrue(report['stopped_stack_line_complete'])
            self.assertEqual(report['diagnostic'], 'COMPLETE')
            self.assertEqual(report['measurement_verdict'], 'FAIL')

    def test_capture_script_syntax_help_and_readable_stats_order(self):
        self.assertTrue(CAPTURE.is_file(), 'missing approved capture wrapper')
        subprocess.run(['bash', '-n', str(CAPTURE)], check=True)
        result = subprocess.run(['bash', str(CAPTURE), '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        code = CAPTURE.read_text()
        ownership = 'chown "$SUDO_UID:$SUDO_GID" "$errfile"'
        self.assertIn(ownership, code)
        self.assertLess(code.index(ownership), code.index('exec 2>>"$errfile"'))
        self.assertLess(code.index('exec 2>>"$errfile"'), code.index('modprobe usbmon'))

    def test_capture_without_sudo_refuses_before_creating_files(self):
        self.assertTrue(CAPTURE.is_file(), 'missing approved capture wrapper')
        if os.geteuid() == 0:
            self.skipTest('non-root gate cannot be exercised as root')
        with tempfile.TemporaryDirectory() as tmp:
            result = subprocess.run(['bash', str(CAPTURE), '1', tmp], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(list(Path(tmp).iterdir()), [])
            self.assertIn('sudo', result.stderr)


if __name__ == '__main__':
    unittest.main()
