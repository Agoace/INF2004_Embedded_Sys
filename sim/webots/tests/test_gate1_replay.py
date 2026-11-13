import copy
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import patch

from pil.gate1 import run_soak
from pil.gate1_replay import RecordedUART, ReplayClock, load_fixture, replay_soak
from pil.wire import Type, decode

FIXTURE = Path(__file__).parent / 'fixtures/pil_gate1_cap4_synthetic_replay.json'


class ReplayTests(unittest.TestCase):
    def test_complete_recorded_soak_is_host_match_not_hardware_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            report = replay_soak(FIXTURE, Path(tmp) / 'replay')
            self.assertTrue(report['host_fixture_match'], report)
            self.assertEqual(report['soak_verdict'], 'INCOMPLETE')
            self.assertFalse(report['physical_gate1_pass'])
            self.assertGreater(report['rx_chunks'], 100)
            self.assertGreater(report['tx_chunks'], 40)

    def test_reference_recording_is_deterministic(self):
        from generate_gate1_replay_fixture import make_fixture
        self.assertEqual(load_fixture(FIXTURE), make_fixture())

    def test_cli_cannot_discover_or_open_a_device(self):
        script = Path(__file__).parents[1] / 'tools/gate1_dry_run.py'
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / 'dry-run'
            argv = [str(script), '--fixture', str(FIXTURE), '--output', str(output)]
            with patch('sys.argv', argv), redirect_stdout(io.StringIO()), \
                    patch('pil.serial_io.discover', side_effect=AssertionError('device discovery')), \
                    patch('pil.serial_io.ProbeSerial', side_effect=AssertionError('serial open')), \
                    self.assertRaises(SystemExit) as error:
                runpy.run_path(str(script), run_name='__main__')
            self.assertEqual(error.exception.code, 0)
            result = json.loads((output / 'dry-run.json').read_text())
            self.assertTrue(result['host_fixture_match'])
            self.assertFalse(result['serial_device_opened'])

    def test_scope_schema_and_time_bounds_are_strict(self):
        original = load_fixture(FIXTURE)
        for field, value in (('hardware_measured', True), ('schema', True),
                             ('duration_us', 1_000_001), ('session', 0)):
            fixture = copy.deepcopy(original)
            fixture[field] = value
            with self.subTest(field=field), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'fixture.json'
                path.write_text(json.dumps(fixture))
                with self.assertRaises(ValueError):
                    load_fixture(path)

    def test_first_output_matches_negotiated_pending_armed_contract(self):
        fixture = load_fixture(FIXTURE)
        outputs = [decode(bytes.fromhex(row['hex'])) for row in fixture['rx']
                   if decode(bytes.fromhex(row['hex'])).type == Type.OUTPUT]
        self.assertEqual(outputs[0].payload[7], 4)
        self.assertEqual(outputs[-1].payload[7], 6)
        self.assertTrue(any(frame.payload[7] == 20 for frame in outputs))
        first_armed = next(frame for frame in outputs if frame.payload[7] == 1)
        self.assertEqual(first_armed.ack, 0)
        self.assertEqual(first_armed.payload[:4], b'\0' * 4)

    def test_corrupt_recorded_uart_fails_the_real_worker(self):
        fixture = load_fixture(FIXTURE)
        fixture = copy.deepcopy(fixture)
        row = next(row for row in fixture['rx']
                   if decode(bytes.fromhex(row['hex'])).type == Type.HELLO_ACK)
        packet = bytearray.fromhex(row['hex'])
        packet[-3] ^= 1
        row['hex'] = packet.hex()
        clock = ReplayClock(fixture['start_pc_us'])
        pair = RecordedUART(fixture, clock)
        with tempfile.TemporaryDirectory() as tmp:
            result = run_soak(pair, Path(tmp) / 'bad', clock=clock,
                              duration_us=fixture['duration_us'], session=fixture['session'],
                              transport_only=True)
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertTrue(pair.closed)

    def test_expected_tx_mismatch_cannot_silently_pass(self):
        fixture = copy.deepcopy(load_fixture(FIXTURE))
        fixture['tx'][0]['hex'] = '00'
        clock = ReplayClock(fixture['start_pc_us'])
        pair = RecordedUART(fixture, clock)
        with tempfile.TemporaryDirectory() as tmp:
            result = run_soak(pair, Path(tmp) / 'wrong', clock=clock,
                              duration_us=fixture['duration_us'], session=fixture['session'],
                              transport_only=True)
        self.assertEqual(result['verdict'], 'FAIL')
        self.assertIn('recorded TX', result['failure'])


if __name__ == '__main__':
    unittest.main()
