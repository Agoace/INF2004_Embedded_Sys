#!/usr/bin/env python3
"""Generate a deterministic SYNTHETIC UART recording. No device or serial access.

From the repository root:
PYTHONPATH=sim/webots python3 sim/webots/tests/generate_gate1_replay_fixture.py
The fixture exercises software only, never a measured Pico/Probe link.
"""

import json
from pathlib import Path
import tempfile

from pil.gate1 import run_soak
from test_gate1 import FakeClock, FakePair


class RecordingPair(FakePair):
    def __init__(self, clock):
        super().__init__(clock, capabilities=4)
        self.rx, self.tx = [], []

    def read(self, maximum):
        data = super().read(maximum)
        if data:
            self.rx.append({'pc_us': self.clock.now_us(), 'hex': data.hex()})
        return data

    def write(self, data):
        count = super().write(data)
        if count:
            self.tx.append({'pc_us': self.clock.now_us(), 'hex': data[:count].hex()})
        return count


def make_fixture():
    clock = FakeClock()
    pair = RecordingPair(clock)
    with tempfile.TemporaryDirectory() as tmp:
        summary = run_soak(pair, Path(tmp) / 'recording', clock=clock,
                           duration_us=40000, session=123, transport_only=True)
    assert summary['verdict'] == 'INCOMPLETE', summary
    assert summary['stopped_status_observed'] is True
    return {'schema': 1, 'provenance': 'SYNTHETIC host reference; NOT a Pico/Probe capture',
               'hardware_measured': False, 'start_pc_us': 10000, 'duration_us': 40000,
               'session': 123, 'transport_only': True, 'expected_verdict': 'INCOMPLETE',
               'rx': pair.rx, 'tx': pair.tx}


def main():
    fixture = make_fixture()
    path = Path(__file__).parent / 'fixtures/pil_gate1_cap4_synthetic_replay.json'
    path.write_text(json.dumps(fixture, indent=2, sort_keys=True) + '\n')
    print(f'Generated synthetic fixture: {path}, RX{len(fixture["rx"])}, TX{len(fixture["tx"])}')


if __name__ == '__main__':
    main()
