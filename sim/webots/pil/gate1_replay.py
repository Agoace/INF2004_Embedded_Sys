"""Recorded byte/timestamp replay through the real no-Webots soak worker.

Only regular fixture files are read. No serial adapter, discovery or Webots
module is imported. Software fixture agreement is not hardware acceptance.
"""

import hashlib
import json
from pathlib import Path

from .gate1 import run_soak


class ReplayMismatch(ValueError):
    pass


class ReplayClock:
    def __init__(self, start):
        self.now = start

    def now_us(self):
        return self.now

    def sleep_us(self, delay):
        if type(delay) is not int or delay < 0:
            raise ReplayMismatch('invalid replay-clock delay')
        self.now += delay


def load_fixture(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > 2_000_000:
        raise ReplayMismatch('replay input must be a regular fixture file within2MB')
    fixture = json.loads(path.read_text())
    fields = {'schema', 'provenance', 'hardware_measured', 'start_pc_us', 'duration_us',
              'session', 'transport_only', 'expected_verdict', 'rx', 'tx'}
    if not isinstance(fixture, dict) or set(fixture) != fields or type(fixture['schema']) is not int:
        raise ReplayMismatch('unknown replay fixture schema')
    if (fixture['schema'] != 1 or fixture['hardware_measured'] is not False or
            fixture['transport_only'] is not True or fixture['expected_verdict'] != 'INCOMPLETE' or
            not isinstance(fixture['provenance'], str) or 'SYNTHETIC' not in fixture['provenance']):
        raise ReplayMismatch('fixture must explicitly declare synthetic host-only scope')
    for field, lo, hi in (('start_pc_us', 0, 2**63-1), ('duration_us', 1000, 1_000_000),
                          ('session', 1, 0xffffffff)):
        if type(fixture[field]) is not int or not lo <= fixture[field] <= hi:
            raise ReplayMismatch('replay time/session outside bounds')
    for name in ('rx', 'tx'):
        rows = fixture[name]
        if not isinstance(rows, list) or not 1 <= len(rows) <= 10000:
            raise ReplayMismatch('replay stream size/type')
        previous = fixture['start_pc_us']
        for row in rows:
            if (not isinstance(row, dict) or set(row) != {'pc_us', 'hex'} or
                    type(row['pc_us']) is not int or row['pc_us'] < previous or
                    row['pc_us'] > fixture['start_pc_us'] + 6_000_000 or
                    not isinstance(row['hex'], str) or len(row['hex']) > 8192):
                raise ReplayMismatch('replay chunk/time bound')
            data = bytes.fromhex(row['hex'])
            if not 1 <= len(data) <= 4096:
                raise ReplayMismatch('replay chunk must be1..4096 bytes')
            previous = row['pc_us']
    return fixture


class RecordedUART:
    """Fake nonblocking endpoint; every PC write must match recorded bytes/time."""
    def __init__(self, fixture, clock):
        self.fixture, self.clock = fixture, clock
        self.rx_index = self.tx_index = 0
        self.closed = False

    def read(self, maximum):
        if self.rx_index == len(self.fixture['rx']):
            return b''
        row = self.fixture['rx'][self.rx_index]
        if self.clock.now_us() < row['pc_us']:
            return b''
        if self.clock.now_us() != row['pc_us']:
            raise ReplayMismatch('recorded RX replay time mismatch')
        data = bytes.fromhex(row['hex'])
        if len(data) > maximum:
            raise ReplayMismatch('recorded RX exceeds requested read capacity')
        self.rx_index += 1
        return data

    def write(self, data):
        if self.tx_index == len(self.fixture['tx']):
            raise ReplayMismatch('unexpected recorded TX after fixture end')
        row = self.fixture['tx'][self.tx_index]
        expected = bytes.fromhex(row['hex'])
        if self.clock.now_us() != row['pc_us'] or not data.startswith(expected):
            raise ReplayMismatch('recorded TX bytes/time mismatch')
        self.tx_index += 1
        return len(expected)

    def close(self):
        self.closed = True

    @property
    def complete(self):
        return (self.closed and self.rx_index == len(self.fixture['rx']) and
                self.tx_index == len(self.fixture['tx']))


def replay_soak(fixture_path, output):
    fixture = load_fixture(fixture_path)
    clock = ReplayClock(fixture['start_pc_us'])
    pair = RecordedUART(fixture, clock)
    result = run_soak(pair, output, clock=clock, duration_us=fixture['duration_us'],
                      session=fixture['session'], transport_only=True)
    matched = (pair.complete and result['verdict'] == fixture['expected_verdict'] and
               result['stopped_status_observed'] is True)
    report = {'scope': 'SYNTHETIC HOST REPLAY ONLY; no Pico/Probe/Webots timing measurement',
              'host_fixture_match': matched, 'soak_verdict': result['verdict'],
              'physical_gate1_pass': False, 'serial_device_opened': False,
              'fixture_sha256': hashlib.sha256(Path(fixture_path).read_bytes()).hexdigest(),
              'rx_chunks': pair.rx_index, 'tx_chunks': pair.tx_index,
              'expected_rx_chunks': len(fixture['rx']), 'expected_tx_chunks': len(fixture['tx']),
              'failure': result['failure']}
    (Path(output) / 'dry-run.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    return report
