#!/usr/bin/env python3
"""Compile the real SW6 D1 producer on the host and verify synthetic PC bounds.

This is NOT a Pico/Probe soak. No serial paths, ports or hardware are accessed.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pil.gate1_diagnostics import decode_dump, evaluate

SOURCES = ('tests/pil_gate1_fixture.c', 'app_program/sim/pil_gate1_metrics.c',
           'app_program/sim/pil_gate1_dump.c', 'app_program/sim/pil_wire.c')
ARTIFACTS = SOURCES + ('app_program/sim/pil_gate1_metrics.h',
                      'app_program/sim/pil_gate1_dump.h', 'app_program/sim/pil_wire.h')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--codec-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-revision')
    parser.add_argument('--compiler', default='cc')
    args = parser.parse_args()
    root = args.codec_root.resolve()
    original = {name: (root / name).read_bytes() for name in ARTIFACTS}
    if args.expected_revision:
        revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse',
                                            args.expected_revision + '^{commit}'], text=True).strip()
        for name, data in original.items():
            if subprocess.check_output(['git', '-C', str(root), 'show', revision + ':' + name]) != data:
                raise AssertionError('C producer differs from expected revision: ' + name)
        print('Verified C producer revision:', revision)
    args.output.mkdir(parents=True, exist_ok=False)
    executable = args.output.resolve() / 'd1-host-producer'
    command = [args.compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Wconversion',
               '-Wshadow', '-Wpedantic', '-Werror',
               '-I', str(root / 'app_program/sim'), *(str(root / name) for name in SOURCES),
               '-o', str(executable)]
    subprocess.run(command, check=True)
    fixture = args.output.resolve() / 'c-produced-d1.bin'
    subprocess.run([str(executable), str(fixture)], check=True)
    dump = decode_dump(fixture.read_bytes())
    epoch, staged, applied = dump.context[2], dump.counts[0], dump.counts[1]
    assert (staged, applied) == (600004, 600001), 'unexpected synthetic fixture cohort'
    # Declared synthetic intervals only: these are NOT measured clock captures.
    pc = {'identity': {'boot_nonce': dump.boot_nonce, 'host_session': dump.session,
                       'epoch_pico_us': epoch}, 'verdict': 'INCOMPLETE',
          'elapsed_us': 600000000, 'ten_minute_duration_met': True,
          'active_output_count': applied, 'active_output_prefix_complete': True,
          'image_sha256': '0' * 64, 'profile_flags': 0x79,
          'required_capabilities': 4, 'stopped_status_observed': True,
          'source_limits_verified': True,
          'sources': [{'source_us': n*1000, 'lower_pico_us': epoch+n*1000-4005,
                       'upper_pico_us': epoch+n*1000-3995} for n in range(staged)]}
    result = evaluate(dump, pc)
    expected = {'availability_age_upper_us': 1008, 'availability_age_lower_us': 995,
                'apply_age_upper_us': 4030, 'apply_age_lower_us': 4015,
                'endpoint_slack_lower_us': 1997, 'apply_jitter_upper_us': 25,
                'native_first_C_entry_jitter_upper_us': 11}
    for field, value in expected.items():
        assert result.get(field) == value, (field, result)
    assert result['verdict'] == 'INCOMPLETE' and not result['physical_gate1_pass']
    assert result['transport_subset_qualified'] is True
    for name, data in original.items():
        assert (root / name).read_bytes() == data, 'producer changed during test'
    report = {'scope': 'SYNTHETIC HOST ONLY; no Pico/Probe timing measurement',
              'C_sources_sha256': {name: hashlib.sha256(data).hexdigest()
                                   for name, data in original.items()},
              'fixture_sha256': hashlib.sha256(fixture.read_bytes()).hexdigest(),
              'fixture_bytes': fixture.stat().st_size,
              'C_cohort_counts': {'staged': staged, 'applied': applied},
              'compiler_command': command, 'PC_result': result}
    (args.output / 'interop-summary.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    print(json.dumps(report, indent=2, sort_keys=True))
    print('PASS: real C D1 production reductions/encode/chain agree with independent PC bounds.')
    print('HOST ONLY: synthetic fixture; no hardware measurement or Gate1 PASS.')


if __name__ == '__main__':
    main()
