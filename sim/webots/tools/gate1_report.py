#!/usr/bin/env python3
"""Offline D1/PC evidence correlation. This command never opens a serial device."""

import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pil.gate1_diagnostics import DiagnosticError, decode_dump, evaluate, load_pc_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc-run', type=Path, required=True)
    parser.add_argument('--usb-dump', type=Path, required=True,
                        help='previously retained Pico USB bytes, not a device path')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('select a new summary file; do not overwrite evidence')
    from pil.gate1_trial import labels
    pc, hint = None, {}
    try:
        summary_path = args.pc_run/'summary.json'
        if summary_path.is_file() and summary_path.stat().st_size <= 1_000_000:
            hint = json.loads(summary_path.read_text())
        pc = load_pc_evidence(args.pc_run)
        if not args.usb_dump.is_file() or args.usb_dump.stat().st_size > 4_000_000:
            raise DiagnosticError('USB input must be a regular file within4MB')
        result = evaluate(decode_dump(args.usb_dump.read_bytes()), pc)
    except (DiagnosticError, OSError, ValueError, KeyError, TypeError, IndexError, csv.Error) as error:
        retained = pc if pc is not None else hint
        failed = retained.get('verdict') == 'FAIL'
        result = {**labels(retained.get('trial_clock_400_sync20') is True),
                  'verdict': 'FAIL' if failed else 'INCOMPLETE', 'physical_gate1_pass': False,
                  'incomplete_reason': str(error)}
        if failed:
            result['failure'] = retained.get('failure', 'PC run failed')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 1 if result['verdict'] == 'FAIL' else 2


if __name__ == '__main__':
    raise SystemExit(main())
