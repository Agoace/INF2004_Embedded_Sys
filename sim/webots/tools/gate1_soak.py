#!/usr/bin/env python3
"""Opt-in Probe transport soak. Hardware use needs separate operator permission."""

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pil.gate1 import run_soak
from pil.serial_io import ProbeSerial, discover


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware-authorized', action='store_true',
                        help='explicitly authorize discovery and opening a real Probe')
    parser.add_argument('--device', help='exact /dev/serial/by-id/*Debug_Probe* selector')
    parser.add_argument('--duration-seconds', type=int, default=600)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.hardware_authorized:
        parser.error('no hardware access: --hardware-authorized requires operator permission')
    if not 1 <= args.duration_seconds <= 1700:
        parser.error('duration must be1..1700seconds')
    # Check output before opening any device. Never append to old evidence.
    if args.output.exists():
        parser.error('output already exists; select a new evidence directory')
    device = discover(args.device)
    summary = run_soak(ProbeSerial(device), args.output,
                       duration_us=args.duration_seconds * 1_000_000)
    print(json.dumps(summary, indent=2))
    return {'FAIL': 1, 'INCOMPLETE': 2}[summary['verdict']]


if __name__ == '__main__':
    raise SystemExit(main())
