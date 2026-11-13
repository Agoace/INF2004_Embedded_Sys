#!/usr/bin/env python3
"""No-device dry-run: replay recorded synthetic UART bytes through the soak worker."""

import argparse
import json
from pathlib import Path
import sys

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE))
from pil.gate1_replay import replay_soak


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path,
                        default=SOURCE / 'tests/fixtures/pil_gate1_cap4_synthetic_replay.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a fresh output directory; do not overwrite evidence')
    result = replay_soak(args.fixture, args.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    print('HOST DRY-RUN ' + ('PASS' if result['host_fixture_match'] else 'FAIL') +
          '; soak verdict ' + result['soak_verdict'] + '; no hardware or serial access')
    return 0 if result['host_fixture_match'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
