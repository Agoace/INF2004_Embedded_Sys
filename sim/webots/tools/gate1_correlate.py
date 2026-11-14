#!/usr/bin/env python3
"""Offline PC/optional usbmon/Pico RX1 timing report; never issues Gate1 PASS."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pil.gate1_measurement import correlate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc', type=Path, required=True, help='soak output directory')
    parser.add_argument('--stopped', type=Path, help='complete raw Pico CDC D1+RX1+stack capture')
    parser.add_argument('--usbmon', type=Path, help='optional full-snaplen classic pcap from tcpdump')
    parser.add_argument('--usb-bus', type=int)
    parser.add_argument('--usb-dev', type=int, help='runtime address, not a pinned device number')
    parser.add_argument('--ep-out', type=lambda x: int(x, 0))
    parser.add_argument('--ep-in', type=lambda x: int(x, 0))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('report exists; select a new path')
    try:
        report = correlate(args.pc, pcap=args.usbmon, usb_bus=args.usb_bus,
                           usb_device=args.usb_dev, out_ep=args.ep_out, in_ep=args.ep_in,
                           stopped=args.stopped)
        with args.output.open('x') as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write('\n')
    except (OSError, ValueError) as error:
        parser.error(str(error))
    from pil.gate1_trial import labels
    print(json.dumps({**labels(report.get('trial_clock_400_sync20') is True),
                      'report': str(args.output), 'pc_verdict': report['original_pc_verdict'],
                      'URB_layer': report['urb_layer'], 'URB_reason': report['urb_reason'],
                      'sync_samples': len(report['sync']), 'RX1': report['diagnostic'],
                      'physical_gate1_pass': False}, indent=2))
    return 0  # Report generated, NOT transport or hardware acceptance.


if __name__ == '__main__':
    raise SystemExit(main())
