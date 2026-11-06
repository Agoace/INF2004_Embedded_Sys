#!/usr/bin/env python3
"""Decode KR1 RAM rows (or VE events with --events) to CSV. No dependencies."""
import argparse
import csv
import re
import sys


def bits(word, offset, width, signed=False):
    value = (word >> offset) & ((1 << width) - 1)
    return value - (1 << width) if signed and value & (1 << (width - 1)) else value


def decode(payload):
    if len(payload) != 32:
        raise ValueError("KR1 requires exactly 16 bytes")
    w = int.from_bytes(bytes.fromhex(payload), "little")
    echo = bits(w, 93, 16)
    status = (echo >> 9) & 3
    return [bits(w, 0, 16) * 20, bits(w, 16, 2), bits(w, 18, 3),
            bits(w, 21, 12), bits(w, 33, 12), bits(w, 45, 8, True),
            bits(w, 53, 10, True), bits(w, 63, 10, True),
            bits(w, 73, 10, True), bits(w, 83, 10, True), echo, status,
            echo & 511 if status == 0 else "", (echo >> 11) & 7,
            bits(w, 109, 7, True), bits(w, 116, 8), bits(w, 124, 4)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default="-")
    parser.add_argument("--events", action="store_true")
    args = parser.parse_args()
    stream = sys.stdin if args.path == "-" else open(args.path, encoding="utf-8")
    writer = csv.writer(sys.stdout)
    if args.events:
        writer.writerow(["time_ms", "kind", "from", "to", "word"])
    else:
        writer.writerow(["row", "time_ms", "state", "module", "raw_L", "raw_C", "position",
                         "asked_L", "asked_R", "measured_L", "measured_R", "echo_word",
                         "echo_status", "echo_mm", "echo_streak", "servo_deg", "events", "flags"])
    try:
        for line in stream:
            if args.events:
                match = re.fullmatch(r"VE (\d+) (\d+) (\d+) (\d+) (\d+)\s*", line)
                if match:
                    writer.writerow(map(int, match.groups()))
            else:
                match = re.fullmatch(r"KR (\d+) ([0-9a-fA-F]{32})\s*", line)
                if match:
                    writer.writerow([int(match[1]), *decode(match[2])])
    finally:
        if stream is not sys.stdin:
            stream.close()


if __name__ == "__main__":
    main()
