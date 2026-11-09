#!/usr/bin/env python3
"""Decode KR1 RAM rows (VE events with --events, hump lines with --hump) to CSV.

No dependencies."""
import argparse
import csv
import re
import sys


def bits(word, offset, width, signed=False):
    value = (word >> offset) & ((1 << width) - 1)
    return value - (1 << width) if signed and value & (1 << (width - 1)) else value


MODULE_NAMES = {0: "none", 1: "line", 2: "search", 3: "scan", 4: "detour",
                5: "scan_reverse"}


def decode(payload):
    if len(payload) != 32:
        raise ValueError("KR1 requires exactly 16 bytes")
    w = int.from_bytes(bytes.fromhex(payload), "little")
    echo = bits(w, 93, 16)
    status = (echo >> 9) & 3
    module = bits(w, 18, 3)
    return [bits(w, 0, 16) * 20, bits(w, 16, 2), module, MODULE_NAMES.get(module, "unknown"),
            bits(w, 21, 12), bits(w, 33, 12), bits(w, 45, 8, True),
            bits(w, 53, 10, True), bits(w, 63, 10, True),
            bits(w, 73, 10, True), bits(w, 83, 10, True), echo, status,
            echo & 511 if status == 0 else "", (echo >> 11) & 7,
            bits(w, 109, 7, True), bits(w, 116, 8), bits(w, 124, 4)]


HUMP_NONE = 65535
HUMP_LEFT = 0x20
# Run end, run end without data, GP20 DUMP and STREAM GP21 drive lines.
HUMP_LINES = (
    ("run_end", re.compile(r"\[robot\] hump highest_mm (\d+)\.(\d) side (\w+) count (\d+) "
                           r"flags ([0-9a-f]{2}) gated (\d+)\s*")),
    ("run_end", re.compile(r"\[robot\] hump unavailable flags ([0-9a-f]{2})\s*")),
    ("dump", re.compile(r"\[robot\] hump peak_0\.1mm (\d+) count (\d+) "
                        r"flags ([0-9a-f]{2}) gated (\d+)\s*")),
    ("stream", re.compile(r"\[S4_HUMP\] peak_0\.1mm (\d+) count (\d+) "
                          r"flags ([0-9a-f]{2}) gated (\d+) long \d+\s*")))


def decode_hump(line):
    """Return [source, peak_mm, side, count, flags_hex, gated] or None."""
    for index, (source, pattern) in enumerate(HUMP_LINES):
        match = pattern.fullmatch(line)
        if not match:
            continue
        if index == 0:
            whole, tenth, side, count, flags, gated = match.groups()
            return [source, f"{whole}.{tenth}", side, int(count), flags, int(gated)]
        if index == 1:
            return [source, "unavailable", "", "", match[1], ""]
        tenths, count, flags, gated = (int(match[1]), int(match[2]), match[3],
                                       int(match[4]))
        if tenths == HUMP_NONE:
            return [source, "unavailable", "", "", flags, ""]
        side = "none" if count == 0 else "left" if int(flags, 16) & HUMP_LEFT else "right"
        return [source, f"{tenths // 10}.{tenths % 10}", side, count, flags, gated]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default="-")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--events", action="store_true")
    mode.add_argument("--hump", action="store_true")
    args = parser.parse_args()
    stream = sys.stdin if args.path == "-" else open(args.path, encoding="utf-8")
    writer = csv.writer(sys.stdout)
    if args.hump:
        writer.writerow(["source", "peak_mm", "side", "count", "flags", "gated"])
    elif args.events:
        writer.writerow(["time_ms", "kind", "from", "to", "word"])
    else:
        writer.writerow(["row", "time_ms", "state", "module", "module_name", "raw_L", "raw_C", "position",
                         "asked_L", "asked_R", "measured_L", "measured_R", "echo_word",
                         "echo_status", "echo_mm", "echo_streak", "servo_deg", "events", "flags"])
    try:
        for line in stream:
            if args.hump:
                row = decode_hump(line)
                if row:
                    writer.writerow(row)
            elif args.events:
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
