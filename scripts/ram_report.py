#!/usr/bin/env python3
"""Report linked static RAM contributions from a GNU ld map (no secrets)."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import re

RAM_START = 0x20000000
RAM_END = 0x20042000
SECTION = re.compile(r"^ (\.[\w.$-]+)(?:\s+(.*))?$")
VALUES = re.compile(r"^\s*(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(\S+.*)$")
OUTPUT = re.compile(r"^([.\w]+)\s+(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)")


def module_for(obj):
    """Keep support groups distinct from application owners."""
    match = re.search(r"app_program/(subsystem[1-5]|vehicle|platform)/", obj)
    if match:
        return match[1]
    if "/lwip/" in obj or "/liblwip/" in obj:
        return "lwIP"
    if "/kernel/" in obj:
        return "kernel"
    if "/app_sdk_hw/" in obj:
        return "SDK"
    if "/tinyusb/" in obj:
        return "USB SDK"
    if "/cyw43/" in obj:
        return "radio SDK"
    if "/libg.a(" in obj or "/libm.a(" in obj:
        return "C library"
    return "other support"


def parse_map(text):
    """Read one-line and wrapped input sections; exclude discarded/flash."""
    rows = []
    outputs = {}
    pending = None
    active = False
    for line in text.splitlines():
        if line.startswith("Linker script and memory map"):
            active = True
        if not active:
            continue
        out = OUTPUT.match(line)
        if out:
            name, addr, size = out.groups()
            if RAM_START <= int(addr, 16) < RAM_END:
                outputs[name] = int(size, 16)
        section = SECTION.match(line)
        if section:
            pending = section[1]
            values = VALUES.match(section[2] or "")
        elif pending:
            values = VALUES.match(line)
        else:
            continue
        if values:
            addr, size, obj = values.groups()
            address, length = int(addr, 16), int(size, 16)
            if (RAM_START <= address < RAM_END and length and
                    ".o" in obj):
                rows.append({"section": pending, "address": address,
                             "bytes": length, "object": obj,
                             "module": module_for(obj)})
            pending = None
        elif not section:
            pending = None
    totals = defaultdict(int)
    for row in rows:
        totals[row["module"]] += row["bytes"]
    static = sum(outputs.values())
    return {"static_ram": static, "output_sections": outputs,
            "groups": dict(sorted(totals.items())),
            "input_bytes": sum(totals.values()),
            "alignment_bytes": static - sum(totals.values()),
            "contributions": sorted(rows, key=lambda r: -r["bytes"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--module", help="Filter largest sections by group")
    args = parser.parse_args()
    report = parse_map(args.map.read_text())
    if not report["output_sections"]:
        parser.error("no linked RAM output sections found in this GNU map")
    if args.json:
        print(json.dumps(report, indent=2))
        return
    print("Group | bytes")
    for name, size in report["groups"].items():
        print(f"{name} | {size}")
    print(f"Static RAM (all RAM output sections) | {report['static_ram']}")
    print(f"Alignment / other output bytes | {report['alignment_bytes']}")
    print("\nLargest linked input sections (symbol suffix, object, bytes):")
    rows = report["contributions"]
    if args.module:
        rows = [row for row in rows if row["module"] == args.module]
    for row in rows[:args.top]:
        print(f"{row['section']} | {row['object']} | {row['bytes']}")


if __name__ == "__main__":
    main()
