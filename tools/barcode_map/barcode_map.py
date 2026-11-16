#!/usr/bin/env python3
"""Convert a readable barcode map to the 26 BW_MAP_VALUES.

Usage: barcode_map.py "F=LEFT,K=RIGHT"
Prints 26 comma-separated action codes (A..Z). Unlisted letters are NONE.
Actions: NONE=0, LEFT=1, RIGHT=2, STRAIGHT=3, UTURN=4 (see
app_program/subsystem3/barcode_window.h, mapped_action). Exit 1 on error.
"""
import sys

ACTIONS = {"NONE": 0, "LEFT": 1, "RIGHT": 2, "STRAIGHT": 3, "UTURN": 4}


class MapError(ValueError):
    """The map text is not valid."""


def convert(text):
    """Return a list of 26 action codes for the map text."""
    values = [0] * 26
    seen = set()
    if not text.strip():
        raise MapError("empty map")
    for entry in text.split(","):
        parts = [p.strip() for p in entry.split("=")]
        if len(parts) != 2 or not parts[0] or not parts[1]:
            raise MapError(f"bad entry '{entry.strip()}': use LETTER=ACTION")
        letter, action = parts[0].upper(), parts[1].upper()
        if len(letter) != 1 or not "A" <= letter <= "Z":
            raise MapError(f"unknown letter '{parts[0]}': use A..Z")
        if action not in ACTIONS:
            raise MapError(f"unknown action '{parts[1]}': use "
                           + "/".join(ACTIONS))
        if letter in seen:
            raise MapError(f"letter {letter} is given twice")
        seen.add(letter)
        values[ord(letter) - ord("A")] = ACTIONS[action]
    return values


def main(argv):
    if len(argv) != 2:
        print("usage: barcode_map.py 'F=LEFT,K=RIGHT'", file=sys.stderr)
        return 1
    try:
        values = convert(argv[1])
    except MapError as err:
        print(f"BARCODE_MAP: {err}", file=sys.stderr)
        return 1
    print(",".join(str(v) for v in values))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
