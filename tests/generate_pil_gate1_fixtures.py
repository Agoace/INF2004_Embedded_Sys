#!/usr/bin/env python3
"""Independent D1 envelopes, NOT accepted/qualified run cohorts."""
import argparse
import json
from pathlib import Path
import struct


def crc32c(data):
    value = 0xFFFFFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
    return value ^ 0xFFFFFFFF


def cobs(data):
    out = bytearray([0])
    code, at = 1, 0
    for byte in data:
        if byte:
            out.append(byte)
            code += 1
        else:
            out[at] = code
            at, code = len(out), 1
            out.append(0)
    out[at] = code
    return b"\0" + out + b"\0"


def generate():
    vectors = []

    def add(name, kind=1, size=104, accepted=True, **changes):
        payload = bytes(i % 256 for i in range(size))
        words = dict(magic=0x31444750, version=1, kind=kind, seq=14,
                     nonce=0x12345678, session=0x9ABCDEF0, length=size)
        bad_crc = changes.pop("bad_crc", False)
        words.update(changes)
        raw = struct.pack("<IHHIIII", *words.values()) + payload
        raw += struct.pack("<I", crc32c(raw) ^ int(bad_crc))
        vectors.append(dict(name=name, accepted_envelope=accepted,
                            decoded_hex=raw.hex(), wire_hex=cobs(raw).hex()))

    for kind, (name, size) in enumerate(zip(
            ("context", "counts", "extremum", "trace", "manifest"),
            (104, 104, 48, 64, 16)), 1):
        add("envelope_" + name, kind, size)
    add("zero_identity_unqualified", nonce=0, session=0)
    add("bad_version", accepted=False, version=2)
    add("bad_magic", accepted=False, magic=0)
    add("unknown_kind", accepted=False, kind=6)
    add("bad_declared_length", accepted=False, length=103)
    add("bad_crc", accepted=False, bad_crc=True)
    add("oversize_decoded", accepted=False, size=105)
    return {"format": "D1-envelope-v1", "note":
            "Envelope acceptance only. Payloads are NOT qualified run evidence.",
            "vectors": vectors}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    path = Path(__file__).parent / "fixtures/pil_gate1_d1.json"
    text = json.dumps(generate(), indent=2) + "\n"
    if args.check:
        if path.read_text() != text:
            raise SystemExit("D1 fixtures differ from independent reference")
        print("D1: 12 reference envelopes match; no hardware qualification")
    else:
        path.write_text(text)
