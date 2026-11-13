#!/usr/bin/env python3
"""Compile SW6's real C byte codec and compare it with the PC codec, host-only."""

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import random
import struct
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pil.wire import ProtocolError, cobs_decode, cobs_encode, crc32c, decode, encode


class CFrame(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint8), ("payload_length", ctypes.c_uint16),
                ("session", ctypes.c_uint32), ("sequence", ctypes.c_uint32),
                ("source_us", ctypes.c_uint32), ("ack", ctypes.c_uint32),
                ("payload", ctypes.c_uint8 * 104)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--codec-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compiler", default="cc")
    parser.add_argument("--expected-revision",
                        help="verify codec/header/fixture against Git commit")
    args = parser.parse_args()
    if args.expected_revision:
        revision = subprocess.check_output(
            ["git", "-C", str(args.codec_root), "rev-parse",
             args.expected_revision + "^{commit}"], text=True).strip()
        for relative in ("app_program/sim/pil_wire.c", "app_program/sim/pil_wire.h",
                         "tests/fixtures/pil_wire_v1.json"):
            committed = subprocess.check_output(
                ["git", "-C", str(args.codec_root), "show", revision + ":" + relative])
            if committed != (args.codec_root / relative).read_bytes():
                raise AssertionError("working codec artifact differs from expected commit")
        print("Verified C codec/header/fixture revision:", revision)
    args.output.mkdir(parents=True, exist_ok=True)
    source = args.codec_root / "app_program/sim/pil_wire.c"
    library = args.output.resolve() / "libpil_wire.so"
    subprocess.run([args.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC",
                    "-shared", str(source), "-o", str(library)], check=True)
    codec = ctypes.CDLL(str(library))
    codec.pil_decode.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(CFrame)]
    codec.pil_decode.restype = ctypes.c_int
    codec.pil_encode.argtypes = [ctypes.POINTER(CFrame), ctypes.c_void_p, ctypes.c_size_t,
                                ctypes.POINTER(ctypes.c_size_t)]
    codec.pil_encode.restype = ctypes.c_int
    fixture = Path(__file__).resolve().parents[1] / "tests/fixtures/pil_wire_v1.json"
    authoritative = args.codec_root / "tests/fixtures/pil_wire_v1.json"
    if fixture.read_bytes() != authoritative.read_bytes():
        raise AssertionError("SW6 authoritative fixture and PC copy differ")
    vectors = json.loads(fixture.read_text())["vectors"]
    print("C source SHA256:", hashlib.sha256(source.read_bytes()).hexdigest())
    print("Golden fixture SHA256:", hashlib.sha256(fixture.read_bytes()).hexdigest())
    checked = 0

    def compare(packet, expected=None):
        nonlocal checked
        result = CFrame()
        ctypes.memset(ctypes.byref(result), 0xA5, ctypes.sizeof(result))
        previous = bytes(result)
        accepted_c = codec.pil_decode(packet, len(packet), ctypes.byref(result)) == 0
        try:
            frame = decode(packet)
            accepted_pc = True
        except ProtocolError:
            accepted_pc = False
        if accepted_c != accepted_pc or (expected is not None and accepted_pc != expected):
            raise AssertionError(f"C/PC acceptance mismatch {packet.hex()}")
        if accepted_pc:
            actual = (result.type, result.session, result.sequence, result.source_us,
                      result.ack, bytes(result.payload[:result.payload_length]))
            wanted = (frame.type, frame.session, frame.sequence, frame.source_us,
                      frame.ack, frame.payload)
            if actual != wanted:
                raise AssertionError("decoded C fields differ")
            output = ctypes.create_string_buffer(130)
            size = ctypes.c_size_t(0)
            status = codec.pil_encode(ctypes.byref(result), output, len(output),
                                      ctypes.byref(size))
            if status or output.raw[:size.value] != encode(frame):
                raise AssertionError("C and PC encode differ")
        elif bytes(result) != previous:
            raise AssertionError("C decoder partially published invalid frame")
        checked += 1

    for vector in vectors:
        compare(bytes.fromhex(vector["wire_hex"]), vector["accepted"])
    rng = random.Random(0x50494C01)
    good = [v for v in vectors if v["accepted"]]
    for _ in range(3000):
        body = bytearray(cobs_decode(bytes.fromhex(rng.choice(good)["wire_hex"])))
        offset = rng.randrange(len(body) - 4)
        body[offset] ^= 1 << rng.randrange(8)
        struct.pack_into("<I", body, len(body) - 4, crc32c(body[:-4]))
        compare(cobs_encode(body))
    print(f"PASS: {checked} C/PC comparisons; byte-exact golden encode/decode;")
    print("CRC-valid semantic mutations; invalid C decode leaves destination unchanged.")
    print("HOST ONLY: no serial port, Webots execution or hardware timing qualification.")


if __name__ == "__main__":
    main()
