#!/usr/bin/env python3
"""Independent reference bytes for frozen PIL v1; --check never writes."""
import argparse
import json
from pathlib import Path
import struct

DEST = Path(__file__).parent / "fixtures/pil_wire_v1.json"
SESSION = 0x11223344
NONCE = 0xA1B2C3D4


def crc32c(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def cobs(data):
    chunks = data.split(b"\0")
    return b"".join(bytes([len(chunk) + 1]) + chunk for chunk in chunks) + b"\0"


def vectors():
    result = []

    def add(name, kind, payload, seq=0, time=0, ack=0, session=SESSION,
            accepted=True, version=1):
        header = struct.pack("<BBHIIII", version, kind, len(payload),
                             session, seq, time, ack)
        raw = header + payload
        body = raw + struct.pack("<I", crc32c(raw))
        result.append(dict(name=name, accepted=accepted, type=kind,
                           session=session, sequence=seq, source_us=time,
                           ack=ack, payload_hex=payload.hex(),
                           decoded_hex=body.hex(), wire_hex=cobs(body).hex()))

    hello = struct.pack("<6I", NONCE, SESSION, 31, 1000, 4000, 5000)
    add("boot", 0, struct.pack("<4I", NONCE, 31, 921600, 0x50494C01),
        time=0xFFFFF000, ack=0xFFFFFFFF, session=0)
    add("hello", 3, hello, time=0x10203040)
    add("hello_ack_timer_wrap", 4, hello + struct.pack("<I", 96000),
        time=0xFFFFF000)
    add("arm_pending", 5, struct.pack("<2I", NONCE, SESSION), time=0x10203040)
    sensor = bytearray(40)
    struct.pack_into("<3H", sensor, 0, 100, 200, 300)
    struct.pack_into("<3h", sensor, 16, -32768, 32752, 16000)
    struct.pack_into("<3h", sensor, 22, 400, -100, 300)
    struct.pack_into("<4H", sensor, 28, 212, 1000, 0, 1500)
    struct.pack_into("<H", sensor, 36, 0x1FFF)
    add("sensor_zero", 1, sensor, ack=0xFFFFFFFF)
    struct.pack_into("<2i", sensor, 8, -3, 7)
    struct.pack_into("<H", sensor, 36, 0x1F9F)
    add("sensor_one_no_new_imu", 1, sensor, seq=1, time=1000)
    sensor[7] = 8
    add("sensor_timeout_overrides_valid", 1, sensor, seq=0, time=0)
    sensor[7] = 0
    struct.pack_into("<H", sensor, 36, 0x179F)
    struct.pack_into("<H", sensor, 38, 65535)
    add("sensor_timing_fault_metadata", 1, sensor, seq=1, time=1000)
    output = bytearray(32)
    struct.pack_into("<hhHBBHBBIII4H", output, 0, 1250, -625, 1500,
                     0, 1, 2, 3, 0, 960, 5, 980, 1, 2, 3, 65535)
    add("output_armed", 2, output, time=1000)
    output = bytearray(32)
    output[7] = 4
    add("output_inhibited_uninitialized", 2, output, time=0xFFFFFFF0,
        ack=0xFFFFFFFF)
    add("disarm_stale_session", 6, struct.pack("<I", 2), session=0,
        ack=0xFFFFFFFF)
    host_time = 0x0102030405060708
    add("sync_request", 7, struct.pack("<Q", host_time), time=host_time & 0xFFFFFFFF,
        ack=0xFFFFFFFF)
    add("sync_reply_timer_wrap", 8, struct.pack("<QII", host_time,
                                               0xFFFFFFF0, 0), time=0x20)
    add("reject_version", 6, struct.pack("<I", 1), version=2, accepted=False)
    add("reject_unknown_type", 255, bytes(104), accepted=False)
    add("reject_control_session_copy", 5, struct.pack("<2I", NONCE, 1),
        accepted=False)
    add("reject_disarm_reason", 6, struct.pack("<I", 0), accepted=False)
    add("reject_capability", 0, struct.pack("<4I", NONCE, 32, 921600, 0x50494C01),
        session=0, ack=0xFFFFFFFF, accepted=False)
    add("reject_schema", 0, struct.pack("<4I", NONCE, 31, 921600, 0),
        session=0, ack=0xFFFFFFFF, accepted=False)
    sensor[6] = 4
    add("reject_button_reserved", 1, sensor, seq=1, time=1000, accepted=False)
    sensor[6] = 0
    sensor[0:2] = struct.pack("<H", 4096)
    add("reject_adc_range", 1, sensor, seq=1, time=1000, accepted=False)
    sensor[0:2] = struct.pack("<H", 100)
    add("reject_source_phase", 1, sensor, seq=1, time=1001, accepted=False)
    add("reject_source_seq", 1, sensor, seq=2, time=1000, accepted=False)
    sensor[36:38] = struct.pack("<H", 0x1FFF)
    add("reject_imu_new_phase", 1, sensor, seq=1, time=1000, accepted=False)
    output[11] = 1
    add("reject_output_reserved", 2, output, accepted=False)
    output[11] = 0
    output[0:2] = struct.pack("<h", 1)
    add("reject_inhibited_duty", 2, output, accepted=False)
    return dict(schema_id="50494c01", version=1,
                crc_check_123456789="e3069283", vectors=result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    assert crc32c(b"123456789") == 0xE3069283
    text = json.dumps(vectors(), indent=2) + "\n"
    if args.check:
        assert DEST.read_text() == text, "PIL fixtures differ from reference"
    else:
        DEST.parent.mkdir(parents=True, exist_ok=True)
        DEST.write_text(text)


if __name__ == "__main__":
    main()
