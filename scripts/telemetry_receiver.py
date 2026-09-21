#!/usr/bin/env python3
"""Receive Pico W version 2 instrument telemetry over UDP."""

import argparse
import json
import socket
import time
from datetime import datetime, timezone

MAX_PACKET_BYTES = 1400
SCHEMA_VERSION = 2
STALE_SECONDS = 5.0
MAX_STRING_LENGTH = 64
MAX_U32 = 0xFFFFFFFF
MIN_I32 = -0x80000000
READING_STRING_FIELDS = {"state", "motor_1_direction", "motor_2_direction"}
READING_INTEGER_FIELDS = {
    "motor_1_speed_mrpm", "motor_2_speed_mrpm", "motor_1_duty_pct",
    "motor_2_duty_pct", "range_mm", "ir_mv", "supply_mv",
    "distance_travelled_mm", "progress_pct",
}
REQUIRED_ENVELOPE = {"ver", "device", "session", "seq", "ms", "link",
                     "test"}


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def is_identifier(value):
    if not isinstance(value, str) or not value or len(value) > MAX_STRING_LENGTH:
        return False
    return all(character.isascii() and
               (character.isalnum() or character in "-_")
               for character in value)


def reject_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def validate_integer(value, key, signed=False):
    if isinstance(value, bool) or not isinstance(value, int):
        return "{0} must be an integer".format(key)
    if signed:
        if value < MIN_I32 or value > MAX_U32:
            return "{0} is outside the integer range".format(key)
    elif value < 0 or value > MAX_U32:
        return "{0} is outside the unsigned integer range".format(key)
    return None


def validate_readings(readings):
    if not isinstance(readings, dict):
        return "readings must be an object"
    if "state" not in readings:
        return "readings requires field 'state'"
    if not is_identifier(readings["state"]):
        return "state must be a safe bounded string"

    for key, value in readings.items():
        if not isinstance(key, str) or not is_identifier(key):
            return "reading key is invalid"
        if key in READING_STRING_FIELDS:
            if not is_identifier(value):
                return "{0} must be a safe bounded string".format(key)
        elif key in READING_INTEGER_FIELDS:
            error = validate_integer(value, key)
            if error is not None:
                return error
            if key.endswith("duty_pct") and value > 100:
                return "{0} is outside the percent range".format(key)
            if key == "progress_pct" and value > 100:
                return "progress_pct is outside the percent range"
        else:
            # Optional readings may be added without a version bump. Keep old
            # receivers safe by accepting only bounded scalar values.
            if isinstance(value, bool) or not isinstance(value, (int, str)):
                return "unknown reading has invalid type"
            if isinstance(value, int):
                error = validate_integer(value, key, signed=value < 0)
                if error is not None:
                    return error
            elif not is_identifier(value):
                return "unknown reading string is invalid"
    return None


def validate_notice(notice):
    if not isinstance(notice, dict):
        return "notice must be an object"
    if set(notice) != {"outcome", "fault"}:
        return "notice must contain only outcome and fault"
    if not is_identifier(notice["outcome"]):
        return "notice outcome must be a safe bounded string"
    error = validate_integer(notice["fault"], "fault")
    if error is not None:
        return error
    if notice["fault"] not in (0, 1):
        return "fault must be 0 or 1"
    return None


def parse_packet(data: bytes):
    if len(data) > MAX_PACKET_BYTES:
        return None, "packet is too large"
    if not data.endswith(b"\n"):
        return None, "packet has no terminating newline"
    try:
        packet = json.loads(data[:-1].decode("utf-8"),
                            object_pairs_hook=reject_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        return None, "invalid JSON: {0}".format(error)

    if not isinstance(packet, dict):
        return None, "packet is not a JSON object"
    if set(packet) - REQUIRED_ENVELOPE - {"readings", "notice"}:
        return None, "unknown envelope field"
    if not REQUIRED_ENVELOPE.issubset(packet):
        return None, "required envelope field is missing"

    error = validate_integer(packet["ver"], "ver")
    if error is not None:
        return None, error
    if packet["ver"] != SCHEMA_VERSION:
        return None, "unsupported schema version: {0}".format(packet["ver"])
    for key in ("session", "seq", "ms"):
        error = validate_integer(packet[key], key)
        if error is not None:
            return None, error
    error = validate_integer(packet["link"], "link", signed=True)
    if error is not None:
        return None, error
    if not is_identifier(packet["device"]):
        return None, "device must be a safe bounded string"
    if not is_identifier(packet["test"]):
        return None, "test must be a safe bounded string"

    has_readings = "readings" in packet
    has_notice = "notice" in packet
    if has_readings == has_notice:
        return None, "packet must contain exactly one of readings or notice"
    error = validate_readings(packet["readings"]) if has_readings \
        else validate_notice(packet["notice"])
    if error is not None:
        return None, error
    return packet, None


def report_packet(packet, source, last_sequences):
    session = packet["session"]
    sequence = packet["seq"]
    previous = last_sequences.get(session)
    if previous is not None:
        if sequence > previous + 1:
            print("[{0}] sequence gap session={1}: missing {2}..{3}".format(
                timestamp(), session, previous + 1, sequence - 1), flush=True)
        elif sequence <= previous:
            print("[{0}] out-of-order or duplicate session={1}: seq={2} "
                  "previous={3}".format(
                      timestamp(), session, sequence, previous), flush=True)
        if sequence > previous:
            last_sequences[session] = sequence
    else:
        last_sequences[session] = sequence

    if "readings" in packet:
        values = packet["readings"]
        details = " ".join("{0}={1}".format(key, values[key])
                           for key in values)
        kind = "readings " + details
    else:
        notice = packet["notice"]
        kind = "notice outcome={0} fault={1}".format(
            notice["outcome"], notice["fault"])
    print("[{0}] {1}:{2} ver={3} device={4} session={5} seq={6} "
          "pico_ms={7} link={8} test={9} {10}".format(
              timestamp(), source[0], source[1], packet["ver"],
              packet["device"], session, sequence, packet["ms"],
              packet["link"], packet["test"], kind), flush=True)


def packet_bytes(packet):
    return (json.dumps(packet, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("ascii")


def self_test() -> int:
    base = {
        "ver": 2, "device": "self-test", "session": 1, "seq": 0,
        "ms": 0, "link": 3, "test": "probe",
    }
    valid_readings = dict(base, readings={
        "state": "running", "motor_1_speed_mrpm": 12345,
        "motor_1_duty_pct": 10, "motor_1_direction": "forward",
        "range_mm": 250, "ir_mv": 1800, "progress_pct": 42,
    })
    valid_notice = dict(base, seq=1,
                        notice={"outcome": "stalled", "fault": 1})
    for packet in (valid_readings, valid_notice):
        parsed, error = parse_packet(packet_bytes(packet))
        assert parsed is not None, error

    unknown_version = dict(valid_readings, ver=3)
    missing_field = dict(valid_readings)
    del missing_field["seq"]
    wrong_type = dict(valid_readings, device=123)
    bad_cases = [
        (b"{\"ver\":2\n", "malformed"),
        (b"x" * (MAX_PACKET_BYTES + 1), "oversized"),
        (packet_bytes(unknown_version), "unknown version"),
        (packet_bytes(missing_field), "missing envelope field"),
        (packet_bytes(wrong_type), "wrong field type"),
    ]
    for data, name in bad_cases:
        parsed, error = parse_packet(data)
        assert parsed is None and error, name
    print("Telemetry receiver self-test passed (schema v2).", flush=True)
    return 0


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0",
                        help="bind address (default: 0.0.0.0)")
    parser.add_argument("--port", default=4242, type=int,
                        help="UDP port (default: 4242)")
    parser.add_argument("--self-test", action="store_true",
                        help="validate accepted and rejected packet examples")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    if arguments.self_test:
        return self_test()
    if not 1 <= arguments.port <= 65535:
        print("port must be between 1 and 65535")
        return 2

    last_sequences = {}
    malformed_packets = 0
    last_packet_time = None
    stale_reported = False
    started = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind((arguments.bind, arguments.port))
        receiver.settimeout(1.0)
        print("Listening on {0}:{1}; telemetry schema v{2}; "
              "unauthenticated LAN UDP.".format(
                  arguments.bind, arguments.port, SCHEMA_VERSION), flush=True)
        try:
            while True:
                try:
                    data, source = receiver.recvfrom(MAX_PACKET_BYTES + 1)
                except socket.timeout:
                    data = None

                now = time.monotonic()
                if data is not None:
                    packet, error = parse_packet(data)
                    if error is not None:
                        malformed_packets += 1
                        print("[{0}] malformed packet #{1} from {2}:{3}: {4}"
                              .format(timestamp(), malformed_packets,
                                      source[0], source[1], error),
                              flush=True)
                    else:
                        last_packet_time = now
                        stale_reported = False
                        report_packet(packet, source, last_sequences)

                reference = last_packet_time if last_packet_time is not None \
                    else started
                if not stale_reported and now - reference > STALE_SECONDS:
                    print("[{0}] WARNING: no telemetry packet for more than "
                          "{1:.0f} seconds.".format(
                              timestamp(), STALE_SECONDS), flush=True)
                    stale_reported = True
        except KeyboardInterrupt:
            print("\nReceiver stopped by operator.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
