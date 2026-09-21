#!/usr/bin/env python3
"""Receive Pico JSON telemetry and report packet loss.

This listener accepts unauthenticated UDP from the LAN. Restrict UDP port
4242 with a firewall rule to the local network or the Pico's reserved DHCP
address. Packet content is untrusted and is never evaluated or used as a path.
"""

import argparse
import json
import socket
import time
from datetime import datetime, timezone

MAX_PACKET_BYTES = 1400
SCHEMA_VERSION = 1
STALE_SECONDS = 5.0
MAX_STRING_LENGTH = 64
MAX_U32 = 0xFFFFFFFF
MAX_I32 = 0x7FFFFFFF
MIN_I32 = -0x80000000
KNOWN_TEST_TYPES = {
    "probe", "rotation_drift", "encoder_calibration", "duty_calibration",
    "speed_calibration", "board_diagnostics",
}
REQUIRED_ENVELOPE = {"ver", "device", "session", "seq", "ms", "link",
                     "test", "payload"}
ROTATION_STRING_FIELDS = {
    "state", "direction", "startup", "stall", "emergency", "error",
}
ROTATION_NUMERIC_FIELDS = {
    "motor", "duty_pct", "revolutions_requested", "revolution_index",
    "absolute_count", "elapsed_ms", "ideal_counts", "target_counts",
    "count_at_stop", "absolute_count_at_stop", "count_at_trigger",
    "absolute_count_at_trigger", "count_after_settle",
    "absolute_count_after_settle", "coast_overshoot_counts",
    "boundary_count", "undershoot_counts", "drift_counts", "drift_mdeg",
    "drift_per_rev_mdeg", "elapsed_us", "mean_us_per_rev",
    "mean_ms_per_rev", "counts_per_rev_milli",
    "sensitivity_millicounts_per_degree", "jog_counts",
    "jog_final_absolute_count", "jog_combined_count",
    "smallest_jog_movement_counts", "alignment_resolution_cpr_milli",
    "jog_primary_cpr_milli", "jog_secondary_cpr_milli", "split_index",
    "split_total", "split_revolution", "split_absolute_count",
    "split_elapsed_us", "split_delta_counts", "split_delta_us", "completed",
}
ROTATION_FIELDS = ROTATION_STRING_FIELDS | ROTATION_NUMERIC_FIELDS
ROTATION_COMMON_TERMINAL_FIELDS = {
    "state", "direction", "startup", "stall", "emergency", "error", "motor",
    "duty_pct", "revolutions_requested", "ideal_counts", "target_counts",
    "count_at_stop", "absolute_count_at_stop", "count_after_settle",
    "absolute_count_after_settle", "coast_overshoot_counts", "boundary_count",
    "elapsed_ms", "completed",
}
ROTATION_STATE_REQUIRED_FIELDS = {
    "running": {
        "state", "direction", "startup", "stall", "emergency", "error", "motor",
        "duty_pct", "revolutions_requested", "revolution_index", "absolute_count",
        "elapsed_ms", "completed",
    },
    "jogging": {
        "state", "direction", "motor", "duty_pct", "absolute_count", "jog_counts",
        "jog_final_absolute_count", "jog_combined_count", "smallest_jog_movement_counts",
        "alignment_resolution_cpr_milli", "jog_primary_cpr_milli",
        "jog_secondary_cpr_milli", "completed",
    },
    "completed": ROTATION_COMMON_TERMINAL_FIELDS | {
        "count_at_trigger", "absolute_count_at_trigger", "undershoot_counts",
        "drift_counts", "drift_mdeg", "drift_per_rev_mdeg", "elapsed_us",
        "mean_us_per_rev", "mean_ms_per_rev", "counts_per_rev_milli",
        "sensitivity_millicounts_per_degree",
    },
    "completed_split": {
        "state", "direction", "motor", "split_index", "split_total",
        "split_revolution", "split_absolute_count", "split_elapsed_us",
        "split_delta_counts", "split_delta_us", "completed",
    },
    "stalled": ROTATION_COMMON_TERMINAL_FIELDS,
    "emergency_stop": ROTATION_COMMON_TERMINAL_FIELDS,
    "safety_timeout": ROTATION_COMMON_TERMINAL_FIELDS,
    "incomplete": ROTATION_COMMON_TERMINAL_FIELDS,
}
DISPLAY_PAYLOAD_ORDER = (
    "state", "direction", "motor", "duty_pct", "revolutions_requested",
    "revolution_index", "absolute_count", "elapsed_ms", "ideal_counts",
    "target_counts", "count_at_stop", "absolute_count_at_stop",
    "count_at_trigger", "absolute_count_at_trigger", "count_after_settle",
    "absolute_count_after_settle", "coast_overshoot_counts", "boundary_count",
    "undershoot_counts", "drift_counts", "drift_mdeg", "drift_per_rev_mdeg",
    "elapsed_us", "mean_us_per_rev", "mean_ms_per_rev", "counts_per_rev_milli",
    "sensitivity_millicounts_per_degree", "jog_counts",
    "jog_final_absolute_count", "jog_combined_count",
    "smallest_jog_movement_counts", "alignment_resolution_cpr_milli",
    "jog_primary_cpr_milli", "jog_secondary_cpr_milli", "split_index",
    "split_total", "split_revolution", "split_absolute_count",
    "split_elapsed_us", "split_delta_counts", "split_delta_us", "startup",
    "stall", "emergency", "error", "completed",
)


def timestamp() -> str:
    """Return a bounded, host-side UTC timestamp."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def is_identifier(value):
    """Accept only the schema's safe identifier alphabet."""
    if not isinstance(value, str) or not value or len(value) > MAX_STRING_LENGTH:
        return False
    return all(character.isascii() and
               (character.isalnum() or character in "-_")
               for character in value)


def reject_duplicate_keys(pairs):
    """Reject duplicate JSON keys instead of silently keeping the last one."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def validate_integer(value, key, signed=False):
    """Validate one protocol integer and return an error string if invalid."""
    if isinstance(value, bool) or not isinstance(value, int):
        return "{0} must be an integer".format(key)
    if signed:
        if value < MIN_I32 or value > MAX_U32:
            return "{0} is outside the integer range".format(key)
    elif value < 0 or value > MAX_U32:
        return "{0} is outside the unsigned integer range".format(key)
    return None


def validate_payload(test_type, payload):
    """Validate the selected test payload without guessing unknown values."""
    if not isinstance(payload, dict):
        return "payload must be an object"

    if test_type == "rotation_drift":
        known_fields = ROTATION_FIELDS
        required = {"state", "completed"}
    else:
        known_fields = {"state"}
        required = {"state"}

    if not required.issubset(payload):
        return "required payload field is missing"

    for key, value in payload.items():
        if not isinstance(key, str) or not is_identifier(key):
            return "payload key is invalid"
        if key == "state" or (key in known_fields and
                               key in ROTATION_STRING_FIELDS):
            if not is_identifier(value):
                return "{0} must be a safe string".format(key)
        elif key in known_fields and key in ROTATION_NUMERIC_FIELDS:
            error = validate_integer(value, key, signed=key in {
                "count_at_stop", "count_at_trigger", "count_after_settle",
                "coast_overshoot_counts", "drift_counts", "drift_mdeg",
                "drift_per_rev_mdeg", "jog_counts", "split_delta_counts",
            })
            if error is not None:
                return error
        elif key not in known_fields:
            # Optional fields may be added without a version bump. Accept only
            # bounded scalar values so an old receiver never guesses a shape.
            if isinstance(value, bool) or not isinstance(value, (int, str)):
                return "unknown payload field has invalid type"
            if isinstance(value, int):
                error = validate_integer(value, key, signed=value < 0)
                if error is not None:
                    return error
            elif not is_identifier(value):
                return "unknown payload string is invalid"

    if test_type == "rotation_drift":
        state = payload["state"]
        if state not in ROTATION_STATE_REQUIRED_FIELDS:
            return "unknown rotation_drift state: {0}".format(state)
        missing = ROTATION_STATE_REQUIRED_FIELDS[state] - set(payload)
        if missing:
            return "state '{0}' requires field '{1}'".format(
                state, sorted(missing)[0])
        completed = payload["completed"]
        if completed not in (0, 1):
            return "completed must be 0 or 1"
        if state in ("running", "jogging") and completed != 0:
            return "running and jogging require completed=0"
        if state not in ("running", "jogging") and completed != 1:
            return "terminal states require completed=1"
        if "split_index" in payload:
            split_fields = {"split_index", "split_total", "split_revolution",
                            "split_absolute_count", "split_elapsed_us",
                            "split_delta_counts", "split_delta_us"}
            if not split_fields.issubset(payload):
                return "split payload field is missing"
            if payload["split_total"] <= 0 or payload["split_total"] > 4096:
                return "split_total is outside the supported range"
            if payload["split_index"] < 0 or \
                    payload["split_index"] >= payload["split_total"]:
                return "split_index is outside split_total"
    return None


def parse_packet(data: bytes):
    """Parse and validate one bounded JSON telemetry packet."""
    if len(data) > MAX_PACKET_BYTES:
        return None, "packet is too large"
    if not data.endswith(b"\n"):
        return None, "packet has no terminating newline"
    try:
        text = data[:-1].decode("utf-8")
        packet = json.loads(text, object_pairs_hook=reject_duplicate_keys)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        return None, "invalid JSON: {0}".format(error)

    if not isinstance(packet, dict):
        return None, "packet is not a JSON object"
    unknown_envelope = set(packet) - REQUIRED_ENVELOPE
    if unknown_envelope:
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
    if packet["test"] not in KNOWN_TEST_TYPES:
        return None, "unknown test type: {0}".format(packet["test"])

    error = validate_payload(packet["test"], packet["payload"])
    if error is not None:
        return None, error
    return packet, None


def missing_ranges(received, total):
    """Return compact missing split ranges."""
    missing = [index for index in range(total) if index not in received]
    ranges = []
    for index in missing:
        if not ranges or index != ranges[-1][1] + 1:
            ranges.append([index, index])
        else:
            ranges[-1][1] = index
    return ",".join(
        str(start) if start == end else "{0}-{1}".format(start, end)
        for start, end in ranges)


def report_split_status(packet, key, series, split_reports):
    """Report split completeness at a completed summary."""
    total = series["total"]
    missing = missing_ranges(series["received"], total)
    previous_report = split_reports.get(key)
    if missing == previous_report:
        return
    split_reports[key] = missing
    if missing:
        print("[{0}] completed split session={1}: missing {2} of {3}"
              .format(timestamp(), packet["session"], missing, total), flush=True)
    else:
        print("[{0}] completed split session={1}: all {2} splits received"
              .format(timestamp(), packet["session"], total), flush=True)
        for split_index in sorted(series["items"]):
            split = series["items"][split_index]["payload"]
            print("[{0}] split session={1} revolution={2} count={3} "
                  "elapsed_us={4} delta_counts={5} delta_us={6}".format(
                      timestamp(), packet["session"],
                      split.get("split_revolution", "?"),
                      split.get("split_absolute_count", "?"),
                      split.get("split_elapsed_us", "?"),
                      split.get("split_delta_counts", "?"),
                      split.get("split_delta_us", "?")), flush=True)


def process_completed_packet(packet, completed_seen, split_series,
                             split_reports):
    """Deduplicate completed rotation packets and track split series."""
    if packet["test"] != "rotation_drift":
        return False, False
    payload = packet["payload"]
    if payload.get("completed") != 1:
        return False, False

    key = (packet["session"], payload["completed"])
    duplicate = key in completed_seen
    completed_seen.add(key)
    if "split_index" not in payload:
        series = split_series.get(key)
        if series is not None:
            report_split_status(packet, key, series, split_reports)
        return duplicate, False

    index = payload["split_index"]
    total = payload["split_total"]
    series = split_series.setdefault(
        key, {"total": total, "received": set(), "items": {}})
    if series["total"] != total:
        print("[{0}] split total changed session={1}: {2} to {3}"
              .format(timestamp(), packet["session"], series["total"], total),
              flush=True)
        return duplicate, True
    series["received"].add(index)
    series["items"][index] = packet
    return duplicate, True


def report_packet(packet, source, last_sequences, display=True):
    """Track sequence gaps and optionally print one valid packet."""
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

    if not display:
        return
    payload_display = " ".join(
        "{0}={1}".format(key, packet["payload"][key])
        for key in DISPLAY_PAYLOAD_ORDER if key in packet["payload"])
    if payload_display:
        payload_display = " " + payload_display
    print("[{0}] {1}:{2} ver={3} device={4} session={5} seq={6} "
          "pico_ms={7} link={8} test={9}{10}".format(
              timestamp(), source[0], source[1], packet["ver"],
              packet["device"], session, sequence, packet["ms"],
              packet["link"], packet["test"], payload_display),
          flush=True)


def packet_bytes(packet):
    """Encode one self-test object in the wire format."""
    return (json.dumps(packet, separators=(",", ":"),
                       ensure_ascii=True) + "\n").encode("ascii")


def self_test() -> int:
    """Validate good packets and all required bad-packet classes."""
    base = {
        "ver": 1, "device": "self-test", "session": 1, "seq": 0,
        "ms": 0, "link": 3,
    }
    valid_packets = []
    for test_type in sorted(KNOWN_TEST_TYPES - {"rotation_drift"}):
        valid_packets.append(dict(base, test=test_type,
                                  payload={"state": "reserved"}))

    running = {
        "state": "running", "direction": "forward", "startup": "complete",
        "stall": "clear", "emergency": "clear", "error": "none", "motor": 1,
        "duty_pct": 10, "revolutions_requested": 50, "revolution_index": 2,
        "absolute_count": 5000, "elapsed_ms": 5000, "completed": 0,
    }
    jogging = {
        "state": "jogging", "direction": "forward", "motor": 1, "duty_pct": 10,
        "absolute_count": 126478, "jog_counts": 28,
        "jog_final_absolute_count": 126478, "jog_combined_count": 126478,
        "smallest_jog_movement_counts": 3, "alignment_resolution_cpr_milli": 60,
        "jog_primary_cpr_milli": 2529560, "jog_secondary_cpr_milli": 2480,
        "completed": 0,
    }
    completed = {
        "state": "completed", "direction": "forward", "startup": "complete",
        "stall": "clear", "emergency": "clear", "error": "none", "motor": 1,
        "duty_pct": 10, "revolutions_requested": 50, "ideal_counts": 126750,
        "target_counts": 126450, "count_at_stop": 126450,
        "absolute_count_at_stop": 126450, "count_at_trigger": 126450,
        "absolute_count_at_trigger": 126450, "count_after_settle": 126475,
        "absolute_count_after_settle": 126475, "coast_overshoot_counts": 25,
        "boundary_count": 49, "undershoot_counts": 300, "drift_counts": -275,
        "drift_mdeg": -39000, "drift_per_rev_mdeg": -780,
        "elapsed_ms": 128000, "elapsed_us": 128000000,
        "mean_us_per_rev": 2560000, "mean_ms_per_rev": 2560,
        "counts_per_rev_milli": 2529500,
        "sensitivity_millicounts_per_degree": 704, "completed": 1,
    }
    split = {
        "state": "completed_split", "direction": "forward", "motor": 1,
        "split_index": 0, "split_total": 49, "split_revolution": 1,
        "split_absolute_count": 2535, "split_elapsed_us": 2600000,
        "split_delta_counts": 2535, "split_delta_us": 2600000, "completed": 1,
    }
    valid_packets.extend(
        dict(base, test="rotation_drift", payload=payload)
        for payload in [running, jogging, completed, split]
    )
    for state in ("stalled", "emergency_stop", "safety_timeout", "incomplete"):
        failure = {
            "state": state, "direction": "forward", "startup": "complete",
            "stall": "active" if state == "stalled" else "clear",
            "emergency": "active" if state == "emergency_stop" else "clear",
            "error": "timeout" if state == "safety_timeout" else "none",
            "motor": 1, "duty_pct": 10, "revolutions_requested": 50,
            "ideal_counts": 126750, "target_counts": 126450,
            "count_at_stop": 100, "absolute_count_at_stop": 100,
            "count_after_settle": 100, "absolute_count_after_settle": 100,
            "coast_overshoot_counts": 0, "boundary_count": 0,
            "elapsed_ms": 1000, "completed": 1,
        }
        valid_packets.append(dict(base, test="rotation_drift", payload=failure))

    for packet in valid_packets:
        parsed, error = parse_packet(packet_bytes(packet))
        assert parsed is not None, error

    unknown_version = dict(valid_packets[0], ver=2)
    missing_field = dict(base, test="rotation_drift",
                         payload=dict(running))
    del missing_field["payload"]["absolute_count"]
    wrong_type = dict(valid_packets[0], device=123)
    bad_cases = [
        (b"{\"ver\":1\n", "malformed"),
        (b"x" * (MAX_PACKET_BYTES + 1), "oversized"),
        (packet_bytes(unknown_version), "unknown version"),
        (packet_bytes(missing_field), "missing field"),
        (packet_bytes(wrong_type), "wrong type"),
    ]
    for data, name in bad_cases:
        parsed, error = parse_packet(data)
        assert parsed is None and error, name
    print("Telemetry receiver self-test passed.", flush=True)
    return 0


def parse_arguments():
    """Parse listener and self-test options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0",
                        help="bind address (default: 0.0.0.0)")
    parser.add_argument("--port", default=4242, type=int,
                        help="UDP port (default: 4242)")
    parser.add_argument("--self-test", action="store_true",
                        help="validate accepted and rejected packet examples")
    return parser.parse_args()


def main() -> int:
    """Run the UDP receiver until interrupted."""
    arguments = parse_arguments()
    if arguments.self_test:
        return self_test()
    if not 1 <= arguments.port <= 65535:
        print("port must be between 1 and 65535")
        return 2

    last_sequences = {}
    completed_seen = set()
    split_series = {}
    split_reports = {}
    malformed_packets = 0
    last_packet_time = None
    stale_reported = False
    started = time.monotonic()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind((arguments.bind, arguments.port))
        receiver.settimeout(1.0)
        print("Listening on {0}:{1}; unauthenticated LAN UDP.".format(
            arguments.bind, arguments.port), flush=True)

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
                        duplicate_completed, is_split = process_completed_packet(
                            packet, completed_seen, split_series, split_reports)
                        report_packet(packet, source, last_sequences,
                                      display=not duplicate_completed and
                                      not is_split)

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
