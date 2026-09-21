#!/usr/bin/env python3
"""Receive unauthenticated Pico UDP telemetry and report packet loss.

This listener accepts unauthenticated UDP from the LAN. Restrict UDP port
4242 with a firewall rule to the local network or the Pico's reserved DHCP
address. Packet content is untrusted and is never evaluated or used as a path.
"""

import argparse
import socket
import time
from datetime import datetime, timezone

MAX_PACKET_BYTES = 2048
STALE_SECONDS = 5.0
REQUIRED_FIELDS = {"ver", "device", "session", "seq", "ms", "link"}
OPTIONAL_FIELDS = {
    "test", "state", "direction", "startup", "stall", "emergency", "error",
    "motor", "duty", "revolutions", "revolution", "absolute_count", "elapsed_ms",
    "ideal_counts", "target_counts", "count_at_stop", "absolute_count_at_stop",
    "count_at_trigger", "absolute_count_at_trigger", "count_after_settle",
    "absolute_count_after_settle", "coast_overshoot", "boundary_count", "undershoot",
    "drift_counts", "drift_mdeg", "drift_per_rev_mdeg", "elapsed_us",
    "mean_us_per_rev", "mean_ms_per_rev", "counts_per_rev_milli",
    "sensitivity_milli_counts", "jog_counts", "jog_final_absolute_count",
    "jog_combined_count",
    "smallest_jog_movement", "alignment_resolution_cpr_milli",
    "jog_primary_cpr_milli", "jog_secondary_cpr_milli", "split_index",
    "split_total", "split_revolution", "split_absolute_count", "split_elapsed_us",
    "split_delta_counts", "split_delta_us", "completed",
}
OPTIONAL_DISPLAY_ORDER = (
    "test", "state", "direction", "motor", "duty", "revolutions", "revolution",
    "absolute_count", "elapsed_ms", "ideal_counts", "target_counts",
    "count_at_stop", "absolute_count_at_stop", "count_at_trigger",
    "absolute_count_at_trigger", "count_after_settle", "absolute_count_after_settle",
    "coast_overshoot", "boundary_count", "undershoot", "drift_counts", "drift_mdeg",
    "drift_per_rev_mdeg", "elapsed_us", "mean_us_per_rev", "mean_ms_per_rev",
    "counts_per_rev_milli", "sensitivity_milli_counts", "jog_counts",
    "jog_final_absolute_count", "jog_combined_count", "smallest_jog_movement", "alignment_resolution_cpr_milli",
    "jog_primary_cpr_milli", "jog_secondary_cpr_milli", "split_index", "split_total",
    "split_revolution", "split_absolute_count", "split_elapsed_us",
    "split_delta_counts", "split_delta_us", "startup", "stall", "emergency", "error",
    "completed",
)
KNOWN_FIELDS = REQUIRED_FIELDS | OPTIONAL_FIELDS
NUMERIC_FIELDS = {
    "ver", "seq", "ms", "link", "motor", "duty", "revolutions", "revolution",
    "absolute_count", "elapsed_ms", "ideal_counts", "target_counts", "count_at_stop",
    "absolute_count_at_stop", "count_at_trigger", "absolute_count_at_trigger",
    "count_after_settle", "absolute_count_after_settle", "coast_overshoot",
    "boundary_count", "undershoot", "drift_counts", "drift_mdeg", "drift_per_rev_mdeg",
    "elapsed_us", "mean_us_per_rev", "mean_ms_per_rev", "counts_per_rev_milli",
    "sensitivity_milli_counts", "jog_counts", "jog_final_absolute_count",
    "jog_combined_count", "smallest_jog_movement",
    "alignment_resolution_cpr_milli", "jog_primary_cpr_milli", "jog_secondary_cpr_milli",
    "split_index", "split_total", "split_revolution", "split_absolute_count",
    "split_elapsed_us", "split_delta_counts", "split_delta_us", "completed",
}


def timestamp() -> str:
    """Return a bounded, host-side UTC timestamp."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def parse_packet(data: bytes):
    """Parse one bounded telemetry line and return fields or an error."""
    if len(data) > MAX_PACKET_BYTES:
        return None, "packet is too large"
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        return None, "packet is not ASCII"
    if not text.endswith("\n"):
        return None, "packet has no terminating newline"

    line = text[:-1]
    if "\n" in line or "\r" in line:
        return None, "packet contains an unexpected line break"

    fields = {}
    for item in line.split(","):
        if item.count("=") != 1:
            return None, "field is not key=value"
        key, value = item.split("=", 1)
        if key not in KNOWN_FIELDS:
            return None, "unknown field"
        if key in fields:
            return None, "duplicate field"
        if not value or len(value) > 64:
            return None, "field value is empty or too long"
        if any(ord(character) < 32 or ord(character) > 126
               for character in value):
            return None, "field value contains control text"
        fields[key] = value

    if not REQUIRED_FIELDS.issubset(fields):
        return None, "required field is missing"

    for key in NUMERIC_FIELDS.intersection(fields):
        try:
            fields[key] = int(fields[key], 10)
        except ValueError:
            return None, "numeric field is invalid"

    if any(fields[key] < 0 for key in ("ver", "seq", "ms")):
        return None, "numeric field is negative"
    return fields, None


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


def report_split_status(fields, key, series, split_reports):
    """Report the current split completeness at a completed summary."""
    total = series["total"]
    missing = missing_ranges(series["received"], total)
    previous_report = split_reports.get(key)
    if missing == previous_report:
        return
    split_reports[key] = missing
    if missing:
        print("[{0}] completed split session={1}: missing {2} of {3}"
              .format(timestamp(), fields["session"], missing, total), flush=True)
    else:
        print("[{0}] completed split session={1}: all {2} splits received"
              .format(timestamp(), fields["session"], total), flush=True)
        for split_index in sorted(series["items"]):
            split = series["items"][split_index]
            print("[{0}] split session={1} revolution={2} count={3} "
                  "elapsed_us={4} delta_counts={5} delta_us={6}".format(
                      timestamp(), fields["session"],
                      split.get("split_revolution", "?"),
                      split.get("split_absolute_count", "?"),
                      split.get("split_elapsed_us", "?"),
                      split.get("split_delta_counts", "?"),
                      split.get("split_delta_us", "?")), flush=True)


def process_completed_packet(fields, completed_seen, split_series,
                             split_reports):
    """Deduplicate completed packets and track their split series."""
    if fields.get("completed") != 1:
        return False, False

    key = (fields["session"], fields["completed"])
    duplicate = key in completed_seen
    completed_seen.add(key)
    if "split_index" not in fields:
        series = split_series.get(key)
        if series is not None:
            report_split_status(fields, key, series, split_reports)
        return duplicate, False

    index = fields["split_index"]
    total = fields.get("split_total")
    if (total is None or total <= 0 or total > 4096 or
            index < 0 or index >= total):
        print("[{0}] invalid completed split session={1} index={2} total={3}"
              .format(timestamp(), fields["session"], index, total), flush=True)
        return duplicate, True

    series = split_series.setdefault(
        key, {"total": total, "received": set(), "items": {}})
    if series["total"] != total:
        print("[{0}] split total changed session={1}: {2} to {3}"
              .format(timestamp(), fields["session"], series["total"], total),
              flush=True)
        return duplicate, True
    series["received"].add(index)
    series["items"][index] = fields
    return duplicate, True


def report_packet(fields, source, last_sequences, display=True):
    """Track sequence gaps and optionally print one valid packet."""
    session = fields["session"]
    sequence = fields["seq"]
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

    device = fields["device"]
    optional_display = " ".join(
        "{0}={1}".format(key, fields[key])
        for key in OPTIONAL_DISPLAY_ORDER if key in fields)
    if optional_display:
        optional_display = " " + optional_display
    print("[{0}] {1}:{2} ver={3} device={4} session={5} seq={6} "
          "pico_ms={7} link={8}{9}".format(
              timestamp(), source[0], source[1], fields["ver"], device,
              session, sequence, fields["ms"], fields["link"],
              optional_display),
          flush=True)


def parse_arguments():
    """Parse the optional bind address and port."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0",
                        help="bind address (default: 0.0.0.0)")
    parser.add_argument("--port", default=4242, type=int,
                        help="UDP port (default: 4242)")
    return parser.parse_args()


def main() -> int:
    """Run the UDP receiver until interrupted."""
    arguments = parse_arguments()
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
                    fields, error = parse_packet(data)
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
                            fields, completed_seen, split_series, split_reports)
                        report_packet(fields, source, last_sequences,
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
