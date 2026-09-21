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
        if key not in REQUIRED_FIELDS:
            return None, "unknown field"
        if key in fields:
            return None, "duplicate field"
        if not value or len(value) > 64:
            return None, "field value is empty or too long"
        if any(ord(character) < 32 or ord(character) > 126
               for character in value):
            return None, "field value contains control text"
        fields[key] = value

    if set(fields) != REQUIRED_FIELDS:
        return None, "required field is missing"

    try:
        fields["ver"] = int(fields["ver"], 10)
        fields["seq"] = int(fields["seq"], 10)
        fields["ms"] = int(fields["ms"], 10)
        fields["link"] = int(fields["link"], 10)
    except ValueError:
        return None, "numeric field is invalid"

    if fields["ver"] < 0 or fields["seq"] < 0 or fields["ms"] < 0:
        return None, "numeric field is negative"
    return fields, None


def report_packet(fields, source, last_sequences):
    """Print one valid packet and report sequence gaps for its session."""
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

    device = fields["device"]
    print("[{0}] {1}:{2} ver={3} device={4} session={5} seq={6} "
          "pico_ms={7} link={8}".format(
              timestamp(), source[0], source[1], fields["ver"], device,
              session, sequence, fields["ms"], fields["link"]),
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
                        report_packet(fields, source, last_sequences)

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
