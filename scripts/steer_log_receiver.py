#!/usr/bin/env python3
"""Receive the steer_test Wi-Fi log (UDP broadcast, port 4243).

Packet format: src/steer_log.h and docs/STEER_WIFI_LOG.md.

The robot sends:
  - TEXT packets: its printf output (spin, calibration, run summary);
  - LOG packets: 50 ms log entries, live during a run, and again as a
    replay when the run stops, so lost live packets can be filled in.

All packet content is untrusted input: sizes are checked, and text is
shown without control characters.

  python3 scripts/steer_log_receiver.py            # listen, print, save
  python3 scripts/steer_log_receiver.py --self-test
"""

import argparse
import pathlib
import socket
import sys
from datetime import datetime

PORT = 4243
VERSION = 1
TYPE_LOG = 1
TYPE_TEXT = 2
HEADER_BYTES = 5
LOG_HEADER_BYTES = 5
ENTRY_BYTES = 11
MAX_ENTRIES = 20
FLAG_REPLAY = 0x01
FLAG_REPLAY_END = 0x02
LOG_PERIOD_MS = 50

MODES = {0: "FOLW", 1: "SRCH"}
SIDES = {0: "-", 1: "L", 2: "R", 3: "?"}
SOURCES = {0: "-", 1: "sensor", 2: "gap", 3: "hint"}

# The same vector is packed by tests/test_steer_log.c.
VECTOR = bytes([0xDB, 0x0D, 0xEC, 0xA5, 0xB1, 0xC0, 0x8F, 0xFF, 0x0F, 0xFF, 0x7F])
VECTOR_FIELDS = dict(position=-37, left_pct=13, right_pct=-20, info=0xA5,
                     raw=(177, 2300, 4095), left_counts=-16, right_counts=511)


def bits(value, pos, width):
    return (value >> pos) & ((1 << width) - 1)


def signed(value, width):
    sign = 1 << (width - 1)
    return (value ^ sign) - sign


def decode_entry(data):
    """88-bit entry, least significant bit first."""
    v = int.from_bytes(data, "little")
    return dict(
        position=signed(bits(v, 0, 8), 8),
        left_pct=signed(bits(v, 8, 8), 8),
        right_pct=signed(bits(v, 16, 8), 8),
        info=bits(v, 24, 8),
        raw=(bits(v, 32, 12), bits(v, 44, 12), bits(v, 56, 12)),
        left_counts=signed(bits(v, 68, 10), 10),
        right_counts=signed(bits(v, 78, 10), 10),
    )


TABLE_HEADER = "  t_ms mode  pos  dutyL dutyR  rawL rawC rawR  side src    hint  cntL cntR"


def format_entry(index, e):
    info = e["info"]
    return ("%6d %s %+4d  %3d %3d  %4d %4d %4d  %s %-6s %s  %4d %4d" % (
        index * LOG_PERIOD_MS, MODES.get(info & 3, "?"), e["position"],
        e["left_pct"], e["right_pct"], e["raw"][0], e["raw"][1], e["raw"][2],
        SIDES[(info >> 2) & 3], SOURCES[(info >> 4) & 3], SIDES[(info >> 6) & 3],
        e["left_counts"], e["right_counts"]))


def clean_text(data):
    text = data.decode("utf-8", errors="replace").replace("\r", "")
    return "".join(c for c in text if c == "\n" or c.isprintable())


class Receiver:
    def __init__(self, out_dir, live):
        self.out_dir = out_dir
        self.live = live
        self.next_seq = {}   # boot id -> expected sequence number
        self.lost = {}       # boot id -> lost packets
        self.runs = {}       # (boot, run) -> {"entries": {index: e}, "live": set()}
        self.text_tail = {}  # boot id -> unfinished text line

    def handle(self, data):
        if len(data) < HEADER_BYTES:
            return
        ptype, version = data[0] & 0x0F, data[0] >> 4
        if version != VERSION:
            print("[recv] ignored packet with version %d" % version)
            return
        boot = int.from_bytes(data[1:3], "little")
        seq = int.from_bytes(data[3:5], "little")
        self.track_seq(boot, seq)
        body = data[HEADER_BYTES:]
        if ptype == TYPE_TEXT:
            self.handle_text(boot, body)
        elif ptype == TYPE_LOG:
            self.handle_log(boot, body)

    def track_seq(self, boot, seq):
        if boot not in self.next_seq:
            print("[recv] robot boot %04x" % boot)
            self.lost[boot] = 0
        else:
            gap = (seq - self.next_seq[boot]) & 0xFFFF
            if 0 < gap < 0x8000:
                self.lost[boot] += gap
        self.next_seq[boot] = (seq + 1) & 0xFFFF

    def handle_text(self, boot, body):
        text = self.text_tail.get(boot, "") + clean_text(body)
        lines = text.split("\n")
        self.text_tail[boot] = lines.pop()
        if not lines:
            return
        path = self.out_dir / ("text_%04x.txt" % boot)
        with path.open("a") as f:
            for line in lines:
                print("pico> " + line)
                f.write(line + "\n")

    def handle_log(self, boot, body):
        if len(body) < LOG_HEADER_BYTES:
            return
        run, flags = body[0], body[1]
        first = int.from_bytes(body[2:4], "little")
        count = body[4]
        if count > MAX_ENTRIES or len(body) != LOG_HEADER_BYTES + count * ENTRY_BYTES:
            print("[recv] ignored LOG packet with a bad size")
            return
        state = self.runs.setdefault((boot, run), {"entries": {}, "live": set()})
        replay = bool(flags & FLAG_REPLAY)
        if self.live and not replay and not state["live"]:
            print("[log] run %d\n[log] %s" % (run, TABLE_HEADER))
        for i in range(count):
            off = LOG_HEADER_BYTES + i * ENTRY_BYTES
            e = decode_entry(body[off:off + ENTRY_BYTES])
            index = first + i
            state["entries"][index] = e
            if not replay:
                state["live"].add(index)
                if self.live:
                    print("[log] " + format_entry(index, e))
        if flags & FLAG_REPLAY_END:
            self.save_run(boot, run, state)

    def save_run(self, boot, run, state):
        entries = state["entries"]
        indices = sorted(entries)
        if not indices:
            return
        expected = indices[-1] - indices[0] + 1
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = self.out_dir / ("run_%s_boot%04x_run%d.txt" % (stamp, boot, run))
        with path.open("w") as f:
            f.write("# steer_test run %d, boot %04x, saved %s\n" % (run, boot, stamp))
            f.write("# entries %d of %d; received live %d; packets lost %d\n" % (
                len(indices), expected, len(state["live"]), self.lost.get(boot, 0)))
            f.write(TABLE_HEADER + "\n")
            for index in indices:
                f.write(format_entry(index, entries[index]) + "\n")
        print("[recv] run %d saved to %s (%d of %d entries; %d live)" % (
            run, path, len(indices), expected, len(state["live"])))
        state["live"] = set()


def self_test():
    failures = 0
    got = decode_entry(VECTOR)
    for key, want in VECTOR_FIELDS.items():
        if got[key] != want:
            print("FAIL %s: %r, want %r" % (key, got[key], want))
            failures += 1
    # A LOG packet with 2 entries, then a replay end, through the receiver.
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        r = Receiver(pathlib.Path(d), live=False)
        header = bytes([TYPE_LOG | (VERSION << 4), 0xEF, 0xBE, 0, 0])
        r.handle(header + bytes([3, 0, 10, 0, 2]) + VECTOR + VECTOR)
        header2 = bytes([TYPE_LOG | (VERSION << 4), 0xEF, 0xBE, 2, 0])  # seq 1 lost
        r.handle(header2 + bytes([3, FLAG_REPLAY | FLAG_REPLAY_END, 10, 0, 1]) + VECTOR)
        files = list(pathlib.Path(d).glob("run_*_bootbeef_run3.txt"))
        if len(files) != 1:
            print("FAIL run file not written")
            failures += 1
        else:
            text = files[0].read_text()
            if "entries 2 of 2; received live 2; packets lost 1" not in text:
                print("FAIL run file header:\n" + text)
                failures += 1
            if format_entry(10, VECTOR_FIELDS | {"raw": (177, 2300, 4095)}) not in text:
                print("FAIL run file entry")
                failures += 1
        # Bad sizes are ignored.
        r.handle(header + bytes([3, 0, 0, 0, 5]) + VECTOR)
        r.handle(b"\x21")
    if failures:
        print("steer_log_receiver self-test: %d failure(s)" % failures)
        return 1
    print("steer_log_receiver self-test: all tests passed")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--out", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parent.parent
                        / "build" / "steer_logs",
                        help="folder for the saved runs and text")
    parser.add_argument("--quiet", action="store_true",
                        help="do not print live log entries")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()

    args.out.mkdir(parents=True, exist_ok=True)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print("[recv] listening on UDP port %d; saving to %s" % (args.port, args.out))
    receiver = Receiver(args.out, live=not args.quiet)
    try:
        while True:
            data, _ = sock.recvfrom(2048)
            receiver.handle(data)
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[recv] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
