#!/usr/bin/env python3
"""Receive the steer_test Wi-Fi log (UDP broadcast, port 4243).

Packet format: src/steer_log.h and docs/STEER_WIFI_LOG.md.

The robot sends:
  - TEXT packets: its printf output (spin, calibration, run summary);
  - LOG packets: 50 ms log entries, live during a run, and again as a
    replay when the run stops, so lost live packets can be filled in.

The receiver sends a HELLO packet to the broadcast address once a second.
The robot then sends to this PC directly (unicast): Wi-Fi repeats lost
unicast frames, but not broadcast frames. A run is saved when the last
replay packet arrives, or 3 s after the last replay packet if it is lost.

All packet content is untrusted input: sizes are checked, and text is
shown without control characters.

  python3 scripts/steer_log_receiver.py            # listen, print, save
  python3 scripts/steer_log_receiver.py --self-test
"""

import argparse
import pathlib
import select
import socket
import sys
import time
from datetime import datetime

PORT = 4243
VERSION = 1
TYPE_LOG = 1
TYPE_TEXT = 2
TYPE_HELLO = 3
TYPE_BYPASS = 4
HELLO_PERIOD_S = 1.0
REPLAY_QUIET_S = 3.0
HEADER_BYTES = 5
LOG_HEADER_BYTES = 5
ENTRY_BYTES = 11
MAX_ENTRIES = 20
FLAG_REPLAY = 0x01
FLAG_REPLAY_END = 0x02
FLAG_BYPASS = 0x04  # run from bypass_test: mode 3 is BYPS
FLAG_DETOUR = 0x08  # with FLAG_BYPASS: run from detour_test
BYPASS_HEADER_BYTES = 4
BYPASS_BYTES = 5
MAX_BYPASS = 40
LOG_PERIOD_MS = 50

# STRT: straight across a junction; CALM: following with the calm settings
MODES = {0: "FOLW", 1: "SRCH", 2: "STRT", 3: "CALM"}
BYPASS_MODES = {0: "FOLW", 1: "SRCH", 2: "STRT", 3: "BYPS"}
BYPASS_STATES = {0: "PIVOT", 1: "AROUND", 2: "DONE", 3: "FAILED"}
BYPASS_PHASES = {0: "DRIVE", 1: "TURN", 2: "EDGE", 3: "?"}
# detour_test: state << 2 | phase is the step (src/obstacle_detour.h).
DETOUR_STEPS = ["BACK", "TURN_OUT", "OUT", "TURN_BACK", "PASS", "TURN_IN", "IN",
                "ON_LINE", "ALIGN", "DONE", "FAILED", "?", "?", "?", "?", "?"]
SIDES = {0: "-", 1: "L", 2: "R", 3: "?"}
SOURCES = {0: "-", 1: "sensor", 2: "gap", 3: "hint"}

# The same vector is packed by tests/test_steer_log.c.
VECTOR = bytes([0xDB, 0x0D, 0xEC, 0xA5, 0xB1, 0xC0, 0x8F, 0xFF, 0x0F, 0xFF, 0x7F])
BYPASS_VECTOR = bytes([0xD2, 0xA4, 0x00, 0x70, 0x2F])  # also in tests/test_steer_log.c
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


def decode_bypass(data):
    """40-bit bypass record, least significant bit first."""
    v = int.from_bytes(data, "little")
    return dict(
        time_10ms=bits(v, 0, 13),
        servo_deg=bits(v, 13, 6) - 30,
        reading=bits(v, 19, 11),
        state=bits(v, 30, 2),
        steer_deg=bits(v, 32, 6) - 30,
        phase=bits(v, 38, 2),
    )


def reading_text(reading):
    """Ultrasonic result bits 0-10: distance 0-8, status 9-10."""
    status, mm = reading >> 9, reading & 0x1FF
    if status == 0:
        return "TOO_NEAR" if mm == 0 else "%d mm" % mm
    return {1: "NO_RISE", 2: "STUCK_HIGH", 3: "NO_RETURN"}[status]


BYPASS_HEADER = "#  t_ms servo reading    state  phase steer"
DETOUR_HEADER = "#  t_ms servo reading    step"


def format_detour(b):
    return "# %6d %+5d %-10s %s" % (
        b["time_10ms"] * 10, b["servo_deg"], reading_text(b["reading"]),
        DETOUR_STEPS[b["state"] << 2 | b["phase"]])


def format_bypass(b):
    phase = BYPASS_PHASES[b["phase"]] if b["state"] == 1 else "-"
    return "# %6d %+5d %-10s %-6s %-5s %+5d" % (
        b["time_10ms"] * 10, b["servo_deg"], reading_text(b["reading"]),
        BYPASS_STATES[b["state"]], phase, b["steer_deg"])


TABLE_HEADER = "  t_ms mode  pos  dutyL dutyR  rawL rawC rawR  side src    hint  cntL cntR"


def format_entry(index, e, modes=MODES):
    info = e["info"]
    return ("%6d %s %+4d  %3d %3d  %4d %4d %4d  %s %-6s %s  %4d %4d" % (
        index * LOG_PERIOD_MS, modes.get(info & 3, "?"), e["position"],
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
        if ptype == TYPE_HELLO:
            return  # our own HELLO, looped back
        boot = int.from_bytes(data[1:3], "little")
        seq = int.from_bytes(data[3:5], "little")
        self.track_seq(boot, seq)
        body = data[HEADER_BYTES:]
        if ptype == TYPE_TEXT:
            self.handle_text(boot, body)
        elif ptype == TYPE_LOG:
            self.handle_log(boot, body)
        elif ptype == TYPE_BYPASS:
            self.handle_bypass(boot, body)

    def run_state(self, boot, run):
        return self.runs.setdefault((boot, run), {
            "entries": {}, "live": set(), "replay_at": None, "modes": MODES, "bypass": {},
            "detour": False})

    def handle_bypass(self, boot, body):
        if len(body) < BYPASS_HEADER_BYTES:
            return
        run = body[0]
        first = int.from_bytes(body[1:3], "little")
        count = body[3]
        if count > MAX_BYPASS or len(body) != BYPASS_HEADER_BYTES + count * BYPASS_BYTES:
            print("[recv] ignored BYPASS packet with a bad size")
            return
        state = self.run_state(boot, run)
        for i in range(count):
            off = BYPASS_HEADER_BYTES + i * BYPASS_BYTES
            state["bypass"][first + i] = decode_bypass(body[off:off + BYPASS_BYTES])

    def track_seq(self, boot, seq):
        if boot not in self.next_seq:
            # The robot counts from 0 at each start: earlier packets were lost
            # (or sent before this receiver started).
            print("[recv] robot boot %04x (first packet %d)" % (boot, seq))
            self.lost[boot] = seq
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
        state = self.run_state(boot, run)
        if flags & FLAG_BYPASS:
            state["modes"] = BYPASS_MODES
        if flags & FLAG_DETOUR:
            state["detour"] = True
        replay = bool(flags & FLAG_REPLAY)
        if replay:
            state["replay_at"] = time.monotonic()
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
                    print("[log] " + format_entry(index, e, state["modes"]))
        if flags & FLAG_REPLAY_END:
            self.save_run(boot, run, state)

    def tick(self, now):
        """Save runs whose last replay packet was lost."""
        for (boot, run), state in self.runs.items():
            if state["replay_at"] is not None and now - state["replay_at"] >= REPLAY_QUIET_S:
                print("[recv] last replay packet of run %d not received" % run)
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
            name = ("detour_test" if state["detour"]
                    else "bypass_test" if state["modes"] is BYPASS_MODES else "steer_test")
            f.write("# %s run %d, boot %04x, saved %s\n" % (name, run, boot, stamp))
            f.write("# entries %d of %d; received live %d; packets lost %d\n" % (
                len(indices), expected, len(state["live"]), self.lost.get(boot, 0)))
            f.write(TABLE_HEADER + "\n")
            for index in indices:
                f.write(format_entry(index, entries[index], state["modes"]) + "\n")
            bypass = state["bypass"]
            if bypass:
                last = max(bypass)
                f.write("# bypass readings %d of %d\n" % (len(bypass), last + 1))
                f.write((DETOUR_HEADER if state["detour"] else BYPASS_HEADER) + "\n")
                fmt = format_detour if state["detour"] else format_bypass
                for i in sorted(bypass):
                    f.write(fmt(bypass[i]) + "\n")
        print("[recv] run %d saved to %s (%d of %d entries; %d live; %d packets lost)" % (
            run, path, len(indices), expected, len(state["live"]), self.lost.get(boot, 0)))
        state["live"] = set()
        state["replay_at"] = None


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
        # Last replay packet lost: saved after the quiet time.
        r.handle(header2[:3] + bytes([3, 0]) + bytes([4, FLAG_REPLAY, 0, 0, 1]) + VECTOR)
        r.tick(time.monotonic())
        if list(pathlib.Path(d).glob("run_*_bootbeef_run4.txt")):
            print("FAIL saved before the quiet time")
            failures += 1
        r.tick(time.monotonic() + REPLAY_QUIET_S)
        if len(list(pathlib.Path(d).glob("run_*_bootbeef_run4.txt"))) != 1:
            print("FAIL not saved after the quiet time")
            failures += 1
        # bypass_test run: BYPS mode name and the bypass table.
        got_b = decode_bypass(BYPASS_VECTOR)
        want_b = dict(time_10ms=1234, servo_deg=-25, reading=0x600, state=1, steer_deg=17,
                      phase=0)
        if decode_bypass(bytes([0, 0, 0, 0, 0x80]))["phase"] != 2:
            print("FAIL bypass phase bits 38-39")
            failures += 1
        if got_b != want_b:
            print("FAIL bypass vector: %r" % got_b)
            failures += 1
        for reading, want in ((0, "TOO_NEAR"), (250, "250 mm"), (0x200, "NO_RISE"),
                              (0x400, "STUCK_HIGH"), (0x600, "NO_RETURN")):
            if reading_text(reading) != want:
                print("FAIL reading %03x: %r" % (reading, reading_text(reading)))
                failures += 1
        hb = bytes([TYPE_BYPASS | (VERSION << 4), 0xEF, 0xBE, 5, 0])
        r.handle(hb + bytes([5, 0, 0, 1]) + BYPASS_VECTOR)
        hl = bytes([TYPE_LOG | (VERSION << 4), 0xEF, 0xBE, 6, 0])
        r.handle(hl + bytes([5, FLAG_REPLAY | FLAG_REPLAY_END | FLAG_BYPASS,
                                                 0, 0, 1]) + VECTOR)
        files = list(pathlib.Path(d).glob("run_*_bootbeef_run5.txt"))
        text = files[0].read_text() if len(files) == 1 else ""
        if " BYPS " not in format_entry(0, VECTOR_FIELDS | {"info": 0xA7}, BYPASS_MODES):
            print("FAIL mode 3 is not BYPS in a bypass_test run")
            failures += 1
        for want in ("# bypass_test run 5", "     0 SRCH", "# bypass readings 1 of 1",
                     "#  12340   -25 NO_RETURN  AROUND DRIVE   +17"):
            if want not in text:
                print("FAIL bypass run file, missing %r:\n%s" % (want, text))
                failures += 1
        # detour_test run: step names from state << 2 | phase.
        r.handle(bytes([TYPE_BYPASS | (VERSION << 4), 0xEF, 0xBE, 7, 0]) +
                 bytes([6, 0, 0, 1]) + BYPASS_VECTOR)  # state 1, phase 0: PASS
        r.handle(bytes([TYPE_LOG | (VERSION << 4), 0xEF, 0xBE, 8, 0]) +
                 bytes([6, FLAG_REPLAY | FLAG_REPLAY_END | FLAG_BYPASS | FLAG_DETOUR,
                        0, 0, 1]) + VECTOR)
        files = list(pathlib.Path(d).glob("run_*_bootbeef_run6.txt"))
        text = files[0].read_text() if len(files) == 1 else ""
        for want in ("# detour_test run 6", DETOUR_HEADER, "#  12340   -25 NO_RETURN  PASS"):
            if want not in text:
                print("FAIL detour run file, missing %r:\n%s" % (want, text))
                failures += 1
        if not format_detour(dict(want_b, state=2, phase=2)).endswith(" FAILED"):
            print("FAIL detour step 10 is not FAILED")
            failures += 1
        # Bad sizes are ignored.
        r.handle(header + bytes([3, 0, 0, 0, 5]) + VECTOR)
        r.handle(b"\x21")
        r.handle(hb + bytes([5, 0, 0, 2]) + BYPASS_VECTOR)
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
    hello = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hello.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    hello_packet = bytes([TYPE_HELLO | (VERSION << 4), 0, 0, 0, 0])
    print("[recv] listening on UDP port %d; saving to %s" % (args.port, args.out))
    receiver = Receiver(args.out, live=not args.quiet)
    next_hello = 0.0
    try:
        while True:
            now = time.monotonic()
            if now >= next_hello:
                next_hello = now + HELLO_PERIOD_S
                try:
                    hello.sendto(hello_packet, ("255.255.255.255", args.port))
                except OSError as error:
                    print("[recv] HELLO not sent: %s" % error)
            receiver.tick(now)
            ready, _, _ = select.select([sock], [], [], 0.2)
            if ready:
                data, _ = sock.recvfrom(2048)
                receiver.handle(data)
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[recv] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
