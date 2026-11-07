#!/usr/bin/env python3
"""Check Python decoding against independently compiled C packer output."""
import contextlib
import csv
import importlib.util
import io
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import robot_wire as wire  # noqa: E402
spec = importlib.util.spec_from_file_location("mqtt_robot", REPO / "scripts/mqtt_robot.py")
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class WireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = {}
        cls.batches = {}
        for line in subprocess.check_output([sys.argv[1]], text=True).splitlines():
            topic, encoded = line.split()
            if topic.startswith("batch/"):
                cls.batches[topic.removeprefix("batch/")] = bytes.fromhex(encoded)
            else:
                cls.samples[topic] = bytes.fromhex(encoded)

    def test_c_samples(self):
        state = wire.decode("robot/state", self.samples["robot/state"])
        self.assertEqual((state["state"], state["module"], state["motion_faults"]), (3, 5, 129))
        self.assertEqual((state["uptime_ms"], state["sequence"]), (0x12345678, 0xFEDCBA98))
        self.assertEqual((state["line_age_ms"], state["motion_age_ms"], state["vehicle_age_ms"]), (10, 20, 65535))
        self.assertEqual((state["raw_bc"], state["norm_l"], state["norm_c"]), (0xABC, 1023, 0x155))
        self.assertEqual((state["count_l"], state["count_r"]), (-2147483648, -1234567))
        self.assertTrue(all(state[name] == 1 for name in wire.STATE_FLAGS))
        self.assertEqual(state["kr1_hex"], "01020304" + "00" * 12)
        sonar = wire.decode("robot/sonar", self.samples["robot/sonar"])
        self.assertEqual(sonar, dict(version=1, status=1, valid=1, pending=1, angle=-30, ping_angle=30,
                                    sequence=0xFFFFFFFF, generation=0x12345678, request_id=33,
                                    trigger_ms=1000, timestamp_ms=1010, echo=0x4567, ping_sequence=123,
                                    echo_status=2, echo_mm=""))
        event = wire.decode("robot/event", self.samples["robot/event"])
        self.assertEqual(event, dict(version=1, kind=6, sequence=0x12345678, uptime_ms=1234,
                                    word=0xABCD, detail=4, run_relative=1, **{"from": 2, "to": 3}))
        heart = wire.decode("robot/heartbeat", self.samples["robot/heartbeat"])
        names = ("uptime_ms", "connect_attempts", "telemetry_lost", "event_lost", "command_lost",
                 "sonar_history_missed", "vehicle_overruns", "motion_late", "motion_missed",
                 "mqtt_hook_max_us", "publish_ok", "publish_failed", "line_busy", "sonar_busy",
                 "motion_busy", "vehicle_busy", "health_busy", "latest_sonar_history_sequence",
                 "wifi_poll_gap_max_us", "wifi_poll_gap_samples")
        self.assertEqual([heart[name] for name in names], list(range(1, 21)))
        self.assertEqual((heart["wifi_link_status"], heart["wifi_start_status"]), (-3, -50))
        self.assertEqual((heart["stack_min_free"], heart["health_age_ms"], heart["reset_reason"]), (0x1234, 65535, 2))
        self.assertEqual((heart["wifi_requested_priority"], heart["wifi_observed_priority"], heart["mqtt_state"]), (20, 20, 3))
        self.assertTrue(all(heart[name] == 1 for name in wire.HEARTBEAT_FLAGS))
        log = wire.decode("robot/log", self.samples["robot/log"])
        self.assertEqual((log["kind"], log["dump_id"], log["generation"], log["index"]), (2, 111, 222, 333))
        self.assertEqual(log["event_word"], 0xABCD)
        self.assertEqual(log["event_run_relative"], 1)
        self.assertEqual(wire.pack_command(4, 0x12345678, bytes(range(16))), self.samples["robot/cmd"])

    def test_reject_malformed(self):
        for topic, data in self.samples.items():
            if topic.endswith("/cmd"):
                continue
            for bad in (b"", data[:-1], data + b"\0", b"\2" + data[1:]):
                with self.subTest(topic=topic, size=len(bad)):
                    with self.assertRaises(ValueError):
                        wire.decode(topic, bad)
        for topic, offset, mask in (("state", 19, 0x80), ("state", 47, 0x80),
                                    ("sonar", 1, 0x80), ("sonar", 26, 1),
                                    ("event", 15, 2), ("heartbeat", 1, 0x80),
                                    ("heartbeat", 3, 1), ("heartbeat", 42, 1),
                                    ("heartbeat", 98, 1), ("log", 2, 1)):
            bad = bytearray(self.samples["robot/" + topic])
            bad[offset] |= mask
            with self.assertRaises(ValueError):
                wire.decode("robot/" + topic, bad)
        for topic in ("state", "robot/cmd", "bad/state"):
            with self.assertRaises(ValueError):
                wire.decode(topic, self.samples["robot/state"])

    def test_log_bodies_and_csv(self):
        rows = []
        for kind in (0, 1, 3, 4):
            data = bytearray(self.samples["robot/log"])
            data[1] = kind
            data[16:32] = bytes(16)
            rows.append(wire.decode("robot/log", data))
        self.assertEqual(rows[0]["rows_total"], 0)
        self.assertEqual(rows[1]["kr1_hex"], "00" * 16)
        self.assertEqual(rows[2]["rows_sent"], 0)
        self.assertEqual(rows[3]["abort_reason"], 0)
        for topic, data in self.samples.items():
            if topic != "robot/cmd":
                rows.append(wire.decode(topic, data))
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=wire.csv_fields())
        writer.writeheader()
        writer.writerows(rows)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(stream.getvalue())))), len(rows))

    def test_token_file(self):
        with tempfile.TemporaryDirectory() as folder:
            token = Path(folder) / "token"
            token.write_text(bytes(range(16)).hex() + "\n")
            token.chmod(0o600)
            self.assertEqual(wire.read_token(token), bytes(range(16)))
            token.chmod(0o644)
            with self.assertRaises(ValueError):
                wire.read_token(token)
            token.chmod(0o600)
            for text in ("00", "z" * 32, "0" * 32 + "\nEXTRA", " " + "0" * 32):
                token.write_text(text)
                with self.assertRaises(ValueError):
                    wire.read_token(token)
            link = Path(folder) / "link"
            link.symlink_to(token)
            with self.assertRaises(OSError):
                wire.read_token(link)

    def test_cli_callback_api_and_publication(self):
        calls = []
        version2 = object()
        sample = self.samples["robot/state"]

        class Client:
            def __init__(self, api):
                calls.append(("api", api))

            def connect(self, host, port, keepalive):
                calls.append(("connect", host, port, keepalive))
                self.on_connect(self, None, None, types.SimpleNamespace(is_failure=False), None)

            def subscribe(self, topics):
                calls.append(("subscribe", topics))

            def loop_start(self):
                pass

            def loop_forever(self):
                self.on_message(self, None, types.SimpleNamespace(topic="robot/state", payload=sample))

            def publish(self, topic, payload, qos, retain):
                calls.append(("publish", topic, payload, qos, retain))
                return types.SimpleNamespace(wait_for_publish=lambda timeout: None, is_published=lambda: True)

            def disconnect(self):
                pass

            def loop_stop(self):
                pass

        modules = {name: types.ModuleType(name) for name in ("paho", "paho.mqtt", "paho.mqtt.client")}
        modules["paho"].mqtt = modules["paho.mqtt"]
        modules["paho.mqtt"].client = modules["paho.mqtt.client"]
        modules["paho.mqtt.client"].Client = Client
        modules["paho.mqtt.client"].CallbackAPIVersion = types.SimpleNamespace(VERSION2=version2)
        with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(cli.main(["--host", "test.invalid"]), 0)
        self.assertEqual(calls[0], ("api", version2))
        self.assertEqual(len(list(csv.DictReader(io.StringIO(output.getvalue())))), 1)
        self.assertEqual(calls[2][1], [("robot/" + name, 0) for name in wire.SIZES])
        with tempfile.TemporaryDirectory() as folder:
            token = Path(folder) / "private-token"
            token.write_text(bytes(range(16)).hex())
            token.chmod(0o600)
            calls.clear()
            with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()) as error:
                self.assertEqual(cli.main(["--host", "test.invalid", "--command", "start",
                                           "--token-file", str(token), "--request-id", "0x12345678"]), 0)
            self.assertEqual(calls[-1], ("publish", "robot/cmd", self.samples["robot/cmd"], 1, False))
            self.assertNotIn(bytes(range(16)).hex(), error.getvalue())
            self.assertIn("not robot acceptance", error.getvalue())
            for optional_file in ([], ["--token-file", str(token)]):
                calls.clear()
                with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(cli.main(["--host", "test.invalid", "--command", "estop",
                                               "--request-id", "123", *optional_file]), 0)
                expected_token = bytes(range(16)) if optional_file else bytes(16)
                self.assertEqual(calls[-1], ("publish", "robot/cmd", wire.pack_command(1, 123, expected_token), 1, False))

    def test_non_estop_requires_token_file(self):
        for command in ("start", "calibrate", "dump"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                cli.main(["--host", "test.invalid", "--command", command])
            self.assertEqual(error.exception.code, 2)

    def test_command_input(self):
        for opcode, request, token in ((0, 1, bytes(16)), (5, 1, bytes(16)),
                                       (1, -1, bytes(16)), (1, 1 << 32, bytes(16)), (1, 1, bytes(15))):
            with self.assertRaises(ValueError):
                wire.pack_command(opcode, request, token)


def build_batch(kind=5, count=1, flags=0, stream=0, generation=1, first=0, publish=1000,
                newest=990, rows=None, skipped=0, records=None, extra=b""):
    """Independent Python frame builder; C-packed frames are compared against it."""
    if records is None:
        records = [bytes([(first + i + j) & 255 for j in range(16)]) for i in range(count)]
        if kind == 6:
            records = [bytes([1, 1 + i % 6, 0, 1]) + bytes(12) for i in range(count)]
    if rows is None:
        rows = first + count
    header = struct.pack("<BBBBIIIIIII", 1, kind, count, flags, stream, generation, first,
                         publish, newest, rows, skipped)
    return header + b"".join(records) + extra


def control(kind, dump_id, generation, body):
    return struct.pack("<BBHIII", 1, kind, 0, dump_id, generation, 0) + body


def begin(dump_id, generation, rows, events):
    return control(0, dump_id, generation, struct.pack("<IIII", rows, events, 0, 0))


def end(dump_id, generation, rows, events):
    return control(3, dump_id, generation, struct.pack("<IIII", rows, events, 0, 0))


def abort(dump_id, generation, reason=1):
    return control(4, dump_id, generation, struct.pack("<IIII", reason, 0, 0, 0))


class LogBatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.batches = {}
        for line in subprocess.check_output([sys.argv[1]], text=True).splitlines():
            topic, encoded = line.split()
            if topic.startswith("batch/"):
                cls.batches[topic.removeprefix("batch/")] = bytes.fromhex(encoded)

    def setUp(self):
        self.reports = []
        self.log = wire.LogStream(report=self.reports.append)

    def test_c_frames_match_python_layout(self):
        live = self.batches["live"]
        records = [bytes([(i * 7 + 3) & 255 for i in range(16 * k, 16 * k + 16)]) for k in range(10)]
        self.assertEqual(live, build_batch(count=10, generation=7, first=100, publish=5000,
                                           newest=4950, rows=125, records=records))
        header, rows = wire.parse_log_batch(live)
        self.assertEqual((header["kind"], header["count"], header["flags"], header["stream_id"]),
                         (5, 10, 0, 0))
        self.assertEqual((header["generation"], header["first_index"], header["log_rows"]),
                         (7, 100, 125))
        self.assertEqual([row["index"] for row in rows], list(range(100, 110)))
        self.assertEqual(rows[0]["kr1_hex"], records[0].hex())
        self.assertEqual(wire.decode("robot/log", live)["first_index"], 100)
        jump = wire.parse_log_batch(self.batches["jump"])[0]
        self.assertEqual((jump["jumped"], jump["skipped_before"], jump["new_generation"]), (1, 115, 0))
        newgen = wire.parse_log_batch(self.batches["newgen"])[0]
        self.assertEqual((newgen["new_generation"], newgen["skipped_before"], newgen["generation"]),
                         (1, 33, 8))
        refill = wire.parse_log_batch(self.batches["refill_rows"])
        self.assertEqual((refill[0]["refill"], refill[0]["stream_id"], len(refill[1]), refill[1][11]["index"]),
                         (1, 111, 12, 35))
        self.assertEqual(len(self.batches["refill_rows"]), 224)
        events = wire.parse_log_batch(self.batches["refill_events"])[1]
        self.assertEqual([(e["index"], e["event_kind"], e["event_word"], e["event_detail"],
                           e["event_run_relative"]) for e in events],
                         [(4, 1, 0x1200, 7, 0), (5, 2, 0x1201, 8, 1), (6, 3, 0x1202, 9, 0)])

    def test_malformed_batches_rejected(self):
        good = build_batch(count=2)
        for bad in (b"", good[:31], good[:-1], good + b"\0", build_batch(count=0, records=[]),
                    build_batch(count=13, records=[bytes(16)] * 13),
                    build_batch(count=3, records=[bytes(16)] * 2), b"\2" + good[1:],
                    build_batch(kind=7), build_batch(kind=5)[:32]):
            with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                wire.parse_log_batch(bad)
        for flag in (8, 16, 32, 64, 128):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                wire.parse_log_batch(build_batch(flags=flag))
        for kwargs in (dict(flags=1, newest=5), dict(flags=1, skipped=1, newest=0),
                       dict(flags=3, newest=0), dict(flags=5, newest=0), dict(stream=9)):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                wire.parse_log_batch(build_batch(**kwargs))
        with self.assertRaises(ValueError):  # a bad event record rejects the whole frame
            wire.parse_log_batch(build_batch(kind=6, records=[b"\2" + bytes(15)]))
        with self.assertRaises(ValueError):
            wire.parse_log_batch(build_batch(kind=6, records=[bytes([1, 9]) + bytes(14)]))
        with self.assertRaises(ValueError):
            wire.decode("robot/log", good[:-1])
        self.assertEqual(self.log.handle(build_batch(count=1))[0]["index"], 0)
        with self.assertRaises(ValueError):
            self.log.handle(build_batch(flags=8))
        self.assertEqual(len(self.log.seen), 1)  # rejected frames change no state

    def test_retired_kinds_still_accepted(self):
        for kind in (1, 2):
            body = bytes([1, 1]) + bytes(14) if kind == 2 else bytes(16)
            rows = self.log.handle(control(kind, 5, 3, body)[:12] + struct.pack("<I", 9) + body)
            self.assertEqual((rows[0]["kind"], rows[0]["index"], rows[0]["source"]), (kind, 9, "refill"))
        self.assertEqual(self.log.handle(control(1, 5, 3, bytes(16))[:12] + struct.pack("<I", 9)
                                         + bytes(16)), [])  # duplicate row

    def test_live_columns(self):
        rows = self.log.handle(build_batch(count=3, generation=4, first=10, publish=5,
                                           newest=0xFFFFFFF0, rows=20, skipped=0))
        self.assertEqual([r["index"] for r in rows], [10, 11, 12])
        self.assertEqual({r["data_age_ms"] for r in rows}, {21})  # wrap-safe
        self.assertEqual({r["rows_behind"] for r in rows}, {7})
        far = self.log.handle(build_batch(count=1, generation=4, first=30, publish=5,
                                          newest=0xF0000000, rows=31))
        self.assertEqual(far[0]["data_age_ms"], 0x10000005)
        self.assertEqual({(r["source"], r["generation"], r["flags"], r["kind"]) for r in rows},
                         {("live", 4, 0, 5)})
        rows = self.log.handle(build_batch(count=2, generation=4, first=20, flags=4, skipped=7,
                                           rows=22))
        self.assertEqual([(r["skipped_before"], r["jumped"], r["rows_behind"]) for r in rows],
                         [(7, 1, 0), (0, 1, 0)])
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=wire.csv_fields())  # no unknown key raises
        for row in rows:
            writer.writerow({"topic": "robot/log", **row})

    def test_csv_fields_cover_every_record_kind(self):
        events = self.log.handle(build_batch(kind=6, count=2, flags=1, stream=5, newest=0))
        self.assertEqual((events[0]["source"], events[0]["data_age_ms"], events[0]["rows_behind"]),
                         ("refill", "", ""))
        names = set(wire.csv_fields())
        for row in events + self.log.handle(self.batches["live"]):
            self.assertLessEqual(set(row) - {"received_unix_s"}, names)

    def test_dedupe_and_merge(self):
        self.assertEqual(len(self.log.handle(build_batch(count=10, first=0, generation=2))), 10)
        refill = build_batch(count=12, first=4, flags=1, stream=77, newest=0, generation=2)
        rows = self.log.handle(refill)
        self.assertEqual([r["index"] for r in rows], [10, 11, 12, 13, 14, 15])
        self.assertEqual(self.log.handle(refill), [])  # QoS 1 retry
        self.assertEqual(self.log.duplicates, 6 + 12)
        # Same index in another kind or generation is not a duplicate.
        self.assertEqual(len(self.log.handle(build_batch(kind=6, count=2, first=0, flags=1,
                                                         stream=77, newest=0, generation=2))), 2)
        self.assertEqual(len(self.log.handle(build_batch(count=2, first=0, generation=3))), 2)
        self.assertEqual(self.log.handle(build_batch(kind=6, count=2, first=0, flags=1,
                                                     stream=78, newest=0, generation=2)), [])

    def gaps(self):
        return [(g["type"], g["generation"], g["start"], g["end"]) for g in self.log.gaps]

    def test_live_gaps(self):
        self.log.handle(build_batch(count=10, first=0))
        self.log.handle(build_batch(count=10, first=10))
        self.assertEqual(self.gaps(), [])
        self.log.handle(build_batch(count=10, first=40))  # frames 20-39 lost on QoS 0
        self.assertEqual(self.gaps(), [("lost", 1, 20, 40)])
        self.log.handle(build_batch(count=10, first=30))  # late/reordered frame: no new gap
        self.log.handle(build_batch(count=10, first=50))
        self.assertEqual(len(self.gaps()), 1)
        self.assertEqual(self.reports[0], "live gap (lost): generation 1 rows 20-39 (20 rows)")
        self.log.handle(build_batch(count=1, first=61))  # a one-row hole is a gap too
        self.assertEqual(self.gaps()[-1], ("lost", 1, 60, 61))

    def test_jump_and_reconnect(self):
        self.log.handle(build_batch(count=10, first=0))
        # Reconnect jump: the robot reports exactly the rows it skipped.
        self.log.handle(build_batch(count=10, first=300, flags=4, skipped=290))
        self.assertEqual(self.gaps(), [("jumped", 1, 10, 300)])
        self.log.handle(build_batch(count=10, first=310))
        self.assertEqual(len(self.gaps()), 1)
        # Skipped count larger than the observed hole: the whole hole is a jump.
        self.log.handle(build_batch(count=10, first=345, flags=4, skipped=30))
        self.assertEqual(self.gaps()[1], ("jumped", 1, 320, 345))

    def test_jump_with_lost_frames_before(self):
        self.log.handle(build_batch(count=10, first=0))
        self.log.handle(build_batch(count=10, first=200, flags=4, skipped=50))
        self.assertEqual(self.gaps(), [("lost", 1, 10, 150), ("jumped", 1, 150, 200)])

    def test_first_frame_is_a_late_join_not_a_gap(self):
        self.log.handle(build_batch(count=10, first=500))
        self.assertEqual(self.gaps(), [])
        late_jump = wire.LogStream(report=lambda message: None)
        late_jump.handle(build_batch(count=10, first=500, flags=4, skipped=480))
        self.assertEqual([(g["type"], g["start"], g["end"]) for g in late_jump.gaps],
                         [("jumped", 20, 500)])

    def test_generation_change(self):
        self.log.handle(build_batch(count=10, first=0, generation=1))
        self.log.handle(build_batch(count=2, first=0, generation=2, flags=2, skipped=33,
                                    publish=2000, newest=1990))
        self.assertEqual((self.gaps(), self.log.old_generation_skipped), ([], 33))
        self.assertEqual(self.log.live_next, {1: 10, 2: 2})
        # NEW_GENERATION frame lost: first frame seen for generation 3 starts at 40.
        self.log.handle(build_batch(count=2, first=40, generation=3))
        self.assertEqual(self.gaps(), [("lost", 3, 0, 40)])
        # NEW_GENERATION that starts after a jump: rows before it were skipped, not lost.
        self.log.handle(build_batch(count=2, first=60, generation=4, flags=6, skipped=5))
        self.assertEqual(self.gaps()[-1], ("jumped", 4, 0, 60))
        self.assertEqual(self.log.old_generation_skipped, 38)
        self.log.handle(build_batch(count=2, first=5, generation=5, flags=2))  # new gen not at zero
        self.assertEqual(self.gaps()[-1], ("lost", 5, 0, 5))

    def test_dump_complete_and_missing(self):
        self.log.handle(begin(9, 2, 30, 4))
        self.log.handle(build_batch(count=12, first=0, flags=1, stream=9, newest=0, generation=2))
        self.log.handle(build_batch(count=12, first=12, flags=1, stream=9, newest=0, generation=2))
        self.log.handle(build_batch(kind=6, count=4, first=0, flags=1, stream=9, newest=0,
                                    generation=2))
        self.log.handle(end(9, 2, 24, 4))
        self.assertEqual(self.reports[-1], "dump 9: missing rows [24-29] events []")
        # Live rows fill part of the hole; the final summary shows what is still absent.
        self.log.handle(build_batch(count=3, first=24, generation=2, rows=40))
        self.assertIn("dump 9: incomplete, missing rows [27-29] events []", self.log.summary())
        # A repeated END (QoS 1 retry) does not report again.
        count = len(self.reports)
        self.log.handle(end(9, 2, 24, 4))
        self.assertEqual(len(self.reports), count)
        self.log.handle(begin(10, 2, 24, 4))
        self.log.handle(end(10, 2, 24, 4))
        self.assertEqual(self.reports[-1], "dump 10: complete, 24 rows, 4 events")

    def test_dump_missing_event_and_abort(self):
        self.log.handle(begin(3, 1, 0, 6))
        self.log.handle(build_batch(kind=6, count=2, first=0, flags=1, stream=3, newest=0))
        self.log.handle(build_batch(kind=6, count=1, first=5, flags=1, stream=3, newest=0))
        self.log.handle(end(3, 1, 0, 3))
        self.assertEqual(self.reports[-1], "dump 3: missing rows [] events [2-4]")
        self.log.handle(begin(4, 1, 99, 0))
        self.log.handle(abort(4, 1))
        self.assertEqual(self.reports[-1], "dump 4: aborted, reason 1")
        self.log.handle(end(4, 1, 0, 0))  # END after ABORT changes nothing
        self.assertIn("dump 4: aborted", self.log.summary())
        self.log.handle(end(55, 1, 0, 0))
        self.assertIn("dump 55: no BEGIN seen before kind 3", self.reports[-1])

    def test_summary(self):
        self.assertEqual(self.log.summary(), [])
        self.log.handle(begin(1, 1, 10, 0))
        self.log.handle(build_batch(count=10, first=0))
        self.log.handle(build_batch(count=10, first=30))
        lines = self.log.summary()
        self.assertEqual(lines[0], "log summary: 20 rows, 0 events, 0 duplicates dropped")
        self.assertEqual(lines[1], "live gap (lost): generation 1 rows 10-29, still missing [10-29]")
        self.assertEqual(lines[2], "dump 1: not finished, missing rows [] events []")

    def test_csv_sink_flushes_by_interval(self):
        class Stream(io.StringIO):
            flushes = 0

            def flush(self):
                self.flushes += 1

        now = [0.0]
        stream = Stream()
        sink = cli.CsvSink(stream, ["a"], interval=1.0, clock=lambda: now[0])
        for _ in range(100):  # many rows inside one interval: no per-row flush
            sink.write({"a": 1})
        self.assertEqual(stream.flushes, 0)
        now[0] = 1.5
        sink.write({"a": 2})
        self.assertEqual(stream.flushes, 1)
        sink.write({"a": 3})
        self.assertEqual(stream.flushes, 1)
        sink.flush()
        self.assertEqual(stream.flushes, 2)
        self.assertEqual(len(stream.getvalue().splitlines()), 1 + 102)

    def test_cli_routes_log_batches_and_summarises(self):
        calls = []
        frames = [self.batches["live"], build_batch(count=1, generation=7, first=300),
                  b"bad", build_batch(count=2, flags=8)]

        class Client:
            def __init__(self, api):
                del api

            def connect(self, host, port, keepalive):
                del host, port, keepalive
                self.on_connect(self, None, None, types.SimpleNamespace(is_failure=False), None)

            def subscribe(self, topics):
                calls.append(topics)

            def loop_forever(self):
                for frame in frames:
                    self.on_message(self, None, types.SimpleNamespace(topic="robot/log",
                                                                       payload=frame))

            def disconnect(self):
                pass

            def loop_stop(self):
                pass

        modules = {name: types.ModuleType(name) for name in ("paho", "paho.mqtt", "paho.mqtt.client")}
        modules["paho"].mqtt = modules["paho.mqtt"]
        modules["paho.mqtt"].client = modules["paho.mqtt.client"]
        modules["paho.mqtt.client"].Client = Client
        modules["paho.mqtt.client"].CallbackAPIVersion = types.SimpleNamespace(VERSION2=None)
        with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()) as out, \
                contextlib.redirect_stderr(io.StringIO()) as error:
            self.assertEqual(cli.main(["--host", "test.invalid"]), 0)
        table = list(csv.DictReader(io.StringIO(out.getvalue())))
        self.assertEqual([row["index"] for row in table], [str(i) for i in range(100, 110)] + ["300"])
        self.assertEqual(table[0]["data_age_ms"], "50")
        self.assertEqual(table[0]["rows_behind"], "15")
        self.assertEqual(table[0]["topic"], "robot/log")
        text = error.getvalue()
        self.assertEqual(text.count("discarded robot/log"), 2)
        self.assertIn("live gap (lost): generation 7 rows 110-299 (190 rows)", text)
        self.assertIn("log summary: 11 rows, 0 events, 0 duplicates dropped", text)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
