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
            elif topic.startswith("robot/"):
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
            for bad in (b"", data[:-1], data + b"\0", b"\x09" + data[1:]):
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
            if topic == "heartbeat" and offset == 98 and bad[0] == 2:
                struct.pack_into("<H", bad, 98, 1001)  # version 2: CPU load above 100.0 %
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
                frames = (("state", sample), ("barcode", barcode_frame()),
                          ("imu", imu_snapshot()), ("imu", imu_event()),
                          ("barcode", bytes([9, 1])), ("imu", bytes([1, 9])))
                for name, data in frames:
                    self.on_message(self, None, types.SimpleNamespace(
                        topic="robot/" + name, payload=data))

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
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual([row["topic"] for row in rows],
                         ["robot/state", "robot/barcode", "robot/imu", "robot/imu"])
        self.assertEqual((rows[1]["code"], rows[2]["pitch_raw"], rows[3]["event"]),
                         ("5", "3", "6"))
        qos_one = ("log", "status", "summary", "barcode")
        self.assertEqual(calls[2][1], [("robot/" + name, int(name in qos_one))
                                       for name in wire.SIZES])
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


def obstacle_frame(flags=0b0001, decision=1, angle=-15, scan=7, uptime=123456, closest=250,
                   word=0x1234, right_step=3, left_step=7, right_clear=15, left_clear=255):
    return struct.pack("<BBBbIIHHBBBB", 1, flags, decision, angle, scan, uptime, closest, word,
                       right_step, left_step, right_clear, left_clear)


class NewTopicTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v1_state = None
        cls.c_state = None
        cls.dash = {}
        for line in subprocess.check_output([sys.argv[1]], text=True).splitlines():
            topic, encoded = line.split()
            if topic == "robot/state":
                cls.c_state = bytes.fromhex(encoded)
            elif topic.startswith("dash/"):
                cls.dash[topic.removeprefix("dash/")] = bytes.fromhex(encoded)
        cls.v1_state = b"\x01" + cls.c_state[1:48]  # first 48 bytes are the same in v1 and v2

    def state_v2(self, left=-1500, right=2500, avg=500):
        return b"\x02" + self.v1_state[1:] + struct.pack("<iii", left, right, avg)

    def test_state_v2_adds_signed_distances_and_keeps_v1_fields(self):
        v1 = wire.decode("robot/state", self.v1_state)
        v2 = wire.decode("robot/state", self.state_v2())
        self.assertEqual((v2["dist_left_mm"], v2["dist_right_mm"], v2["dist_avg_mm"]),
                         (-1500, 2500, 500))
        self.assertEqual((v1["version"], v2["version"]), (1, 2))
        self.assertNotIn("dist_left_mm", v1)  # old captures still decode, no invented values
        self.assertEqual({k: v for k, v in v2.items() if not k.startswith("dist_") and k != "version"},
                         {k: v for k, v in v1.items() if k != "version"})
        extreme = wire.decode("robot/state", self.state_v2(-2 ** 31, 2 ** 31 - 1, 0))
        self.assertEqual((extreme["dist_left_mm"], extreme["dist_right_mm"]), (-2 ** 31, 2 ** 31 - 1))

    def test_c_packed_state_v2_and_old_version_bytes(self):
        self.assertIn((len(self.c_state), self.c_state[0]), ((60, 2), (72, 3)))
        result = wire.decode("robot/state", self.c_state)
        self.assertEqual((result["dist_left_mm"], result["dist_right_mm"], result["dist_avg_mm"]),
                         (-123456, 2 ** 31 - 1, -7))
        self.assertEqual((result["uptime_ms"], result["count_l"], result["raw_bc"]),
                         (0x12345678, -2 ** 31, 0xABC))
        for version in (0, 3, 9, 255):  # a version that is neither 1 nor 2 never decodes
            for size in (48, 60):
                with self.assertRaises(ValueError):
                    wire.decode("robot/state", bytes([version]) + self.c_state[1:size])

    def test_c_batch_two_frames_match_independent_builders(self):
        self.assertEqual(self.dash["state_v3"], state_v3_frame(
            self.c_state, duty_left=-800, duty_right=32767, heading=-1234567, rate=-4749,
            dist=(-123456, 2 ** 31 - 1, -7)))
        for suffix, outcome, payload, flags in (("done_line", 3, 0x567, 1),
                                                ("done_no_line", 4, 1, 1),
                                                ("failed", 7, 3, 0),
                                                ("aborted", 8, 0, 1)):
            frame = self.dash["detour_" + suffix]
            self.assertEqual(frame, detour_frame(
                outcome=outcome, side=2, flags=flags, sequence=0x01020304, start=0xFFFFFFF0,
                end=16, duration=32, distance=0xA1B2C3D4, lateral=0x1234, payload=payload,
                scan=0x0A0B0C0D))
            self.assertEqual(wire.decode("robot/detour", frame)["outcome"], outcome)
        for suffix, flags, source, reason, error in (("normal", 2, 1, 1, 41),
                                                    ("aborted_detour", 6, 2, 2, 41),
                                                    ("no_speed_error", 0, 4, 3, 65535)):
            frame = self.dash["summary_" + suffix]
            self.assertEqual(frame, summary_frame(
                flags=flags, source=source, reason=reason, run=0x01020304, duration=61234,
                distance=4321, speed_error=error, motion_faults=9, shell=4, shell_detail=258))
            self.assertEqual(wire.decode("robot/summary", frame)["stop_reason"], reason)
        for suffix, cpu in (("", 0x123), ("_unknown", 65535)):
            frame = self.dash["heartbeat_v2" + suffix]
            self.assertEqual(wire.decode("robot/heartbeat", frame)["cpu_load_permille"], cpu)
        junction = wire.decode("robot/event", self.dash["event_junction"])
        self.assertEqual((junction["kind"], junction["sequence"], junction["word"]), (7, 20, 2))

    def test_c_packed_obstacle_and_status(self):
        result = wire.decode("robot/obstacle", self.dash["obstacle"])
        self.assertEqual((result["closest_valid"], result["too_near"], result["left_blocked"],
                          result["right_blocked"]), (1, 1, 1, 1))
        self.assertEqual((result["decision"], result["closest_angle_deg"]), (2, -30))
        self.assertEqual((result["scan_id"], result["uptime_ms"]), (0x01020304, 0xFFFFFFF0))
        self.assertEqual((result["closest_mm"], result["profile_word"]), (510, 0x5A5A))
        self.assertEqual((result["right_free_step"], result["left_free_step"]), (7, 3))
        self.assertEqual((result["right_clear_deg"], result["left_clear_deg"]), (255, 15))
        self.assertEqual(self.dash["obstacle"], obstacle_frame(
            flags=0x0F, decision=2, angle=-30, scan=0x01020304, uptime=0xFFFFFFF0, closest=510,
            word=0x5A5A, right_step=7, left_step=3, right_clear=255, left_clear=15))
        self.assertEqual(wire.decode("robot/status", self.dash["status_online"])["robot_online"], 1)
        self.assertEqual(wire.decode("robot/status", self.dash["status_offline"])["robot_online"], 0)
        self.assertEqual((self.dash["status_online"], self.dash["status_offline"]),
                         (bytes([1, 1]), bytes([1, 2])))

    def test_state_version_and_length_must_agree(self):
        good = self.state_v2()
        bad_frames = (b"", good[:-1], good + b"\0", b"\x01" + good[1:], b"\x03" + good[1:],
                      b"\x02" + self.v1_state[1:], b"\x00" + good[1:], self.v1_state + b"\0",
                      self.v1_state[:-1])
        for bad in bad_frames:
            with self.subTest(size=len(bad), version=bad[:1]), self.assertRaises(ValueError):
                wire.decode("robot/state", bad)
        for offset, mask in ((19, 0x80), (47, 0x80)):  # v1 reserved bits stay checked in v2
            frame = bytearray(good)
            frame[offset] |= mask
            with self.assertRaises(ValueError):
                wire.decode("robot/state", frame)

    def test_obstacle_decode(self):
        result = wire.decode("robot/obstacle", obstacle_frame())
        self.assertEqual((result["closest_valid"], result["too_near"], result["left_blocked"],
                          result["right_blocked"]), (1, 0, 0, 0))
        self.assertEqual((result["decision"], result["closest_angle_deg"], result["scan_id"]),
                         (1, -15, 7))
        self.assertEqual((result["uptime_ms"], result["closest_mm"], result["profile_word"]),
                         (123456, 250, 0x1234))
        self.assertEqual((result["right_free_step"], result["left_free_step"]), (3, 7))
        self.assertEqual((result["right_clear_deg"], result["left_clear_deg"]), (15, 255))
        flags = wire.decode("robot/obstacle", obstacle_frame(flags=0b1110, decision=0, angle=0,
                                                             closest=0, scan=0xFFFFFFFF))
        self.assertEqual((flags["closest_valid"], flags["too_near"], flags["left_blocked"],
                          flags["right_blocked"], flags["scan_id"]), (0, 1, 1, 1, 0xFFFFFFFF))
        none = wire.decode("robot/obstacle", obstacle_frame(closest=511, angle=30))
        self.assertEqual(none["closest_mm"], 511)

    def test_obstacle_rejects_bad_frames(self):
        good = obstacle_frame()
        for bad in (b"", good[:-1], good + b"\0", b"\x02" + good[1:], b"\x00" + good[1:]):
            with self.assertRaises(ValueError):
                wire.decode("robot/obstacle", bad)
        for kwargs in (dict(flags=0x40), dict(flags=0x80), dict(flags=0x30), dict(decision=3), dict(closest=512),
                       dict(right_step=8), dict(left_step=8), dict(right_clear=36),
                       dict(left_clear=254)):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                wire.decode("robot/obstacle", obstacle_frame(**kwargs))

    def test_status_decode(self):
        online = wire.decode("robot/status", bytes([1, 1]))
        offline = wire.decode("robot/status", bytes([1, 2]))
        self.assertEqual((online["robot_online"], online["status_state"]), (1, 1))
        self.assertEqual((offline["robot_online"], offline["status_state"]), (0, 2))
        for bad in (b"", b"\x01", bytes([1, 0]), bytes([1, 3]), bytes([2, 1]), bytes([0, 1]),
                    bytes([1, 1, 0])):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                wire.decode("robot/status", bad)

    def test_reserved_topics_ignore_unknown_versions_and_kinds(self):
        for topic in ("robot/barcode", "robot/imu"):
            for frame in (bytes([2, 1, 5]), bytes([1, 9, 7, 7, 7]), bytes([0, 1])):
                result = wire.decode(topic, frame)
                self.assertEqual((result["known"], result["length"]), (0, len(frame)))
                self.assertEqual((result["version"], result["kind"]), (frame[0], frame[1]))
            for bad in (b"", b"\x01"):
                with self.assertRaises(ValueError):
                    wire.decode(topic, bad)
        with self.assertRaises(ValueError):
            wire.decode("robot/nothing", bytes([1, 1]))

    def test_csv_covers_new_fields(self):
        fields = set(wire.csv_fields())
        for name in ("dist_left_mm", "dist_right_mm", "dist_avg_mm", "closest_mm", "scan_id",
                     "decision", "left_clear_deg", "right_clear_deg", "robot_online",
                     "status_state", "profile_word", "closest_valid", "too_near"):
            self.assertIn(name, fields)
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=wire.csv_fields())
        for topic, data in (("robot/state", self.state_v2()), ("robot/state", self.v1_state),
                            ("robot/obstacle", obstacle_frame()),
                            ("robot/status", bytes([1, 1]))):
            writer.writerow(wire.decode(topic, data))


def state_v3_frame(base, duty_left=-250, duty_right=1000, heading=-4500, rate=-123,
                   dist=(10, 20, 15)):
    """Version 3 state: the first 48 bytes of the C sample, distances, duty and heading."""
    return (b"\x03" + base[1:48] + struct.pack("<iii", *dist) +
            struct.pack("<hhihH", duty_left, duty_right, heading, rate, 0))


def heartbeat_frame(base, version=2, cpu=0x123):
    frame = bytearray(base)
    frame[0] = version
    struct.pack_into("<H", frame, 98, cpu if version == 2 else 0)
    return bytes(frame)


def detour_frame(outcome=3, side=1, flags=1, sequence=4, start=1000, end=3500, duration=2500,
                 distance=1200, lateral=340, payload=0, scan=9):
    return struct.pack("<BBBBIIIIIHHI", 1, outcome, side, flags, sequence, start, end, duration,
                       distance, lateral, payload, scan)


def summary_frame(flags=0b010, source=1, reason=1, run=2, start=5000, duration=60000,
                  distance=8000, losses=3, obstacles=2, done=2, succeeded=1, detour_ms=7000,
                  started=3, speed_error=45, motion_faults=0x05, shell=2, shell_detail=6,
                  barcode_ok=0, barcode_failed=0, hump=0xFFFF):
    return struct.pack("<BBBBIIIIHHHHIHHBBHBBH", 1, flags, source, reason, run, start, duration,
                       distance, losses, obstacles, done, succeeded, detour_ms, started,
                       speed_error, motion_faults, shell, shell_detail, barcode_ok,
                       barcode_failed, hump)


def barcode_frame(flags=0b00111, code=5, votes=3, reads=4, direction=1, nav=9, result=-45,
                  sequence=12, uptime=777):
    return struct.pack("<BBBBBBBBhHII", 1, 1, flags, code, votes, reads, direction, nav, result,
                       0, sequence, uptime)


def imu_snapshot(pitch=3, heading=200, rate=250, hump=2, collision=1, valid=1, height=7,
                 peak=9, age=0xBEEF):
    word1 = pitch | heading << 8 | rate << 16 | hump << 24 | collision << 26 | valid << 27
    word2 = height | peak << 8 | age << 16
    return struct.pack("<BBHII", 1, 1, 0, word1, word2)


def imu_event(event=6, sequence=77, uptime=4242):
    return struct.pack("<BBBBII", 1, 2, event, 0, sequence, uptime)


class BatchTwoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = {}
        for line in subprocess.check_output([sys.argv[1]], text=True).splitlines():
            topic, encoded = line.split()
            if topic in ("robot/state", "robot/heartbeat", "robot/event"):
                cls.samples[topic] = bytes.fromhex(encoded)

    def test_state_v3_fields_and_signs(self):
        frame = state_v3_frame(self.samples["robot/state"])
        result = wire.decode("robot/state", frame)
        self.assertEqual(result["version"], 3)
        self.assertEqual((result["duty_left_permille"], result["duty_right_permille"]),
                         (-250, 1000))
        self.assertEqual((result["heading_cdeg"], result["turn_rate_ddps"]), (-4500, -123))
        self.assertEqual((result["dist_left_mm"], result["dist_right_mm"],
                          result["dist_avg_mm"]), (10, 20, 15))
        v2 = wire.decode("robot/state", b"\x02" + frame[1:60])
        self.assertEqual({k: v for k, v in result.items() if k in v2 and k != "version"},
                         {k: v for k, v in v2.items() if k != "version"})
        self.assertNotIn("heading_cdeg", v2)
        extreme = wire.decode("robot/state", state_v3_frame(
            self.samples["robot/state"], -32768, 32767, -2 ** 31, 32767))
        self.assertEqual((extreme["duty_left_permille"], extreme["duty_right_permille"],
                          extreme["heading_cdeg"], extreme["turn_rate_ddps"]),
                         (-32768, 32767, -2 ** 31, 32767))

    def test_state_v3_rejects_bad_frames(self):
        good = state_v3_frame(self.samples["robot/state"])
        for bad in (good[:-1], good + b"\0", b"\x03" + good[1:60], b"\x04" + good[1:],
                    b"\x02" + good[1:], good[:70] + b"\x01\x00", good[:71] + b"\x01"):
            with self.assertRaises(ValueError):
                wire.decode("robot/state", bad)

    def test_heartbeat_cpu_load(self):
        base = self.samples["robot/heartbeat"]
        for cpu in (0, 1, 0x123, 1000, 0xFFFF):
            result = wire.decode("robot/heartbeat", heartbeat_frame(base, 2, cpu))
            self.assertEqual((result["version"], result["cpu_load_permille"]), (2, cpu))
        for cpu in (1001, 0xFFFE):
            with self.assertRaises(ValueError):
                wire.decode("robot/heartbeat", heartbeat_frame(base, 2, cpu))
        old = wire.decode("robot/heartbeat", heartbeat_frame(base, 1))
        self.assertNotIn("cpu_load_permille", old)
        bad = bytearray(heartbeat_frame(base, 1))
        bad[99] = 1  # version 1 keeps the bytes reserved
        with self.assertRaises(ValueError):
            wire.decode("robot/heartbeat", bad)
        with self.assertRaises(ValueError):
            wire.decode("robot/heartbeat", heartbeat_frame(base, 3))

    def test_detour_decode(self):
        result = wire.decode("robot/detour", detour_frame())
        self.assertEqual((result["outcome"], result["side"], result["lateral_valid"]), (3, 1, 1))
        self.assertEqual((result["sequence"], result["start_ms"], result["end_ms"]), (4, 1000, 3500))
        self.assertEqual((result["duration_ms"], result["distance_mm"]), (2500, 1200))
        self.assertEqual((result["max_lateral_mm"], result["payload"], result["scan_id"]),
                         (340, 0, 9))
        for outcome in (3, 4, 7, 8):
            self.assertEqual(wire.decode("robot/detour", detour_frame(outcome))["outcome"],
                             outcome)
        wrap = wire.decode("robot/detour", detour_frame(start=0xFFFFFFF0, end=5, sequence=2 ** 32 - 1,
                                                        duration=21, distance=2 ** 32 - 1,
                                                        lateral=65535, scan=0))
        self.assertEqual((wrap["start_ms"], wrap["distance_mm"], wrap["max_lateral_mm"]),
                         (0xFFFFFFF0, 2 ** 32 - 1, 65535))

    def test_detour_rejects_bad_frames(self):
        good = detour_frame()
        for bad in (b"", good[:-1], good + b"\0", b"\x02" + good[1:]):
            with self.assertRaises(ValueError):
                wire.decode("robot/detour", bad)
        for kwargs in (dict(outcome=0), dict(outcome=1), dict(outcome=5), dict(outcome=6),
                       dict(outcome=9), dict(side=3), dict(flags=2), dict(flags=0x80)):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                wire.decode("robot/detour", detour_frame(**kwargs))

    def test_summary_decode(self):
        result = wire.decode("robot/summary", summary_frame())
        expected = dict(version=1, barcode_hump_implemented=0, speed_error_valid=1,
                        detour_aborted=0, stop_source=1, stop_reason=1, run_id=2, start_ms=5000,
                        duration_ms=60000, distance_mm=8000, line_losses=3, obstacles=2,
                        detours_done=2, detours_succeeded=1, detour_total_ms=7000,
                        detours_started=3, max_speed_error_mm_s=45, motion_fault_bits=5,
                        shell_fault=2, shell_fault_detail=6, barcode_ok=0, barcode_failed=0,
                        hump_peak=0xFFFF)
        self.assertEqual(result, expected)
        high = wire.decode("robot/summary", summary_frame(flags=0b101, source=4, reason=12,
                                                          run=2 ** 32 - 1, distance=2 ** 32 - 1,
                                                          speed_error=0xFFFF, barcode_ok=3,
                                                          barcode_failed=1, hump=15))
        self.assertEqual((high["barcode_hump_implemented"], high["speed_error_valid"],
                          high["detour_aborted"]), (1, 0, 1))
        self.assertEqual((high["stop_source"], high["stop_reason"], high["run_id"],
                          high["max_speed_error_mm_s"], high["hump_peak"]),
                         (4, 12, 2 ** 32 - 1, 0xFFFF, 15))

    def test_summary_rejects_bad_frames(self):
        good = summary_frame()
        for bad in (b"", good[:-1], good + b"\0", b"\x02" + good[1:]):
            with self.assertRaises(ValueError):
                wire.decode("robot/summary", bad)
        for kwargs in (dict(flags=0x08), dict(flags=0x80), dict(source=5), dict(reason=13)):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                wire.decode("robot/summary", summary_frame(**kwargs))

    def test_obstacle_scan_stage(self):
        for stage in (0, 1, 2):
            flags = 0b0101 | stage << 4
            result = wire.decode("robot/obstacle", obstacle_frame(flags=flags))
            self.assertEqual(result["scan_stage"], stage)
            self.assertEqual((result["closest_valid"], result["too_near"], result["left_blocked"],
                              result["right_blocked"]), (1, 0, 1, 0))
        with self.assertRaises(ValueError):
            wire.decode("robot/obstacle", obstacle_frame(flags=0x30))
        with self.assertRaises(ValueError):
            wire.decode("robot/obstacle", obstacle_frame(flags=0x40 | 0x10))

    def test_event_junction_is_accepted_and_kind_8_is_not(self):
        frame = bytearray(self.samples["robot/event"])
        frame[1] = 7
        self.assertEqual(wire.decode("robot/event", frame)["kind"], 7)
        for kind in (0, 8):
            frame[1] = kind
            with self.assertRaises(ValueError):
                wire.decode("robot/event", frame)

    def test_barcode_decode_and_rejects(self):
        result = wire.decode("robot/barcode", barcode_frame())
        self.assertEqual((result["known"], result["detected"], result["decoded"],
                          result["unreadable"], result["command_issued"], result["executed"]),
                         (1, 1, 1, 1, 0, 0))
        self.assertEqual((result["code"], result["votes"], result["reads"], result["direction"],
                          result["nav_command"], result["result"]), (5, 3, 4, 1, 9, -45))
        self.assertEqual((result["sequence"], result["uptime_ms"]), (12, 777))
        done = wire.decode("robot/barcode", barcode_frame(flags=0b11011, result=32767))
        self.assertEqual((done["command_issued"], done["executed"], done["result"]), (1, 1, 32767))
        good = barcode_frame()
        for bad in (good[:-1], good + b"\0"):
            with self.assertRaises(ValueError):
                wire.decode("robot/barcode", bad)
        for kwargs in (dict(flags=0x20), dict(code=8), dict(direction=3)):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                wire.decode("robot/barcode", barcode_frame(**kwargs))
        with self.assertRaises(ValueError):
            wire.decode("robot/barcode", good[:10] + b"\x01\x00" + good[12:])  # reserved u16

    def test_imu_decode_and_rejects(self):
        result = wire.decode("robot/imu", imu_snapshot())
        self.assertEqual((result["known"], result["kind"]), (1, 1))
        self.assertEqual((result["pitch_raw"], result["heading_raw"], result["turn_rate_raw"]),
                         (3, 200, 250))
        self.assertEqual((result["hump_state"], result["collision"], result["valid"]), (2, 1, 1))
        self.assertEqual((result["height_raw"], result["peak_raw"], result["age_or_seq"]),
                         (7, 9, 0xBEEF))
        edge = wire.decode("robot/imu", imu_snapshot(255, 255, 255, 3, 0, 0, 255, 255, 0xFFFF))
        self.assertEqual((edge["pitch_raw"], edge["hump_state"], edge["age_or_seq"]),
                         (255, 3, 0xFFFF))
        event = wire.decode("robot/imu", imu_event())
        self.assertEqual((event["known"], event["kind"], event["event"], event["sequence"],
                          event["uptime_ms"]), (1, 2, 6, 77, 4242))
        for bad in (imu_snapshot()[:-1], imu_snapshot() + b"\0", imu_event()[:-1],
                    imu_snapshot()[:2] + b"\x01\x00" + imu_snapshot()[4:],  # reserved bytes
                    imu_snapshot()[:7] + b"\x10" + imu_snapshot()[8:],  # spare bits
                    imu_event(event=0), imu_event(event=7),
                    imu_event()[:3] + b"\x01" + imu_event()[4:]):
            with self.assertRaises(ValueError):
                wire.decode("robot/imu", bad)

    def test_csv_has_every_new_field_and_accepts_every_new_row(self):
        fields = set(wire.csv_fields())
        for name in ("duty_left_permille", "duty_right_permille", "heading_cdeg",
                     "turn_rate_ddps", "cpu_load_permille", "scan_stage", "outcome", "side",
                     "lateral_valid", "max_lateral_mm", "payload", "duration_ms", "stop_source",
                     "stop_reason", "run_id", "detours_succeeded", "max_speed_error_mm_s",
                     "shell_fault_detail", "hump_peak", "barcode_hump_implemented"):
            self.assertIn(name, fields)
        base = self.samples["robot/state"]
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=wire.csv_fields())
        for topic, data in (("robot/state", state_v3_frame(base)),
                            ("robot/heartbeat", heartbeat_frame(self.samples["robot/heartbeat"])),
                            ("robot/detour", detour_frame()), ("robot/summary", summary_frame()),
                            ("robot/obstacle", obstacle_frame(flags=0x11)),
                            ("robot/barcode", barcode_frame()),
                            ("robot/imu", imu_snapshot()), ("robot/imu", imu_event())):
            writer.writerow(wire.decode(topic, data))

    def test_recorder_subscribes_to_the_new_topics_with_qos(self):
        self.assertEqual(wire.SIZES["state"], 72)
        self.assertEqual((wire.SIZES["detour"], wire.SIZES["summary"]), (32, 44))
        self.assertEqual(cli.SUBSCRIBE_QOS, {"log": 1, "status": 1, "summary": 1, "barcode": 1})


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
