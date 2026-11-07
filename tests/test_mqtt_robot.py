#!/usr/bin/env python3
"""Check Python decoding against independently compiled C packer output."""
import contextlib
import csv
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("mqtt_robot", REPO / "scripts/mqtt_robot.py")
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)


class WireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = {}
        for line in subprocess.check_output([sys.argv[1]], text=True).splitlines():
            topic, encoded = line.split()
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
            self.assertEqual(wire.main(["--host", "test.invalid"]), 0)
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
                self.assertEqual(wire.main(["--host", "test.invalid", "--command", "start",
                                           "--token-file", str(token), "--request-id", "0x12345678"]), 0)
            self.assertEqual(calls[-1], ("publish", "robot/cmd", self.samples["robot/cmd"], 1, False))
            self.assertNotIn(bytes(range(16)).hex(), error.getvalue())
            self.assertIn("not robot acceptance", error.getvalue())
            for optional_file in ([], ["--token-file", str(token)]):
                calls.clear()
                with mock.patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(wire.main(["--host", "test.invalid", "--command", "estop",
                                               "--request-id", "123", *optional_file]), 0)
                expected_token = bytes(range(16)) if optional_file else bytes(16)
                self.assertEqual(calls[-1], ("publish", "robot/cmd", wire.pack_command(1, 123, expected_token), 1, False))

    def test_non_estop_requires_token_file(self):
        for command in ("start", "calibrate", "dump"):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                wire.main(["--host", "test.invalid", "--command", command])
            self.assertEqual(error.exception.code, 2)

    def test_command_input(self):
        for opcode, request, token in ((0, 1, bytes(16)), (5, 1, bytes(16)),
                                       (1, -1, bytes(16)), (1, 1 << 32, bytes(16)), (1, 1, bytes(15))):
            with self.assertRaises(ValueError):
                wire.pack_command(opcode, request, token)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
