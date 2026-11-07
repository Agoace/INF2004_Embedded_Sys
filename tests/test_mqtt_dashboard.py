#!/usr/bin/env python3
"""Dashboard tests: no broker and no Paho. C-packed robot messages feed the model.

Usage: test_mqtt_dashboard.py <path to test_mqtt_wire>
"""
import contextlib
import http.client
import io
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import dashboard_model as model_module  # noqa: E402
import mqtt_dashboard as dashboard  # noqa: E402
import robot_wire  # noqa: E402

TOKEN = bytes.fromhex("00112233445566778899aabbccddeeff")  # public fixture, not a secret
BINARY = sys.argv[1]


class FakeClock:
    def __init__(self):
        self.seconds = 100.0

    def __call__(self):
        return self.seconds

    def advance(self, milliseconds):
        self.seconds += milliseconds / 1000.0


class FakeLink:
    def __init__(self, connected=True):
        self.connected = connected
        self.sent = []
        self.puback = True
        self.error = None

    def publish(self, payload, timeout):
        del timeout
        if self.error:
            raise self.error
        self.sent.append(payload)
        return self.puback


def c_samples():
    samples = {}
    for line in subprocess.check_output([BINARY], text=True).splitlines():
        topic, encoded = line.split()
        if topic.startswith("dash/"):
            samples[topic.removeprefix("dash/")] = bytes.fromhex(encoded)
    return samples


SAMPLES = c_samples()
TOPICS = {"state": "robot/state", "sonar": "robot/sonar", "event": "robot/event",
          "heartbeat": "robot/heartbeat", "obstacle": "robot/obstacle",
          "status": "robot/status"}


def topic_for(name):
    return TOPICS[name.split("_")[0]]


def batch(count=1, first=0, generation=1, flags=0, stream=0, skipped=0, newest=990,
          publish=1000, rows=None, kind=5):
    records = b"".join(bytes([(first + i + j) & 255 for j in range(16)]) for i in range(count))
    if kind == 6:
        records = bytes([1, 1, 0, 1] + [0] * 12) * count
    rows = first + count if rows is None else rows
    head = struct.pack("<BBBBIIIIIII", 1, kind, count, flags, stream, generation, first,
                       publish, newest, rows, skipped)
    return head + records


def control(kind, dump_id, generation, body):
    return struct.pack("<BBHIII", 1, kind, 0, dump_id, generation, 0) + body


def state_v2(name="state_idle", left=1500, right=1700, average=1600):
    """Version 2 state: the first 48 bytes are the C-packed v1 layout, then three i32."""
    return b"\x02" + SAMPLES[name][1:48] + struct.pack("<iii", left, right, average)


def obstacle(flags=0b0001, decision=1, angle=-15, scan=7, uptime=5000, closest=250, word=0x1234,
             right_step=3, left_step=7, right_clear=15, left_clear=255):
    return struct.pack("<BBBbIIHHBBBB", 1, flags, decision, angle, scan, uptime, closest, word,
                       right_step, left_step, right_clear, left_clear)


ONLINE = bytes([1, 1])
OFFLINE = bytes([1, 2])


class ModelCase(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.model = model_module.DashboardModel(clock=self.clock, token_configured=True)
        self.model.set_broker_connected(True)
        self.model.ingest("robot/status", ONLINE)

    def feed(self, name):
        self.model.ingest(topic_for(name), SAMPLES[name])

    def panel(self, panel_id):
        for panel in self.model.snapshot()["panels"]:
            if panel["id"] == panel_id:
                return panel
        raise AssertionError(panel_id)

    def rows(self, panel_id):
        return {row["label"]: row["value"] for row in self.panel(panel_id)["rows"]}

    def button(self, name):
        return self.model.snapshot()["buttons"][name]


class PanelTests(ModelCase):
    def test_every_panel_exists_without_data(self):
        snapshot = self.model.snapshot()
        ids = [panel["id"] for panel in snapshot["panels"]]
        self.assertEqual(ids, ["vehicle", "line", "wheels", "ultrasonic", "obstacle", "events",
                               "health", "barcode", "imu", "log"])
        for panel in snapshot["panels"]:
            self.assertTrue(panel["stale"])
        self.assertEqual(snapshot["robot"]["level"], "silent")  # status online, no state yet
        self.assertFalse(snapshot["robot"]["online"])
        json.dumps(snapshot)  # must be plain JSON

    def test_state_panels_from_c_packed_message(self):
        self.feed("state_idle")
        vehicle, line, wheels = self.rows("vehicle"), self.rows("line"), self.rows("wheels")
        self.assertEqual((vehicle["State"], vehicle["Module"], vehicle["Running"]),
                         ("IDLE", "none", "no"))
        self.assertEqual(vehicle["Vehicle board"], "fresh")
        self.assertEqual(line["Position"], -25)
        self.assertEqual(line["Raw left / centre"], "1234 / 2345")
        self.assertEqual(line["Normalised left / centre"], "500 / 600")
        self.assertEqual(wheels["Requested left / right"], "100 / -100 mm/s")
        self.assertEqual(wheels["Measured left / right"], "90 / -95 mm/s")
        self.assertEqual(wheels["Encoder counts left / right"], "1000 / -2000")
        self.assertEqual(wheels["Servo"], "-15 deg")
        self.assertEqual(self.rows("barcode")["Raw GP28 sample"], 777)
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")
        self.assertEqual(self.panel("line")["chart"]["values"], [-25])
        self.feed("state_running")
        vehicle = self.rows("vehicle")
        self.assertEqual((vehicle["State"], vehicle["Module"], vehicle["Running"]),
                         ("RUNNING", "line tracing", "yes"))

    def test_ultrasonic_latest_and_profile(self):
        self.feed("sonar_valid")
        self.assertEqual(self.rows("ultrasonic")["Latest"], "345 mm")
        self.clock.advance(200)
        self.feed("sonar_error")
        panel = self.panel("ultrasonic")
        self.assertEqual(self.rows("ultrasonic")["Latest"], "no rising edge")
        points = {p[0]: p for p in panel["chart"]["points"]}
        self.assertEqual(points[-30][1:3], [345, 200])
        self.assertEqual((points[30][1], points[30][3]), (None, "no rising edge"))
        self.assertEqual(list(points), [-30, 30])  # sorted by angle
        self.feed("sonar_error")  # same history sequence again: ignored
        self.assertEqual(len(self.panel("ultrasonic")["chart"]["points"]), 2)

    def test_events_and_command_decisions(self):
        self.feed("event_run_end")
        self.feed("event_start_not_idle")
        panel = self.panel("events")
        self.assertEqual(len(panel["items"]), 2)
        self.assertIn("START refused: robot not idle", panel["items"][0]["text"])
        self.assertIn("run end", panel["items"][1]["text"])
        self.assertEqual(self.rows("vehicle")["Last robot decision"],
                         "START refused: robot not idle")
        self.assertFalse(panel["stale"])  # events are sporadic and never grey out
        self.clock.advance(60000)
        self.assertFalse(self.panel("events")["stale"])

    def test_event_sequence_gaps(self):
        for name in ("event_start_ok", "event_start_not_idle", "event_calibrate_ok"):
            self.feed(name)  # sequence 10, 11, 14: two events missing
        self.assertEqual(self.model.snapshot()["gaps"]["event_gaps"], 2)

    def test_health_panel(self):
        self.feed("heartbeat")
        rows = self.rows("health")
        self.assertEqual(rows["Wi-Fi"], "up")
        self.assertEqual(rows["MQTT (robot side)"], "subscribed")
        self.assertEqual(rows["Reset reason"], "watchdog")
        self.assertEqual(rows["Wi-Fi poll gap max"], "25.0 ms (600 polls)")
        self.assertEqual(rows["Lost: telemetry / event / command"], "3 / 4 / 5")
        self.assertEqual(rows["Busy reads: line / sonar / motion / vehicle / health"],
                         "6 / 7 / 8 / 9 / 10")
        self.assertEqual(rows["Stack minimum free"], "1234 bytes")
        self.assertEqual(rows["Publish ok / failed"], "100 / 2")
        self.assertEqual(rows["Motion late / missed"], "12 / 13")

    def test_imu_is_a_placeholder(self):
        panel = self.panel("imu")
        self.assertTrue(panel["placeholder"] and panel["stale"])
        self.assertEqual(panel["rows"], [])

    def test_new_panel_needs_only_a_builder(self):
        @model_module.register_panel
        def extra_panel(model, now):
            del model, now
            return model_module.make_panel("extra", "Extra", 0, rows=[("answer", 42)])

        try:
            ids = [panel["id"] for panel in self.model.snapshot()["panels"]]
            self.assertEqual(ids[-1], "extra")
            self.assertFalse(self.panel("extra")["stale"])
        finally:
            model_module.PANEL_BUILDERS.remove(extra_panel)


class StaleTests(ModelCase):
    def test_state_panel_ages_add_reported_board_age(self):
        self.feed("state_idle")
        self.assertEqual(self.panel("line")["age_ms"], 5)
        self.assertEqual(self.panel("wheels")["age_ms"], 8)
        self.assertEqual(self.panel("vehicle")["age_ms"], 10)
        self.assertEqual(self.panel("barcode")["age_ms"], 0)
        self.clock.advance(990)
        self.assertEqual(self.panel("line")["age_ms"], 995)
        self.assertFalse(self.panel("line")["stale"])
        self.clock.advance(10)  # 1005 ms: older than one second
        self.assertTrue(self.panel("line")["stale"])
        self.assertTrue(self.panel("wheels")["stale"])  # 1010 + 8 ms

    def test_boundary_is_one_second(self):
        self.feed("state_idle")
        self.clock.advance(992)
        self.assertEqual(self.panel("wheels")["age_ms"], 1000)
        self.assertFalse(self.panel("wheels")["stale"])
        self.clock.advance(1)
        self.assertTrue(self.panel("wheels")["stale"])

    def test_missing_board_is_stale_even_when_message_is_new(self):
        self.feed("state_vehicle_stale")
        panel = self.panel("vehicle")
        self.assertEqual(panel["age_ms"], 65535)
        self.assertTrue(panel["stale"])
        self.assertFalse(self.panel("line")["stale"])

    def test_heartbeat_uses_a_longer_limit(self):
        self.feed("heartbeat")
        self.clock.advance(2000)
        self.assertFalse(self.panel("health")["stale"])
        self.clock.advance(600)
        self.assertTrue(self.panel("health")["stale"])

    def test_sonar_goes_stale(self):
        self.feed("sonar_valid")
        self.clock.advance(1100)
        self.assertTrue(self.panel("ultrasonic")["stale"])
        self.assertEqual(self.rows("ultrasonic")["Latest"], "345 mm")  # old value is still shown

    def test_robot_status(self):
        self.assertEqual(self.model.snapshot()["robot"]["level"], "silent")
        self.feed("heartbeat")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "silent")  # heartbeat is not state
        self.feed("state_idle")
        robot = self.model.snapshot()["robot"]
        self.assertEqual((robot["online"], robot["level"], robot["text"]),
                         (True, "online", "ROBOT ONLINE"))
        self.clock.advance(3100)
        robot = self.model.snapshot()["robot"]
        self.assertFalse(robot["online"])
        self.assertEqual((robot["level"], robot["text"]), ("silent", "robot silent: no state for 3 s"))

    def test_malformed_messages_are_counted_and_change_nothing(self):
        self.feed("state_idle")
        good = SAMPLES["state_idle"]
        for bad in (b"", good[:-1], b"\x09" + good[1:]):
            self.model.ingest("robot/state", bad)
        self.model.ingest("robot/other", good)
        self.model.ingest("robot/log", b"\x01\x05\x00")
        snapshot = self.model.snapshot()
        self.assertEqual(snapshot["discarded"], 5)
        self.assertEqual(self.rows("line")["Position"], -25)
        self.assertEqual(self.model.last_discard[:10], "robot/log:")


class GapTests(ModelCase):
    def test_row_gap_counter(self):
        def log(**kwargs):
            self.model.ingest("robot/log", batch(**kwargs))

        log(count=10, first=0)
        log(count=10, first=10)
        self.assertEqual(self.model.snapshot()["gaps"]["total"], 0)
        log(count=10, first=40)  # 20 rows lost
        log(count=10, first=300, flags=4, skipped=250)  # reconnect jump, 250 skipped
        gaps = self.model.snapshot()["gaps"]
        self.assertEqual((gaps["lost_rows"], gaps["jumped_rows"], gaps["total"]), (20, 250, 270))
        rows = self.rows("log")
        self.assertEqual(rows["Live rows received"], 40)
        self.assertEqual((rows["Rows lost (index gaps)"], rows["Rows skipped (jumps)"]), (20, 250))
        self.assertEqual(rows["Jump frames"], 1)
        self.assertEqual(rows["Data age"], "10 ms")
        self.assertEqual(rows["Rows behind"], 0)
        self.assertEqual(rows["Generation / last index"], "1 / 309")

    def test_refill_and_duplicates_do_not_add_gaps(self):
        self.model.ingest("robot/log", batch(count=10, first=0))
        refill = batch(count=10, first=0, flags=1, stream=7, newest=0)
        self.model.ingest("robot/log", refill)
        self.model.ingest("robot/log", refill)
        self.model.ingest("robot/log", batch(count=10, first=20, flags=1, stream=7, newest=0))
        self.assertEqual(self.model.snapshot()["gaps"]["total"], 0)
        rows = self.rows("log")
        self.assertEqual((rows["Live rows received"], rows["Refill rows (DUMP) merged"]),
                         (10, 10))
        self.assertEqual(rows["Duplicates dropped"], 20)

    def test_log_data_age_grows_and_goes_stale(self):
        self.model.ingest("robot/log", batch(count=2, publish=1500, newest=1000))
        self.assertEqual(self.panel("log")["age_ms"], 500)
        self.clock.advance(600)
        panel = self.panel("log")
        self.assertEqual(panel["age_ms"], 1100)
        self.assertTrue(panel["stale"])

    def test_dump_report_appears_in_log_panel(self):
        self.model.ingest("robot/log", control(0, 5, 1, struct.pack("<IIII", 12, 0, 0, 0)))
        self.model.ingest("robot/log", batch(count=10, first=0, flags=1, stream=5, newest=0))
        self.model.ingest("robot/log", control(3, 5, 1, struct.pack("<IIII", 10, 0, 0, 0)))
        self.assertEqual(self.panel("log")["items"][0]["text"],
                         "dump 5: missing rows [10-11] events []")


class ButtonTests(ModelCase):
    def idle_fresh(self):
        self.feed("state_idle")

    def test_estop_is_always_enabled(self):
        self.assertTrue(self.button("estop")["enabled"])  # no data at all
        self.model.set_broker_connected(False)
        estop = self.button("estop")
        self.assertTrue(estop["enabled"])
        self.assertIn("broker not connected", estop["reason"])
        self.feed("state_running")
        self.assertTrue(self.button("estop")["enabled"])
        self.assertFalse(self.button("estop")["needs_confirm"])

    def test_start_and_calibrate_need_idle_and_fresh_data(self):
        self.assertIn("robot is silent", self.button("start")["reason"])  # status, no state
        self.idle_fresh()
        for name in ("start", "calibrate"):
            self.assertEqual(self.button(name), {"enabled": True, "reason": "",
                                                 "needs_confirm": True})
        self.assertTrue(self.button("dump")["enabled"])
        self.assertFalse(self.button("dump")["needs_confirm"])
        self.clock.advance(1100)  # state is stale
        for name in ("start", "calibrate"):
            self.assertFalse(self.button(name)["enabled"])
            self.assertIn("ms old", self.button(name)["reason"])
        self.assertTrue(self.button("dump")["enabled"])  # DUMP may run while driving

    def test_not_idle_disables(self):
        self.feed("state_running")
        for name in ("start", "calibrate"):
            self.assertEqual(self.button(name)["reason"], "robot is not idle")
        self.assertTrue(self.button("dump")["enabled"])

    def test_vehicle_board_not_fresh_disables(self):
        self.feed("state_vehicle_stale")
        self.assertEqual(self.button("start")["reason"], "vehicle board data is not fresh")

    def test_broker_down_and_missing_token(self):
        self.idle_fresh()
        self.model.set_broker_connected(False)
        for name in ("start", "calibrate", "dump"):
            self.assertEqual(self.button(name)["reason"], "broker not connected")
        self.model.set_broker_connected(True)
        self.model.token_configured = False
        for name in ("start", "calibrate", "dump"):
            self.assertEqual(self.button(name)["reason"], "no token file: only ESTOP is available")

    def test_dump_disabled_while_dump_active(self):
        self.feed("state_idle")
        self.feed("heartbeat_dump")
        self.assertEqual(self.button("dump")["reason"], "a RAM dump is already running")
        self.clock.advance(2900)  # stale heartbeat: do not trust the flag
        self.assertTrue(self.button("dump")["enabled"])

    def test_pending_command_blocks_until_decision_or_timeout(self):
        self.idle_fresh()
        self.model.note_command("calibrate", 77, True)
        self.assertIn("waiting for the robot decision", self.button("start")["reason"])
        self.assertTrue(self.button("estop")["enabled"])
        self.feed("event_start_ok")  # a decision for a different command
        self.assertFalse(self.button("start")["enabled"])
        self.feed("event_calibrate_ok")
        self.feed("state_idle")
        self.assertTrue(self.button("start")["enabled"])
        self.model.note_command("start", 78, True)
        self.clock.advance(5100)
        self.feed("state_idle")
        self.assertTrue(self.button("start")["enabled"])

    def test_command_result_status(self):
        self.assertIsNone(self.model.snapshot()["last_command"])
        self.feed("event_start_ok")  # earlier decision must not match a later command
        self.clock.advance(10)
        self.model.note_command("start", 5, True)
        status = self.model.snapshot()["last_command"]
        self.assertEqual((status["command"], status["request_id"], status["puback"]),
                         ("START", 5, True))
        self.assertEqual(status["status"], "sent; waiting for the robot decision")
        self.feed("event_start_not_idle")
        self.assertEqual(self.model.snapshot()["last_command"]["status"],
                         "robot decision: refused: robot not idle")
        self.assertEqual(self.rows("vehicle")["Command result"],
                         "robot decision: refused: robot not idle")
        self.model.note_command("start", 6, False)
        self.clock.advance(5100)
        text = self.model.snapshot()["last_command"]["status"]
        self.assertIn("no robot decision received", text)
        self.assertIn("PUBACK is not acceptance", text)


class WheelDistanceTests(ModelCase):
    def test_distance_from_state_v2(self):
        self.model.ingest("robot/state", state_v2(left=-1500, right=2500, average=500))
        rows = self.rows("wheels")
        self.assertEqual(rows["Distance left / right / average"], "-1500 / 2500 / 500 mm")
        self.assertEqual(self.panel("wheels")["age_ms"], 8)  # motion board age
        self.assertIn("since robot boot", self.panel("wheels")["note"])
        self.assertEqual(rows["Encoder counts left / right"], "1000 / -2000")

    def test_distance_follows_the_newest_state(self):
        self.model.ingest("robot/state", state_v2(left=10, right=20, average=15))
        self.model.ingest("robot/state", state_v2(left=11, right=21, average=16))
        self.assertEqual(self.rows("wheels")["Distance left / right / average"],
                         "11 / 21 / 16 mm")

    def test_old_v1_state_does_not_invent_distance(self):
        self.model.ingest("robot/state", b"\x01" + SAMPLES["state_idle"][1:48])
        self.assertEqual(self.rows("wheels")["Distance left / right / average"],
                         "not reported (old state v1)")

    def test_v2_state_still_drives_every_other_panel_and_the_button_rules(self):
        self.model.ingest("robot/state", state_v2())
        self.assertEqual(self.rows("line")["Position"], -25)
        self.assertEqual(self.rows("vehicle")["State"], "IDLE")
        self.assertTrue(self.button("start")["enabled"])
        self.model.ingest("robot/state", state_v2("state_running"))
        self.assertFalse(self.button("start")["enabled"])


class ObstacleTests(ModelCase):
    def feed_obstacle(self, **kwargs):
        self.model.ingest("robot/obstacle", obstacle(**kwargs))

    def test_before_any_scan(self):
        panel = self.panel("obstacle")
        self.assertEqual((panel["age_ms"], panel["note"]), (None, "no completed scan yet"))

    def test_summary_rows(self):
        self.feed_obstacle()
        rows = self.rows("obstacle")
        self.assertEqual(rows["Closest"], "250 mm at -15 deg (right)")
        self.assertEqual(rows["Right clearance"], "15 deg (3 steps free)")
        self.assertEqual(rows["Left clearance"], "blocked")
        self.assertEqual(rows["Detour decision"], "right")
        self.assertEqual((rows["Scan ID"], rows["Scans missed (ID gaps)"]), (7, 0))
        self.assertEqual(rows["Profile word"], "0x1234")
        self.feed_obstacle(scan=8, angle=30, decision=2, right_clear=255, left_clear=20,
                           left_step=4, right_step=7)
        rows = self.rows("obstacle")
        self.assertEqual(rows["Closest"], "250 mm at 30 deg (left)")
        self.assertEqual((rows["Detour decision"], rows["Left clearance"],
                          rows["Right clearance"]), ("left", "20 deg (4 steps free)", "blocked"))
        self.feed_obstacle(scan=9, angle=0, decision=0, flags=0b1101, closest=100)
        self.assertEqual(self.rows("obstacle")["Closest"], "100 mm at 0 deg (ahead)")
        self.assertEqual(self.rows("obstacle")["Detour decision"], "none (both sides blocked)")

    def test_closest_special_cases(self):
        self.feed_obstacle(flags=0b0010, closest=0, angle=0, scan=1)  # too near
        self.assertEqual(self.rows("obstacle")["Closest"], "too near (below sensor range)")
        self.feed_obstacle(flags=0b0000, closest=511, angle=0, scan=2)  # nothing seen
        self.assertEqual(self.rows("obstacle")["Closest"], "none seen")
        self.assertIsNone(self.panel("obstacle")["chart"]["closest"])

    def test_scan_id_gaps_are_counted(self):
        for scan in (1, 2, 5, 6, 10):
            self.feed_obstacle(scan=scan)
        self.assertEqual(self.rows("obstacle")["Scans missed (ID gaps)"], 2 + 3)
        self.feed_obstacle(scan=1)  # the robot restarted: a lower ID is not a gap
        self.assertEqual(self.rows("obstacle")["Scans missed (ID gaps)"], 5)
        self.feed_obstacle(scan=2)
        self.assertEqual(self.rows("obstacle")["Scans missed (ID gaps)"], 5)

    def test_chart_overlays_the_scan_profile(self):
        self.feed("sonar_valid")
        self.feed("sonar_error")
        self.feed_obstacle(angle=-30, closest=345)
        chart = self.panel("obstacle")["chart"]
        self.assertEqual(chart["type"], "obstacle")
        self.assertEqual([p[0] for p in chart["points"]], [-30, 30])
        self.assertEqual(chart["closest"], {"angle": -30, "mm": 345})
        self.assertEqual((chart["right_clear_deg"], chart["left_clear_deg"], chart["decision"]),
                         (15, 255, 1))

    def test_age_is_time_since_scan_and_never_grey(self):
        self.feed_obstacle()
        self.clock.advance(60000)
        panel = self.panel("obstacle")
        self.assertEqual(panel["age_ms"], 60000)
        self.assertFalse(panel["stale"])  # a scan result is a point in time

    def test_bad_frame_is_discarded(self):
        self.feed_obstacle()
        self.model.ingest("robot/obstacle", obstacle(decision=3))
        self.model.ingest("robot/obstacle", obstacle()[:-1])
        self.assertEqual(self.model.snapshot()["discarded"], 2)
        self.assertEqual(self.rows("obstacle")["Scan ID"], 7)


class CPackedNewTopicTests(ModelCase):
    def test_c_packed_obstacle_drives_the_panel(self):
        self.feed("obstacle")
        rows = self.rows("obstacle")
        self.assertEqual(rows["Closest"], "too near (below sensor range)")
        self.assertEqual(rows["Right clearance"], "blocked")
        self.assertEqual(rows["Left clearance"], "15 deg (3 steps free)")
        self.assertEqual(rows["Detour decision"], "left")
        self.assertEqual((rows["Scan ID"], rows["Profile word"]), (0x01020304, "0x5a5a"))
        chart = self.panel("obstacle")["chart"]
        self.assertEqual(chart["closest"], {"angle": -30, "mm": 510})

    def test_c_packed_status_drives_the_robot_link(self):
        self.model.status = None
        self.feed("state_idle")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "unknown")
        self.feed("status_online")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "online")
        self.feed("status_offline")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "offline")
        self.assertFalse(self.button("start")["enabled"])
        self.assertTrue(self.button("estop")["enabled"])

    def test_c_packed_state_is_v2_with_distance_fields(self):
        self.assertEqual((len(SAMPLES["state_idle"]), SAMPLES["state_idle"][0]), (60, 2))
        self.feed("state_idle")
        self.assertEqual(self.rows("wheels")["Distance left / right / average"], "0 / 0 / 0 mm")
        self.assertTrue(self.button("start")["enabled"])


class ReservedTopicTests(ModelCase):
    def test_not_implemented_until_data_arrives(self):
        self.assertEqual(self.panel("imu")["note"], "not implemented: no data on robot/imu yet")
        self.assertTrue(self.panel("imu")["placeholder"])
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")
        self.feed("state_idle")
        self.assertEqual(self.rows("barcode")["Raw GP28 sample"], 777)
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")

    def test_data_is_acknowledged_but_not_decoded(self):
        self.model.ingest("robot/barcode", bytes([1, 3, 9, 9]))
        self.model.ingest("robot/imu", bytes([1, 1, 0, 0, 0, 0]))
        self.assertEqual(self.rows("barcode")["Decoded command"],
                         "data received (kind 3, 4 bytes); decoder not implemented")
        imu = self.panel("imu")
        self.assertFalse(imu["placeholder"])
        self.assertEqual(imu["rows"][0]["value"],
                         "data received (kind 1, 6 bytes); decoder not implemented")
        self.assertFalse(imu["stale"])
        self.clock.advance(1500)
        self.assertTrue(self.panel("imu")["stale"])

    def test_unknown_versions_are_ignored_not_counted_as_errors(self):
        self.model.ingest("robot/barcode", bytes([2, 1, 5]))
        self.model.ingest("robot/imu", bytes([9, 1, 5]))
        self.assertEqual((self.model.ignored_reserved, self.model.snapshot()["discarded"]), (2, 0))
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")
        self.assertTrue(self.panel("imu")["placeholder"])
        self.model.ingest("robot/imu", b"\x01")  # too short to read a header
        self.assertEqual(self.model.snapshot()["discarded"], 1)


class RobotStatusTests(ModelCase):
    def fresh_idle(self):
        self.feed("state_idle")

    def test_online_needs_status_and_recent_state(self):
        self.fresh_idle()
        robot = self.model.snapshot()["robot"]
        self.assertEqual((robot["level"], robot["status"]["retained"]), ("online", False))
        self.clock.advance(3100)
        self.assertEqual(self.model.snapshot()["robot"]["level"], "silent")

    def test_retained_offline_shows_at_once_and_blocks_commands(self):
        self.fresh_idle()
        self.model.ingest("robot/status", OFFLINE, retained=True)
        robot = self.model.snapshot()["robot"]
        self.assertEqual((robot["level"], robot["online"]), ("offline", False))
        self.assertEqual(robot["text"], "ROBOT OFFLINE (broker last will)")
        self.assertTrue(robot["status"]["retained"])
        for name in ("start", "calibrate", "dump"):
            self.assertEqual(self.button(name), {"enabled": False, "reason": "robot is offline",
                                                 "needs_confirm": name != "dump"})

    def test_estop_stays_enabled_with_a_warning_when_offline(self):
        self.fresh_idle()
        self.model.ingest("robot/status", OFFLINE)
        estop = self.button("estop")
        self.assertTrue(estop["enabled"])
        self.assertIn("delivery is unlikely", estop["reason"])
        self.assertIn("GP21", estop["reason"])
        self.model.ingest("robot/status", ONLINE)
        self.assertEqual(self.button("estop")["reason"], "")

    def test_robot_comes_back_online(self):
        self.fresh_idle()
        self.model.ingest("robot/status", OFFLINE, retained=True)
        self.assertFalse(self.button("start")["enabled"])
        self.model.ingest("robot/status", ONLINE)  # robot reconnected, retained online
        self.fresh_idle()
        self.assertEqual(self.model.snapshot()["robot"]["level"], "online")
        self.assertTrue(self.button("start")["enabled"])

    def test_silent_robot_blocks_everything_except_estop(self):
        self.fresh_idle()
        self.clock.advance(3500)
        for name in ("start", "calibrate", "dump"):
            self.assertEqual(self.button(name)["reason"], "robot is silent (no state data)")
        self.assertTrue(self.button("estop")["enabled"])

    def test_unknown_status_keeps_the_older_rules_when_data_is_recent(self):
        self.model.status = None  # an old firmware: no robot/status was ever sent
        self.fresh_idle()
        robot = self.model.snapshot()["robot"]
        self.assertEqual((robot["level"], robot["online"]), ("unknown", False))
        self.assertTrue(self.button("start")["enabled"])
        self.clock.advance(3500)
        self.assertEqual(self.button("dump")["reason"], "no recent robot data")

    def test_broker_disconnect_forgets_the_status(self):
        self.model.ingest("robot/status", OFFLINE, retained=True)
        self.model.set_broker_connected(False)
        self.assertIsNone(self.model.snapshot()["robot"]["status"])
        self.model.set_broker_connected(True)
        self.assertEqual(self.model.snapshot()["robot"]["level"], "unknown")  # retained comes again

    def test_bad_status_is_discarded(self):
        for bad in (b"", bytes([1, 0]), bytes([1, 3]), bytes([2, 1]), bytes([1, 1, 1])):
            self.model.ingest("robot/status", bad)
        self.assertEqual(self.model.snapshot()["discarded"], 5)
        self.assertTrue(self.model.status["online"])

    def test_commander_refuses_gated_commands_when_offline_but_sends_estop(self):
        link = FakeLink()
        commander = dashboard.Commander(link, self.model, TOKEN, lambda: 5)
        self.fresh_idle()
        self.model.ingest("robot/status", OFFLINE, retained=True)
        for name in ("start", "calibrate", "dump"):
            status, result = commander.send(name, True)
            self.assertEqual((status, result["error"]), (409, "robot is offline"))
        self.assertEqual(link.sent, [])
        self.assertEqual(commander.send("estop", None)[0], 200)
        self.assertEqual(len(link.sent), 1)


class CommanderTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.model = model_module.DashboardModel(clock=self.clock, token_configured=True)
        self.model.set_broker_connected(True)
        self.link = FakeLink()
        self.commander = dashboard.Commander(self.link, self.model, TOKEN, lambda: 0x12345678)
        self.model.ingest("robot/status", ONLINE)
        self.model.ingest("robot/state", SAMPLES["state_idle"])

    def test_payload_follows_the_wire_contract(self):
        status, result = self.commander.send("start", True)
        self.assertEqual(status, 200)
        self.assertEqual(self.link.sent,
                         [struct.pack("<BBI", 1, 4, 0x12345678) + TOKEN])
        self.assertEqual((result["request_id"], result["puback"]), (0x12345678, True))
        self.assertIn("PUBACK only means the broker", dashboard.NOTE)
        self.assertEqual(self.model.snapshot()["last_command"]["command"], "START")

    def test_opcodes(self):
        for name, code in (("dump", 2), ("calibrate", 3)):
            self.clock.advance(6000)
            self.model.ingest("robot/state", SAMPLES["state_idle"])
            self.assertEqual(self.commander.send(name, True)[0], 200)
            self.assertEqual(self.link.sent[-1][1], code)
        self.assertEqual(self.commander.send("estop", False)[0], 200)
        self.assertEqual(self.link.sent[-1][1], 1)

    def test_confirm_is_required_for_start_and_calibrate(self):
        for confirm in (False, None, "yes", 1):
            status, result = self.commander.send("start", confirm)
            self.assertEqual((status, result["error"]), (409, "confirmation required"))
        self.assertEqual(self.link.sent, [])

    def test_gate_is_rechecked_before_publishing(self):
        self.model.ingest("robot/state", SAMPLES["state_running"])
        status, result = self.commander.send("start", True)
        self.assertEqual((status, result["error"]), (409, "robot is not idle"))
        self.assertEqual(self.link.sent, [])

    def test_estop_ignores_gates_but_needs_a_connection(self):
        self.model.ingest("robot/state", SAMPLES["state_running"])
        self.assertEqual(self.commander.send("estop", None)[0], 200)
        self.link.connected = False
        status, result = self.commander.send("estop", None)
        self.assertEqual((status, result["error"]), (503, "broker not connected"))

    def test_estop_without_token_file_sends_zero_token(self):
        commander = dashboard.Commander(self.link, self.model, None, lambda: 9)
        self.model.token_configured = False
        self.assertEqual(commander.send("estop", None)[0], 200)
        self.assertEqual(self.link.sent[-1], struct.pack("<BBI", 1, 1, 9) + bytes(16))
        self.assertEqual(commander.send("dump", None)[0], 409)

    def test_publish_failure_and_unknown_command(self):
        self.link.error = ConnectionError("down")
        self.assertEqual(self.commander.send("estop", None), (503, {"ok": False,
                                                                    "error": "publish failed"}))
        self.assertIsNone(self.model.snapshot()["last_command"])
        self.assertEqual(self.commander.send("reboot", True)[0], 400)

    def test_no_puback_is_reported_not_hidden(self):
        self.link.puback = False
        status, result = self.commander.send("dump", None)
        self.assertEqual((status, result["puback"]), (200, False))


class AppCase(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.model = model_module.DashboardModel(clock=self.clock, token_configured=True)
        self.model.set_broker_connected(True)
        self.model.ingest("robot/status", ONLINE)
        self.model.ingest("robot/state", SAMPLES["state_idle"])
        self.link = FakeLink()
        commander = dashboard.Commander(self.link, self.model, TOKEN, lambda: 4242)
        self.app = dashboard.DashboardApp(self.model, commander, 0, csrf="fixed-test-csrf")
        self.server = dashboard.DashboardServer(self.app, 0)
        self.port = self.server.server_address[1]
        self.app.set_port(self.port)
        self.thread = threading.Thread(target=self.server.serve_forever, args=(0.02,), daemon=True)
        self.thread.start()
        self.log = io.StringIO()
        self.log_patch = contextlib.redirect_stderr(self.log)
        self.log_patch.__enter__()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.log_patch.__exit__(None, None, None)

    def request(self, method, path, body=None, headers=None, host=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        merged = {"Host": host or f"127.0.0.1:{self.port}"}
        merged.update(headers or {})
        try:
            data = body.encode("utf-8") if isinstance(body, str) else body
            connection.putrequest(method, path, skip_host=True, skip_accept_encoding=True)
            for name, value in merged.items():
                connection.putheader(name, value)
            if data is not None:
                connection.putheader("Content-Length", str(len(data)))
            connection.endheaders(data)
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def good_headers(self, **extra):
        headers = {"Origin": f"http://127.0.0.1:{self.port}", "Content-Type": "application/json",
                   "X-CSRF-Token": "fixed-test-csrf"}
        headers.update(extra)
        return headers

    def post(self, command="estop", confirm=None, **extra):
        body = json.dumps({"command": command, "confirm": confirm})
        return self.request("POST", "/api/command", body, self.good_headers(**extra))


class SecurityTests(AppCase):
    def test_binds_loopback_only(self):
        self.assertEqual(self.server.server_address[0], "127.0.0.1")

    def test_valid_post_publishes_once(self):
        status, _, body = self.post("estop")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["request_id"], 4242)
        self.assertEqual(self.link.sent, [struct.pack("<BBI", 1, 1, 4242) + TOKEN])
        self.assertEqual(self.post("start", True)[0], 409)  # waits for the ESTOP decision
        self.clock.advance(5100)
        self.model.ingest("robot/state", SAMPLES["state_idle"])
        self.assertEqual(self.post("start", True)[0], 200)
        self.assertEqual(len(self.link.sent), 2)

    def test_post_rejections_publish_nothing(self):
        cases = {
            "no origin": {"Origin": None},
            "other origin": {"Origin": "http://evil.example"},
            "null origin": {"Origin": "null"},
            "origin port": {"Origin": f"http://127.0.0.1:{self.port + 1}"},
            "https origin": {"Origin": f"https://127.0.0.1:{self.port}"},
            "no csrf": {"X-CSRF-Token": None},
            "wrong csrf": {"X-CSRF-Token": "guess"},
            "form type": {"Content-Type": "application/x-www-form-urlencoded"},
            "plain type": {"Content-Type": "text/plain"},
            "no type": {"Content-Type": None},
        }
        for name, change in cases.items():
            headers = self.good_headers()
            for key, value in change.items():
                if value is None:
                    headers.pop(key)
                else:
                    headers[key] = value
            body = json.dumps({"command": "start", "confirm": True})
            status, _, _ = self.request("POST", "/api/command", body, headers)
            self.assertEqual(status, 403, name)
        status, _, _ = self.request("POST", "/api/command", "{}", self.good_headers(),
                                    host="evil.example")
        self.assertEqual(status, 403)
        status, _, _ = self.request("POST", "/api/command", "{}", self.good_headers(),
                                    host=f"evil.example:{self.port}")
        self.assertEqual(status, 403)
        self.assertEqual(self.link.sent, [])

    def test_cross_site_form_post_cannot_start_the_robot(self):
        headers = {"Origin": "http://evil.example", "Content-Type": "text/plain"}
        body = json.dumps({"command": "start", "confirm": True})
        self.assertEqual(self.request("POST", "/api/command", body, headers)[0], 403)
        self.assertEqual(self.link.sent, [])

    def test_localhost_name_is_accepted(self):
        headers = self.good_headers(Origin=f"http://localhost:{self.port}")
        body = json.dumps({"command": "estop"})
        status, _, _ = self.request("POST", "/api/command", body, headers,
                                    host=f"localhost:{self.port}")
        self.assertEqual(status, 200)

    def test_bad_bodies(self):
        for body in ("", "not json", "[]", "{}", '{"command": 5}', '{"command": "reboot"}'):
            status, _, _ = self.request("POST", "/api/command", body, self.good_headers())
            self.assertIn(status, (400,), body)
        self.assertEqual(self.request("POST", "/api/command", "x" * 2000,
                                      self.good_headers())[0], 400)
        self.assertEqual(self.link.sent, [])

    def test_unconfirmed_and_gated_commands_are_refused(self):
        status, _, body = self.post("start", False)
        self.assertEqual((status, json.loads(body)["error"]), (409, "confirmation required"))
        self.model.ingest("robot/state", SAMPLES["state_running"])
        status, _, body = self.post("calibrate", True)
        self.assertEqual((status, json.loads(body)["error"]), (409, "robot is not idle"))
        self.assertEqual(self.link.sent, [])

    def test_get_needs_the_right_host(self):
        for path in ("/", "/api/state", "/events"):
            self.assertEqual(self.request("GET", path, host="rebind.example")[0], 403)
        self.assertEqual(self.request("GET", "/other")[0], 404)
        self.assertEqual(self.request("PUT", "/api/command")[0], 501)

    def test_page_has_csp_and_csrf_but_no_remote_resources(self):
        status, headers, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        page = body.decode()
        self.assertIn('content="fixed-test-csrf"', page)
        policy = headers["Content-Security-Policy"]
        self.assertIn("default-src 'none'", policy)
        self.assertIn("frame-ancestors 'none'", policy)
        nonce = policy.split("'nonce-")[1].split("'")[0]
        self.assertGreaterEqual(page.count(f'nonce="{nonce}"'), 2)
        self.assertNotIn("__NONCE__", page)
        self.assertNotIn("__CSRF__", page)
        for remote in ("http://", "https://", "//cdn", "src=", "@import"):
            self.assertNotIn(remote, page.replace("http://www.w3.org", ""))
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertNotIn("Access-Control-Allow-Origin", headers)
        self.assertNotEqual(self.request("GET", "/")[1]["Content-Security-Policy"], policy)

    def test_token_is_never_sent_or_logged(self):
        secret_forms = (TOKEN.hex().encode(), TOKEN, TOKEN.hex().upper().encode())
        responses = [self.request("GET", "/"), self.request("GET", "/api/state"),
                     self.post("estop"), self.post("start", True), self.post("start", False),
                     self.post("dump", None), self.post("reboot", True),
                     self.request("POST", "/api/command", "{}", {"Origin": "http://x"}),
                     self.request("GET", "/nope")]
        events = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        events.request("GET", "/events", headers={"Host": f"127.0.0.1:{self.port}"})
        stream = events.getresponse()
        frame = stream.fp.readline() + stream.fp.readline()
        events.close()
        self.assertTrue(frame.startswith(b"data: "))
        blobs = [body + repr(headers).encode() for _, headers, body in responses] + [frame]
        blobs.append(self.log.getvalue().encode())
        blobs.append(json.dumps(self.model.snapshot()).encode())
        for blob in blobs:
            for secret in secret_forms:
                self.assertNotIn(secret, blob)
        self.assertIn("http POST /api/command", self.log.getvalue())
        self.assertNotIn("fixed-test-csrf", self.log.getvalue())

    def test_event_stream_and_poll_agree(self):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request("GET", "/events", headers={"Host": f"127.0.0.1:{self.port}"})
        response = connection.getresponse()
        self.assertEqual(response.getheader("Content-Type"), "text/event-stream")
        data = json.loads(response.fp.readline().decode().removeprefix("data: "))
        connection.close()
        polled = json.loads(self.request("GET", "/api/state")[2])
        self.assertEqual(data["buttons"], polled["buttons"])
        self.assertEqual([p["id"] for p in data["panels"]], [p["id"] for p in polled["panels"]])
        self.assertTrue(data["buttons"]["start"]["enabled"])


class PageTests(unittest.TestCase):
    def script(self):
        import dashboard_page
        text = dashboard_page.PAGE
        return text.split("<script nonce=\"__NONCE__\">")[1].split("</script>")[0]

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_inline_script_is_valid_javascript(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "page.js"
            path.write_text(self.script())
            result = subprocess.run(["node", "--check", str(path)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_page_draws_any_panel_and_never_assigns_html_from_data(self):
        script = self.script()
        self.assertIn("CHART_RENDERERS", script)
        self.assertNotIn("eval(", script)
        self.assertEqual(script.count("innerHTML"), 1)  # one static template, no data in it
        self.assertIn('id="banner"', __import__("dashboard_page").PAGE)
        self.assertIn("obstacle(canvas, chart)", script)
        for command in ("estop", "dump", "start", "calibrate"):
            self.assertIn(f'data-command="{command}"', __import__("dashboard_page").PAGE)


class PahoLinkTests(unittest.TestCase):
    def fake_paho(self, calls):
        class Info:
            def wait_for_publish(self, timeout):
                calls.append(("wait", timeout))

            def is_published(self):
                return True

        class Client:
            def __init__(self, api):
                calls.append(("api", api))

            def reconnect_delay_set(self, min_delay, max_delay):
                calls.append(("delay", min_delay, max_delay))

            def connect_async(self, host, port, keepalive):
                calls.append(("connect_async", host, port, keepalive))

            def loop_start(self):
                calls.append(("loop_start",))

            def subscribe(self, topics):
                calls.append(("subscribe", topics))

            def publish(self, topic, payload, qos, retain):
                calls.append(("publish", topic, payload, qos, retain))
                return Info()

            def disconnect(self):
                calls.append(("disconnect",))

            def loop_stop(self):
                calls.append(("loop_stop",))

        modules = {name: types.ModuleType(name) for name in ("paho", "paho.mqtt",
                                                             "paho.mqtt.client")}
        modules["paho"].mqtt = modules["paho.mqtt"]
        modules["paho.mqtt"].client = modules["paho.mqtt.client"]
        modules["paho.mqtt.client"].Client = Client
        modules["paho.mqtt.client"].CallbackAPIVersion = types.SimpleNamespace(VERSION2="v2")
        return modules

    def test_connect_subscribe_publish_disconnect(self):
        calls = []
        model = model_module.DashboardModel(token_configured=True)
        link = dashboard.PahoLink("broker.test", 1883, model)
        with mock.patch.dict(sys.modules, self.fake_paho(calls)):
            link.start()
        self.assertEqual(calls[0], ("api", "v2"))
        self.assertIn(("connect_async", "broker.test", 1883, 60), calls)
        with self.assertRaises(ConnectionError):
            link.publish(b"x", 1)
        link._on_connect(link.client, None, None, types.SimpleNamespace(is_failure=False), None)
        self.assertTrue(model.broker_connected)
        subscribe = [call for call in calls if call[0] == "subscribe"][-1][1]
        self.assertEqual(dict(subscribe), {"robot/state": 0, "robot/sonar": 0, "robot/event": 0,
                                           "robot/heartbeat": 0, "robot/log": 1,
                                           "robot/obstacle": 0, "robot/status": 1,
                                           "robot/barcode": 1, "robot/imu": 0})
        self.assertTrue(link.publish(b"payload", 3))
        self.assertIn(("publish", "robot/cmd", b"payload", 1, False), calls)
        link._on_message(None, None, types.SimpleNamespace(topic="robot/state",
                                                           payload=SAMPLES["state_idle"],
                                                           retain=False))
        self.assertIsNotNone(model.state)
        link._on_message(None, None, types.SimpleNamespace(topic="robot/status", payload=OFFLINE,
                                                           retain=True))
        self.assertEqual(model.status, {"online": False, "retained": True,
                                        "rx": model.status["rx"]})
        link._on_disconnect(None, None, None, None, None)
        self.assertFalse(model.broker_connected)
        link._on_connect(link.client, None, None, types.SimpleNamespace(is_failure=True), None)
        self.assertFalse(model.broker_connected)
        link.stop()
        self.assertEqual(calls[-2:], [("disconnect",), ("loop_stop",)])


class CliTests(unittest.TestCase):
    def run_main(self, args):
        started = []

        class Link:
            def __init__(self, host, port, model):
                started.append((host, port, model.token_configured))
                self.connected = False

            def start(self):
                pass

            def stop(self):
                pass

        with mock.patch.object(dashboard, "PahoLink", Link), \
                mock.patch.object(dashboard.DashboardServer, "serve_forever",
                                  side_effect=KeyboardInterrupt), \
                contextlib.redirect_stderr(io.StringIO()) as error:
            code = dashboard.main(args)
        return code, started, error.getvalue()

    def test_token_file_is_read_and_never_printed(self):
        with tempfile.TemporaryDirectory() as folder:
            token = Path(folder) / "token"
            token.write_text(TOKEN.hex() + "\n")
            token.chmod(0o600)
            code, started, error = self.run_main(["--http-port", "0", "--token-file", str(token)])
            self.assertEqual((code, started), (0, [("127.0.0.1", 1883, True)]))
            self.assertNotIn(TOKEN.hex(), error)
            token.chmod(0o644)
            with contextlib.redirect_stderr(io.StringIO()) as error, \
                    self.assertRaises(SystemExit) as raised:
                dashboard.main(["--http-port", "0", "--token-file", str(token)])
            self.assertEqual(raised.exception.code, 2)
            self.assertNotIn(TOKEN.hex(), error.getvalue())

    def test_without_token_file_only_estop(self):
        code, started, error = self.run_main(["--http-port", "0", "--host", "192.0.2.10"])
        self.assertEqual((code, started), (0, [("192.0.2.10", 1883, False)]))
        self.assertIn("http://127.0.0.1:0/", error)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
