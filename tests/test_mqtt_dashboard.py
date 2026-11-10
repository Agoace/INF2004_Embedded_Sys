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
import dashboard_page  # noqa: E402
import dashboard_replay  # noqa: E402
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
          "status": "robot/status", "detour": "robot/detour", "summary": "robot/summary"}


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


def state_v3(name="state_idle", duty=(-250, 1000), heading=-4500, rate=-123,
             dist=(1500, 1700, 1600)):
    """Version 3 state: the C-packed first 48 bytes, distances, duty, heading and turn rate."""
    return (b"\x03" + SAMPLES[name][1:48] + struct.pack("<iii", *dist) +
            struct.pack("<hhihH", duty[0], duty[1], heading, rate, 0))


def heartbeat_v(version=2, cpu=123, name="heartbeat"):
    frame = bytearray(SAMPLES[name][:100])
    frame[0] = version
    struct.pack_into("<H", frame, 98, cpu if version == 2 else 0)
    return bytes(frame)


def detour(outcome=3, side=1, flags=1, sequence=4, start=1000, end=3500, duration=2500,
           distance=1200, lateral=340, payload=0, scan=9):
    return struct.pack("<BBBBIIIIIHHI", 1, outcome, side, flags, sequence, start, end, duration,
                       distance, lateral, payload, scan)


def summary(flags=0b010, source=1, reason=1, run=2, start=5000, duration=60000, distance=8000,
            losses=3, obstacles=2, done=2, succeeded=1, detour_ms=7000, started=3,
            speed_error=45, motion_faults=0x05, shell=2, shell_detail=6, barcode_ok=0,
            barcode_failed=0, hump=0xFFFF):
    return struct.pack("<BBBBIIIIHHHHIHHBBHBBH", 1, flags, source, reason, run, start, duration,
                       distance, losses, obstacles, done, succeeded, detour_ms, started,
                       speed_error, motion_faults, shell, shell_detail, barcode_ok,
                       barcode_failed, hump)


def summary_v3(hump=108, count=1, hump_flags=1, gated=0, boot=0, **kwargs):
    """Summary v3: the v1 layout, boot ID at 44 and the hump tail at 48..51."""
    old = summary(hump=hump, **kwargs)
    return b"\x03" + old[1:] + struct.pack("<IHBB", boot, count, hump_flags, gated)


def barcode(flags=0b00111, code=5, votes=3, reads=4, direction=1, nav=9, result=-45,
            sequence=12, uptime=777):
    return struct.pack("<BBBBBBBBhHII", 1, 1, flags, code, votes, reads, direction, nav, result,
                       0, sequence, uptime)


def imu_snapshot(pitch=3, heading=200, rate=250, hump=2, collision=1, valid=1, height=7,
                 peak=9, age=0xBEEF):
    word1 = pitch | heading << 8 | rate << 16 | hump << 24 | collision << 26 | valid << 27
    return struct.pack("<BBHII", 1, 1, 0, word1, height | peak << 8 | age << 16)


def imu_event(event=6, sequence=77, uptime=4242):
    return struct.pack("<BBBBII", 1, 2, event, 0, sequence, uptime)


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
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 1, 0))

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
        self.assertEqual(ids, ["vehicle", "line", "wheels", "odometry", "ultrasonic",
                               "obstacle", "events", "health", "barcode", "imu", "summary",
                               "hump", "log"])
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
        self.assertEqual(list(self.rows("imu")),
                         ["Pitch", "Heading", "Turn rate", "Hump state", "Height", "Peak",
                          "Collision", "Motion event"])
        self.assertTrue(all(row["value"] == "not implemented yet" for row in panel["rows"]))

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
                         "[boot:00000000:0] dump 5: missing rows [10-11] events []")


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
        self.assertEqual((panel["age_ms"], panel["note"]),
                         (None, "no completed scan or detour yet"))

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
        self.feed("state_boot_a")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "unknown")
        self.feed("status_online")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "online")
        self.feed("status_offline")
        self.assertEqual(self.model.snapshot()["robot"]["level"], "offline")
        self.assertFalse(self.button("start")["enabled"])
        self.assertTrue(self.button("estop")["enabled"])

    def test_c_packed_state_is_v2_with_distance_fields(self):
        self.assertIn((len(SAMPLES["state_idle"]), SAMPLES["state_idle"][0]),
                      ((60, 2), (72, 3), (76, 4), (76, 5)))
        self.feed("state_idle")
        self.assertEqual(self.rows("wheels")["Distance left / right / average"], "0 / 0 / 0 mm")
        self.assertTrue(self.button("start")["enabled"])


class ReservedTopicTests(ModelCase):
    def test_not_implemented_until_data_arrives(self):
        self.assertEqual(self.panel("imu")["note"], "not implemented: no data on robot/imu yet")
        self.assertTrue(self.panel("imu")["placeholder"])
        for label in ("Decoded command", "Confidence", "Direction", "Navigation command",
                      "Result", "Detected / decoded / unreadable", "Command issued / executed"):
            self.assertEqual(self.rows("barcode")[label], "not implemented yet")
        self.feed("state_idle")
        self.assertEqual(self.rows("barcode")["Raw GP28 sample"], 777)
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")

    def test_barcode_layout_once_data_arrives(self):
        self.model.ingest("robot/barcode", barcode())
        rows = self.rows("barcode")
        self.assertEqual(rows["Detected / decoded / unreadable"], "yes / yes / yes")
        self.assertEqual(rows["Decoded command"], "code 5 (raw)")
        self.assertEqual(rows["Confidence"], "3 of 4 reads agree")
        self.assertEqual((rows["Direction"], rows["Navigation command"], rows["Result"]),
                         ("forward", "9 (raw)", "-45 (raw)"))
        self.assertEqual(rows["Command issued / executed"], "no / no")
        self.assertEqual(rows["Report"], "#12 at robot uptime 777 ms")
        self.model.ingest("robot/barcode", barcode(flags=0b11011, direction=2))
        rows = self.rows("barcode")
        self.assertEqual((rows["Command issued / executed"], rows["Direction"]),
                         ("yes / yes", "reverse"))

    def test_barcode_age_comes_from_its_own_topic(self):
        self.model.ingest("robot/barcode", barcode())
        self.assertEqual(self.panel("barcode")["age_ms"], 0)
        self.assertFalse(self.panel("barcode")["stale"])
        self.clock.advance(1500)
        self.feed("state_idle")
        self.assertTrue(self.panel("barcode")["stale"])

    def test_imu_layout_once_data_arrives(self):
        self.model.ingest("robot/imu", imu_snapshot())
        panel = self.panel("imu")
        rows = self.rows("imu")
        self.assertFalse(panel["placeholder"])
        self.assertEqual((rows["Pitch (raw)"], rows["Heading (raw)"], rows["Turn rate (raw)"]),
                         (3, 200, 250))
        self.assertEqual((rows["Hump state (raw)"], rows["Height (raw)"], rows["Peak (raw)"]),
                         (2, 7, 9))
        self.assertEqual((rows["Collision"], rows["Sample valid"]), ("yes", "yes"))
        self.assertEqual(rows["Sample age or sequence (raw)"], 0xBEEF)
        self.assertEqual(rows["Motion event"], "not implemented yet")
        self.assertIn("units are not fixed", panel["note"])
        self.assertFalse(panel["stale"])
        self.clock.advance(1500)
        self.assertTrue(self.panel("imu")["stale"])

    def test_imu_motion_events_alone_and_with_a_snapshot(self):
        self.model.ingest("robot/imu", imu_event(event=4, sequence=5, uptime=99))
        rows = self.rows("imu")
        self.assertEqual(rows["Motion event"], "climbing #5 at robot uptime 99 ms")
        self.assertEqual(rows["Pitch"], "not implemented yet")
        self.assertFalse(self.panel("imu")["placeholder"])
        self.model.ingest("robot/imu", imu_snapshot())
        self.model.ingest("robot/imu", imu_event(event=6, sequence=6, uptime=120))
        rows = self.rows("imu")
        self.assertEqual(rows["Motion event"], "impact #6 at robot uptime 120 ms")
        self.assertEqual(rows["Pitch (raw)"], 3)  # the snapshot is kept

    def test_unknown_versions_and_kinds_are_ignored_not_counted_as_errors(self):
        self.model.ingest("robot/barcode", bytes([2, 1, 5]))
        self.model.ingest("robot/imu", bytes([9, 1, 5]))
        self.model.ingest("robot/imu", bytes([1, 9, 5, 5]))
        self.assertEqual((self.model.ignored_reserved, self.model.snapshot()["discarded"]), (3, 0))
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")
        self.assertTrue(self.panel("imu")["placeholder"])
        self.model.ingest("robot/imu", b"\x01")  # too short to read a header
        self.model.ingest("robot/barcode", barcode()[:-1])  # known frame, wrong size
        self.assertEqual(self.model.snapshot()["discarded"], 2)
        self.assertEqual(self.rows("barcode")["Decoded command"], "not implemented yet")


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
        self.fresh_idle()  # a session boundary invalidates the previous idle state
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


class DutyAndOdometryTests(ModelCase):
    def test_wheels_panel_shows_duty_from_state_v3(self):
        self.model.ingest("robot/state", state_v3(duty=(-250, 1000)))
        self.assertEqual(self.rows("wheels")["PWM duty left / right"], "-25.0 % / +100.0 %")
        self.model.ingest("robot/state", state_v3(duty=(5, -1)))
        self.assertEqual(self.rows("wheels")["PWM duty left / right"], "+0.5 % / -0.1 %")
        self.assertEqual(self.rows("wheels")["Distance left / right / average"],
                         "1500 / 1700 / 1600 mm")

    def test_older_states_say_that_duty_is_not_reported(self):
        self.model.ingest("robot/state", state_v2())
        self.assertEqual(self.rows("wheels")["PWM duty left / right"],
                         "not reported (old state version)")

    def test_odometry_before_data(self):
        panel = self.panel("odometry")
        self.assertEqual((panel["age_ms"], panel["note"]), (None, "waiting for robot/state"))

    def test_odometry_heading_dial_turn_rate_and_distance(self):
        self.model.ingest("robot/state", state_v3(heading=-4500, rate=-123, dist=(10, 30, 20)))
        panel = self.panel("odometry")
        rows = self.rows("odometry")
        self.assertEqual(rows["Heading (left +, since boot)"], "-45.00 deg")
        self.assertEqual(rows["Heading on the dial"], "315.00 deg")
        self.assertEqual(rows["Turn rate (left +)"], "-12.3 deg/s")
        self.assertEqual((rows["Distance average"], rows["Distance left / right"]),
                         ("20 mm", "10 / 30 mm"))
        self.assertIn("encoder-derived", rows["Source"])
        self.assertIn("encoder-derived", panel["title"])
        self.assertEqual(panel["chart"], {"type": "dial", "heading_deg": 315.0,
                                          "turn_rate_dps": -12.3})
        self.assertEqual(panel["age_ms"], 8)  # the motion board age in the C sample
        self.assertEqual((rows["IMU heading (raw)"], rows["IMU turn rate (raw)"]),
                         ("not implemented yet", "not implemented yet"))

    def test_heading_wraps_both_ways(self):
        cases = {0: 0.0, 36050: 0.5, -1: 359.99, 36000: 0.0, -36000: 0.0, 2 ** 31 - 1: 116.47,
                 -2 ** 31: 243.52}
        for cdeg, dial in cases.items():
            self.model.ingest("robot/state", state_v3(heading=cdeg))
            self.assertAlmostEqual(self.panel("odometry")["chart"]["heading_deg"], dial,
                                   places=2, msg=str(cdeg))
            self.assertTrue(0 <= self.panel("odometry")["chart"]["heading_deg"] < 360)

    def test_imu_values_appear_beside_the_encoder_values_for_comparison(self):
        self.model.ingest("robot/state", state_v3())
        self.model.ingest("robot/imu", imu_snapshot(heading=77, rate=88))
        rows = self.rows("odometry")
        self.assertEqual((rows["IMU heading (raw)"], rows["IMU turn rate (raw)"]), (77, 88))
        self.assertEqual(rows["Heading (left +, since boot)"], "-45.00 deg")

    def test_old_state_has_no_heading(self):
        self.model.ingest("robot/state", state_v2())
        panel = self.panel("odometry")
        rows = self.rows("odometry")
        self.assertIsNone(panel["chart"])
        self.assertEqual(rows["Heading and turn rate"], "not reported (old state version)")
        self.assertEqual(rows["Distance average"], "1600 mm")
        self.model.ingest("robot/state", b"\x01" + SAMPLES["state_idle"][1:48])
        self.assertEqual(self.rows("odometry")["Distance"], "not reported (old state version)")

    def test_odometry_goes_stale_with_the_state(self):
        self.model.ingest("robot/state", state_v3())
        self.assertFalse(self.panel("odometry")["stale"])
        self.clock.advance(1100)
        self.assertTrue(self.panel("odometry")["stale"])


class DetourTests(ModelCase):
    def rows_of_table(self):
        return self.panel("obstacle")["table"]["rows"]

    def test_table_columns_and_one_row_per_detour(self):
        self.model.ingest("robot/detour", detour())
        table = self.panel("obstacle")["table"]
        self.assertEqual(table["columns"], ["Detour", "Side", "Outcome", "Duration", "Distance",
                                            "Max offset", "Scan"])
        self.assertEqual(table["rows"], [[4, "right", "done, line found", "2.50 s", "1200 mm",
                                          "340 mm", 9]])
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 0)

    def test_every_outcome_and_payload_meaning(self):
        cases = ((3, 0, 0, "done, line found"),
                 (4, 1, 1, "done, no line (search left)"),
                 (4, 2, 2, "done, no line (search right)"),
                 (4, 0, 0, "done, no line (search unknown)"),
                 (7, 2, 1, "FAILED: both sides blocked"), (7, 1, 2, "FAILED: turn limit reached"),
                 (7, 1, 3, "FAILED: took too long"), (7, 1, 4, "FAILED: ultrasonic fault"),
                 (7, 1, 5, "FAILED: echo too near"), (7, 1, 6, "FAILED: no line found"),
                 (7, 1, 0, "FAILED: none"), (7, 1, 99, "FAILED: unknown (99)"),
                 (8, 1, 0, "aborted (run ended)"))
        for sequence, (outcome, side, payload, text) in enumerate(cases, start=1):
            self.model.ingest("robot/detour", detour(outcome=outcome, side=side, payload=payload,
                                                     sequence=sequence))
            self.assertEqual(self.rows_of_table()[0][2], text)
        self.assertEqual(self.rows_of_table()[0][1], "right")

    def test_side_texts(self):
        for side, text in ((0, "none"), (1, "right"), (2, "left")):
            self.model.ingest("robot/detour", detour(side=side, sequence=side + 1))
            self.assertEqual(self.rows_of_table()[0][1], text)

    def test_offset_not_measured_and_unknown_scan(self):
        self.model.ingest("robot/detour", detour(flags=0, lateral=0, scan=0))
        row = self.rows_of_table()[0]
        self.assertEqual((row[5], row[6]), ("not measured", "unknown"))
        self.model.ingest("robot/detour", detour(flags=0, lateral=500, sequence=5))
        # A value sent while the valid flag is off must not show.
        self.assertEqual(self.rows_of_table()[0][5], "not measured")

    def test_newest_first_and_history_limit(self):
        for sequence in range(1, 13):
            self.model.ingest("robot/detour", detour(sequence=sequence))
        rows = self.rows_of_table()
        self.assertEqual([row[0] for row in rows], list(range(12, 4, -1)))
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 0)

    def test_sequence_gaps_are_counted_and_a_restart_is_not_a_gap(self):
        for sequence in (1, 2, 5, 9):
            self.model.ingest("robot/detour", detour(sequence=sequence))
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 2 + 3)
        self.model.ingest("robot/detour", detour(sequence=1))
        self.model.ingest("robot/detour", detour(sequence=2))
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 5)
        self.model.ingest("robot/detour", detour(sequence=2))  # a repeated sequence
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 5)
        self.model.ingest("robot/detour", detour(sequence=3))
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 5)

    def test_panel_works_with_detours_but_without_a_scan_and_with_both(self):
        self.model.ingest("robot/detour", detour())
        panel = self.panel("obstacle")
        self.assertIsNone(panel["chart"])
        self.assertEqual(panel["age_ms"], 0)
        self.assertNotIn("Closest", self.rows("obstacle"))
        self.model.ingest("robot/obstacle", obstacle(flags=0b0001 | 2 << 4))
        panel = self.panel("obstacle")
        self.assertEqual(panel["chart"]["type"], "obstacle")
        self.assertEqual(self.rows("obstacle")["Scan stage"], "fine")
        self.assertEqual(len(panel["table"]["rows"]), 1)

    def test_obstacle_age_uses_the_newest_scan_or_detour(self):
        self.model.ingest("robot/detour", detour())
        self.clock.advance(2000)
        self.model.ingest("robot/obstacle", obstacle())
        self.assertEqual(self.panel("obstacle")["age_ms"], 0)
        self.clock.advance(1000)
        self.model.ingest("robot/detour", detour(sequence=5))
        self.assertEqual(self.panel("obstacle")["age_ms"], 0)

    def test_scan_stage_texts(self):
        for stage, text in ((0, "not reported"), (1, "coarse"), (2, "fine")):
            self.model.ingest("robot/obstacle", obstacle(flags=0b0001 | stage << 4, scan=stage))
            self.assertEqual(self.rows("obstacle")["Scan stage"], text)

    def test_no_table_before_a_detour_and_the_panel_never_goes_grey(self):
        self.model.ingest("robot/obstacle", obstacle())
        self.assertIsNone(self.panel("obstacle")["table"])
        self.model.ingest("robot/detour", detour())
        self.clock.advance(60000)
        panel = self.panel("obstacle")
        self.assertFalse(panel["stale"])
        self.assertEqual(panel["age_ms"], 60000)

    def test_bad_detour_is_discarded(self):
        self.model.ingest("robot/detour", detour(outcome=5))
        self.model.ingest("robot/detour", detour()[:-1])
        self.assertEqual(self.model.snapshot()["discarded"], 2)
        self.assertIsNone(self.panel("obstacle")["table"])


class SummaryTests(ModelCase):
    def test_before_any_run(self):
        panel = self.panel("summary")
        self.assertEqual((panel["age_ms"], panel["rows"]), (None, []))
        self.assertIn("appears when a run ends", panel["note"])

    def test_rows_of_a_finished_run(self):
        self.model.ingest("robot/summary", summary(), retained=False)
        rows = self.rows("summary")
        self.assertEqual((rows["Run ID"], rows["Stop source"], rows["Stop reason"]),
                         (2, "GP21 button", "GP21 button"))
        self.assertEqual((rows["Started (robot uptime)"], rows["Duration"]),
                         ("5.00 s", "60.00 s"))
        self.assertEqual(rows["Distance (path length)"], "8000 mm")
        self.assertEqual((rows["Line losses"], rows["Obstacles seen"]), (3, 2))
        self.assertEqual(rows["Detours started / done / succeeded"], "3 / 2 / 1")
        self.assertEqual(rows["Detour time in total"], "7.00 s")
        self.assertEqual(rows["Max speed error"], "45 mm/s")
        self.assertEqual(rows["Motion faults seen"], "stall left, ESTOP")
        self.assertEqual(rows["Shell fault"], "module failed (detail 6)")
        self.assertEqual(rows["Barcode ok / failed"], "not implemented yet")
        self.assertNotIn("Hump peak", rows)  # the hump card shows it
        self.assertNotIn("Detour", rows)
        self.assertNotIn("retained", self.panel("summary")["note"])

    def test_stop_texts(self):
        cases = ((0, 0, "none or unknown", "unknown"), (2, 2, "MQTT ESTOP", "MQTT ESTOP command"),
                 (3, 5, "fault", "vehicle deadline missed"), (4, 3, "time limit", "90 s limit"),
                 (3, 12, "fault", "other"), (3, 4, "fault", "shell left RUNNING"))
        for run, (source, reason, source_text, reason_text) in enumerate(cases, start=1):
            self.model.ingest("robot/summary", summary(source=source, reason=reason, run=run))
            rows = self.rows("summary")
            self.assertEqual((rows["Stop source"], rows["Stop reason"]),
                             (source_text, reason_text))
            self.assertEqual(rows["Run ID"], run)

    def test_all_stop_reasons_have_a_name(self):
        for reason in range(13):
            self.model.ingest("robot/summary", summary(reason=reason))
            self.assertNotIn("unknown (", self.rows("summary")["Stop reason"])

    def test_speed_error_not_measured(self):
        self.model.ingest("robot/summary", summary(flags=0, speed_error=45))  # flag off
        self.assertEqual(self.rows("summary")["Max speed error"], "not measured")
        self.model.ingest("robot/summary", summary(flags=0b010, speed_error=0xFFFF))
        self.assertEqual(self.rows("summary")["Max speed error"], "not measured")
        self.model.ingest("robot/summary", summary(flags=0b010, speed_error=0))
        self.assertEqual(self.rows("summary")["Max speed error"], "0 mm/s")

    def test_shell_and_motion_faults(self):
        self.model.ingest("robot/summary", summary(shell=0, shell_detail=0, motion_faults=0))
        rows = self.rows("summary")
        self.assertEqual((rows["Shell fault"], rows["Motion faults seen"]), ("none", "none"))
        self.model.ingest("robot/summary", summary(shell=1, shell_detail=3, motion_faults=0x0F))
        rows = self.rows("summary")
        self.assertEqual(rows["Shell fault"], "safety (motor fault bits) (detail 3)")
        self.assertEqual(rows["Motion faults seen"], "stall left, stall right, ESTOP, "
                                                     "stale command")
        self.model.ingest("robot/summary", summary(motion_faults=0x30))
        self.assertEqual(self.rows("summary")["Motion faults seen"], "unknown bits 0x30")

    def test_aborted_detour_row(self):
        self.model.ingest("robot/summary", summary(flags=0b110))
        self.assertIn("Detour", self.rows("summary"))
        self.assertIn("still running", self.rows("summary")["Detour"])

    def test_reserved_barcode_fields_when_implemented(self):
        self.model.ingest("robot/summary", summary(flags=0b011, barcode_ok=3, barcode_failed=1,
                                                   hump=12))
        self.assertEqual(self.rows("summary")["Barcode ok / failed"], "3 / 1")
        # An old v1/v2 hump value has no fixed unit: the card does not show it.
        self.assertEqual(self.rows("hump")["Highest hump"], "not reported (summary v1 firmware)")
        # Values sent while the flag is off must not show as results.
        self.model.ingest("robot/summary", summary(flags=0b010, barcode_ok=3, hump=12))
        self.assertEqual(self.rows("summary")["Barcode ok / failed"], "not implemented yet")
        # v3 bit 0 is barcode only; the hump has its own implemented flag.
        self.model.ingest("robot/summary", summary_v3(flags=0b010, hump=108))
        self.assertEqual(self.rows("summary")["Barcode ok / failed"], "not implemented yet")
        self.assertEqual(self.rows("hump")["Highest hump"], "10.8 mm")


class HumpTests(ModelCase):
    def test_before_any_run(self):
        panel = self.panel("hump")
        self.assertEqual((panel["age_ms"], panel["rows"], panel["stale"]), (None, [], True))
        self.assertIn("appears when a run ends", panel["note"])

    def test_c_packed_summary_and_events(self):
        self.model.ingest("robot/summary", SAMPLES["summary_hump"])
        rows = self.rows("hump")
        self.assertEqual((rows["Highest hump"], rows["Humps counted"], rows["Side of highest"]),
                         ("10.8 mm", 2, "right wheel"))
        self.assertEqual(rows["Hump starts blocked (curve or slow)"], 1)
        note = self.panel("hump")["note"]
        self.assertIn("stopped on a hump", note)
        self.assertNotIn("IMU data was missing", note)
        self.assertFalse(self.panel("hump")["stale"])
        self.model.ingest("robot/summary", SAMPLES["summary_hump_unavailable"])
        self.assertEqual(self.rows("hump")["Highest hump"], "unavailable (no fresh IMU data)")
        self.assertIn("IMU data was missing", self.panel("hump")["note"])
        self.model.ingest("robot/event", SAMPLES["event_hump"])
        self.model.ingest("robot/event", SAMPLES["event_hump_left"])
        texts = [item["text"] for item in self.panel("events")["items"]]
        self.assertIn("#21 HUMP #3: 10.8 mm, right wheel", texts)
        self.assertIn("#22 HUMP #4: 25.0 mm, left wheel (not proven)", texts)

    def test_flags_side_and_no_hump(self):
        self.model.ingest("robot/summary", summary_v3(hump=0, count=0, hump_flags=0b10001))
        rows = self.rows("hump")
        self.assertEqual((rows["Highest hump"], rows["Humps counted"]), ("no hump detected", 0))
        self.assertNotIn("Side of highest", rows)
        self.assertIn("longer than 600 mm", self.panel("hump")["note"])
        self.model.ingest("robot/summary", summary_v3(hump=250, count=3, hump_flags=0b101001))
        rows = self.rows("hump")
        self.assertEqual((rows["Highest hump"], rows["Side of highest"]),
                         ("25.0 mm", "left wheel (not proven)"))
        self.assertIn("curve froze", self.panel("hump")["note"])
        self.model.ingest("robot/summary", summary_v3(hump_flags=0))
        self.assertEqual(self.rows("hump")["Highest hump"], "not implemented yet")
        self.model.ingest("robot/summary", summary_v3(hump_flags=0x40))  # reserved bit
        self.assertEqual(self.model.snapshot()["discarded"], 1)

    def test_live_events_during_a_run_then_the_summary(self):
        self.model.ingest("robot/summary", summary_v3(hump=90, count=1))
        self.feed("state_running")
        self.clock.advance(10)
        self.model.ingest("robot/event", SAMPLES["event_hump"])
        rows = self.rows("hump")
        self.assertEqual(rows["This run so far (live)"], "10.8 mm, right wheel, 3 counted")
        self.assertEqual(rows["Highest hump"], "9.0 mm")  # previous run until this one ends
        self.clock.advance(10)
        self.model.ingest("robot/summary", summary_v3(hump=108, count=3))
        rows = self.rows("hump")
        self.assertNotIn("This run so far (live)", rows)
        self.assertEqual(rows["Highest hump"], "10.8 mm")
        self.feed("state_idle")
        self.feed("state_running")  # a new run clears the live value
        self.assertNotIn("This run so far (live)", self.rows("hump"))

    def test_retained_copy_is_labelled(self):
        self.model.ingest("robot/summary", summary(), retained=True)
        self.assertIn("retained copy", self.panel("summary")["note"])

    def test_new_run_in_progress_labels_the_old_summary(self):
        self.model.ingest("robot/summary", summary())
        self.feed("state_running")
        self.assertIn("previous run", self.panel("summary")["note"])
        self.feed("state_idle")
        self.assertNotIn("previous run", self.panel("summary")["note"])
        self.feed("state_running")
        self.clock.advance(2000)  # state is stale: do not claim a run is in progress
        self.assertNotIn("previous run", self.panel("summary")["note"])

    def test_never_stale_and_survives_a_broker_drop(self):
        self.model.ingest("robot/summary", summary())
        self.clock.advance(600000)
        panel = self.panel("summary")
        self.assertFalse(panel["stale"])
        self.assertEqual(panel["age_ms"], 65535)  # age is capped like every other age
        self.model.set_broker_connected(False)
        self.assertEqual(self.rows("summary")["Run ID"], 2)

    def test_newest_summary_replaces_the_old_one_and_bad_ones_are_discarded(self):
        self.model.ingest("robot/summary", summary(run=1))
        self.model.ingest("robot/summary", summary(run=2))
        self.assertEqual(self.rows("summary")["Run ID"], 2)
        self.model.ingest("robot/summary", summary(run=3)[:-1])
        self.model.ingest("robot/summary", summary(source=9, run=4))
        self.assertEqual((self.model.snapshot()["discarded"], self.rows("summary")["Run ID"]),
                         (2, 2))


class CpuAndJunctionTests(ModelCase):
    def cpu_text(self):
        return self.rows("health")["CPU load"]

    def test_cpu_load_texts(self):
        self.model.ingest("robot/heartbeat", heartbeat_v(2, 123))
        self.assertEqual(self.cpu_text(), "12.3 %")
        self.model.ingest("robot/heartbeat", heartbeat_v(2, 0))
        self.assertEqual(self.cpu_text(), "0.0 %")
        self.model.ingest("robot/heartbeat", heartbeat_v(2, 1000))
        self.assertEqual(self.cpu_text(), "100.0 %")
        self.model.ingest("robot/heartbeat", heartbeat_v(2, 0xFFFF))
        self.assertEqual(self.cpu_text(), "not available")

    def test_old_heartbeat_has_no_cpu(self):
        self.model.ingest("robot/heartbeat", heartbeat_v(1))
        self.assertEqual(self.cpu_text(), "not reported (old heartbeat)")
        self.assertEqual(self.rows("health")["Wi-Fi"], "up")  # the other rows still work

    def test_bad_cpu_value_discards_the_heartbeat(self):
        self.model.ingest("robot/heartbeat", heartbeat_v(2, 1001))
        self.assertEqual(self.model.snapshot()["discarded"], 1)
        self.assertIsNone(self.model.heartbeat)

    def test_legacy_junction_event_is_listed_with_its_reserved_name(self):
        frame = bytearray(b"\x01" + SAMPLES["event_run_end"][1:16])
        frame[1] = 7
        self.model.ingest("robot/event", bytes(frame))
        self.assertEqual(self.model.snapshot()["discarded"], 0)
        items = self.panel("events")["items"]
        self.assertEqual(len(items), 1)
        self.assertIn("junction (reserved)", items[0]["text"])
        frame[1] = 8
        self.model.ingest("robot/event", bytes(frame))
        self.assertEqual(self.model.snapshot()["discarded"], 1)
        self.assertEqual(len(self.panel("events")["items"]), 1)

    def test_junction_event_does_not_touch_command_decisions(self):
        frame = bytearray(SAMPLES["event_junction"])
        frame[1] = 7
        self.model.ingest("robot/event", bytes(frame))
        self.assertIsNone(self.model.last_decision)


class BatchTwoCFixtureTests(ModelCase):
    def test_c_state_v3_duty_and_odometry(self):
        self.feed("state_v3")
        rows = self.rows("wheels")
        self.assertEqual(rows["PWM duty left / right"], "-80.0 % / +3276.7 %")
        rows = self.rows("odometry")
        self.assertEqual(rows["Heading (left +, since boot)"], "-12345.67 deg")
        self.assertEqual(rows["Turn rate (left +)"], "-474.9 deg/s")
        self.assertEqual(rows["Distance average"], "-7 mm")
        self.assertAlmostEqual(self.panel("odometry")["chart"]["heading_deg"], 254.33)

    def test_c_heartbeat_cpu(self):
        self.feed("heartbeat_v2")
        self.assertEqual(self.rows("health")["CPU load"], "29.1 %")
        self.feed("heartbeat_v2_unknown")
        self.assertEqual(self.rows("health")["CPU load"], "not available")

    def test_c_detour_outcomes_and_opposite_side_enums(self):
        cases = (("done_line", "done, line found", "4660 mm"),
                 ("done_no_line", "done, no line (search left)", "4660 mm"),
                 ("failed", "FAILED: took too long", "not measured"),
                 ("aborted", "aborted (run ended)", "4660 mm"))
        for suffix, outcome, offset in cases:
            self.feed("detour_" + suffix)
            row = self.panel("obstacle")["table"]["rows"][0]
            self.assertEqual(row, [0x01020304, "left", outcome, "0.03 s",
                                   f"{0xA1B2C3D4} mm", offset, 0x0A0B0C0D])

    def test_c_run_summaries(self):
        self.model.ingest("robot/summary", SAMPLES["summary_normal"], retained=True)
        rows = self.rows("summary")
        self.assertEqual((rows["Run ID"], rows["Duration"], rows["Distance (path length)"]),
                         (0x01020304, "61.23 s", "4321 mm"))
        self.assertEqual(rows["Max speed error"], "41 mm/s")
        self.assertEqual(rows["Motion faults seen"], "stall left, stale command")
        self.assertEqual(rows["Shell fault"], "no module (detail 258)")
        self.assertEqual(rows["Barcode ok / failed"], "not implemented yet")
        self.assertIn("retained copy", self.panel("summary")["note"])
        self.feed("summary_aborted_detour")
        self.assertEqual(self.rows("summary")["Stop source"], "MQTT ESTOP")
        self.assertIn("still running", self.rows("summary")["Detour"])
        self.feed("summary_no_speed_error")
        rows = self.rows("summary")
        self.assertEqual((rows["Stop source"], rows["Stop reason"], rows["Max speed error"]),
                         ("time limit", "90 s limit", "not measured"))

    def test_c_junction_event(self):
        self.feed("event_junction")
        self.assertIn("#20 JUNCTION B: bar clear", self.panel("events")["items"][0]["text"])
        self.assertEqual(self.model.discarded, 0)


class JunctionActionDashboardTests(ModelCase):
    def test_line_angle_labels(self):
        for name in ("first", "accepted"):
            self.feed(f"event_junction_{name}")
            self.assertIn(f"JUNCTION B: {name} line at -33.00 deg",
                          self.panel("events")["items"][0]["text"])
        self.assertEqual(self.model.discarded, 0)

    def test_all_commands_and_terminal_reason(self):
        for number, command in enumerate("ABCD", 1):
            frame = bytearray(SAMPLES["event_junction"])
            frame[12:15] = bytes((number, 8, 6))
            self.model.ingest("robot/event", bytes(frame))
            self.assertIn(f"JUNCTION {command}: failed, reason 6",
                          self.panel("events")["items"][0]["text"])
        self.assertEqual(self.model.discarded, 0)
        self.assertIsNone(self.model.last_decision)


class ScanRetryDashboardTests(ModelCase):
    def test_scan_retry_labels_and_summary_count(self):
        self.model.ingest("robot/event", SAMPLES["event_scan_reverse_begin"])
        self.model.ingest("robot/event", SAMPLES["event_scan_reverse_end"])
        self.model.ingest("robot/event", SAMPLES["event_scan_rescan_begin"])
        texts = [item["text"] for item in self.panel("events")["items"]]
        self.assertEqual(texts[0], "#32 SCAN retry 1: rescan begins (both sides blocked)")
        self.assertEqual(texts[-1], "#30 SCAN retry 1: reverse begins (both sides blocked)")
        self.model.ingest("robot/obstacle", SAMPLES["obstacle_retry2"])
        self.assertEqual(self.rows("obstacle")["Scan attempt"], "retry 2")
        self.model.ingest("robot/summary", SAMPLES["summary_scan_reverse_timeout"])
        rows = self.rows("summary")
        self.assertEqual(rows["SCAN reverse-and-rescan attempts"], 2)
        self.assertEqual(rows["Stop reason"], "scan reverse timed out")


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

    def test_allowed_proxy_origin_is_accepted_exactly(self):
        proxy = "https://robot.example.ts.net:8443"
        self.app.add_origin(proxy)
        self.app.set_port(self.port)
        body = json.dumps({"command": "estop"})
        headers = self.good_headers(Origin=proxy)
        status, _, _ = self.request("POST", "/api/command", body, headers,
                                    host="robot.example.ts.net:8443")
        self.assertEqual(status, 200)
        for origin, host in (("http://robot.example.ts.net:8443", "robot.example.ts.net:8443"),
                             ("https://robot.example.ts.net", "robot.example.ts.net"),
                             ("https://evil.ts.net:8443", "robot.example.ts.net:8443"),
                             (proxy, "evil.ts.net:8443")):
            status, _, _ = self.request("POST", "/api/command", body,
                                        self.good_headers(Origin=origin), host=host)
            self.assertEqual(status, 403, (origin, host))
        status, _, _ = self.request("POST", "/api/command", body,
                                    self.good_headers(Origin=proxy, **{"X-CSRF-Token": "guess"}),
                                    host="robot.example.ts.net:8443")
        self.assertEqual(status, 403)
        self.assertEqual(len(self.link.sent), 1)

    def test_bad_proxy_origin_is_refused(self):
        for origin in ("robot.ts.net", "ftp://robot.ts.net", "https://robot.ts.net/path",
                       "https://user@robot.ts.net", "https://robot.ts.net?x=1"):
            with self.assertRaises(ValueError, msg=origin):
                self.app.add_origin(origin)

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
        self.assertIn("dial(canvas, chart)", script)
        self.assertIn("renderGrid", script)
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
                                           "robot/barcode": 1, "robot/imu": 0,
                                           "robot/detour": 0, "robot/summary": 1})
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


def boot_frame(frame, boot, version=2):
    return bytes([version]) + frame[1:] + struct.pack("<I", boot)


def reboot_state(boot, uptime=1000, name="state_idle"):
    frame = bytearray(state_v3(name))
    struct.pack_into("<I", frame, 4, uptime)
    return boot_frame(frame, boot, 4)


def reboot_heartbeat(boot, uptime=1000, mask=8, unavailable=7):
    frame = bytearray(heartbeat_v(2))
    frame[0], frame[3] = 3, mask
    struct.pack_into("<I", frame, 4, uptime)
    return bytes(frame) + struct.pack("<II", boot, unavailable)


def reboot_batch(boot, **kwargs):
    frame = batch(**kwargs)
    return b"\x02" + frame[1:32] + struct.pack("<I", boot) + frame[32:]


class DashboardRebootTests(ModelCase):
    def online(self, boot, uptime=1000):
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 1, boot))
        self.model.ingest("robot/state", reboot_state(boot, uptime))

    def test_boot_change_invalidates_idle_state_and_old_state_cannot_reenable_start(self):
        self.online(100)
        self.assertTrue(self.button("start")["enabled"])
        self.model.ingest("robot/heartbeat", reboot_heartbeat(200))
        self.assertIsNone(self.model.state)
        self.assertFalse(self.button("start")["enabled"])
        self.model.ingest("robot/state", reboot_state(100))  # queued old idle
        self.assertIsNone(self.model.state)
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 1, 100), retained=True)
        self.assertIsNone(self.model.status)  # old retained status cannot override boot B
        self.online(200, uptime=2000)
        self.assertTrue(self.button("start")["enabled"])
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 200)

    def test_summary_is_labelled_previous_boot_and_cannot_replace_current_summary(self):
        self.online(100)
        self.model.ingest("robot/summary", boot_frame(summary(run=8), 100))
        self.online(200)
        self.assertIn("previous boot/session", self.panel("summary")["note"])
        self.assertEqual(self.rows("summary")["Run ID"], 8)
        self.model.ingest("robot/summary", boot_frame(summary(run=1), 200))
        self.model.ingest("robot/summary", boot_frame(summary(run=8), 100), retained=True)
        self.assertEqual(self.rows("summary")["Run ID"], 1)
        self.assertNotIn("previous boot/session", self.panel("summary")["note"])

    def test_retained_summary_does_not_select_live_boot(self):
        self.model.ingest("robot/summary", boot_frame(summary(), 99), retained=True)
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 0)
        self.online(200)
        self.assertIn("previous boot/session", self.panel("summary")["note"])
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 200)

    def test_detours_events_scan_and_pending_command_do_not_cross_boots(self):
        self.online(100)
        self.model.ingest("robot/detour", boot_frame(detour(sequence=10), 100))
        self.model.ingest("robot/obstacle", boot_frame(obstacle(scan=10), 100))
        self.model.ingest("robot/event", boot_frame(SAMPLES["event_start_ok"][:16], 100))
        self.model.note_command("start", 9, True)
        self.online(200)
        self.assertIsNone(self.model.pending)
        self.assertIsNone(self.model.last_decision)
        self.assertEqual(len(self.model.events), 0)
        self.assertEqual(len(self.model.detours), 0)
        self.assertIsNone(self.model.obstacle)
        self.model.ingest("robot/detour", boot_frame(detour(sequence=1), 200))
        self.model.ingest("robot/detour", boot_frame(detour(sequence=11), 100))
        self.assertEqual(len(self.model.detours), 1)
        self.assertEqual(self.rows("obstacle")["Detours missed (sequence gaps)"], 0)
        self.model.ingest("robot/obstacle", boot_frame(obstacle(scan=1), 200))
        self.assertEqual(self.rows("obstacle")["Scans missed (ID gaps)"], 0)

    def test_sonar_sequence_reuse_and_imu_placeholders_reset(self):
        self.online(100)
        self.model.ingest("robot/sonar", boot_frame(SAMPLES["sonar_valid"][:32], 100))
        self.model.ingest("robot/imu", imu_snapshot())
        self.assertTrue(self.model.sonar_profile)
        self.online(200)
        self.assertFalse(self.model.sonar_profile)
        self.assertTrue(self.panel("imu")["placeholder"])
        self.model.ingest("robot/sonar", boot_frame(SAMPLES["sonar_valid"][:32], 200))
        self.assertTrue(self.model.sonar_profile)
        self.model.ingest("robot/sonar", boot_frame(SAMPLES["sonar_error"][:32], 100))
        self.assertEqual(self.model.sonar["fields"]["boot_id"], 200)

    def test_two_boots_both_generation_two_rows_survive_in_dashboard_log(self):
        self.model.ingest("robot/log", reboot_batch(100, generation=2, count=2))
        self.model.ingest("robot/log", reboot_batch(200, generation=2, count=2))
        self.assertIn("4 rows", self.model.log.summary()[0])
        self.assertEqual(self.rows("log")["Live rows received"], 2)  # current boot only
        self.assertEqual(self.model.log.sessions.boot_id, 200)
        self.model.ingest("robot/log", reboot_batch(100, generation=2, count=2, first=2))
        self.assertEqual(self.rows("log")["Live rows received"], 2)
        self.assertEqual(self.model.log.sessions.boot_id, 200)

    def test_legacy_uptime_rollback_clears_live_panels_and_labels_inference(self):
        self.model = model_module.DashboardModel(clock=self.clock, token_configured=True)
        self.model.set_broker_connected(True)
        a = bytearray(state_v3())
        struct.pack_into("<I", a, 4, 5000)
        self.model.ingest("robot/state", a)
        self.model.ingest("robot/detour", detour())
        self.model.ingest("robot/summary", summary())
        struct.pack_into("<I", a, 4, 100)
        self.model.ingest("robot/state", a)
        self.assertEqual(len(self.model.detours), 0)
        self.assertIn("previous boot/session", self.panel("summary")["note"])
        self.assertTrue(self.model.snapshot()["boot"]["inferred"])
        self.assertIn("inferred legacy session", self.rows("vehicle")["Boot / session"])

    def test_broker_disconnect_alone_does_not_invent_boot(self):
        self.online(100)
        before = self.model.snapshot()["boot"]
        self.model.set_broker_connected(False)
        self.model.set_broker_connected(True)
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 1, 100), retained=True)
        self.assertEqual(self.model.snapshot()["boot"], before)

    def test_new_retained_status_before_live_boot_is_cached_not_applied_early(self):
        self.online(100)
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 1, 200), retained=True)
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 100)
        self.model.ingest("robot/state", reboot_state(200))
        self.assertEqual(self.model.snapshot()["robot"]["level"], "online")
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 200)

    def test_foreign_offline_does_not_block_current_boot(self):
        self.online(100)
        self.online(200)
        self.model.ingest("robot/status", struct.pack("<BBI", 2, 2, 100), retained=True)
        self.assertTrue(self.button("start")["enabled"])

    def test_c_boot_fixtures_keep_dashboard_state_and_logs_separate(self):
        self.model = model_module.DashboardModel(clock=self.clock, token_configured=True)
        self.model.set_broker_connected(True)
        self.model.ingest("robot/status", SAMPLES["status_boot_a"])
        self.model.ingest("robot/state", SAMPLES["state_boot_a"])
        self.model.ingest("robot/summary", SAMPLES["summary_boot_a"])
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 0x11223344)
        self.model.ingest("robot/heartbeat", SAMPLES["heartbeat_boot_b"])
        self.assertEqual(self.model.snapshot()["boot"]["boot_id"], 0xAABBCCDD)
        self.assertIsNone(self.model.state)
        self.assertEqual(self.rows("health")["Unavailable boards (last explicit read)"],
                         "vehicle")
        self.assertEqual(self.rows("health")["Unavailable observations (total)"], 233)
        self.assertIn("previous boot/session", self.panel("summary")["note"])
        self.model.ingest("robot/state", SAMPLES["state_boot_a"])
        self.assertIsNone(self.model.state)
        self.model.ingest("robot/log", SAMPLES["log_begin_boot_a"])
        self.model.ingest("robot/log", SAMPLES["log_begin_boot_b"])
        self.assertEqual(len(self.model.log.dumps), 2)

    def test_absent_and_busy_are_separate_health_rows(self):
        self.model.ingest("robot/heartbeat", reboot_heartbeat(100, mask=8, unavailable=42))
        rows = self.rows("health")
        self.assertEqual(rows["Unavailable boards (last explicit read)"], "vehicle")
        self.assertEqual(rows["Unavailable observations (total)"], 42)
        self.assertEqual(rows["Busy reads: line / sonar / motion / vehicle / health"],
                         "6 / 7 / 8 / 9 / 10")
        self.model.ingest("robot/heartbeat", reboot_heartbeat(100, mask=31))
        self.assertEqual(self.rows("health")["Unavailable boards (last explicit read)"],
                         "line, sonar, motion, vehicle, health")
        self.model.ingest("robot/heartbeat", reboot_heartbeat(100, mask=0))
        self.assertEqual(self.rows("health")["Unavailable boards (last explicit read)"], "none")

    def test_retained_old_summary_is_not_live_presence_for_dump(self):
        self.model.status = None
        self.model.last_message_ms = None
        self.model.ingest("robot/summary", boot_frame(summary(), 99), retained=True)
        self.assertFalse(self.button("dump")["enabled"])
        self.assertIsNone(self.model.last_message_ms)

    def test_old_heartbeat_does_not_invent_absence(self):
        self.model.ingest("robot/heartbeat", heartbeat_v(2))
        self.assertEqual(self.rows("health")["Unavailable boards (last explicit read)"],
                         "not reported (old heartbeat)")


def idle_mode(frame, idle=True):
    """Return a state v5 or heartbeat v4 copy of a C fixture with the idle bit set or clear."""
    out = bytearray(frame)
    if len(out) == 76:
        out[0] = 5
        struct.pack_into("<H", out, 70, 1 if idle else 0)
    else:
        out[0] = 4
        out[1] = (out[1] | 0x40) if idle else (out[1] & 0x3F)
    return bytes(out)


class TelemetryModeTests(ModelCase):
    def ingest(self, name, idle=True):
        self.model.ingest(topic_for(name), idle_mode(SAMPLES[name], idle))

    def test_old_firmware_uses_run_limits(self):
        state, beat = bytearray(SAMPLES["state_idle"]), bytearray(SAMPLES["heartbeat"])
        state[0], beat[0] = 4, 3  # state v4 and heartbeat v3 carry no mode bit
        self.model.ingest("robot/state", bytes(state))
        self.model.ingest("robot/heartbeat", bytes(beat))
        telemetry = self.model.snapshot()["telemetry"]
        self.assertEqual((telemetry["mode"], telemetry["state_stale_ms"]), ("unknown", 1000))
        self.clock.advance(1500)
        self.assertIn("ms old", self.button("start")["reason"])

    def test_idle_limits_keep_start_enabled_and_pause_streams(self):
        self.ingest("state_idle")
        self.ingest("heartbeat")
        self.clock.advance(2400)  # one missed 1 Hz idle state is not stale
        snapshot = self.model.snapshot()
        self.assertEqual(snapshot["telemetry"]["mode"], "idle")
        self.assertEqual(snapshot["telemetry"]["state_age_ms"], 2400)
        self.assertTrue(snapshot["buttons"]["start"]["enabled"])
        self.assertTrue(snapshot["buttons"]["calibrate"]["enabled"])
        self.assertFalse(self.panel("line")["stale"])
        self.assertFalse(self.panel("health")["stale"])
        for panel_id in ("ultrasonic", "log"):
            panel = self.panel(panel_id)
            self.assertTrue(panel["paused"])
            self.assertFalse(panel["stale"])
        self.clock.advance(200)  # 2600 ms: past the idle state limit
        self.assertIn("limit 2500 ms", self.button("start")["reason"])
        self.assertTrue(self.panel("line")["stale"])

    def test_idle_heartbeat_limit(self):
        self.ingest("heartbeat")
        self.clock.advance(11900)
        self.assertFalse(self.panel("health")["stale"])
        self.clock.advance(200)
        self.assertTrue(self.panel("health")["stale"])

    def test_run_frame_restores_full_rate_limits(self):
        self.ingest("state_idle")
        self.clock.advance(100)
        self.ingest("state_running", idle=False)
        snapshot = self.model.snapshot()
        self.assertEqual(snapshot["telemetry"]["mode"], "run")
        self.assertFalse(self.panel("ultrasonic")["paused"])
        self.clock.advance(1500)
        self.assertTrue(self.panel("line")["stale"])

    def test_newest_frame_decides(self):
        self.ingest("state_idle")
        self.clock.advance(50)
        self.ingest("heartbeat", idle=False)
        self.assertEqual(self.model.telemetry_mode(), "run")
        self.clock.advance(50)
        self.ingest("state_idle")
        self.assertEqual(self.model.telemetry_mode(), "idle")

    def test_offline_and_silent_gates_still_apply_in_idle(self):
        self.ingest("state_idle")
        self.clock.advance(3100)  # past ROBOT_OFFLINE_MS: about 3 missed idle states
        self.assertEqual(self.model.snapshot()["robot"]["level"], "silent")
        self.assertFalse(self.button("start")["enabled"])
        self.assertTrue(self.button("estop")["enabled"])

    def test_dump_gate_uses_idle_heartbeat_limit(self):
        self.ingest("state_idle")
        self.ingest("heartbeat_dump")
        self.clock.advance(2000)
        self.ingest("state_idle")
        self.assertIn("already running", self.button("dump")["reason"])

    def test_page_shows_the_paused_text(self):
        html = dashboard_page.PAGE
        self.assertIn("idle (paused)", html)
        self.assertIn("idle, telemetry paused", html)


NODE = shutil.which("node")
HARNESS = REPO / "tests" / "dashboard_page_harness.js"


class InstanceTests(AppCase):
    """The server instance ID is public, per process and separate from CSRF."""

    def test_instance_is_on_page_state_and_events_but_is_not_the_csrf_token(self):
        page = self.request("GET", "/")[2].decode()
        self.assertIn(f'<meta name="instance" content="{self.app.instance}">', page)
        self.assertNotIn("__INSTANCE__", page)
        self.assertRegex(self.app.instance, r"^[0-9a-f]{16}$")
        self.assertNotEqual(self.app.instance, self.app.csrf)
        state = json.loads(self.request("GET", "/api/state")[2])
        self.assertEqual(state["server_instance"], self.app.instance)
        events = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        events.request("GET", "/events", headers={"Host": f"127.0.0.1:{self.port}"})
        stream = events.getresponse()
        frame = stream.fp.readline()
        events.close()
        self.assertEqual(json.loads(frame[len(b"data: "):])["server_instance"], self.app.instance)
        self.assertNotIn("fixed-test-csrf", json.dumps(state))

    def test_each_process_gets_a_new_instance(self):
        other = dashboard.DashboardApp(self.model, None, 0, csrf="fixed-test-csrf")
        self.assertNotEqual(other.instance, self.app.instance)

    def test_old_page_token_is_still_refused(self):
        # A restart is detected by instance ID; the CSRF check itself is unchanged.
        status, _, body = self.post("estop", **{"X-CSRF-Token": "stale-token"})
        self.assertEqual((status, json.loads(body)["error"]), (403, "bad CSRF token"))
        self.assertEqual(self.link.sent, [])


@unittest.skipUnless(NODE, "node is not installed; page script checks skipped")
class PageScriptTests(AppCase):
    """Run the served page script in node with a fake DOM and scripted replies."""

    def run_page(self, steps, session=None, now=None, views=None):
        page = self.request("GET", "/")[2].decode()
        harness = {"page": page, "steps": steps, "session": session or {}, "now": now,
                   "views": views or []}
        result = subprocess.run([NODE, str(HARNESS)], input=json.dumps(harness),
                                capture_output=True, text=True, timeout=20, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def state(self, instance=None):
        snapshot = json.loads(self.request("GET", "/api/state")[2])
        if instance is not None:
            snapshot["server_instance"] = instance
        return snapshot

    def test_403_shows_persistent_red_error_naming_the_command(self):
        state = self.state()
        trace = self.run_page([{"render": state}, {"click": "estop"},
                               {"respond": {"status": 403, "body": {"ok": False, "error": "bad CSRF token"}}},
                               {"render": state}, {"render": state}])
        self.assertEqual(len(trace["posts"]), 1)
        self.assertEqual(trace["posts"][0]["headers"]["X-CSRF-Token"], "fixed-test-csrf")
        self.assertFalse(trace["cmderror"]["hidden"])  # still shown after later renders
        self.assertEqual(trace["cmderror"]["text"],
                         "ESTOP NOT SENT: bad CSRF token (HTTP 403). Reload the page. "
                         "Use GP21 to stop the robot.")

    def test_network_error_and_bad_reply_are_not_silent(self):
        state = self.state()
        trace = self.run_page([{"render": state}, {"click": "estop"}, {"respond": "throw"}])
        self.assertEqual(trace["cmderror"]["text"],
                         "ESTOP NOT SENT: dashboard server unreachable. Reload the page. "
                         "Use GP21 to stop the robot.")
        trace = self.run_page([{"render": state}, {"click": "estop"},
                               {"respond": {"status": 502, "body": None}}])
        self.assertIn("ESTOP NOT SENT: no reply (HTTP 502)", trace["cmderror"]["text"])

    def test_robot_refusal_names_command_without_reload_advice(self):
        state = self.state()
        trace = self.run_page([{"render": state}, {"click": "estop"},
                               {"respond": {"status": 409, "body": {"ok": False,
                                                                    "error": "robot is not idle"}}}])
        self.assertEqual(trace["cmderror"]["text"],
                         "ESTOP NOT SENT: robot is not idle (HTTP 409). Use GP21 to stop the robot.")

    def test_success_clears_an_earlier_error(self):
        state = self.state()
        ok = {"ok": True, "puback": True, "request_id": 1, "note": "note."}
        trace = self.run_page([{"render": state}, {"click": "estop"}, {"respond": "throw"},
                               {"click": "estop"}, {"respond": {"status": 200, "body": ok}}])
        self.assertTrue(trace["cmderror"]["hidden"])
        self.assertEqual(trace["message"]["text"], "ESTOP sent (broker acknowledged). note.")

    def test_instance_change_reloads_once_and_disables_commands(self):
        state, other = self.state(), self.state("0123456789abcdef")
        trace = self.run_page([{"render": state}, {"render": other}, {"click": "estop"}])
        self.assertEqual(trace["reloads"], 1)
        self.assertEqual(trace["posts"], [])  # disabled: the stale token is never used
        self.assertTrue(trace["estop"]["disabled"])
        self.assertFalse(trace["restart"]["hidden"])
        self.assertIn("DASHBOARD RESTARTED", trace["restart"]["text"])

    def test_repeated_restart_within_guard_shows_banner_instead_of_reload_loop(self):
        other = self.state("0123456789abcdef")
        first = self.run_page([{"render": other}], now=1000000)
        self.assertEqual(first["reloads"], 1)
        stamp = first["session"]["dashboard-reload-at"]
        # The reloaded page sees a mismatch again within 5 s: banner, no second reload.
        again = self.run_page([{"render": other}, {"click": "estop"}],
                              session={"dashboard-reload-at": stamp}, now=1003000)
        self.assertEqual((again["reloads"], again["posts"]), (0, []))
        self.assertIn("DASHBOARD RESTARTED: reload the page", again["restart"]["text"])
        self.assertTrue(again["estop"]["disabled"])
        # After the guard expires, a new mismatch may reload again.
        later = self.run_page([{"render": other}], session={"dashboard-reload-at": stamp},
                              now=1006000)
        self.assertEqual(later["reloads"], 1)

    def test_same_instance_never_reloads(self):
        state = self.state()
        trace = self.run_page([{"render": state}, {"render": state}])
        self.assertEqual(trace["reloads"], 0)
        self.assertTrue(trace["restart"]["hidden"])
        self.assertFalse(trace["estop"]["disabled"])

    IDLE_VIEWS = [["telemetry", None], ["panel-ultrasonic", ".age"], ["panel-log", ".age"],
                  ["panel-line", ".age"]]

    def idle_state(self):
        self.model.ingest("robot/state", idle_mode(SAMPLES["state_idle"]))
        self.model.ingest("robot/sonar", SAMPLES["sonar_valid"])
        self.clock.advance(1500)
        return self.state()

    def test_idle_paused_rendering_with_the_instance_guard(self):
        state = self.idle_state()
        trace = self.run_page([{"render": state}], views=self.IDLE_VIEWS)
        views = {view["id"]: view for view in trace["views"]}
        self.assertEqual(views["telemetry"]["text"], "idle, telemetry paused, state 1.5 s")
        self.assertEqual(views["telemetry"]["className"], "pill warn")
        self.assertEqual(views["panel-ultrasonic"]["text"], "idle (paused), data 1.5 s")
        self.assertEqual(views["panel-log"]["text"], "idle (paused)")  # no live rows yet
        self.assertEqual(views["panel-line"]["text"], "1.5 s")  # within the 2.5 s idle limit
        self.assertTrue(trace["restart"]["hidden"])
        self.assertEqual(trace["reloads"], 0)

    def test_restarted_server_does_not_render_idle_data(self):
        state = self.idle_state()
        trace = self.run_page([{"render": self.state("other-instance")}, {"render": state}],
                              views=self.IDLE_VIEWS)
        views = {view["id"]: view for view in trace["views"]}
        self.assertFalse(trace["restart"]["hidden"])
        self.assertIsNone(views["telemetry"]["text"])  # the guard returned first
        self.assertIsNone(views["panel-ultrasonic"]["text"])
        self.assertEqual(trace["reloads"], 1)
        self.assertTrue(trace["estop"]["disabled"])  # stale CSRF: commands are blocked


class RedesignModelTests(ModelCase):
    def test_visual_header_keeps_existing_api(self):
        self.feed("state_running")
        snap = self.model.snapshot()
        self.assertEqual(snap["view"]["mode"], "RUNNING")
        self.assertEqual(snap["view"]["module"], snap["view"]["state"]["module"])
        self.assertIn("panels", snap)
        self.assertIn("buttons", snap)
        self.clock.advance(3000)
        self.assertTrue(self.model.snapshot()["view"]["state_stale"])

    def test_sonar_recent_and_frozen_profile_have_explicit_missing_slots(self):
        self.feed("sonar_valid")
        self.model.ingest("robot/obstacle", obstacle(decision=2))
        v = self.model.snapshot()["view"]["sonar"]
        self.assertEqual(len(v["profile"]["points"]), 13)
        self.assertEqual(v["profile"]["decision"], 2)
        self.assertIn("not observed", [p["status"] for p in v["profile"]["points"]])
        frozen = v["profile"]["points"]
        self.feed("sonar_valid")
        self.assertEqual(self.model.snapshot()["view"]["sonar"]["profile"]["points"], frozen)
        self.clock.advance(10001)
        self.assertEqual(self.model.snapshot()["view"]["sonar"]["recent"], [])
        self.assertEqual(self.model.snapshot()["view"]["sonar"]["profile"]["points"], frozen)
        self.model._new_boot()
        self.assertIsNone(self.model.snapshot()["view"]["sonar"]["profile"])

    def test_reverse_events_and_state_are_visible(self):
        self.feed("state_running")
        fields = {"kind":8, "sequence":1, "uptime_ms":5000, "word":1, "detail":1}
        self.model._store_event(fields, self.model.now_ms())
        v = self.model.snapshot()["view"]["sonar"]
        self.assertTrue(v["reversing"])
        self.assertEqual(v["attempt"], 2)
        self.model._store_event(dict(fields, sequence=2, detail=3), self.model.now_ms())
        self.assertFalse(self.model.snapshot()["view"]["sonar"]["reversing"])


    def test_idle_retains_last_echo_window_without_claiming_fresh_data(self):
        self.feed("state_idle_mode")
        self.model._store_sonar(robot_wire.decode("robot/sonar",SAMPLES["sonar_valid"]),self.model.now_ms())
        before=self.model.snapshot()["view"]["sonar"]["recent"]
        self.assertEqual(len(before),1)
        self.clock.advance(20000)
        after=self.model.snapshot()
        self.assertTrue(after["view"]["paused"])
        self.assertEqual(after["view"]["sonar"]["recent"],before)
        self.assertGreater(next(p for p in after["panels"] if p["id"] == "ultrasonic")["age_ms"],10000)

    def test_new_state_clears_reverse_if_end_event_is_missing(self):
        self.feed("state_running")
        self.model._store_event({"kind":8,"sequence":1,"uptime_ms":0,"word":1,"detail":1},0)
        self.assertTrue(self.model.snapshot()["view"]["sonar"]["reversing"])
        self.feed("state_running")
        self.assertFalse(self.model.snapshot()["view"]["sonar"]["reversing"])

    def test_no_return_and_failed_echo_do_not_become_distance_samples(self):
        fields = robot_wire.decode("robot/sonar", SAMPLES["sonar_valid"])
        self.model._store_sonar(dict(fields, sequence=800, angle=0, echo_status=3, valid=0),0)
        self.model._store_sonar(dict(fields, sequence=801, angle=5, echo_status=1, valid=0),0)
        self.model._store_obstacle(robot_wire.decode("robot/obstacle",SAMPLES["obstacle"]),0)
        points = self.model.snapshot()["view"]["sonar"]["profile"]["points"]
        zero = next(p for p in points if p["angle"] == 0)
        failed = next(p for p in points if p["angle"] == 5)
        self.assertTrue(zero["no_return"])
        self.assertIsNone(zero["mm"])
        self.assertFalse(failed["no_return"])
        self.assertIsNone(failed["mm"])


@unittest.skipUnless(NODE, "node required for drawing checks")
class RedesignPageTests(AppCase):
    def trace(self, expressions=(), views=(), snapshot=None):
        page = self.request("GET", "/")[2].decode()
        snap = snapshot or json.loads(self.request("GET", "/api/state")[2])
        snap["server_instance"] = self.app.instance
        result = subprocess.run([NODE, str(HARNESS)], text=True, capture_output=True,
                                input=json.dumps({"page":page, "steps":[{"render":snap}],
                                                  "expressions":expressions, "views":views}),
                                timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_reverse_and_paused_replay_preserve_command_header(self):
        snap = dashboard_replay.scene(SAMPLES, "reattempt")
        trace = self.trace(snapshot=snap, views=[["panel-ultrasonic", ".visual-summary"]])
        self.assertIn("REVERSING",trace["views"][0]["text"])
        self.assertIn("attempt 2/3",trace["views"][0]["text"])
        self.assertFalse(trace["estop"]["disabled"])
        idle = dashboard_replay.scene(SAMPLES, "idle")
        trace = self.trace(snapshot=idle, views=[["panel-ultrasonic", None],
                                                ["panel-ultrasonic", ".age"], ["mode",None]])
        self.assertIn("paused",trace["views"][0]["className"])
        self.assertIn("idle (paused)",trace["views"][1]["text"])
        self.assertEqual(trace["views"][2]["text"],"IDLE")
        self.assertFalse(trace["estop"]["disabled"])

    def test_near_echo_red_beam_no_return_is_edge_square(self):
        snap = dashboard_replay.scene(SAMPLES, "scan-left")
        snap["view"]["sonar"]["latest_mm"] = 200
        trace = self.trace(snapshot=snap)
        self.assertIn(["set:strokeStyle", "#ff625b"],trace["canvasCalls"])
        self.assertTrue(any(c[0] == "strokeRect" for c in trace["canvasCalls"]))

    def test_health_retains_details_and_module_five_highlights_scan(self):
        snap = dashboard_replay.scene(SAMPLES, "scan-left")
        snap["view"]["module"] = 5
        trace = self.trace(snapshot=snap, views=[["module-3",None],
                                               ["panel-wheels-details",".title"],
                                               ["panel-summary-details",".title"]])
        self.assertEqual(trace["views"][0]["className"],"active")
        self.assertIn("details",trace["views"][1]["text"])
        self.assertIn("details",trace["views"][2]["text"])

    def test_hump_card_is_a_primary_card(self):
        self.model.ingest("robot/summary", SAMPLES["summary_hump"])
        page = self.request("GET", "/")[2].decode()
        # The fake DOM does not model parents: check the layout source directly.
        self.assertIn('["ultrasonic", "line", "wheels", "summary", "hump"]', page)
        self.assertIn("#panel-hump { grid-column:3; grid-row:2; }", page)
        self.assertIn("#panel-summary { grid-column:2; grid-row:2; }", page)
        trace = self.trace(views=[["panel-hump", ".title"]])
        self.assertEqual(trace["views"][0]["text"], "HIGHEST HUMP")

    def test_collapsed_health_does_not_draw_zero_size_dial(self):
        expr = """(() => {
          const node=document.getElementById('panel-odometry').querySelector('canvas');
          node.clientWidth=0; node.clientHeight=0;
          renderPanel({id:'odometry',title:'dial',age_ms:0,rows:[],items:[],note:'',
            chart:{type:'dial',heading_deg:0,turn_rate_dps:0}});
          return true;
        })()"""
        self.assertEqual(self.trace(expressions=[expr])["values"],[True])

    def test_line_and_drive_use_numeric_state_and_signed_speed(self):
        self.model.ingest("robot/state", SAMPLES["state_running"])
        trace = self.trace(views=[["panel-wheels", ".title"]])
        self.assertEqual(trace["views"][0]["text"], "DRIVE")
        labels = [c[1] for c in trace["canvasCalls"] if c[0] == "fillText"]
        self.assertIn("0 floor → 1000 line", labels)
        self.assertTrue(any(str(v).startswith("asked ") for v in labels))
        self.assertIn("signed mm/s · 0 at centre", labels)
        self.assertTrue(any("position " in str(v) for v in labels))

    def test_fan_coordinates_put_positive_angles_on_left(self):
        v = self.trace(["sonarPoint(30, 100, 200, 200, 1)",
                        "sonarPoint(-30, 100, 200, 200, 1)"])["values"]
        self.assertAlmostEqual(v[0][0], 150)
        self.assertAlmostEqual(v[1][0], 250)
        self.assertLess(v[0][1], 200)

    def test_fan_draws_and_names_left_decision(self):
        self.model.ingest("robot/obstacle", obstacle(decision=2))
        trace = self.trace(views=[["panel-ultrasonic", ".visual-summary"],
                                 ["panel-ultrasonic", ".legend"]])
        self.assertIn("← LEFT", trace["views"][0]["text"])
        self.assertIn("observed 0/13", trace["views"][0]["text"])
        self.assertIn("approximate, measured 2026-11-07", trace["views"][1]["text"])
        self.assertTrue(any(c[0] == "arc" for c in trace["canvasCalls"]))


class RunPathTests(ModelCase):
    @staticmethod
    def path_row(index=0, time=0, left=100, right=100, generation=1, module=1, state=1):
        return {"generation":generation, "index":index, "kr1_time_ms":time,
                "kr1_measured_l":left, "kr1_measured_r":right, "kr1_state":state,
                "kr1_module":module, "source":"live", "kind":5, "data_age_ms":0,
                "rows_behind":0, "jumped":False}

    def add(self, **kwargs):
        self.model._log_rows([self.path_row(**kwargs)], self.model.now_ms())
        return self.model.path_visual()

    def test_straight_distance_and_reverse(self):
        self.add()
        self.add(index=1, time=100, left=-100, right=-100)
        p=self.add(index=2, time=200)
        self.assertAlmostEqual(p["points"][1]["y"],10)
        self.assertAlmostEqual(p["points"][-1]["y"],0)
        self.assertAlmostEqual(p["points"][-1]["x"],0)

    def test_turn_track_and_positive_left_heading(self):
        self.add(left=0,right=111)
        p=self.add(index=1,time=100,left=0,right=111)
        self.assertAlmostEqual(self.model.path["heading"],.1)
        self.assertLess(p["points"][-1]["x"],0)
        self.assertGreater(p["points"][-1]["y"],0)

    def test_gap_and_long_dt_are_not_connected(self):
        self.add()
        p=self.add(index=3,time=100)
        self.assertEqual(p["gaps"],1)
        self.assertFalse(p["points"][-1]["connected"])
        self.assertEqual(p["points"][-1]["y"],0)
        self.assertEqual(self.add(index=4,time=1000)["gaps"],2)

    def test_resets_per_generation_and_stopped_run(self):
        self.add()
        self.add(index=1,time=100)
        p=self.add(index=0,time=200,generation=2)
        self.assertEqual(len(p["points"]),1)
        self.add(index=1,time=220,generation=2,state=0)
        p=self.add(index=2,time=240,generation=2)
        self.assertEqual((len(p["points"]),p["points"][0]["y"]),(1,0))

    def test_refill_duplicate_and_delayed_generation_cannot_move_path(self):
        self.add(generation=2)
        self.model._log_rows([dict(self.path_row(index=1,time=100,generation=2),source="refill")],0)
        self.add(generation=2)
        self.add(generation=1)
        self.assertEqual(len(self.model.path_visual()["points"]),1)

    def test_quantized_timer_wrap_and_boot_reset(self):
        self.add(time=65535*20)
        p=self.add(index=1,time=0)
        self.assertAlmostEqual(p["points"][-1]["y"],2)
        self.model._new_boot()
        self.assertEqual(self.model.path_visual()["points"],[])

    def test_module_segments_and_point_bound(self):
        self.add(module=3)
        self.add(index=1,time=20,module=4)
        self.assertEqual([p["module"] for p in self.model.path_visual()["points"]],[3,4])
        for i in range(2,4602):
            self.add(index=i,time=i*20)
        self.assertEqual(len(self.model.path_visual()["points"]),4600)
        self.assertEqual(self.model.path_visual()["trimmed"],2)


class RedesignLayoutTests(unittest.TestCase):
    def test_offline_responsive_layout_and_health_section(self):
        page = dashboard_page.PAGE
        self.assertIn('name="viewport"', page)
        self.assertIn('@media(max-width:800px)', page)
        self.assertIn('<details id="health"', page)
        self.assertIn('id="module-3">SCAN', page)
        self.assertIn('id="estop" data-command="estop"', page)
        self.assertNotIn('https://', page)



class ReplayTests(unittest.TestCase):
    def test_replay_is_public_offline_and_does_not_start_a_live_connection(self):
        page = dashboard_replay.replay_page(dashboard_replay.scene(SAMPLES,"idle"))
        self.assertNotIn("\nstart();\n",page)
        self.assertIn("\nrender({",page)
        self.assertIn("REPLAY · public fixtures",page)
        self.assertNotIn("__CSRF__",page)
        self.assertNotIn(TOKEN.hex(),page)
        self.assertNotIn("https://",page)



if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
