"""Pure dashboard model: robot MQTT messages in, panel data and button rules out.

No network, no Paho and no HTTP here, so every rule runs in a host test.
Add a panel by writing one builder function and decorating it with
register_panel; the page draws any panel that appears in the snapshot.
"""
import collections
import threading
import time

import robot_wire

STALE_MS = 1000  # data older than this is greyed out
HEARTBEAT_STALE_MS = 2500  # the robot sends a heartbeat once per second
# Idle telemetry mode (state v5 / heartbeat v4 bit set): state at 1 Hz, heartbeat every 5 s.
IDLE_STALE_MS = 2500
IDLE_HEARTBEAT_STALE_MS = 12000
ROBOT_OFFLINE_MS = 3000  # no robot message at all for this long
PENDING_TIMEOUT_MS = 5000  # wait this long for a robot decision on a command
MISSING_AGE_MS = 65535  # robot value for a board with no data
EVENT_LIMIT = 50
PROFILE_LIMIT = 256
TREND_LIMIT = 100
LOG_MESSAGE_LIMIT = 20
CONFIRM_COMMANDS = ("start", "calibrate")
OBSTACLE_DECISION_TEXT = {0: "none (both sides blocked)", 1: "right", 2: "left"}

STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "FAULT"}
MODULE_NAMES = {0: "none", 1: "line tracing", 2: "line search", 3: "obstacle scan",
                4: "obstacle detour", 5: "scan reverse"}
EVENT_KINDS = {1: "outcome", 2: "module change", 3: "ESTOP or fault", 4: "scan complete",
               5: "run end", 6: "command decision", 7: "junction (reserved)",
               8: "scan retry"}
SCAN_RETRY_TEXT = {1: "reverse begins", 2: "reverse ends", 3: "rescan begins"}
DETOUR_LIMIT = 8
SCAN_STAGES = {0: "not reported", 1: "coarse", 2: "fine"}
SIDE_TEXT = {0: "none", 1: "right", 2: "left"}  # obstacle and detour side bytes
LINE_SIDE_TEXT = {0: "unknown", 1: "left", 2: "right"}  # vc_line_side_t, used in payloads
DETOUR_FAIL_TEXT = {0: "none", 1: "both sides blocked", 2: "turn limit reached",
                    3: "took too long", 4: "ultrasonic fault", 5: "echo too near",
                    6: "no line found"}
STOP_SOURCES = {0: "none or unknown", 1: "GP21 button", 2: "MQTT ESTOP", 3: "fault",
                4: "time limit"}
STOP_REASONS = {0: "unknown", 1: "GP21 button", 2: "MQTT ESTOP command", 3: "90 s limit",
                4: "shell left RUNNING", 5: "vehicle deadline missed", 6: "motion wake lost",
                7: "motion status failed", 8: "line snapshot stale", 9: "line samples lost",
                10: "sonar failure", 11: "telemetry failure", 12: "other",
                13: "scan reverse timed out"}
SHELL_FAULTS = {0: "none", 1: "safety (motor fault bits)", 2: "module failed",
                3: "no route", 4: "no module", 5: "handoff loop"}
MOTION_FAULT_BITS = ((0x01, "stall left"), (0x02, "stall right"), (0x04, "ESTOP"),
                     (0x08, "stale command"))
BARCODE_DIRECTIONS = {0: "unknown", 1: "forward", 2: "reverse"}
IMU_EVENTS = {1: "stationary", 2: "accelerating", 3: "turning", 4: "climbing",
              5: "descending", 6: "impact"}
NOT_IMPLEMENTED = "not implemented yet"
OPCODE_NAMES = {code: name.upper() for name, code in robot_wire.COMMANDS.items()}
DECISIONS = {0: "accepted", 1: "refused: robot not idle", 2: "refused: not calibrated",
             3: "refused: log busy", 4: "duplicate request",
             5: "refused: backend unavailable"}
RESET_REASONS = {0: "power or RUN pin", 1: "watchdog", 2: "other"}
MQTT_STATES = {0: "disconnected", 1: "connecting", 2: "connected", 3: "subscribed"}
SONAR_STATUS = {0: "empty", 1: "echo", 2: "deadline", 3: "cancelled"}
ECHO_STATUS = {0: "valid", 1: "no rising edge", 2: "stuck high", 3: "no return"}

PANEL_BUILDERS = []


def register_panel(builder):
    """Add a panel builder: builder(model, now_ms) returns the dict from make_panel."""
    PANEL_BUILDERS.append(builder)
    return builder


def make_panel(panel_id, title, age_ms, stale_ms=STALE_MS, rows=None, chart=None,
               items=None, note="", placeholder=False, table=None, paused=False):
    """Build one panel. age_ms None means no data yet; stale_ms None means never stale.

    paused True: the robot does not publish this stream in idle telemetry mode, so the
    panel is not stale; the page shows "idle (paused)" and the data age.
    """
    if paused:
        stale = False
    elif placeholder:
        stale = True
    elif age_ms is None:
        stale = True
    else:
        stale = stale_ms is not None and age_ms > stale_ms
    return {"id": panel_id, "title": title, "age_ms": age_ms, "stale": stale,
            "rows": [{"label": label, "value": value} for label, value in (rows or [])],
            "chart": chart, "items": items or [], "note": note, "placeholder": placeholder,
            "table": table, "paused": paused}


def _name(table, value):
    return table.get(value, f"unknown ({value})")


class DashboardModel:
    """Latest robot data, command bookkeeping and button gating. Thread safe."""

    def __init__(self, clock=time.monotonic, token_configured=False):
        self.clock = clock
        self.token_configured = token_configured
        self.lock = threading.RLock()
        self.broker_connected = False
        self.discarded = 0
        self.last_discard = ""
        self.state = None  # {"fields": dict, "rx": ms}
        self.heartbeat = None
        self.status = None  # robot/status: {"online": bool, "retained": bool, "rx": ms}
        self.statuses = {}  # retained status can arrive before that boot's first live frame
        self.obstacle = None
        self.obstacle_scan_id = None
        self.obstacle_missed = 0
        self.detours = collections.deque(maxlen=DETOUR_LIMIT)
        self.detour_sequence = None
        self.detour_missed = 0
        self.summary = None  # {"fields": dict, "rx": ms, "retained": bool}
        self.reserved = {"barcode": None, "imu": None, "imu_event": None}
        self.ignored_reserved = 0
        self.sonar = None
        self.sonar_profile = {}  # angle -> {"mm", "text", "rx"}
        self.sonar_recent = collections.deque(maxlen=128)
        self.scan_samples = {}
        self.completed_scan = None
        self.scan_retry = 0
        self.scan_reversing = False
        self.events = collections.deque(maxlen=EVENT_LIMIT)
        self.last_decision = None
        self.event_sequence = None
        self.event_gaps = 0
        self.trend = collections.deque(maxlen=TREND_LIMIT)
        self.pending = None
        self.last_message_ms = None
        self.log_messages = collections.deque(maxlen=LOG_MESSAGE_LIMIT)
        self.log = robot_wire.LogStream(report=self.log_messages.append)
        self.log_info = {"live_rows": 0, "refill_rows": 0, "jumped_frames": 0, "last": None}

    def now_ms(self):
        return self.clock() * 1000.0

    # ---- input -------------------------------------------------------------

    def set_broker_connected(self, connected):
        with self.lock:
            self.broker_connected = bool(connected)
            if not connected:
                self.status = None  # the broker will send the retained status again
                self.statuses.clear()

    def ingest(self, topic, payload, retained=False):
        """Decode one message. A malformed message is counted and never changes state."""
        with self.lock:
            now = self.now_ms()
            try:
                previous = self.log.sessions.current
                if topic == "robot/log":
                    rows = self.log.handle(payload)
                    if self.log.sessions.current != previous:
                        self._new_boot()
                    current_rows = [r for r in rows
                                    if r["boot_session"] == self.log.sessions.current]
                    self._log_rows(current_rows, now)
                    heard_live = bool(current_rows)
                else:
                    fields = robot_wire.decode(topic, payload)
                    scope = self.log.observe(topic, fields, retained)
                    if self.log.sessions.current != previous:
                        self._new_boot()
                    # Old retained history is labelled, never used as live state.
                    if (scope != self.log.sessions.current and topic != "robot/summary"
                            and self.log.sessions.boot_id is not None):
                        if topic == "robot/status":
                            self._store(topic, dict(fields, boot_session=scope), now, retained)
                        return
                    self._store(topic, dict(fields, boot_session=scope), now, retained)
                    heard_live = (not retained and fields.get("known") != 0 and
                                  topic != "robot/summary")
            except ValueError as error:
                self.discarded += 1
                self.last_discard = f"{topic}: {error}"
                return
            if heard_live:
                self.last_message_ms = now

    def _new_boot(self):
        """Invalidate all current readings and command decisions, but retain scoped logs."""
        self.state = self.heartbeat = self.sonar = self.obstacle = None
        self.status = self.statuses.get(self.log.sessions.current)
        self.sonar_profile.clear()
        self.sonar_recent.clear()
        self.scan_samples.clear()
        self.completed_scan = None
        self.scan_retry = 0
        self.scan_reversing = False
        self.trend.clear()
        self.events.clear()
        self.detours.clear()
        self.obstacle_scan_id = self.detour_sequence = self.event_sequence = None
        self.obstacle_missed = self.detour_missed = self.event_gaps = 0
        self.pending = self.last_decision = self.last_message_ms = None
        self.reserved = {"barcode": None, "imu": None, "imu_event": None}
        self.log_info = {"live_rows": 0, "refill_rows": 0, "jumped_frames": 0, "last": None}

    def _store(self, topic, fields, now, retained=False):
        if topic == "robot/status":
            status = {"online": bool(fields["robot_online"]), "retained": retained, "rx": now}
            self.statuses[fields["boot_session"]] = status
            if (fields["boot_session"] == self.log.sessions.current or
                    self.log.sessions.boot_id is None):
                self.status = status
        elif topic == "robot/obstacle":
            self._store_obstacle(fields, now)
        elif topic in ("robot/barcode", "robot/imu"):
            name = topic.removeprefix("robot/")
            if name == "imu" and fields["kind"] == 2:
                name = "imu_event"
            if fields["known"]:
                self.reserved[name] = {"fields": fields, "rx": now}
            else:
                self.ignored_reserved += 1
        elif topic == "robot/detour":
            self._store_detour(fields, now)
        elif topic == "robot/summary":
            if (self.summary is not None and
                    self.summary["fields"]["boot_session"] == self.log.sessions.current and
                    fields["boot_session"] != self.log.sessions.current):
                return  # do not replace a current summary with an older retained copy
            self.summary = {"fields": fields, "rx": now, "retained": retained}
        elif topic == "robot/state":
            previous_module = None if self.state is None else self.state["fields"]["module"]
            if fields["module"] == 3 and previous_module not in (3, 5):
                self.scan_samples.clear()
                self.scan_retry = 0
                self.scan_reversing = False
            if fields["state"] != 1:
                self.scan_reversing = False
            self.state = {"fields": fields, "rx": now}
            self.trend.append(fields["kr1_position"])
        elif topic == "robot/heartbeat":
            self.heartbeat = {"fields": fields, "rx": now}
        elif topic == "robot/sonar":
            self._store_sonar(fields, now)
        elif topic == "robot/event":
            self._store_event(fields, now)

    def _store_obstacle(self, fields, now):
        scan_id = fields["scan_id"]
        if self.obstacle_scan_id is not None and scan_id > self.obstacle_scan_id + 1:
            self.obstacle_missed += scan_id - self.obstacle_scan_id - 1
        self.obstacle_scan_id = scan_id  # a lower number means the robot restarted
        self.obstacle = {"fields": fields, "rx": now}
        self.completed_scan = {"fields": dict(fields), "samples": dict(self.scan_samples), "rx": now}
        self.scan_retry = fields.get("scan_attempt", 0)
        self.scan_samples.clear()

    def _store_detour(self, fields, now):
        sequence = fields["sequence"]
        if self.detour_sequence is not None and sequence > self.detour_sequence + 1:
            self.detour_missed += sequence - self.detour_sequence - 1
        self.detour_sequence = sequence  # a lower number means the robot restarted
        self.detours.append({"fields": fields, "rx": now})

    def _store_sonar(self, fields, now):
        if self.sonar is not None and self.sonar["fields"]["sequence"] == fields["sequence"]:
            return
        self.sonar = {"fields": fields, "rx": now}
        valid = bool(fields["valid"]) and fields["echo_status"] == 0
        if len(self.sonar_profile) >= PROFILE_LIMIT and fields["angle"] not in self.sonar_profile:
            return
        reading = {"angle": fields["angle"], "mm": fields["echo_mm"] if valid else None,
                   "status": _sonar_distance(fields), "rx": now,
                   "no_return": fields["status"] == 1 and fields["echo_status"] == 3}
        self.sonar_recent.append(reading)
        if fields["angle"] in range(-30, 31, 5):
            self.scan_samples[fields["angle"]] = reading
        self.sonar_profile[fields["angle"]] = {
            "mm": fields["echo_mm"] if valid else None,
            "text": _name(ECHO_STATUS, fields["echo_status"]) if fields["status"] == 1
            else _name(SONAR_STATUS, fields["status"]), "rx": now}

    def _store_event(self, fields, now):
        sequence = fields["sequence"]
        if self.event_sequence is not None and sequence > self.event_sequence + 1:
            self.event_gaps += sequence - self.event_sequence - 1
        self.event_sequence = sequence
        kind = fields["kind"]
        event = {"rx": now, "sequence": sequence, "kind": kind, "uptime_ms": fields["uptime_ms"]}
        if kind == 6:
            opcode = fields["word"] & 0xFF
            event["opcode"] = opcode
            event["decision"] = fields["detail"]
            event["text"] = f"{_name(OPCODE_NAMES, opcode)} {_name(DECISIONS, fields['detail'])}"
            self.last_decision = event
            pending = self.pending
            if (pending is not None and pending["result"] is None
                    and pending["opcode"] == opcode and now >= pending["sent"]):
                pending["result"] = event
        elif kind == robot_wire.EVENT_SCAN_RETRY:
            attempt = fields["word"] & 0xFF
            self.scan_retry = attempt
            self.scan_reversing = fields["detail"] == 1
            if fields["detail"] == 3:
                self.scan_samples.clear()
            event["text"] = (f"SCAN retry {attempt}: {_name(SCAN_RETRY_TEXT, fields['detail'])}"
                             " (both sides blocked)")
        else:
            event["text"] = (f"{_name(EVENT_KINDS, kind)}: {_name(MODULE_NAMES, fields['from'])}"
                             f" to {_name(MODULE_NAMES, fields['to'])},"
                             f" word {fields['word']}, detail {fields['detail']}")
        self.events.append(event)

    def _log_rows(self, rows, now):
        live = [row for row in rows if row.get("source") == "live" and row.get("kind") == 5]
        refill = [row for row in rows if row.get("source") == "refill"]
        self.log_info["live_rows"] += len(live)
        self.log_info["refill_rows"] += len(refill)
        if live:
            last = live[-1]
            self.log_info["last"] = {"rx": now, "data_age_ms": last["data_age_ms"],
                                     "rows_behind": last["rows_behind"],
                                     "generation": last["generation"], "index": last["index"]}
            if any(row["jumped"] for row in live):
                self.log_info["jumped_frames"] += 1

    def note_command(self, command, request_id, puback):
        """Record a command the dashboard just published."""
        with self.lock:
            self.pending = {"command": command, "request_id": request_id, "puback": puback,
                            "opcode": robot_wire.COMMANDS[command], "sent": self.now_ms(),
                            "result": None}

    # ---- derived values ----------------------------------------------------

    def state_age(self, now, key=None):
        """Age of robot/state in ms, plus the reported board age for key; None if unknown."""
        if self.state is None:
            return None
        age = int(now - self.state["rx"])
        if key is not None:
            reported = self.state["fields"][key]
            if reported >= MISSING_AGE_MS:
                return MISSING_AGE_MS
            age += reported
        return min(age, MISSING_AGE_MS)

    def record_age(self, now, record, extra=0):
        if record is None:
            return None
        return min(int(now - record["rx"]) + extra, MISSING_AGE_MS)

    def telemetry_mode(self):
        """Return "idle", "run" or "unknown" from the newest frame that carries the mode bit.

        robot/state v5 and robot/heartbeat v4 carry it. Older firmware: "unknown", and the
        dashboard uses the run (full rate) limits.
        """
        newest = None
        for record, version in ((self.state, 5), (self.heartbeat, 4)):
            if record is not None and record["fields"]["version"] >= version and (
                    newest is None or record["rx"] >= newest["rx"]):
                newest = record
        if newest is None:
            return "unknown"
        return "idle" if newest["fields"]["telemetry_idle"] else "run"

    def limits(self):
        """Freshness limits in ms for the current telemetry mode."""
        if self.telemetry_mode() == "idle":
            return {"state_stale_ms": IDLE_STALE_MS, "heartbeat_stale_ms": IDLE_HEARTBEAT_STALE_MS}
        return {"state_stale_ms": STALE_MS, "heartbeat_stale_ms": HEARTBEAT_STALE_MS}

    def telemetry(self, now):
        mode = self.telemetry_mode()
        return {"mode": mode, "paused": mode == "idle", **self.limits(),
                "state_age_ms": self.record_age(now, self.state),
                "heartbeat_age_ms": self.record_age(now, self.heartbeat)}

    def robot_status(self, now):
        """Return the robot link level: online, offline, silent or unknown.

        online  = robot/status says online AND robot/state is recent
        offline = robot/status says offline (shown at once, also when retained)
        silent  = robot/status says online but robot/state stopped (link may be dying)
        unknown = no robot/status message yet
        """
        status = self.status
        state_age = None if self.state is None else int(now - self.state["rx"])
        last_age = None if self.last_message_ms is None else int(now - self.last_message_ms)
        if status is None:
            level, text = "unknown", "robot status unknown (no robot/status yet)"
        elif not status["online"]:
            level, text = "offline", "ROBOT OFFLINE (broker last will)"
        elif state_age is None or state_age > ROBOT_OFFLINE_MS:
            level = "silent"
            text = "robot silent: no state for " + ("ever" if state_age is None
                                                    else f"{state_age // 1000} s")
        else:
            level, text = "online", "ROBOT ONLINE"
        return {"online": level == "online", "level": level, "text": text, "age_ms": last_age,
                "heard_recently": last_age is not None and last_age <= ROBOT_OFFLINE_MS,
                "status": None if status is None else
                {"online": status["online"], "retained": status["retained"],
                 "age_ms": int(now - status["rx"])}}

    def _state_problem(self, now):
        """Return why robot/state is not usable for a gated button, or an empty string."""
        if self.state is None:
            return "no state data from the robot"
        age = int(now - self.state["rx"])
        limit = self.limits()["state_stale_ms"]
        if age > limit:
            return f"robot state is {age} ms old (limit {limit} ms)"
        fields = self.state["fields"]
        if fields["vehicle_busy"]:
            return "vehicle board read busy"
        if not fields["vehicle_fresh"]:
            return "vehicle board data is not fresh"
        return ""

    def _is_idle(self):
        fields = self.state["fields"]
        return fields["active"] == 0 and fields["state"] == 0

    def _pending_open(self, now):
        pending = self.pending
        return (pending is not None and pending["result"] is None
                and now - pending["sent"] <= PENDING_TIMEOUT_MS)

    def button_state(self, command, now=None):
        """Return {"enabled", "reason", "needs_confirm"}; the server asks again before sending."""
        with self.lock:
            now = self.now_ms() if now is None else now
            result = {"enabled": False, "reason": "", "needs_confirm": command in CONFIRM_COMMANDS}
            if command == "estop":
                result["enabled"] = True
                link = self.robot_status(now)["level"]
                if not self.broker_connected:
                    result["reason"] = "broker not connected: ESTOP cannot be sent now"
                elif link in ("offline", "silent"):
                    result["reason"] = ("robot is not reachable: ESTOP delivery is unlikely. "
                                        "Use GP21.")
                return result
            reason = self._gate_reason(command, now)
            result["enabled"] = reason == ""
            result["reason"] = reason
            return result

    def _gate_reason(self, command, now):
        if command not in robot_wire.COMMANDS:
            return "unknown command"
        if not self.broker_connected:
            return "broker not connected"
        if not self.token_configured:
            return "no token file: only ESTOP is available"
        robot = self.robot_status(now)
        if robot["level"] == "offline":
            return "robot is offline"
        if robot["level"] == "silent":
            return "robot is silent (no state data)"
        if robot["level"] == "unknown" and not robot["heard_recently"]:
            return "no recent robot data"
        if self._pending_open(now):
            return "waiting for the robot decision on the previous command"
        if command == "dump":
            heartbeat = self.heartbeat
            if (heartbeat is not None
                    and now - heartbeat["rx"] <= self.limits()["heartbeat_stale_ms"]
                    and heartbeat["fields"]["log_dump_active"]):
                return "a RAM dump is already running"
            return ""
        problem = self._state_problem(now)
        if problem:
            return problem
        if not self._is_idle():
            return "robot is not idle"
        return ""

    def last_command(self, now):
        pending = self.pending
        if pending is None:
            return None
        if pending["result"] is not None:
            status = "robot decision: " + _name(DECISIONS, pending["result"]["decision"])
        elif now - pending["sent"] > PENDING_TIMEOUT_MS:
            status = ("no robot decision received "
                      "(token refused, link lost or event lost); PUBACK is not acceptance")
        else:
            status = "sent; waiting for the robot decision"
        return {"command": pending["command"].upper(), "request_id": pending["request_id"],
                "puback": pending["puback"], "age_ms": int(now - pending["sent"]),
                "status": status}

    # ---- output ------------------------------------------------------------

    def gap_counts(self):
        lost = sum(g["end"] - g["start"] for g in self.log.gaps if g["type"] == "lost")
        jumped = sum(g["end"] - g["start"] for g in self.log.gaps if g["type"] == "jumped")
        return {"lost_rows": lost, "jumped_rows": jumped, "total": lost + jumped,
                "event_gaps": self.event_gaps, "duplicates": self.log.duplicates}

    def sonar_visual(self, now):
        """Observed echo points, not inferred obstacle geometry or reconstructed scans."""
        record = self.completed_scan
        profile = None
        if record is not None:
            f = record["fields"]
            samples = record["samples"]
            profile = {"scan_id": f["scan_id"], "decision": f["decision"],
                       "attempt": f.get("scan_attempt", 0) + 1,
                       "age_ms": int(now - record["rx"]),
                       "free_threshold_mm": (f["profile_word"] & 511) + 150,
                       "points": [dict(samples[a], angle=a) if a in samples else
                                  {"angle": a, "mm": None, "no_return": False,
                                   "status": "not observed"} for a in range(-30, 31, 5)],
                       "closest": {"angle": f["closest_angle_deg"], "mm": f["closest_mm"]}
                       if f["closest_valid"] else None}
        f = {} if self.state is None else self.state["fields"]
        latest = None if self.sonar is None else self.sonar["fields"]
        servo = f.get("kr1_servo_deg", 0)
        if latest is not None and (self.state is None or self.sonar["rx"] > self.state["rx"]):
            servo = latest["angle"]
        return {"beam_deg": servo,
                "latest_mm": None if latest is None or not latest["valid"] or
                latest["echo_status"] != 0 else latest["echo_mm"],
                "recent": [dict(p, age_ms=int(now - p["rx"])) for p in self.sonar_recent
                           if now - p["rx"] <= 10000],
                "profile": profile, "attempt": min(3, self.scan_retry + 1),
                "reversing": bool(f.get("state") == 1 and
                                  (self.scan_reversing or f.get("module") == 5))}

    def visual_state(self, now):
        """Numeric presentation fields; existing panel and command API stays intact."""
        f = {} if self.state is None else self.state["fields"]
        return {"mode": _name(STATE_NAMES, f.get("state")),
                "module": f.get("module", 0),
                "paused": self.telemetry_mode() == "idle",
                "state": dict(f),
                "sonar": self.sonar_visual(now),
                "state_stale": self.state is None or
                self.state_age(now) > self.limits()["state_stale_ms"]}

    def snapshot(self):
        """Return everything the page needs as one JSON-ready dict."""
        with self.lock:
            now = self.now_ms()
            return {"boot": {"boot_id": self.log.sessions.boot_id,
                             "session": self.log.sessions.current,
                             "inferred": self.log.sessions.boot_id is None},
                    "broker": {"connected": self.broker_connected},
                    "robot": self.robot_status(now),
                    "telemetry": self.telemetry(now),
                    "view": self.visual_state(now),
                    "token_configured": self.token_configured,
                    "gaps": self.gap_counts(),
                    "discarded": self.discarded,
                    "last_command": self.last_command(now),
                    "buttons": {name: self.button_state(name, now)
                                for name in ("estop", "dump", "start", "calibrate")},
                    "panels": [builder(self, now) for builder in PANEL_BUILDERS]}


# ---- panels ------------------------------------------------------------------


def _sonar_distance(fields):
    if fields["valid"] and fields["echo_status"] == 0:
        return f"{fields['echo_mm']} mm"
    return _name(ECHO_STATUS, fields["echo_status"]) if fields["status"] == 1 \
        else _name(SONAR_STATUS, fields["status"])


def _unavailable_text(fields):
    if "board_unavailable_mask" not in fields:
        return "not reported (old heartbeat)"
    names = ("line", "sonar", "motion", "vehicle", "health")
    absent = [name for bit, name in enumerate(names)
              if fields["board_unavailable_mask"] & (1 << bit)]
    return ", ".join(absent) if absent else "none"


def _cpu_text(fields):
    if "cpu_load_permille" not in fields:
        return "not reported (old heartbeat)"
    value = fields["cpu_load_permille"]
    return "not available" if value == robot_wire.CPU_UNKNOWN else f"{value / 10:.1f} %"


def _stack_text(value):
    return "unknown" if value == MISSING_AGE_MS else f"{value} bytes"


@register_panel
def vehicle_panel(model, now):
    rows = []
    age = None
    if model.state is not None:
        fields = model.state["fields"]
        age = model.state_age(now, "vehicle_age_ms")
        rows += [("State", _name(STATE_NAMES, fields["state"])),
                 ("Module", _name(MODULE_NAMES, fields["module"])),
                 ("Running", "yes" if fields["active"] else "no"),
                 ("Motion faults", fields["motion_faults"]),
                 ("Vehicle board", "fresh" if fields["vehicle_fresh"] else "not fresh")]
    rows.append(("Boot / session", model.log.sessions.current +
                 (" (inferred legacy session)" if model.log.sessions.boot_id is None else "")))
    command = model.last_command(now)
    rows.append(("Last command", "none sent" if command is None else
                 f"{command['command']} #{command['request_id']}"))
    if command is not None:
        rows.append(("Command result", command["status"]))
    decision = model.last_decision
    rows.append(("Last robot decision", "none" if decision is None else decision["text"]))
    return make_panel("vehicle", "Vehicle", age, rows=rows,
                      stale_ms=model.limits()["state_stale_ms"],
                      note="" if model.state else "waiting for robot/state")


@register_panel
def line_panel(model, now):
    if model.state is None:
        return make_panel("line", "Line sensor", None, note="waiting for robot/state")
    fields = model.state["fields"]
    rows = [("Position", fields["kr1_position"]),
            ("Raw left / centre", f"{fields['kr1_raw_l']} / {fields['kr1_raw_c']}"),
            ("Normalised left / centre", f"{fields['norm_l']} / {fields['norm_c']}"),
            ("Line valid", "yes" if fields["line_valid"] else "no"),
            ("Line board", "fresh" if fields["line_fresh"] else "not fresh")]
    chart = {"type": "trend", "values": list(model.trend), "min": -128, "max": 127,
             "label": "line position"}
    return make_panel("line", "Line sensor", model.state_age(now, "line_age_ms"),
                      stale_ms=model.limits()["state_stale_ms"], rows=rows, chart=chart)


def _signed(value, scale, unit, digits=1):
    return f"{value / scale:+.{digits}f} {unit}"


@register_panel
def wheels_panel(model, now):
    if model.state is None:
        return make_panel("wheels", "Wheels", None, note="waiting for robot/state")
    fields = model.state["fields"]
    rows = [("Requested left / right", f"{fields['kr1_asked_l']} / {fields['kr1_asked_r']} mm/s"),
            ("Measured left / right",
             f"{fields['kr1_measured_l']} / {fields['kr1_measured_r']} mm/s")]
    if "duty_left_permille" in fields:
        rows.append(("PWM duty left / right",
                     f"{_signed(fields['duty_left_permille'], 10, '%')} / "
                     f"{_signed(fields['duty_right_permille'], 10, '%')}"))
    else:
        rows.append(("PWM duty left / right", "not reported (old state version)"))
    if "dist_avg_mm" in fields:
        rows.append(("Distance left / right / average",
                     f"{fields['dist_left_mm']} / {fields['dist_right_mm']} / "
                     f"{fields['dist_avg_mm']} mm"))
    else:
        rows.append(("Distance left / right / average", "not reported (old state v1)"))
    rows += [("Encoder counts left / right", f"{fields['count_l']} / {fields['count_r']}"),
             ("Servo", f"{fields['kr1_servo_deg']} deg")]
    return make_panel("wheels", "Wheels", model.state_age(now, "motion_age_ms"), rows=rows,
                      stale_ms=model.limits()["state_stale_ms"],
                      note="distance: signed mm since robot boot, forward is positive. "
                      "duty: forward is positive")


@register_panel
def odometry_panel(model, now):
    if model.state is None:
        return make_panel("odometry", "Odometry (encoder-derived)", None,
                          note="waiting for robot/state")
    fields = model.state["fields"]
    age = model.state_age(now, "motion_age_ms")
    imu = model.reserved["imu"]
    imu_fields = None if imu is None else imu["fields"]
    rows = []
    chart = None
    if "heading_cdeg" in fields:
        heading = fields["heading_cdeg"]
        rate = fields["turn_rate_ddps"]
        wrapped = (heading % 36000) / 100
        rows += [("Heading (left +, since boot)", _signed(heading, 100, "deg", 2)),
                 ("Heading on the dial", f"{wrapped:.2f} deg"),
                 ("Turn rate (left +)", _signed(rate, 10, "deg/s"))]
        chart = {"type": "dial", "heading_deg": wrapped, "turn_rate_dps": rate / 10}
    else:
        rows.append(("Heading and turn rate", "not reported (old state version)"))
    if "dist_avg_mm" in fields:
        rows.append(("Distance average", f"{fields['dist_avg_mm']} mm"))
        rows.append(("Distance left / right",
                     f"{fields['dist_left_mm']} / {fields['dist_right_mm']} mm"))
    else:
        rows.append(("Distance", "not reported (old state version)"))
    rows.append(("Source", "encoder-derived (wheel counts), drifts when a wheel slips"))
    rows.append(("IMU heading (raw)", NOT_IMPLEMENTED if imu_fields is None
                 else imu_fields["heading_raw"]))
    rows.append(("IMU turn rate (raw)", NOT_IMPLEMENTED if imu_fields is None
                 else imu_fields["turn_rate_raw"]))
    return make_panel("odometry", "Odometry (encoder-derived)", age, rows=rows, chart=chart,
                      stale_ms=model.limits()["state_stale_ms"],
                      note="heading = (right - left counts) / 24.05 counts per degree; "
                      "origin is robot boot. IMU rows are for comparison later")


@register_panel
def ultrasonic_panel(model, now):
    if model.sonar is None:
        return make_panel("ultrasonic", "Ultrasonic", None, note="no sonar result yet",
                          paused=model.telemetry_mode() == "idle")
    fields = model.sonar["fields"]
    points = [[angle, entry["mm"], int(now - entry["rx"]), entry["text"]]
              for angle, entry in sorted(model.sonar_profile.items())]
    rows = [("Latest", _sonar_distance(fields)), ("Angle", f"{fields['angle']} deg"),
            ("Pending request", "yes" if fields["pending"] else "no")]
    return make_panel("ultrasonic", "Ultrasonic", model.record_age(now, model.sonar), rows=rows,
                      chart={"type": "profile", "points": points, "max_mm": 511},
                      paused=model.telemetry_mode() == "idle")


def _clear_text(deg, step):
    return "blocked" if deg == robot_wire.BLOCKED_CLEAR_DEG else f"{deg} deg ({step} steps free)"


def _closest_text(fields):
    if fields["too_near"]:
        return "too near (below sensor range)"
    if not fields["closest_valid"] or fields["closest_mm"] == robot_wire.NO_CLOSEST_MM:
        return "none seen"
    angle = fields["closest_angle_deg"]
    side = "ahead" if angle == 0 else ("left" if angle > 0 else "right")
    return f"{fields['closest_mm']} mm at {angle} deg ({side})"


def _seconds(ms):
    return f"{ms / 1000:.2f} s"


def _detour_outcome(fields):
    outcome, payload = fields["outcome"], fields["payload"]
    if outcome == 3:
        return "done, line found"
    if outcome == 4:
        return f"done, no line (search {_name(LINE_SIDE_TEXT, payload)})"
    if outcome == 7:
        return f"FAILED: {_name(DETOUR_FAIL_TEXT, payload)}"
    return "aborted (run ended)"


def _detour_table(model):
    rows = []
    for entry in reversed(model.detours):
        f = entry["fields"]
        offset = f"{f['max_lateral_mm']} mm" if f["lateral_valid"] else "not measured"
        rows.append([f["sequence"], _name(SIDE_TEXT, f["side"]), _detour_outcome(f),
                     _seconds(f["duration_ms"]), f"{f['distance_mm']} mm", offset,
                     f["scan_id"] if f["scan_id"] else "unknown"])
    return {"columns": ["Detour", "Side", "Outcome", "Duration", "Distance", "Max offset",
                        "Scan"], "rows": rows}


@register_panel
def obstacle_panel(model, now):
    if model.obstacle is None and not model.detours:
        return make_panel("obstacle", "Obstacle profile and detours", None, stale_ms=None,
                          note="no completed scan or detour yet")
    rows = []
    chart = None
    age = None
    if model.obstacle is not None:
        fields = model.obstacle["fields"]
        age = model.record_age(now, model.obstacle)
        rows = [("Closest", _closest_text(fields)),
                ("Right clearance", _clear_text(fields["right_clear_deg"],
                                                fields["right_free_step"])),
                ("Left clearance", _clear_text(fields["left_clear_deg"],
                                               fields["left_free_step"])),
                ("Detour decision", _name(OBSTACLE_DECISION_TEXT, fields["decision"])),
                ("Scan ID", fields["scan_id"]),
                ("Scan stage", _name(SCAN_STAGES, fields["scan_stage"])),
                ("Scan attempt", "initial" if fields.get("scan_attempt", 0) == 0
                 else f"retry {fields['scan_attempt']}"),
                ("Scans missed (ID gaps)", model.obstacle_missed),
                ("Profile word", f"0x{fields['profile_word']:04x}")]
        points = [[angle, entry["mm"], int(now - entry["rx"]), entry["text"]]
                  for angle, entry in sorted(model.sonar_profile.items())]
        chart = {"type": "obstacle", "points": points, "max_mm": 511,
                 "closest": None if not fields["closest_valid"] else
                 {"angle": fields["closest_angle_deg"], "mm": fields["closest_mm"]},
                 "right_clear_deg": fields["right_clear_deg"],
                 "left_clear_deg": fields["left_clear_deg"], "decision": fields["decision"]}
    if model.detours:
        detour_age = model.record_age(now, model.detours[-1])
        age = detour_age if age is None else min(age, detour_age)
        rows.append(("Detours missed (sequence gaps)", model.detour_missed))
    return make_panel("obstacle", "Obstacle profile and detours", age, stale_ms=None, rows=rows,
                      chart=chart, table=_detour_table(model) if model.detours else None,
                      note="age = time since the newest scan or detour finished; "
                      "angle: right is negative. Max offset is the sideways distance from "
                      "the heading at detour start")


@register_panel
def events_panel(model, now):
    items = [{"age_ms": int(now - e["rx"]), "text": f"#{e['sequence']} {e['text']}"}
             for e in reversed(model.events)]
    age = None if not model.events else int(now - model.events[-1]["rx"])
    rows = [("Event sequence gaps", model.event_gaps)]
    return make_panel("events", "Events", age, stale_ms=None, rows=rows, items=items,
                      note="" if model.events else "no events yet")


@register_panel
def health_panel(model, now):
    if model.heartbeat is None:
        return make_panel("health", "Health", None,
                          stale_ms=model.limits()["heartbeat_stale_ms"],
                          note="waiting for robot/heartbeat")
    f = model.heartbeat["fields"]
    rows = [("Wi-Fi", "up" if f["wifi_up"] else "down"),
            ("MQTT (robot side)", _name(MQTT_STATES, f["mqtt_state"])),
            ("Reset reason", _name(RESET_REASONS, f["reset_reason"])),
            ("Uptime", f"{f['uptime_ms'] / 1000:.1f} s"),
            ("Wi-Fi poll gap max", f"{f['wifi_poll_gap_max_us'] / 1000:.1f} ms"
             f" ({f['wifi_poll_gap_samples']} polls)"),
            ("Lost: telemetry / event / command",
             f"{f['telemetry_lost']} / {f['event_lost']} / {f['command_lost']}"),
            ("Publish ok / failed", f"{f['publish_ok']} / {f['publish_failed']}"),
            ("Busy reads: line / sonar / motion / vehicle / health",
             f"{f['line_busy']} / {f['sonar_busy']} / {f['motion_busy']} / "
             f"{f['vehicle_busy']} / {f['health_busy']}"),
            ("Unavailable boards (last explicit read)", _unavailable_text(f)),
            ("Unavailable observations (total)", f.get("board_unavailable", "not reported")),
            ("Vehicle overruns", f["vehicle_overruns"]),
            ("Motion late / missed", f"{f['motion_late']} / {f['motion_missed']}"),
            ("CPU load", _cpu_text(f)),
            ("Stack minimum free", _stack_text(f["stack_min_free"])),
            ("Restricted commands", "configured" if f["commands_enabled"] else "not configured"),
            ("Robot health age", f"{f['health_age_ms']} ms")]
    return make_panel("health", "Health", model.record_age(now, model.heartbeat),
                      stale_ms=model.limits()["heartbeat_stale_ms"], rows=rows)


def _yes_no(value):
    return "yes" if value else "no"


@register_panel
def barcode_panel(model, now):
    rows = []
    age = None
    if model.state is not None:
        rows.append(("Raw GP28 sample", model.state["fields"]["raw_bc"]))
        age = model.state_age(now)
    record = model.reserved["barcode"]
    if record is None:
        rows += [(label, NOT_IMPLEMENTED) for label in
                 ("Detected / decoded / unreadable", "Decoded command", "Confidence",
                  "Direction", "Navigation command", "Result", "Command issued / executed")]
    else:
        age = model.record_age(now, record)
        f = record["fields"]
        rows += [("Detected / decoded / unreadable",
                  f"{_yes_no(f['detected'])} / {_yes_no(f['decoded'])} / "
                  f"{_yes_no(f['unreadable'])}"),
                 ("Decoded command", f"code {f['code']} (raw)"),
                 ("Confidence", f"{f['votes']} of {f['reads']} reads agree"),
                 ("Direction", _name(BARCODE_DIRECTIONS, f["direction"])),
                 ("Navigation command", f"{f['nav_command']} (raw)"),
                 ("Result", f"{f['result']} (raw)"),
                 ("Command issued / executed",
                  f"{_yes_no(f['command_issued'])} / {_yes_no(f['executed'])}"),
                 ("Report", f"#{f['sequence']} at robot uptime {f['uptime_ms']} ms")]
    return make_panel("barcode", "Barcode", age, rows=rows,
                      note="" if model.state else "waiting for robot/state")


@register_panel
def imu_panel(model, now):
    snapshot, event = model.reserved["imu"], model.reserved["imu_event"]
    if snapshot is None and event is None:
        rows = [(label, NOT_IMPLEMENTED) for label in
                ("Pitch", "Heading", "Turn rate", "Hump state", "Height", "Peak",
                 "Collision", "Motion event")]
        return make_panel("imu", "IMU", None, placeholder=True, rows=rows,
                          note="not implemented: no data on robot/imu yet")
    rows = []
    if snapshot is None:
        rows += [(label, NOT_IMPLEMENTED) for label in
                 ("Pitch", "Heading", "Turn rate", "Hump state", "Height", "Peak",
                  "Collision")]
    else:
        f = snapshot["fields"]
        rows += [("Pitch (raw)", f["pitch_raw"]), ("Heading (raw)", f["heading_raw"]),
                 ("Turn rate (raw)", f["turn_rate_raw"]),
                 ("Hump state (raw)", f["hump_state"]), ("Height (raw)", f["height_raw"]),
                 ("Peak (raw)", f["peak_raw"]), ("Collision", _yes_no(f["collision"])),
                 ("Sample valid", _yes_no(f["valid"])),
                 ("Sample age or sequence (raw)", f["age_or_seq"])]
    if event is None:
        rows.append(("Motion event", NOT_IMPLEMENTED))
    else:
        f = event["fields"]
        rows.append(("Motion event", f"{_name(IMU_EVENTS, f['event'])} #{f['sequence']} at "
                     f"robot uptime {f['uptime_ms']} ms"))
    newest = max((r for r in (snapshot, event) if r is not None), key=lambda r: r["rx"])
    return make_panel("imu", "IMU", model.record_age(now, newest), rows=rows,
                      note="raw values: units are not fixed yet")


def _fault_text(bits):
    names = [name for mask, name in MOTION_FAULT_BITS if bits & mask]
    unknown = bits & ~sum(mask for mask, _ in MOTION_FAULT_BITS)
    if unknown:
        names.append(f"unknown bits 0x{unknown:02x}")
    return ", ".join(names) if names else "none"


@register_panel
def summary_panel(model, now):
    record = model.summary
    if record is None:
        return make_panel("summary", "Run summary", None, stale_ms=None,
                          note="no run summary yet: it appears when a run ends")
    f = record["fields"]
    implemented = f["barcode_hump_implemented"]
    shell = _name(SHELL_FAULTS, f["shell_fault"])
    if f["shell_fault"]:
        shell += f" (detail {f['shell_fault_detail']})"
    unknown_speed = (not f["speed_error_valid"]
                     or f["max_speed_error_mm_s"] == robot_wire.SPEED_ERROR_NONE)
    rows = [("Run ID", f["run_id"]),
            ("Stop source", _name(STOP_SOURCES, f["stop_source"])),
            ("Stop reason", _name(STOP_REASONS, f["stop_reason"])),
            ("Started (robot uptime)", _seconds(f["start_ms"])),
            ("Duration", _seconds(f["duration_ms"])),
            ("Distance (path length)", f"{f['distance_mm']} mm"),
            ("Line losses", f["line_losses"]), ("Obstacles seen", f["obstacles"]),
            ("Detours started / done / succeeded",
             f"{f['detours_started']} / {f['detours_done']} / {f['detours_succeeded']}"),
            ("Detour time in total", _seconds(f["detour_total_ms"])),
            ("SCAN reverse-and-rescan attempts", f.get("scan_reattempts", "not reported")),
            ("Max speed error", "not measured" if unknown_speed
             else f"{f['max_speed_error_mm_s']} mm/s"),
            ("Motion faults seen", _fault_text(f["motion_fault_bits"])),
            ("Shell fault", shell),
            ("Barcode ok / failed", f"{f['barcode_ok']} / {f['barcode_failed']}" if implemented
             else NOT_IMPLEMENTED),
            ("Hump peak", NOT_IMPLEMENTED if not implemented else
             ("none" if f["hump_peak"] == robot_wire.HUMP_PEAK_NONE else f["hump_peak"]))]
    if f["detour_aborted"]:
        rows.append(("Detour", "a detour was still running when the run ended"))
    notes = ["age = time since this summary arrived"]
    scope = f["boot_session"]
    rows.append(("Boot / session", scope))
    if scope != model.log.sessions.current:
        notes.append("previous boot/session: not the current robot's run summary")
    if record["retained"]:
        notes.append("this is the broker's retained copy: it may be from an earlier run")
    state = model.state
    if (scope == model.log.sessions.current and state is not None and
            now - state["rx"] <= model.limits()["state_stale_ms"]
            and state["fields"]["active"]):
        notes.append("a new run is in progress: this summary is the previous run")
    return make_panel("summary", "Run summary", model.record_age(now, record), stale_ms=None,
                      rows=rows, note="; ".join(notes))


@register_panel
def log_panel(model, now):
    info = model.log_info
    gaps = model.gap_counts()
    last = info["last"]
    rows = [("Boot / session", model.log.sessions.current),
            ("Live rows received", info["live_rows"]),
            ("Refill rows (DUMP) merged", info["refill_rows"]),
            ("Rows lost (index gaps)", gaps["lost_rows"]),
            ("Rows skipped (jumps)", gaps["jumped_rows"]),
            ("Jump frames", info["jumped_frames"]),
            ("Duplicates dropped", gaps["duplicates"])]
    age = None
    if last is not None:
        age = min(int(now - last["rx"]) + last["data_age_ms"], MISSING_AGE_MS)
        rows += [("Data age", f"{age} ms"), ("Rows behind", last["rows_behind"]),
                 ("Generation / last index", f"{last['generation']} / {last['index']}")]
    items = [{"age_ms": None, "text": message} for message in reversed(model.log_messages)]
    return make_panel("log", "RAM log stream", age, rows=rows, items=items,
                      note=("" if last else "no live log rows yet; ") +
                      "receive counts: current session; gaps and duplicates: all sessions",
                      paused=model.telemetry_mode() == "idle")
