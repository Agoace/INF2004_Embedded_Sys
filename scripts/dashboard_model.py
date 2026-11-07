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
ROBOT_OFFLINE_MS = 3000  # no robot message at all for this long
PENDING_TIMEOUT_MS = 5000  # wait this long for a robot decision on a command
MISSING_AGE_MS = 65535  # robot value for a board with no data
EVENT_LIMIT = 50
PROFILE_LIMIT = 256
TREND_LIMIT = 100
LOG_MESSAGE_LIMIT = 20
CONFIRM_COMMANDS = ("start", "calibrate")

STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "FAULT"}
MODULE_NAMES = {0: "none", 1: "line tracing", 2: "line search", 3: "obstacle scan",
                4: "obstacle detour"}
EVENT_KINDS = {1: "outcome", 2: "module change", 3: "ESTOP or fault", 4: "scan complete",
               5: "run end", 6: "command decision"}
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
               items=None, note="", placeholder=False):
    """Build one panel. age_ms None means no data yet; stale_ms None means never stale."""
    if placeholder:
        stale = True
    elif age_ms is None:
        stale = True
    else:
        stale = stale_ms is not None and age_ms > stale_ms
    return {"id": panel_id, "title": title, "age_ms": age_ms, "stale": stale,
            "rows": [{"label": label, "value": value} for label, value in (rows or [])],
            "chart": chart, "items": items or [], "note": note, "placeholder": placeholder}


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
        self.sonar = None
        self.sonar_profile = {}  # angle -> {"mm", "text", "rx"}
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

    def ingest(self, topic, payload):
        """Decode one message. A malformed message is counted and never changes state."""
        with self.lock:
            now = self.now_ms()
            try:
                if topic == "robot/log":
                    rows = self.log.handle(payload)
                    self._log_rows(rows, now)
                else:
                    self._store(topic, robot_wire.decode(topic, payload), now)
            except ValueError as error:
                self.discarded += 1
                self.last_discard = f"{topic}: {error}"
                return
            self.last_message_ms = now

    def _store(self, topic, fields, now):
        if topic == "robot/state":
            self.state = {"fields": fields, "rx": now}
            self.trend.append(fields["kr1_position"])
        elif topic == "robot/heartbeat":
            self.heartbeat = {"fields": fields, "rx": now}
        elif topic == "robot/sonar":
            self._store_sonar(fields, now)
        elif topic == "robot/event":
            self._store_event(fields, now)

    def _store_sonar(self, fields, now):
        if self.sonar is not None and self.sonar["fields"]["sequence"] == fields["sequence"]:
            return
        self.sonar = {"fields": fields, "rx": now}
        valid = bool(fields["valid"]) and fields["echo_status"] == 0
        if len(self.sonar_profile) >= PROFILE_LIMIT and fields["angle"] not in self.sonar_profile:
            return
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

    def robot_status(self, now):
        if self.last_message_ms is None:
            return {"online": False, "text": "no robot data yet", "age_ms": None}
        age = int(now - self.last_message_ms)
        online = age <= ROBOT_OFFLINE_MS
        return {"online": online, "age_ms": age,
                "text": "robot online" if online else f"robot silent for {age // 1000} s"}

    def _state_problem(self, now):
        """Return why robot/state is not usable for a gated button, or an empty string."""
        if self.state is None:
            return "no state data from the robot"
        age = int(now - self.state["rx"])
        if age > STALE_MS:
            return f"robot state is {age} ms old"
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
                if not self.broker_connected:
                    result["reason"] = "broker not connected: ESTOP cannot be sent now"
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
        if self._pending_open(now):
            return "waiting for the robot decision on the previous command"
        if command == "dump":
            heartbeat = self.heartbeat
            if (heartbeat is not None and now - heartbeat["rx"] <= HEARTBEAT_STALE_MS
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

    def snapshot(self):
        """Return everything the page needs as one JSON-ready dict."""
        with self.lock:
            now = self.now_ms()
            return {"broker": {"connected": self.broker_connected},
                    "robot": self.robot_status(now),
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
    command = model.last_command(now)
    rows.append(("Last command", "none sent" if command is None else
                 f"{command['command']} #{command['request_id']}"))
    if command is not None:
        rows.append(("Command result", command["status"]))
    decision = model.last_decision
    rows.append(("Last robot decision", "none" if decision is None else decision["text"]))
    return make_panel("vehicle", "Vehicle", age, rows=rows,
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
                      rows=rows, chart=chart)


@register_panel
def wheels_panel(model, now):
    if model.state is None:
        return make_panel("wheels", "Wheels", None, note="waiting for robot/state")
    fields = model.state["fields"]
    rows = [("Requested left / right", f"{fields['kr1_asked_l']} / {fields['kr1_asked_r']} mm/s"),
            ("Measured left / right",
             f"{fields['kr1_measured_l']} / {fields['kr1_measured_r']} mm/s"),
            ("Encoder counts left / right", f"{fields['count_l']} / {fields['count_r']}"),
            ("Servo", f"{fields['kr1_servo_deg']} deg")]
    return make_panel("wheels", "Wheels", model.state_age(now, "motion_age_ms"), rows=rows)


@register_panel
def ultrasonic_panel(model, now):
    if model.sonar is None:
        return make_panel("ultrasonic", "Ultrasonic", None, note="no sonar result yet")
    fields = model.sonar["fields"]
    points = [[angle, entry["mm"], int(now - entry["rx"]), entry["text"]]
              for angle, entry in sorted(model.sonar_profile.items())]
    rows = [("Latest", _sonar_distance(fields)), ("Angle", f"{fields['angle']} deg"),
            ("Pending request", "yes" if fields["pending"] else "no")]
    return make_panel("ultrasonic", "Ultrasonic", model.record_age(now, model.sonar), rows=rows,
                      chart={"type": "profile", "points": points, "max_mm": 511})


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
        return make_panel("health", "Health", None, stale_ms=HEARTBEAT_STALE_MS,
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
            ("Vehicle overruns", f["vehicle_overruns"]),
            ("Motion late / missed", f"{f['motion_late']} / {f['motion_missed']}"),
            ("Stack minimum free", _stack_text(f["stack_min_free"])),
            ("Restricted commands", "configured" if f["commands_enabled"] else "not configured"),
            ("Robot health age", f"{f['health_age_ms']} ms")]
    return make_panel("health", "Health", model.record_age(now, model.heartbeat),
                      stale_ms=HEARTBEAT_STALE_MS, rows=rows)


@register_panel
def barcode_panel(model, now):
    if model.state is None:
        return make_panel("barcode", "Barcode", None, note="waiting for robot/state")
    rows = [("Raw GP28 sample", model.state["fields"]["raw_bc"]),
            ("Decoded command", "not implemented yet")]
    return make_panel("barcode", "Barcode", model.state_age(now), rows=rows)


@register_panel
def imu_panel(model, now):
    del model, now
    return make_panel("imu", "IMU", None, placeholder=True,
                      note="placeholder: no IMU data on the wire yet")


@register_panel
def log_panel(model, now):
    info = model.log_info
    gaps = model.gap_counts()
    last = info["last"]
    rows = [("Live rows received", info["live_rows"]),
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
                      note="" if last else "no live log rows yet")
