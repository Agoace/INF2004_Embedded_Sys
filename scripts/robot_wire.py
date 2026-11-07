#!/usr/bin/env python3
"""Pure robot binary v1 decoder, log merge store and command packer.

No network and no Paho import: the recorder and the dashboard both use this module.
"""
import os
import re
import stat
import struct
import sys

VERSION = 1
# Recorded topics and their newest payload size. The log has variable batch sizes.
SIZES = {"state": 76, "sonar": 36, "event": 20, "heartbeat": 108, "log": 36,
         "obstacle": 24, "status": 6, "detour": 36, "summary": 48,
         "barcode": 20, "imu": 12}
# State version -> payload size. Version 2 adds distances, version 3 adds duty and heading.
STATE_SIZES = {1: 48, 2: 60, 3: 72, 4: 76}
HEARTBEAT_VERSIONS = (1, 2, 3)
TOPIC_SIZES = {
    "state": STATE_SIZES, "heartbeat": {1: 100, 2: 100, 3: 108},
    "sonar": {1: 32, 2: 36}, "event": {1: 16, 2: 20},
    "obstacle": {1: 20, 2: 24}, "detour": {1: 32, 2: 36},
    "summary": {1: 44, 2: 48}, "status": {1: 2, 2: 6}, "log": {1: 32, 2: 36},
    "barcode": {1: 20}, "imu": {1: 12}}
BOOT_OFFSETS = {"state": (4, 72), "heartbeat": (3, 100), "sonar": (2, 32),
                "event": (2, 16), "obstacle": (2, 20), "detour": (2, 32),
                "summary": (2, 44), "status": (2, 2), "log": (2, 32)}
CPU_UNKNOWN = 0xFFFF
RESERVED_TOPICS = ("barcode", "imu")  # reserved, never published yet
RESERVED_SIZES = {("barcode", 1): 20, ("imu", 1): 12, ("imu", 2): 12}  # (topic, kind) -> size
BARCODE_FLAGS = ("detected", "decoded", "unreadable", "command_issued", "executed")
DETOUR_OUTCOMES = (3, 4, 7, 8)  # DONE_LINE, DONE_NO_LINE, FAILED, ABORTED
SUMMARY_FLAGS = ("barcode_hump_implemented", "speed_error_valid", "detour_aborted")
SPEED_ERROR_NONE = 0xFFFF
HUMP_PEAK_NONE = 0xFFFF
EVENT_JUNCTION = 7  # reserved, not emitted yet
OBSTACLE_DECISIONS = ("none", "right", "left")
OBSTACLE_FLAGS = ("closest_valid", "too_near", "left_blocked", "right_blocked")
NO_CLOSEST_MM = 511
BLOCKED_CLEAR_DEG = 255
COMMANDS = {"estop": 1, "dump": 2, "calibrate": 3, "start": 4}
STATE_FLAGS = ("line_valid", "line_fresh", "motion_fresh", "vehicle_fresh",
               "line_busy", "motion_busy", "vehicle_busy", "active")
HEARTBEAT_FLAGS = ("wifi_up", "mqtt_connected", "subscribed", "log_dump_active",
                   "commands_enabled", "health_fresh")
LOG_ROWS, LOG_EVENTS = 5, 6
LOG_HEADER_SIZE, LOG_RECORD_SIZE, LOG_MAX_COUNT = 32, 16, 12
LOG_FLAGS = ("refill", "new_generation", "jumped")


def _u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data, offset):
    return struct.unpack_from("<i", data, offset)[0]


def _i16(data, offset):
    return struct.unpack_from("<h", data, offset)[0]


def _i8(data, offset):
    return struct.unpack_from("<b", data, offset)[0]


def _flags(value, names):
    return {name: (value >> bit) & 1 for bit, name in enumerate(names)}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def decode_kr1(data):
    """Decode the unchanged 128-bit RAM row, including signed wheel values."""
    _require(len(data) == 16, "KR1 length")
    word = int.from_bytes(data, "little")

    def field(offset, width, signed=False):
        value = (word >> offset) & ((1 << width) - 1)
        return value - (1 << width) if signed and value & (1 << (width - 1)) else value

    result = {"kr1_hex": data.hex(), "kr1_time_ms": field(0, 16) * 20,
              "kr1_state": field(16, 2), "kr1_module": field(18, 3),
              "kr1_raw_l": field(21, 12), "kr1_raw_c": field(33, 12),
              "kr1_position": field(45, 8, True), "kr1_echo": field(93, 16),
              "kr1_servo_deg": field(109, 7, True), "kr1_events": field(116, 8),
              "kr1_flags": field(124, 4)}
    wheel_fields = (("asked_l", 53), ("asked_r", 63), ("measured_l", 73), ("measured_r", 83))
    for name, offset in wheel_fields:
        result["kr1_" + name] = field(offset, 10, True)
    return result


def decode(topic, data):
    """Return flat CSV fields. Reject unknown versions, sizes and reserved bits."""
    name = topic.removeprefix("robot/")
    if topic == "robot/" + name and name in RESERVED_TOPICS:
        return decode_reserved(topic, data)
    _require(topic == "robot/" + name and name in SIZES, "unknown topic")
    if name == "log" and len(data) >= 2 and data[1] in (LOG_ROWS, LOG_EVENTS):
        return parse_log_batch(data)[0]
    _require(len(data) > 0, "wrong payload length")
    _require(data[0] in TOPIC_SIZES[name], "unknown version")
    _require(len(data) == TOPIC_SIZES[name][data[0]], "wrong payload length")
    result = {"version": data[0]}
    version, offset = BOOT_OFFSETS[name]
    if data[0] >= version:
        result["boot_id"] = _u32(data, offset)
    if name == "state":
        _require((_u16(data, 18) & 0xF000) == 0 and (_u32(data, 44) >> 20) == 0,
                 "state reserved bits")
        result.update(_flags(data[1], STATE_FLAGS))
        result.update(state=data[2] & 15, module=data[2] >> 4, motion_faults=data[3],
                      uptime_ms=_u32(data, 4), sequence=_u32(data, 8),
                      line_age_ms=_u16(data, 12), motion_age_ms=_u16(data, 14),
                      vehicle_age_ms=_u16(data, 16), raw_bc=_u16(data, 18),
                      count_l=_i32(data, 36), count_r=_i32(data, 40),
                      norm_l=_u32(data, 44) & 1023, norm_c=(_u32(data, 44) >> 10) & 1023)
        result.update(decode_kr1(data[20:36]))
        if data[0] >= 2:
            result.update(dist_left_mm=_i32(data, 48), dist_right_mm=_i32(data, 52),
                          dist_avg_mm=_i32(data, 56))
        if data[0] >= 3:
            _require(_u16(data, 70) == 0, "state reserved word")
            result.update(duty_left_permille=_i16(data, 60), duty_right_permille=_i16(data, 62),
                          heading_cdeg=_i32(data, 64), turn_rate_ddps=_i16(data, 68))
    elif name == "obstacle":
        result.update(_decode_obstacle(data))
    elif name == "detour":
        result.update(_decode_detour(data))
    elif name == "summary":
        result.update(_decode_summary(data))
    elif name == "status":
        _require(data[1] in (1, 2), "status state")
        result.update(status_state=data[1], robot_online=int(data[1] == 1))
    elif name == "sonar":
        _require((data[1] & 0xF0) == 0 and _u16(data, 26) == 0, "sonar reserved bits")
        result.update(status=data[1] & 3, valid=(data[1] >> 2) & 1,
                      pending=(data[1] >> 3) & 1,
                      angle=_i8(data, 2), ping_angle=_i8(data, 3),
                      sequence=_u32(data, 4), generation=_u32(data, 8),
                      request_id=_u32(data, 12),
                      trigger_ms=_u32(data, 16), timestamp_ms=_u32(data, 20),
                      echo=_u16(data, 24), ping_sequence=_u32(data, 28))
        result.update(echo_status=(result["echo"] >> 9) & 3,
                      echo_mm=(result["echo"] & 511) if ((result["echo"] >> 9) & 3) == 0 else "")
    elif name == "event":
        _require(1 <= data[1] <= EVENT_JUNCTION and (data[15] & 0xFE) == 0,
                 "event kind or reserved flags")
        result.update(kind=data[1], **{"from": data[2], "to": data[3]},
                      sequence=_u32(data, 4), uptime_ms=_u32(data, 8), word=_u16(data, 12),
                      detail=data[14], run_relative=data[15] & 1)
    elif name == "heartbeat":
        unavailable_mask = data[3] if data[0] >= 3 else 0
        _require((data[1] & 0xC0) == 0 and _u16(data, 42) == 0 and
                 (data[3] & (0xE0 if data[0] >= 3 else 0xFF)) == 0 and
                 data[2] <= 2 and data[83] <= 3, "heartbeat reserved bits or enum")
        if data[0] >= 3:
            result.update(board_unavailable_mask=unavailable_mask,
                          board_unavailable=_u32(data, 104))
        cpu = _u16(data, 98)
        if data[0] == 1:
            _require(cpu == 0, "heartbeat reserved bits or enum")
        else:
            _require(cpu <= 1000 or cpu == CPU_UNKNOWN, "heartbeat CPU load range")
            result["cpu_load_permille"] = cpu
        result.update(_flags(data[1], HEARTBEAT_FLAGS))
        result["reset_reason"] = data[2]
        names = ("uptime_ms", "connect_attempts", "telemetry_lost", "event_lost",
                 "command_lost", "sonar_history_missed", "vehicle_overruns", "motion_late",
                 "motion_missed")
        result.update({key: _u32(data, 4 + 4 * i) for i, key in enumerate(names)})
        result["stack_min_free"] = _u16(data, 40)
        names = ("mqtt_hook_max_us", "publish_ok", "publish_failed", "line_busy", "sonar_busy",
                 "motion_busy", "vehicle_busy", "health_busy", "latest_sonar_history_sequence")
        result.update({key: _u32(data, 44 + 4 * i) for i, key in enumerate(names)})
        result.update(health_age_ms=_u16(data, 80), wifi_link_status=_i8(data, 82),
                      mqtt_state=data[83], wifi_poll_gap_max_us=_u32(data, 84),
                      wifi_poll_gap_samples=_u32(data, 88), wifi_start_status=_i32(data, 92),
                      wifi_requested_priority=data[96], wifi_observed_priority=data[97])
    else:
        _require(data[1] <= 4 and _u16(data, 2) == 0, "log kind or reserved flags")
        result.update(kind=data[1], dump_id=_u32(data, 4), generation=_u32(data, 8),
                      index=_u32(data, 12))
        body = data[16:32]
        result["body_hex"] = body.hex()
        if data[1] == 1:
            result.update(decode_kr1(body))
        elif data[1] == 2:
            result.update({"event_" + k: v for k, v in decode("robot/event", body).items()})
        else:
            _require(_u32(body, 12) == 0, "log body reserved word")
            names = {0: ("rows_total", "events_total", "lost_total"),
                     3: ("rows_sent", "events_sent", "lost_total"),
                     4: ("abort_reason", "rows_sent", "events_sent")}[data[1]]
            result.update({key: _u32(body, 4 * i) for i, key in enumerate(names)})
    return result


def _decode_obstacle(data):
    """Decode the 20-byte obstacle profile summary of one completed scan."""
    _require(data[1] & 0xC0 == 0, "obstacle reserved flag bits")
    scan_stage = (data[1] >> 4) & 3
    _require(scan_stage != 3, "obstacle scan stage")
    _require(data[2] <= 2, "obstacle decision")
    closest_mm, right_step, left_step = _u16(data, 12), data[16], data[17]
    _require(closest_mm <= NO_CLOSEST_MM and right_step <= 7 and left_step <= 7,
             "obstacle range")
    for clear in (data[18], data[19]):
        _require(clear <= 35 or clear == BLOCKED_CLEAR_DEG, "obstacle clearance")
    result = _flags(data[1] & 15, OBSTACLE_FLAGS)
    result.update(scan_stage=scan_stage, decision=data[2], closest_angle_deg=_i8(data, 3),
                  scan_id=_u32(data, 4), uptime_ms=_u32(data, 8), closest_mm=closest_mm,
                  profile_word=_u16(data, 14),
                  right_free_step=right_step, left_free_step=left_step,
                  right_clear_deg=data[18], left_clear_deg=data[19])
    return result


def _decode_detour(data):
    """Decode the 32-byte report of one finished detour."""
    _require(data[1] in DETOUR_OUTCOMES, "detour outcome")
    _require(data[2] <= 2, "detour side")
    _require(data[3] & 0xFE == 0, "detour reserved flags")
    return {"outcome": data[1], "side": data[2], "lateral_valid": data[3] & 1,
            "sequence": _u32(data, 4), "start_ms": _u32(data, 8), "end_ms": _u32(data, 12),
            "duration_ms": _u32(data, 16), "distance_mm": _u32(data, 20),
            "max_lateral_mm": _u16(data, 24), "payload": _u16(data, 26),
            "scan_id": _u32(data, 28)}


def _decode_summary(data):
    """Decode the 44-byte summary of one finished run."""
    _require(data[1] & ~7 == 0, "summary reserved flags")
    _require(data[2] <= 4, "summary stop source")
    _require(data[3] <= 12, "summary stop reason")
    result = _flags(data[1], SUMMARY_FLAGS)
    result.update(stop_source=data[2], stop_reason=data[3], run_id=_u32(data, 4),
                  start_ms=_u32(data, 8), duration_ms=_u32(data, 12),
                  distance_mm=_u32(data, 16), line_losses=_u16(data, 20),
                  obstacles=_u16(data, 22), detours_done=_u16(data, 24),
                  detours_succeeded=_u16(data, 26), detour_total_ms=_u32(data, 28),
                  detours_started=_u16(data, 32), max_speed_error_mm_s=_u16(data, 34),
                  motion_fault_bits=data[36], shell_fault=data[37],
                  shell_fault_detail=_u16(data, 38), barcode_ok=data[40],
                  barcode_failed=data[41], hump_peak=_u16(data, 42))
    return result


def decode_reserved(topic, data):
    """Decode a reserved topic (robot/barcode or robot/imu).

    A frame with an unknown version or kind is ignored (known 0): the client shows
    "not implemented". A known version and kind must have the right size and zero
    reserved bits, or the frame is rejected.
    """
    name = topic.removeprefix("robot/")
    _require(len(data) >= 2, "reserved topic header")
    result = {"version": data[0], "kind": data[1], "length": len(data), "known": 0}
    size = RESERVED_SIZES.get((name, data[1])) if data[0] == 1 else None
    if size is None:
        return result
    _require(len(data) == size, "wrong payload length")
    result["known"] = 1
    if name == "barcode":
        result.update(_decode_barcode(data))
    elif data[1] == 1:
        result.update(_decode_imu_snapshot(data))
    else:
        _require(data[3] == 0, "imu reserved byte")
        _require(1 <= data[2] <= 6, "imu event")
        result.update(event=data[2], sequence=_u32(data, 4), uptime_ms=_u32(data, 8))
    return result


def _decode_barcode(data):
    _require(data[2] & ~0x1F == 0, "barcode reserved flags")
    _require(data[3] <= 7 and data[6] <= 2 and _u16(data, 10) == 0, "barcode range")
    result = _flags(data[2], BARCODE_FLAGS)
    result.update(code=data[3], votes=data[4], reads=data[5], direction=data[6],
                  nav_command=data[7], result=_i16(data, 8), sequence=_u32(data, 12),
                  uptime_ms=_u32(data, 16))
    return result


def _decode_imu_snapshot(data):
    _require(_u16(data, 2) == 0, "imu reserved bytes")
    word1, word2 = _u32(data, 4), _u32(data, 8)
    _require(word1 >> 28 == 0, "imu reserved bits")
    return {"pitch_raw": word1 & 255, "heading_raw": (word1 >> 8) & 255,
            "turn_rate_raw": (word1 >> 16) & 255, "hump_state": (word1 >> 24) & 3,
            "collision": (word1 >> 26) & 1, "valid": (word1 >> 27) & 1,
            "height_raw": word2 & 255, "peak_raw": (word2 >> 8) & 255,
            "age_or_seq": word2 >> 16}


def parse_log_batch(data):
    """Decode a ROWS/EVENTS batch frame to (header, records); reject any malformed frame.

    Each record is a flat dict with its index; event fields use the event_ prefix.
    """
    _require(len(data) >= LOG_HEADER_SIZE, "log batch shorter than header")
    _require(data[0] in (1, 2), "unknown version")
    header_size = 36 if data[0] == 2 else LOG_HEADER_SIZE
    _require(len(data) >= header_size, "log batch shorter than header")
    kind, count, flags = data[1], data[2], data[3]
    _require(kind in (LOG_ROWS, LOG_EVENTS), "log batch kind")
    _require(1 <= count <= LOG_MAX_COUNT, "log batch count")
    _require(len(data) == header_size + LOG_RECORD_SIZE * count, "log batch length")
    _require(flags & ~7 == 0, "log batch unknown flags")
    header = {"version": data[0], "kind": kind, "count": count, "flags": flags,
              "stream_id": _u32(data, 4), "generation": _u32(data, 8),
              "first_index": _u32(data, 12), "publish_ms": _u32(data, 16),
              "newest_ms": _u32(data, 20), "log_rows": _u32(data, 24),
              "skipped_before": _u32(data, 28)}
    if data[0] == 2:
        header["boot_id"] = _u32(data, 32)
    header.update(_flags(flags, LOG_FLAGS))
    if header["refill"]:
        _require(not header["new_generation"] and not header["jumped"],
                 "refill batch with live flags")
        _require(header["newest_ms"] == 0 and header["skipped_before"] == 0,
                 "refill batch with live-only fields")
    else:
        _require(header["stream_id"] == 0, "live batch with stream ID")
    records = []
    for i in range(count):
        body = bytes(data[header_size + LOG_RECORD_SIZE * i:][:LOG_RECORD_SIZE])
        record = {"index": header["first_index"] + i, "body_hex": body.hex()}
        if kind == LOG_ROWS:
            record.update(decode_kr1(body))
        else:
            record.update({"event_" + k: v for k, v in decode("robot/event", body).items()})
        records.append(record)
    return header, records


def _ranges(indexes):
    """Return sorted indexes as inclusive (start, end) ranges."""
    result = []
    for index in sorted(indexes):
        if result and index == result[-1][1] + 1:
            result[-1][1] = index
        else:
            result.append([index, index])
    return [tuple(item) for item in result]


def _format_ranges(ranges):
    return ", ".join(f"{a}" if a == b else f"{a}-{b}" for a, b in ranges)


class _LogSession:
    """Merge live and DUMP records within one boot/session, and track gaps.

    handle() returns the CSV rows to write: each new record once, and every control
    frame. Duplicates (QoS 1 retries, live and refill overlap) are dropped.
    """

    def __init__(self, report=None):
        self.report = report or (lambda message: print(message, file=sys.stderr))
        self.seen = {}  # (generation, kind) -> set of indexes
        self.duplicates = 0
        self.live_next = {}  # generation -> next live row index, None before the first frame
        self.gaps = []  # dicts: type "lost" or "jumped", generation, start, end (exclusive)
        self.old_generation_skipped = 0
        self.dumps = {}  # dump ID -> dict

    def handle(self, data):
        if len(data) >= 2 and data[1] in (LOG_ROWS, LOG_EVENTS):
            header, records = parse_log_batch(data)
            return self._batch(header, records)
        fields = decode("robot/log", data)
        kind = fields["kind"]
        if kind in (1, 2):
            record = {k: v for k, v in fields.items() if k not in ("version", "kind")}
            row = dict(fields, source="refill", stream_id=fields["dump_id"])
            return [row] if self._store(fields["generation"], kind + 4, record["index"]) else []
        dump = self.dumps.get(fields["dump_id"])
        if kind == 0:
            self.dumps[fields["dump_id"]] = {
                "generation": fields["generation"], "rows_total": fields["rows_total"],
                "events_total": fields["events_total"], "state": "open"}
        elif dump is None:
            self.report(f"dump {fields['dump_id']}: no BEGIN seen before kind {kind}")
        elif kind == 3 and dump["state"] == "open":
            self._finish(fields["dump_id"], dump)
        elif kind == 4 and dump["state"] == "open":
            dump["state"] = "aborted"
            self.report(f"dump {fields['dump_id']}: aborted, reason {fields['abort_reason']}")
        return [fields]

    def _store(self, generation, kind, index):
        indexes = self.seen.setdefault((generation, kind), set())
        if index in indexes:
            self.duplicates += 1
            return False
        indexes.add(index)
        return True

    def _batch(self, header, records):
        if not header["refill"] and header["kind"] == LOG_ROWS:
            self._track_live(header)
        age = rows_behind = ""
        if not header["refill"]:
            age = (header["publish_ms"] - header["newest_ms"]) & 0xFFFFFFFF
            rows_behind = header["log_rows"] - (header["first_index"] + header["count"])
        rows = []
        for i, record in enumerate(records):
            if not self._store(header["generation"], header["kind"], record["index"]):
                continue
            rows.append({**header, **record, "source": "refill" if header["refill"] else "live",
                         "data_age_ms": age, "rows_behind": rows_behind,
                         "skipped_before": header["skipped_before"] if i == 0 else 0})
        return rows

    def _gap(self, kind, generation, start, end):
        if end > start:
            self.gaps.append({"type": kind, "generation": generation, "start": start, "end": end})
            self.report(f"live gap ({kind}): generation {generation} rows {start}-{end - 1}"
                        f" ({end - start} rows)")

    def _track_live(self, header):
        generation, first = header["generation"], header["first_index"]
        skipped = header["skipped_before"]
        if header["new_generation"]:
            # For this flag skipped_before counts unpublished rows of the old generation.
            self.old_generation_skipped += skipped
            skipped = first if header["jumped"] else 0
            self.live_next.setdefault(generation, 0)
        elif generation not in self.live_next:
            # An earlier generation was seen: the flagged first frames were lost.
            self.live_next[generation] = 0 if self.live_next else None
        expected = self.live_next[generation]
        if expected is None:
            self._gap("jumped", generation, max(0, first - skipped), first)
            expected = first
        elif first > expected:
            jump_start = max(expected, first - skipped)
            self._gap("lost", generation, expected, jump_start)
            self._gap("jumped", generation, jump_start, first)
        self.live_next[generation] = max(expected, first + header["count"])

    def _missing(self, generation, kind, total):
        indexes = self.seen.get((generation, kind), set())
        return _ranges(set(range(total)) - indexes) if total else []

    def _finish(self, dump_id, dump):
        rows = self._missing(dump["generation"], LOG_ROWS, dump["rows_total"])
        events = self._missing(dump["generation"], LOG_EVENTS, dump["events_total"])
        dump["state"] = "incomplete" if rows or events else "complete"
        if dump["state"] == "complete":
            self.report(f"dump {dump_id}: complete, {dump['rows_total']} rows,"
                        f" {dump['events_total']} events")
        else:
            self.report(f"dump {dump_id}: missing rows [{_format_ranges(rows)}]"
                        f" events [{_format_ranges(events)}]")

    def summary(self):
        """Return final report lines; empty when no log traffic was seen."""
        if not (self.seen or self.dumps or self.gaps):
            return []
        rows = sum(len(v) for (_, kind), v in self.seen.items() if kind == LOG_ROWS)
        events = sum(len(v) for (_, kind), v in self.seen.items() if kind == LOG_EVENTS)
        lines = [f"log summary: {rows} rows, {events} events, "
                 f"{self.duplicates} duplicates dropped"]
        for gap in self.gaps:
            absent = sorted(set(range(gap["start"], gap["end"])) -
                            self.seen.get((gap["generation"], LOG_ROWS), set()))
            lines.append(f"live gap ({gap['type']}): generation {gap['generation']} rows "
                         f"{gap['start']}-{gap['end'] - 1}, still missing "
                         f"[{_format_ranges(_ranges(absent))}]")
        if self.old_generation_skipped:
            lines.append(f"old generation rows never published: at least "
                         f"{self.old_generation_skipped}")
        for dump_id, dump in self.dumps.items():
            if dump["state"] == "aborted":
                lines.append(f"dump {dump_id}: aborted")
                continue
            rows_missing = self._missing(dump["generation"], LOG_ROWS, dump["rows_total"])
            events_missing = self._missing(dump["generation"], LOG_EVENTS, dump["events_total"])
            state = "not finished" if dump["state"] == "open" else dump["state"]
            lines.append(f"dump {dump_id}: {state}, missing rows "
                         f"[{_format_ranges(rows_missing)}] events "
                         f"[{_format_ranges(events_missing)}]")
        return lines


class BootSession:
    """Track live boot identity; retained/history messages never move the live boot.

    A local session suffix also separates legacy resets and accidental boot-ID
    reuse. Uptime comparisons are per source, modulo 2**32, never between topics.
    Only state, heartbeat and live log publish times establish an uptime boundary.
    """

    def __init__(self):
        self.current = "legacy:0"
        self.boot_id = None
        self.serial = 0
        self.scopes = {}
        self.active_ids = set()
        self.clocks = {}
        self.offline = False

    def scope_for(self, fields):
        boot = fields.get("boot_id")
        if boot is None:
            return self.current
        if boot not in self.scopes:
            self.scopes[boot] = f"boot:{boot:08x}:0"
        return self.scopes[boot]

    def _start(self, boot, renew=False):
        self.serial += 1
        if boot is None:
            self.current = f"legacy:{self.serial}"
        else:
            if renew:
                self.scopes[boot] = f"boot:{boot:08x}:{self.serial}"
            self.current = self.scopes[boot]
            self.active_ids.add(boot)
        self.boot_id = boot
        self.clocks.clear()
        self.offline = False

    def observe(self, topic, fields, retained=False):
        scope = self.scope_for(fields)
        # Summaries, completed detours and refill rows can refer to old history.
        historical = (topic in ("robot/summary", "robot/detour") or
                      (topic == "robot/log" and (fields.get("refill") or
                                                "publish_ms" not in fields)))
        if retained or historical or fields.get("known") == 0:
            if topic == "robot/status" and (self.boot_id is None or scope == self.current):
                if not fields["robot_online"]:
                    self.offline = True
                elif self.offline:
                    self._start(fields.get("boot_id", self.boot_id), renew=True)
                    return self.current
            return scope
        boot = fields.get("boot_id", self.boot_id)
        if boot is not None and boot != self.boot_id:
            if boot in self.active_ids:
                return scope  # delayed frame of a boot already superseded
            self._start(boot)
        if topic == "robot/status":
            if not fields["robot_online"]:
                self.offline = True
            elif self.offline:
                self._start(boot, renew=True)
            return self.current
        timestamp = None
        if topic in ("robot/state", "robot/heartbeat"):
            timestamp = fields["uptime_ms"]
        elif topic == "robot/log":
            timestamp = fields.get("publish_ms")
        if timestamp is not None:
            previous = self.clocks.get(topic)
            if previous is not None and ((timestamp - previous) & 0xFFFFFFFF) > 0x80000000:
                self._start(boot, renew=True)
            self.clocks[topic] = timestamp
        return self.scope_for(fields)


class LogStream:
    """Keep independent log stores per boot/session, including DUMP cursors and gaps.

    Refill can repair a gap only in the same boot. Every returned CSV row carries
    boot_session; explicit boot_id is inherited from the outer log frame.
    """

    def __init__(self, report=None):
        self.report = report or (lambda message: print(message, file=sys.stderr))
        self.sessions = BootSession()
        self.stores = {}
        self.boot_ids = {}

    def observe(self, topic, fields, retained=False):
        return self.sessions.observe(topic, fields, retained)

    def _session(self, fields):
        scope = self.observe("robot/log", fields)
        if scope not in self.stores:
            prefix = "" if scope == "legacy:0" else f"[{scope}] "
            self.stores[scope] = _LogSession(report=lambda msg: self.report(prefix + msg))
            self.boot_ids[scope] = fields.get("boot_id")
        return scope, self.stores[scope]

    def handle(self, data):
        if len(data) >= 2 and data[1] in (LOG_ROWS, LOG_EVENTS):
            return self._batch(*parse_log_batch(data))
        fields = decode("robot/log", data)
        scope, store = self._session(fields)
        return [dict(row, boot_session=scope) for row in store.handle(data)]

    def _batch(self, header, records):
        scope, store = self._session(header)
        return [dict(row, boot_session=scope) for row in store._batch(header, records)]

    @property
    def seen(self):
        return {(scope, generation, kind): indexes
                for scope, store in self.stores.items()
                for (generation, kind), indexes in store.seen.items()}

    @property
    def live_next(self):
        return {(scope, generation): index for scope, store in self.stores.items()
                for generation, index in store.live_next.items()}

    @property
    def gaps(self):
        return [dict(gap, boot_session=scope, boot_id=self.boot_ids[scope])
                for scope, store in self.stores.items() for gap in store.gaps]

    @property
    def dumps(self):
        return {(scope, dump_id): dump for scope, store in self.stores.items()
                for dump_id, dump in store.dumps.items()}

    @property
    def duplicates(self):
        return sum(store.duplicates for store in self.stores.values())

    @property
    def old_generation_skipped(self):
        return sum(store.old_generation_skipped for store in self.stores.values())

    def summary(self):
        if not self.stores:
            return []
        rows = sum(len(v) for (_, _, kind), v in self.seen.items() if kind == LOG_ROWS)
        events = sum(len(v) for (_, _, kind), v in self.seen.items() if kind == LOG_EVENTS)
        lines = [f"log summary: {rows} rows, {events} events, "
                 f"{self.duplicates} duplicates dropped"]
        for scope, store in self.stores.items():
            prefix = "" if scope == "legacy:0" else f"[{scope}] "
            lines.extend(prefix + line for line in store.summary()[1:])
        return lines


def pack_command(opcode, request_id, token):
    _require(opcode in COMMANDS.values(), "unknown command")
    _require(0 <= request_id <= 0xFFFFFFFF, "request ID range")
    _require(len(token) == 16, "token length")
    return struct.pack("<BBI", VERSION, opcode, request_id) + token


def read_token(path):
    """Read one owner-only regular file; never include token bytes in errors."""
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(fd)
        _require(stat.S_ISREG(info.st_mode) and stat.S_IMODE(info.st_mode) == 0o600 and
                 info.st_uid == os.getuid(), "token file must be owned by you with mode 0600")
        value = os.read(fd, 35)
        _require(re.fullmatch(rb"[0-9a-fA-F]{32}(?:\r?\n)?", value) is not None,
                 "token file must contain 32 hex digits")
        return bytes.fromhex(value.decode("ascii").strip())
    finally:
        os.close(fd)


def csv_fields():
    fields = {"received_unix_s", "topic", "boot_session"}
    for name, versions in TOPIC_SIZES.items():
        for version, size in versions.items():
            sample = bytearray(size)
            sample[0] = version
            if name in ("event", "status", "barcode", "imu"):
                sample[1] = 1
            elif name == "detour":
                sample[1] = 3
            fields.update(decode("robot/" + name, sample))
    # IMU motion events have different fields from snapshots.
    sample = bytearray(12)
    sample[0], sample[1], sample[2] = 1, 2, 1
    fields.update(decode("robot/imu", sample))
    for kind in (1, 2, 3, 4):
        sample = bytearray(32)
        sample[0], sample[1] = VERSION, kind
        if kind == 2:
            sample[16], sample[17] = VERSION, 1
        fields.update(decode("robot/log", sample))
    for kind in (LOG_ROWS, LOG_EVENTS):
        sample = bytearray(LOG_HEADER_SIZE + LOG_RECORD_SIZE)
        sample[0], sample[1], sample[2] = VERSION, kind, 1
        if kind == LOG_EVENTS:
            sample[32], sample[33] = VERSION, 1
        header, records = parse_log_batch(sample)
        stream = LogStream(report=lambda message: None)
        fields.update(stream._batch(header, records)[0])
    return ["received_unix_s", "topic", *sorted(fields - {"received_unix_s", "topic"})]
