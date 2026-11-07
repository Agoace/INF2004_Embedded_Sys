#!/usr/bin/env python3
"""Receive robot binary v1 telemetry as CSV; send token-guarded commands."""
import argparse
import csv
import os
import re
import secrets
import stat
import struct
import sys
import threading
import time

VERSION = 1
SIZES = {"state": 48, "sonar": 32, "event": 16, "heartbeat": 100, "log": 32}
COMMANDS = {"estop": 1, "dump": 2, "calibrate": 3, "start": 4}
STATE_FLAGS = ("line_valid", "line_fresh", "motion_fresh", "vehicle_fresh",
               "line_busy", "motion_busy", "vehicle_busy", "active")
HEARTBEAT_FLAGS = ("wifi_up", "mqtt_connected", "subscribed", "log_dump_active",
                   "commands_enabled", "health_fresh")


def _u16(data, offset):
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data, offset):
    return struct.unpack_from("<I", data, offset)[0]


def _i32(data, offset):
    return struct.unpack_from("<i", data, offset)[0]


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
    for name, offset in (("asked_l", 53), ("asked_r", 63), ("measured_l", 73), ("measured_r", 83)):
        result["kr1_" + name] = field(offset, 10, True)
    return result


def decode(topic, data):
    """Return flat CSV fields. Reject unknown versions, sizes and reserved bits."""
    name = topic.removeprefix("robot/")
    _require(topic == "robot/" + name and name in SIZES, "unknown topic")
    _require(len(data) == SIZES[name], "wrong payload length")
    _require(data[0] == VERSION, "unknown version")
    result = {"version": data[0]}
    if name == "state":
        _require((_u16(data, 18) & 0xF000) == 0 and (_u32(data, 44) >> 20) == 0, "state reserved bits")
        result.update(_flags(data[1], STATE_FLAGS))
        result.update(state=data[2] & 15, module=data[2] >> 4, motion_faults=data[3],
                      uptime_ms=_u32(data, 4), sequence=_u32(data, 8),
                      line_age_ms=_u16(data, 12), motion_age_ms=_u16(data, 14),
                      vehicle_age_ms=_u16(data, 16), raw_bc=_u16(data, 18),
                      count_l=_i32(data, 36), count_r=_i32(data, 40),
                      norm_l=_u32(data, 44) & 1023, norm_c=(_u32(data, 44) >> 10) & 1023)
        result.update(decode_kr1(data[20:36]))
    elif name == "sonar":
        _require((data[1] & 0xF0) == 0 and _u16(data, 26) == 0, "sonar reserved bits")
        result.update(status=data[1] & 3, valid=(data[1] >> 2) & 1, pending=(data[1] >> 3) & 1,
                      angle=_i8(data, 2), ping_angle=_i8(data, 3),
                      sequence=_u32(data, 4), generation=_u32(data, 8), request_id=_u32(data, 12),
                      trigger_ms=_u32(data, 16), timestamp_ms=_u32(data, 20),
                      echo=_u16(data, 24), ping_sequence=_u32(data, 28))
        result.update(echo_status=(result["echo"] >> 9) & 3,
                      echo_mm=(result["echo"] & 511) if ((result["echo"] >> 9) & 3) == 0 else "")
    elif name == "event":
        _require(1 <= data[1] <= 6 and (data[15] & 0xFE) == 0, "event kind or reserved flags")
        result.update(kind=data[1], **{"from": data[2], "to": data[3]}, sequence=_u32(data, 4),
                      uptime_ms=_u32(data, 8), word=_u16(data, 12), detail=data[14], run_relative=data[15] & 1)
    elif name == "heartbeat":
        _require((data[1] & 0xC0) == 0 and data[3] == 0 and _u16(data, 42) == 0 and
                 _u16(data, 98) == 0 and data[2] <= 2 and data[83] <= 3, "heartbeat reserved bits or enum")
        result.update(_flags(data[1], HEARTBEAT_FLAGS))
        result["reset_reason"] = data[2]
        names = ("uptime_ms", "connect_attempts", "telemetry_lost", "event_lost", "command_lost",
                 "sonar_history_missed", "vehicle_overruns", "motion_late", "motion_missed")
        result.update({key: _u32(data, 4 + 4 * i) for i, key in enumerate(names)})
        result["stack_min_free"] = _u16(data, 40)
        names = ("mqtt_hook_max_us", "publish_ok", "publish_failed", "line_busy", "sonar_busy",
                 "motion_busy", "vehicle_busy", "health_busy", "latest_sonar_history_sequence")
        result.update({key: _u32(data, 44 + 4 * i) for i, key in enumerate(names)})
        result.update(health_age_ms=_u16(data, 80), wifi_link_status=_i8(data, 82), mqtt_state=data[83],
                      wifi_poll_gap_max_us=_u32(data, 84), wifi_poll_gap_samples=_u32(data, 88),
                      wifi_start_status=_i32(data, 92), wifi_requested_priority=data[96],
                      wifi_observed_priority=data[97])
    else:
        _require(data[1] <= 4 and _u16(data, 2) == 0, "log kind or reserved flags")
        result.update(kind=data[1], dump_id=_u32(data, 4), generation=_u32(data, 8), index=_u32(data, 12))
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
        _require(re.fullmatch(rb"[0-9a-fA-F]{32}(?:\r?\n)?", value) is not None, "token file must contain 32 hex digits")
        return bytes.fromhex(value.decode("ascii").strip())
    finally:
        os.close(fd)


def csv_fields():
    fields = {"received_unix_s", "topic"}
    for name, size in SIZES.items():
        sample = bytearray(size)
        sample[0] = VERSION
        if name == "event":
            sample[1] = 1
        fields.update(decode("robot/" + name, sample))
    for kind in (1, 2, 3, 4):
        sample = bytearray(32)
        sample[0], sample[1] = VERSION, kind
        if kind == 2:
            sample[16], sample[17] = VERSION, 1
        fields.update(decode("robot/log", sample))
    return ["received_unix_s", "topic", *sorted(fields - {"received_unix_s", "topic"})]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="existing trusted MQTT broker")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--csv", default="-", help="CSV destination; default stdout")
    parser.add_argument("--command", choices=COMMANDS)
    parser.add_argument("--token-file", help="owner-only 0600 file with 32 hex digits")
    parser.add_argument("--request-id", type=lambda value: int(value, 0))
    args = parser.parse_args(argv)
    if args.command and not args.token_file:
        parser.error("--command requires --token-file")
    if args.request_id is not None and not 0 <= args.request_id <= 0xFFFFFFFF:
        parser.error("--request-id must be an unsigned 32-bit integer")
    command = None
    request_id = args.request_id if args.request_id is not None else secrets.randbits(32)
    if args.command:
        try:
            command = pack_command(COMMANDS[args.command], request_id, read_token(args.token_file))
        except (OSError, ValueError):
            parser.error("cannot read valid private token file")
    # Pure decode/tests do not require Paho or a network connection.
    import paho.mqtt.client as mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    connected = threading.Event()
    stream = sys.stdout if args.csv == "-" else open(args.csv, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(stream, fieldnames=csv_fields())
    writer.writeheader()

    def on_connect(mqtt_client, userdata, flags, reason_code, properties):
        del userdata, flags, properties
        if reason_code.is_failure:
            print("broker connection rejected", file=sys.stderr)
            return
        mqtt_client.subscribe([("robot/" + name, 0) for name in SIZES])
        connected.set()

    def on_message(mqtt_client, userdata, message):
        del mqtt_client, userdata
        try:
            fields = decode(message.topic, message.payload)
        except ValueError as error:
            print(f"discarded {message.topic}: {error}", file=sys.stderr)
            return
        writer.writerow({"received_unix_s": f"{time.time():.6f}", "topic": message.topic, **fields})
        stream.flush()

    client.on_connect, client.on_message = on_connect, on_message
    try:
        client.connect(args.host, args.port, 60)
        if command is not None:
            client.loop_start()
            if not connected.wait(10):
                raise TimeoutError("broker connection timeout")
            publication = client.publish("robot/cmd", command, qos=1, retain=False)
            publication.wait_for_publish(timeout=10)
            if not publication.is_published():
                raise TimeoutError("command publication timeout")
            print(f"command published request_id={request_id}; broker ACK is not robot acceptance", file=sys.stderr)
        else:
            client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        client.loop_stop()
        if stream is not sys.stdout:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
