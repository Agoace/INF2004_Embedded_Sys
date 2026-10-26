"""Live MQTT bench tool. Every received command result is diagnostic only."""
import argparse
import json
import math
import os
import re
import secrets
import threading
import time

import buddy1_command as command_codec
import buddy1_telemetry as telemetry_codec
from buddy1_telemetry import decode_status

OUTCOMES = ("BENCH_RECORDED", "DUPLICATE", "CONFLICT", "OLD", "WRONG_SESSION",
            "MALFORMED", "BUSY", "NOT_READY")


def decode_result(data):
    if not data or data[0] >> 6 != 1 or data[0] & 7:
        raise ValueError("invalid bench result header")
    code = (data[0] >> 3) & 7
    if code == 5:
        if len(data) != 1:
            raise ValueError("malformed-frame result has no command echo")
        return dict(outcome=OUTCOMES[code])
    if len(data) != 8:
        raise ValueError("bench result requires a seven-byte command echo")
    opcode, sequence, session = command_codec.decode(data[1:])
    return dict(outcome=OUTCOMES[code], opcode=opcode, sequence=sequence, session=session)


class LiveSession:
    """A retained online message alone cannot establish a live command session."""
    def __init__(self):
        self.session = None
        self.observed = 0.0

    def observe(self, status, retained, heartbeat, now):
        if (not status["online"] or not status["ready"]) and status["session"] == self.session:
            self.session = None
        if heartbeat and not retained and status["online"] and status["ready"]:
            self.session, self.observed = status["session"], now

    def current(self, now):
        if self.session is None or not 0 <= now - self.observed <= 3:
            raise ValueError("no fresh command-ready heartbeat")
        return self.session


class BenchClient:
    def __init__(self, args, mqtt):
        self.args, self.mqtt = args, mqtt
        self.prefix = f"robot/{args.robot}/"
        self.lock = threading.Lock()
        self.live = LiveSession()
        self.connected = self.subscribed = False
        self.result = self.expected = None
        self.waiting_raw = False
        self.frames = self.gaps = self.duplicates = self.bad_frames = 0
        self.ignored = 0
        self.last_frame = self.last_status = None
        self.last_display = 0.0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="buddy1-tool-" + secrets.token_hex(4),
                                  clean_session=True, reconnect_on_failure=args.mode == "monitor")
        self.client.reconnect_delay_set(1, 5)
        self.client.connect_timeout = 5
        if args.username:
            self.client.username_pw_set(args.username, os.environ.get("BUDDY1_MQTT_PASSWORD"))
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, reason, properties):
        del userdata, flags, properties
        with self.lock:
            self.connected = not reason.is_failure
            self.subscribed = False
            self.live = LiveSession()
        if reason.is_failure:
            print(json.dumps({"broker_connection": str(reason)}), flush=True)
            return
        client.subscribe([(self.prefix + name, 1 if name in ("status", "event") else 0)
                          for name in ("status", "heartbeat", "event", "telemetry")])

    def on_disconnect(self, client, userdata, flags, reason, properties):
        del client, userdata, flags, reason, properties
        with self.lock:
            self.connected = self.subscribed = False
            self.live = LiveSession()
        print(json.dumps({"broker_connected": False}), flush=True)

    def on_subscribe(self, client, userdata, mid, reasons, properties):
        del client, userdata, mid, properties
        with self.lock:
            self.subscribed = not any(reason.is_failure for reason in reasons)

    def on_message(self, client, userdata, message):
        del client, userdata
        now = time.monotonic()
        try:
            suffix = message.topic.removeprefix(self.prefix)
            if suffix in ("status", "heartbeat"):
                status = decode_status(message.payload)
                with self.lock:
                    self.live.observe(status, message.retain, suffix == "heartbeat", now)
                if suffix == "status" or status != self.last_status:
                    print(json.dumps({suffix: status, "retained": bool(message.retain)}), flush=True)
                self.last_status = status
            elif suffix == "event":
                result = decode_result(message.payload)
                print(json.dumps({"bench_result": result}), flush=True)
                with self.lock:
                    identity = (result.get("opcode"), result.get("sequence"), result.get("session"))
                    if (self.expected is not None and identity == self.expected) or self.waiting_raw:
                        self.result = result
            elif suffix == "telemetry":
                if message.retain:
                    self.ignored += 1
                    return
                frame = telemetry_codec.decode(message.payload)
                if self.last_frame is not None and self.last_frame[0] == frame["session"]:
                    delta = (frame["sequence"] - self.last_frame[1]) & 65535
                    if delta == 0:
                        self.duplicates += 1
                        return
                    if delta < 32768:
                        self.gaps += delta - 1
                    else:
                        self.ignored += 1
                        return
                self.last_frame = (frame["session"], frame["sequence"])
                self.frames += 1
                if self.args.mode == "monitor" and now - self.last_display >= self.args.display_period:
                    label = "sensor_telemetry" if self.args.source == "sensors" else "simulated_telemetry"
                    print(json.dumps({label: telemetry_codec.readable(frame),
                                      "received": self.frames, "sequence_gaps": self.gaps}), flush=True)
                    self.last_display = now
        except (ValueError, TypeError) as error:
            self.bad_frames += 1
            print(json.dumps({"invalid_message": str(error)}), flush=True)

    def send(self):
        deadline = time.monotonic() + self.args.timeout
        while time.monotonic() < deadline:
            with self.lock:
                try:
                    token = self.live.current(time.monotonic()) if self.subscribed else None
                except ValueError:
                    token = None
            if token is not None:
                break
            time.sleep(0.05)
        else:
            raise ValueError("no live, command-ready Pico heartbeat; check firmware/topics/network")
        if self.args.mode == "send":
            token = token if self.args.session is None else self.args.session
            opcode = command_codec.COMMANDS.index(self.args.command)
            payload = command_codec.encode(opcode, self.args.sequence, token)
            expected = (opcode, self.args.sequence, token)
        else:
            payload, expected = bytes.fromhex(self.args.hex_payload), None
        with self.lock:
            self.live.current(time.monotonic())
            if not self.connected:
                raise ValueError("broker disconnected before sending")
            self.expected, self.waiting_raw, self.result = expected, self.args.mode == "raw", None
        info = self.client.publish(self.prefix + "cmd", payload, qos=1, retain=False)
        if info.rc != self.mqtt.MQTT_ERR_SUCCESS:
            raise ValueError(f"publish failed: {info.rc}")
        deadline = time.monotonic() + self.args.timeout
        while time.monotonic() < deadline:
            with self.lock:
                result, connected = self.result, self.connected
            if result is not None:
                return 0 if result["outcome"] in ("BENCH_RECORDED", "DUPLICATE") else 2
            if not connected:
                break
            time.sleep(0.05)
        raise ValueError("no matching bench result; outcome unknown (no robot execution is implied)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot", help="Default: aai01, or aai01-sensors with --source sensors")
    parser.add_argument("--source", choices=("simulated", "sensors"), default="simulated")
    parser.add_argument("--username", default=os.environ.get("BUDDY1_MQTT_USERNAME"))
    parser.add_argument("--timeout", type=float, default=10)
    sub = parser.add_subparsers(dest="mode", required=True)
    monitor = sub.add_parser("monitor")
    monitor.add_argument("--duration", type=float, default=0, help="Seconds; zero means until Ctrl+C")
    monitor.add_argument("--display-period", type=float, default=1)
    sender = sub.add_parser("send", help="Send a diagnostic-only bench command")
    sender.add_argument("command", choices=command_codec.COMMANDS)
    sender.add_argument("--sequence", type=lambda value: int(value, 0), required=True)
    sender.add_argument("--session", type=lambda value: int(value, 0), help="Override to test wrong sessions")
    raw = sub.add_parser("raw", help="Send malformed hexadecimal test data, non-retained")
    raw.add_argument("hex_payload")
    args = parser.parse_args()
    args.robot = args.robot or ("aai01-sensors" if args.source == "sensors" else "aai01")
    if (not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", args.robot) or not 1 <= args.port <= 65535 or
            not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("invalid robot, port or timeout")
    if args.mode == "monitor" and (not math.isfinite(args.duration) or not math.isfinite(args.display_period) or
                                   args.duration < 0 or args.display_period <= 0):
        parser.error("duration must be nonnegative and display period positive")
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.error("install scripts/buddy1-requirements.txt in the Buddy 1 virtual environment")
    bench = BenchClient(args, mqtt)
    try:
        if args.mode == "monitor":
            bench.client.connect_async(args.host, args.port, 10)
        else:
            bench.client.connect(args.host, args.port, 10)
        bench.client.loop_start()
        if args.mode != "monitor":
            return bench.send()
        start = time.monotonic()
        while args.duration == 0 or time.monotonic() - start < args.duration:
            time.sleep(0.1)
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, OSError) as error:
        print(json.dumps({"error": str(error)}), flush=True)
        return 1
    finally:
        bench.client.disconnect()
        bench.client.loop_stop()
        print(json.dumps({"summary": {"frames": bench.frames, "sequence_gaps": bench.gaps,
                                     "duplicates": bench.duplicates, "ignored": bench.ignored,
                                     "invalid": bench.bad_frames}}), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
