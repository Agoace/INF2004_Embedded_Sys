#!/usr/bin/env python3
"""Receive robot binary v1 telemetry as CSV; send commands (ESTOP token optional)."""
import argparse
import csv
import os
import secrets
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_wire import (COMMANDS, SIZES, LogStream, csv_fields, decode,  # noqa: E402
                        pack_command, read_token)

CSV_FLUSH_SECONDS = 1.0
SUBSCRIBE_QOS = {"log": 1, "status": 1, "summary": 1}


class CsvSink:
    """Write CSV rows; flush on a time interval, never one flush per row."""

    def __init__(self, stream, fieldnames, interval=CSV_FLUSH_SECONDS, clock=time.monotonic):
        self.stream, self.interval, self.clock = stream, interval, clock
        self.writer = csv.DictWriter(stream, fieldnames=fieldnames)
        self.writer.writeheader()
        self.last_flush = clock()

    def write(self, row):
        self.writer.writerow(row)
        if self.clock() - self.last_flush >= self.interval:
            self.flush()

    def flush(self):
        self.stream.flush()
        self.last_flush = self.clock()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="existing trusted MQTT broker")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--csv", default="-", help="CSV destination; default stdout")
    parser.add_argument("--command", choices=COMMANDS)
    parser.add_argument("--token-file",
                        help="0600 file with 32 hex digits; "
                             "optional only for ESTOP on an unconfigured robot")
    parser.add_argument("--request-id", type=lambda value: int(value, 0))
    args = parser.parse_args(argv)
    if args.command and args.command != "estop" and not args.token_file:
        parser.error("this command requires --token-file")
    if args.request_id is not None and not 0 <= args.request_id <= 0xFFFFFFFF:
        parser.error("--request-id must be an unsigned 32-bit integer")
    command = None
    request_id = args.request_id if args.request_id is not None else secrets.randbits(32)
    if args.command:
        try:
            token = read_token(args.token_file) if args.token_file else bytes(16)
            command = pack_command(COMMANDS[args.command], request_id, token)
        except (OSError, ValueError):
            parser.error("cannot read valid private token file")
    # Pure decode/tests do not require Paho or a network connection.
    import paho.mqtt.client as mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    connected = threading.Event()
    stream = sys.stdout if args.csv == "-" else open(args.csv, "w", newline="", encoding="utf-8")
    sink = CsvSink(stream, csv_fields())
    log_stream = LogStream()

    def on_connect(mqtt_client, userdata, flags, reason_code, properties):
        del userdata, flags, properties
        if reason_code.is_failure:
            print("broker connection rejected", file=sys.stderr)
            return
        # A subscription caps delivery QoS: robot/log (DUMP refill), robot/status and
        # robot/summary are QoS 1.
        mqtt_client.subscribe([("robot/" + name, SUBSCRIBE_QOS.get(name, 0)) for name in SIZES])
        connected.set()

    def on_message(mqtt_client, userdata, message):
        del mqtt_client, userdata
        try:
            if message.topic == "robot/log":
                rows = log_stream.handle(message.payload)
            else:
                rows = [decode(message.topic, message.payload)]
        except ValueError as error:
            print(f"discarded {message.topic}: {error}", file=sys.stderr)
            return
        received = f"{time.time():.6f}"
        for fields in rows:
            sink.write({"received_unix_s": received, "topic": message.topic, **fields})

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
            print(f"command published request_id={request_id}; "
                  "broker ACK is not robot acceptance", file=sys.stderr)
        else:
            client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        client.loop_stop()
        sink.flush()
        for line in log_stream.summary():
            print(line, file=sys.stderr)
        if stream is not sys.stdout:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
