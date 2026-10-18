#!/usr/bin/env python3
"""Serve a read-only telemetry dashboard on localhost for Tailscale Serve.

The HTTP server binds to 127.0.0.1 only. It does not provide motor controls;
GP20 and GP21 remain the only physical controls. Tailscale Serve may proxy the
local dashboard to authorised tailnet devices without exposing the HTTP server
on the LAN or Internet.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import http.server
import json
import socket
import threading
import time
from urllib.parse import urlsplit

# Import the receiver's parser. There must be one schema validation
# implementation for the UDP receiver and this dashboard.
import telemetry_receiver

HTTP_BIND_ADDRESS = "127.0.0.1"
DEFAULT_UDP_BIND = "0.0.0.0"
DEFAULT_UDP_PORT = 4242
DEFAULT_HTTP_PORT = 8080
HISTORY_LIMIT = 300
STALE_SECONDS = telemetry_receiver.STALE_SECONDS

HTML_PAGE = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pico W instrument dashboard</title>
<style>
:root { color-scheme: light dark; font-family: sans-serif; }
body { margin: 1rem auto; max-width: 1100px; padding: 0 1rem; }
h1 { margin-bottom: .25rem; }
.subtitle { margin-top: 0; color: #666; }
.panel { border: 1px solid #888; border-radius: .5rem; margin: .8rem 0; padding: .8rem; }
.grid { display: grid; gap: .6rem; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); }
.metric { border: 1px solid #aaa; border-radius: .35rem; padding: .6rem; }
.label { display: block; font-size: .85rem; opacity: .75; }
.value { display: block; font-size: 1.35rem; font-variant-numeric: tabular-nums; margin-top: .2rem; }
.good { color: #087f23; }
.bad { color: #b00020; }
.notice { font-size: 1.1rem; font-weight: bold; }
.sparkline { height: 42px; width: 100%; }
.sparkline polyline { fill: none; stroke: currentColor; stroke-width: 2; }
.history-row { align-items: center; display: grid; gap: .7rem; grid-template-columns: 180px 1fr 220px; margin: .4rem 0; }
.history-summary { font-variant-numeric: tabular-nums; white-space: nowrap; }
@media (max-width: 700px) { .history-row { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<h1>Pico W instrument dashboard</h1>
<p class="subtitle">Read-only. GP20 and GP21 remain the physical controls.</p>
<section class="panel">
<h2>Connection</h2>
<div class="grid">
<div class="metric"><span class="label">Status</span><strong id="connection-status" class="value">starting</strong></div>
<div class="metric"><span class="label">Last packet</span><strong id="last-packet" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Device</span><strong id="device" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Session</span><strong id="session" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Sequence</span><strong id="sequence" class="value">not applicable</strong></div>
</div>
<p id="session-note">Waiting for a telemetry session.</p>
</section>
<section class="panel">
<h2>Run status</h2>
<p id="terminal-notice" class="notice">No terminal notice received.</p>
</section>
<section class="panel">
<h2>Live readings</h2>
<div class="grid">
<div class="metric"><span class="label">Motor 1 speed</span><strong id="motor_1_speed_mrpm" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Motor 2 speed</span><strong id="motor_2_speed_mrpm" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Motor 1 duty</span><strong id="motor_1_duty_pct" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Motor 2 duty</span><strong id="motor_2_duty_pct" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Motor 1 direction</span><strong id="motor_1_direction" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Motor 2 direction</span><strong id="motor_2_direction" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Obstacle range</span><strong id="range_mm" class="value">not applicable</strong></div>
<div class="metric"><span class="label">IR reading</span><strong id="ir_mv" class="value">not applicable</strong></div>
<div class="metric"><span class="label">State</span><strong id="state" class="value">not applicable</strong></div>
<div class="metric"><span class="label">Progress</span><strong id="progress_pct" class="value">not applicable</strong></div>
</div>
</section>
<section class="panel">
<h2>Packet health</h2>
<div class="grid">
<div class="metric"><span class="label">Datagrams received</span><strong id="packets-received" class="value">0</strong></div>
<div class="metric"><span class="label">Malformed packets</span><strong id="malformed-packets" class="value">0</strong></div>
<div class="metric"><span class="label">Missing sequence numbers</span><strong id="sequence-gaps" class="value">0</strong></div>
</div>
</section>
<section class="panel">
<h2>Recent numeric history</h2>
<p>Up to 300 samples per field. History is cleared when the Pico session changes.</p>
<div id="history"></div>
</section>
<script>
(function () {
  "use strict";
  var fields = [
    ["motor_1_speed_mrpm", "Motor 1 speed", " RPM", function (value) { return value / 1000; }],
    ["motor_2_speed_mrpm", "Motor 2 speed", " RPM", function (value) { return value / 1000; }],
    ["motor_1_duty_pct", "Motor 1 duty", " %", null],
    ["motor_2_duty_pct", "Motor 2 duty", " %", null],
    ["range_mm", "Obstacle range", " mm", null],
    ["ir_mv", "IR reading", " mV", null],
    ["progress_pct", "Progress", " %", null]
  ];

  function text(id, value) {
    var element = document.getElementById(id);
    if (element) { element.textContent = value; }
  }

  function valueText(readings, key, suffix, convert) {
    if (!Object.prototype.hasOwnProperty.call(readings, key)) {
      return "not applicable";
    }
    var value = readings[key];
    if (convert) { value = convert(value); }
    return String(value) + suffix;
  }

  function renderHistory(history) {
    var root = document.getElementById("history");
    root.replaceChildren();
    fields.forEach(function (definition) {
      var key = definition[0];
      var samples = history[key] || [];
      var row = document.createElement("div");
      row.className = "history-row";
      var label = document.createElement("strong");
      label.textContent = definition[1];
      var svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
      svg.setAttribute("class", "sparkline");
      svg.setAttribute("viewBox", "0 0 300 42");
      svg.setAttribute("role", "img");
      svg.setAttribute("aria-label", definition[1] + " history");
      var summary = document.createElement("span");
      summary.className = "history-summary";
      if (samples.length === 0) {
        summary.textContent = "not applicable";
      } else {
        var values = samples.map(function (sample) { return Number(sample.value); });
        var minimum = Math.min.apply(null, values);
        var maximum = Math.max.apply(null, values);
        var total = values.reduce(function (sum, value) { return sum + value; }, 0);
        var mean = total / values.length;
        var convert = definition[3];
        if (convert) {
          minimum = convert(minimum); maximum = convert(maximum); mean = convert(mean);
        }
        summary.textContent = "min " + minimum.toFixed(2) + " / mean " + mean.toFixed(2) + " / max " + maximum.toFixed(2) + definition[2];
        var low = Math.min.apply(null, values);
        var high = Math.max.apply(null, values);
        var span = high - low || 1;
        var points = samples.map(function (sample, index) {
          var x = samples.length === 1 ? 150 : index * 300 / (samples.length - 1);
          var y = 38 - ((Number(sample.value) - low) / span) * 34;
          return x.toFixed(2) + "," + y.toFixed(2);
        }).join(" ");
        var line = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
        line.setAttribute("points", points);
        svg.appendChild(line);
      }
      row.appendChild(label); row.appendChild(svg); row.appendChild(summary);
      root.appendChild(row);
    });
  }

  function render(data) {
    var readings = data.readings || {};
    var connected = data.connected;
    var status = document.getElementById("connection-status");
    status.textContent = connected ? "connected" : "stale";
    status.className = "value " + (connected ? "good" : "bad");
    text("last-packet", data.last_packet_age_s === null ? "not applicable" : data.last_packet_age_s.toFixed(1) + " s ago");
    text("device", data.device === null ? "not applicable" : String(data.device));
    text("session", data.session === null ? "not applicable" : String(data.session));
    text("sequence", data.sequence === null ? "not applicable" : String(data.sequence));
    text("session-note", data.session_note);
    if (data.notice === null) {
      text("terminal-notice", "No terminal notice received.");
    } else {
      text("terminal-notice", "Outcome: " + String(data.notice.outcome) + " | Fault: " + (data.notice.fault ? "yes" : "no"));
    }
    text("motor_1_speed_mrpm", valueText(readings, "motor_1_speed_mrpm", " RPM", function (value) { return value / 1000; }));
    text("motor_2_speed_mrpm", valueText(readings, "motor_2_speed_mrpm", " RPM", function (value) { return value / 1000; }));
    text("motor_1_duty_pct", valueText(readings, "motor_1_duty_pct", " %", null));
    text("motor_2_duty_pct", valueText(readings, "motor_2_duty_pct", " %", null));
    text("motor_1_direction", valueText(readings, "motor_1_direction", "", null));
    text("motor_2_direction", valueText(readings, "motor_2_direction", "", null));
    text("range_mm", valueText(readings, "range_mm", " mm", null));
    text("ir_mv", valueText(readings, "ir_mv", " mV", null));
    text("state", valueText(readings, "state", "", null));
    text("progress_pct", valueText(readings, "progress_pct", " %", null));
    text("packets-received", String(data.packets_received));
    text("malformed-packets", String(data.malformed_packets));
    text("sequence-gaps", String(data.sequence_gaps));
    renderHistory(data.history || {});
  }

  function poll() {
    fetch("/api/state", { cache: "no-store" }).then(function (response) {
      if (!response.ok) { throw new Error("state request failed"); }
      return response.json();
    }).then(render).catch(function () {
      var status = document.getElementById("connection-status");
      status.textContent = "dashboard error";
      status.className = "value bad";
    });
  }
  poll();
  window.setInterval(poll, 1000);
}());
</script>
</body>
</html>
'''


class DashboardState:
    """Thread-safe current readings, health counters, and rolling history."""

    def __init__(self):
        self.lock = threading.Lock()
        self.packets_received = 0
        self.malformed_packets = 0
        self.sequence_gaps = 0
        self.session = None
        self.device = None
        self.sequence = None
        self.readings = {}
        self.notice = None
        self.session_note = "Waiting for a telemetry session."
        self.history = {}
        self.last_sequence_by_session = {}
        self.last_packet_monotonic = None
        self.last_packet_epoch = None
        self.udp_error = None

    def ingest(self, data, source=None, received_at=None):
        """Validate and apply one untrusted UDP datagram."""
        if received_at is None:
            received_at = time.time()
        with self.lock:
            self.packets_received += 1
        packet, error = telemetry_receiver.parse_packet(data)
        with self.lock:
            if error is not None:
                self.malformed_packets += 1
                return False, error
            session = packet["session"]
            if self.session != session:
                self.session = session
                self.history = {}
                self.last_sequence_by_session = {}
                self.notice = None
                self.session_note = "New session {0}; history cleared.".format(session)
            previous = self.last_sequence_by_session.get(session)
            sequence = packet["seq"]
            if previous is not None and sequence > previous + 1:
                self.sequence_gaps += sequence - previous - 1
            if previous is None or sequence > previous:
                self.last_sequence_by_session[session] = sequence
            self.device = packet["device"]
            self.sequence = sequence
            self.last_packet_monotonic = time.monotonic()
            self.last_packet_epoch = received_at
            if "readings" in packet:
                self.readings = dict(packet["readings"])
                for key, value in self.readings.items():
                    if isinstance(value, int) and not isinstance(value, bool):
                        self.history.setdefault(key, deque(maxlen=HISTORY_LIMIT)).append({
                            "time": received_at,
                            "value": value,
                        })
            else:
                self.notice = dict(packet["notice"])
            return True, None

    def set_udp_error(self, error):
        with self.lock:
            self.udp_error = str(error)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = None if self.last_packet_monotonic is None \
                else max(0.0, now - self.last_packet_monotonic)
            history = {
                key: list(samples) for key, samples in self.history.items()
            }
            return {
                "connected": age is not None and age <= STALE_SECONDS,
                "last_packet_age_s": age,
                "last_packet_time": self._format_time(self.last_packet_epoch),
                "device": self.device,
                "session": self.session,
                "sequence": self.sequence,
                "readings": dict(self.readings),
                "notice": None if self.notice is None else dict(self.notice),
                "session_note": self.session_note,
                "packets_received": self.packets_received,
                "malformed_packets": self.malformed_packets,
                "sequence_gaps": self.sequence_gaps,
                "history": history,
                "udp_error": self.udp_error,
            }

    @staticmethod
    def _format_time(epoch):
        if epoch is None:
            return None
        return datetime.fromtimestamp(epoch, timezone.utc).isoformat(timespec="milliseconds")


class UdpTelemetryThread(threading.Thread):
    """Receive UDP packets without sharing a socket or loop with HTTP."""

    daemon = True

    def __init__(self, state, bind_address, port, stop_event):
        super().__init__(name="telemetry-udp")
        self.state = state
        self.bind_address = bind_address
        self.port = port
        self.stop_event = stop_event

    def run(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
                receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                receiver.bind((self.bind_address, self.port))
                receiver.settimeout(0.5)
                while not self.stop_event.is_set():
                    try:
                        data, source = receiver.recvfrom(
                            telemetry_receiver.MAX_PACKET_BYTES + 1)
                    except socket.timeout:
                        continue
                    self.state.ingest(data, source)
        except OSError as error:
            if not self.stop_event.is_set():
                self.state.set_udp_error(error)


def make_http_handler(state):
    class DashboardHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                body = HTML_PAGE.encode("utf-8")
                content_type = "text/html; charset=utf-8"
            elif path == "/api/state":
                body = (json.dumps(state.snapshot(), separators=(",", ":")) + "\n").encode("utf-8")
                content_type = "application/json; charset=utf-8"
            else:
                self.send_error(404, "not found")
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format_string, *args):
            # Keep the dashboard useful on a serial terminal without echoing
            # received packet fields into an HTML response.
            print("dashboard HTTP: " + format_string % args, flush=True)

    return DashboardHandler


def packet_bytes(packet):
    return telemetry_receiver.packet_bytes(packet)


def self_test():
    state = DashboardState()
    base = {
        "ver": 2, "device": "self-test", "session": 100, "seq": 0,
        "ms": 1, "link": 3, "test": "probe",
    }
    first = dict(base, readings={
        "state": "running", "motor_1_speed_mrpm": 12500,
        "motor_1_duty_pct": 10,
    })
    accepted, error = state.ingest(packet_bytes(first))
    assert accepted and error is None
    snapshot = state.snapshot()
    assert snapshot["readings"]["motor_1_speed_mrpm"] == 12500
    assert "motor_2_speed_mrpm" not in snapshot["readings"]
    assert "motor_2_speed_mrpm" not in snapshot["history"]
    assert snapshot["packets_received"] == 1

    notice = dict(base, seq=1,
                  notice={"outcome": "completed", "fault": 0})
    accepted, error = state.ingest(packet_bytes(notice))
    assert accepted and error is None
    snapshot = state.snapshot()
    assert snapshot["notice"] == {"outcome": "completed", "fault": 0}

    second = dict(base, session=101, seq=0, readings={"state": "idle"})
    accepted, error = state.ingest(packet_bytes(second))
    assert accepted and error is None
    snapshot = state.snapshot()
    assert snapshot["session"] == 101
    assert snapshot["history"] == {}
    assert "cleared" in snapshot["session_note"]
    assert "motor_1_speed_mrpm" not in snapshot["readings"]

    accepted, error = state.ingest(b"not-json\n")
    assert not accepted and error is not None
    snapshot = state.snapshot()
    assert snapshot["malformed_packets"] == 1
    assert snapshot["packets_received"] == 4
    json.dumps(snapshot)
    print("Telemetry dashboard self-test passed.")


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default=DEFAULT_UDP_BIND,
                        help="UDP bind address (default: 0.0.0.0)")
    parser.add_argument("--port", default=DEFAULT_UDP_PORT, type=int,
                        help="UDP port (default: 4242)")
    parser.add_argument("--http-port", default=DEFAULT_HTTP_PORT, type=int,
                        help="localhost HTTP port (default: 8080)")
    parser.add_argument("--self-test", action="store_true",
                        help="run state pipeline tests and exit")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.self_test:
        self_test()
        return 0
    for name, port in (("UDP", arguments.port), ("HTTP", arguments.http_port)):
        if not 1 <= port <= 65535:
            print("{0} port must be between 1 and 65535".format(name))
            return 2

    state = DashboardState()
    stop_event = threading.Event()
    udp_thread = UdpTelemetryThread(state, arguments.bind, arguments.port,
                                    stop_event)
    try:
        server = http.server.ThreadingHTTPServer(
            (HTTP_BIND_ADDRESS, arguments.http_port), make_http_handler(state))
    except OSError as error:
        print("Could not bind HTTP server to {0}:{1}: {2}".format(
            HTTP_BIND_ADDRESS, arguments.http_port, error))
        return 1
    server.daemon_threads = True
    udp_thread.start()
    print("Dashboard: http://{0}:{1}/ (read-only; GP20/GP21 physical controls)".format(
        HTTP_BIND_ADDRESS, arguments.http_port), flush=True)
    print("UDP telemetry: {0}:{1}; schema v{2}".format(
        arguments.bind, arguments.port, telemetry_receiver.SCHEMA_VERSION), flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nDashboard stopped by operator.", flush=True)
    finally:
        server.server_close()
        stop_event.set()
        udp_thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
