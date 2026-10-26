"""Read-only localhost dashboard for provisional Buddy 1 MQTT telemetry."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import re
import threading
import time
import uuid

import buddy1_telemetry
from buddy1_telemetry import decode_status


class DashboardState:
    """Decode messages and hold a small view model; no MQTT or HTTP dependencies."""
    def __init__(self, stale_after=3.0, clock=time.monotonic, wall_clock=time.time,
                 source="simulated"):
        if not math.isfinite(stale_after) or stale_after <= 0:
            raise ValueError("stale-after must be positive and finite")
        if source not in ("simulated", "sensors"):
            raise ValueError("source must be simulated or sensors")
        self.source = source
        self.lock = threading.Lock()
        self.clock, self.wall_clock, self.stale_after = clock, wall_clock, stale_after
        self.connected = False
        self.subscribed = False
        self.network_error = None
        self.frame = self.frame_time = self.last_update = None
        self.current_frame = False
        self.report = self.report_time = None
        self.retained_report = False
        self.session = None
        self.invalid = self.ignored = 0
        self.last_error = None

    def connection(self, connected, error=None):
        with self.lock:
            self.connected = connected
            self.subscribed = False
            self.network_error = error
            self.report = self.report_time = self.session = None
            self.current_frame = False

    def subscription(self, success):
        with self.lock:
            self.subscribed = success
            self.network_error = None if success else "MQTT subscription failed"

    def receive(self, kind, payload, retained=False):
        if kind not in ("telemetry", "status", "heartbeat"):
            return
        try:
            decoded = (buddy1_telemetry.readable(buddy1_telemetry.decode(payload))
                       if kind == "telemetry" else decode_status(payload))
        except (ValueError, TypeError) as error:
            with self.lock:
                self.invalid += 1
                self.last_error = f"{kind}: {error}"
            return
        with self.lock:
            now = self.clock()
            if kind == "telemetry":
                if retained or (self.session is not None and decoded["session"] != self.session):
                    self.ignored += 1
                    return
                if self.current_frame and self.frame["session"] == decoded["session"]:
                    delta = (decoded["sequence"] - self.frame["sequence"]) & 65535
                    if not 0 < delta < 32768:
                        self.ignored += 1
                        return
                self.session = decoded["session"]
                self.frame, self.frame_time = decoded, now
                self.current_frame = True
                self.last_update = datetime.fromtimestamp(self.wall_clock(), timezone.utc).isoformat()
            else:
                # Retained online status is a report, never evidence of live presence.
                # A delayed old-session will must not replace current-session status.
                if ((kind == "heartbeat" and retained) or
                    (self.session is not None and decoded["session"] != self.session
                     and (retained or not decoded["online"]))):
                    self.ignored += 1
                    return
                if not retained and decoded["online"]:
                    if self.session != decoded["session"]:
                        self.current_frame = False
                    self.session = decoded["session"]
                self.report, self.report_time = decoded, now
                self.retained_report = retained

    def snapshot(self):
        with self.lock:
            now = self.clock()
            age = None if self.frame_time is None else max(0, now - self.frame_time)
            report_age = None if self.report_time is None else max(0, now - self.report_time)
            robot_state = "unknown"
            if self.connected and self.report is not None:
                if not self.report["online"]:
                    robot_state = "offline report"
                elif self.retained_report:
                    robot_state = "online report (retained)"
                elif report_age >= self.stale_after:
                    robot_state = "unknown (stale report)"
                else:
                    robot_state = "online"
            fresh = (self.connected and self.current_frame and age is not None
                     and age < self.stale_after and robot_state != "offline report")
            # Copy through JSON: callers cannot mutate data held by the model.
            return json.loads(json.dumps(dict(
                simulated=self.source == "simulated", source=self.source,
                broker_connected=self.connected, mqtt_subscribed=self.subscribed,
                network_error=self.network_error, robot_state=robot_state,
                robot_report=self.report, robot_report_age_seconds=report_age,
                freshness=("waiting" if self.frame is None else "fresh" if fresh else "stale"),
                telemetry=self.frame, telemetry_age_seconds=age, last_update=self.last_update,
                invalid_messages=self.invalid, ignored_messages=self.ignored,
                last_error=self.last_error)))


class MqttReceiver:
    """Transport only; validated state and presentation live outside this adapter."""
    def __init__(self, state, host, port, robot):
        import paho.mqtt.client as mqtt
        self.state = state
        self.prefix = f"robot/{robot}/"
        self.topics = {self.prefix + name: name for name in ("telemetry", "status", "heartbeat")}
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="buddy1-dashboard-" + uuid.uuid4().hex[:12])
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        user = os.environ.get("BUDDY1_MQTT_USERNAME")
        if user:
            self.client.username_pw_set(user, os.environ.get("BUDDY1_MQTT_PASSWORD"))
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_connect_fail = self.on_connect_fail
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message
        self.host, self.port = host, port

    def on_connect(self, client, userdata, flags, reason, properties):
        self.state.connection(not reason.is_failure, str(reason) if reason.is_failure else None)
        if not reason.is_failure:
            result, _ = client.subscribe([(topic, 1) for topic in self.topics])
            if result != 0:
                self.state.subscription(False)

    def on_subscribe(self, client, userdata, mid, reasons, properties):
        self.state.subscription(bool(reasons) and all(not reason.is_failure for reason in reasons))

    def on_disconnect(self, client, userdata, flags, reason, properties):
        self.state.connection(False, str(reason) if reason.is_failure else None)

    def on_connect_fail(self, client, userdata):
        self.state.connection(False, "Broker unavailable; retrying")

    def on_message(self, client, userdata, message):
        kind = self.topics.get(message.topic)
        if kind is not None:
            self.state.receive(kind, message.payload, message.retain)

    def start(self):
        self.client.connect_async(self.host, self.port, keepalive=20)
        self.client.loop_start()

    def stop(self):
        self.client.disconnect()
        self.client.loop_stop()


HTML_PAGE = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Buddy 1 MQTT bench</title><style>
:root { font-family:system-ui,sans-serif; color:#202a33; background:#f5f6f8; }
body { max-width:1000px; margin:2rem auto; padding:0 1rem; }
h1 { margin-bottom:.3rem; } h2 { font-size:1.1rem; }
.notice { padding:1rem; background:#fff1c2; border-left:5px solid #976600; font-weight:bold; }
.panel { background:white; border:1px solid #d3d9df; border-radius:.5rem; margin:1rem 0; padding:1rem; }
.grid { display:grid; gap:1rem; grid-template-columns:repeat(auto-fit,minmax(190px,1fr)); }
.label { display:block; font-size:.85rem; color:#536170; }
.value { display:block; margin-top:.3rem; font-size:1.35rem; font-variant-numeric:tabular-nums; }
.note { color:#536170; font-size:.9rem; } .warning { color:#8b3600; }
</style></head><body>
<h1>Buddy 1 · MQTT telemetry</h1>
<p class="notice">__DATA_NOTICE__</p>
<section class="panel"><h2>Connection and freshness</h2><div class="grid">
<div><span class="label">Broker connection</span><strong class="value" id="broker">waiting</strong></div>
<div><span class="label">Robot status</span><strong class="value" id="robot">unknown</strong></div>
<div><span class="label">Telemetry freshness</span><strong class="value" id="freshness">waiting</strong></div>
</div><p id="update" class="note">No valid telemetry received.</p>
<p id="ready" class="note">Bench command readiness: unknown. No robot execution is implied.</p>
<p id="http-error" class="warning" role="status"></p></section>
<section class="panel"><h2>Line sensors · raw ADC (0–4095)</h2><div class="grid">
<div><span class="label">Left</span><strong class="value" id="left">—</strong></div>
<div><span class="label">Centre</span><strong class="value" id="centre">—</strong></div>
<div><span class="label">Right</span><strong class="value" id="right">—</strong></div>
<div><span class="label">Line pattern · raw bits</span><strong class="value" id="pattern">—</strong></div>
</div><p id="line-quality">Line sensor quality: unavailable</p></section>
<section class="panel"><h2>Barcode sensor</h2><div class="grid">
<div><span class="label">Raw sample · 0 or 1</span><strong class="value" id="barcode">—</strong></div>
</div><p id="barcode-quality">Barcode sensor quality: unavailable</p></section>
<section class="panel"><h2>Packet details</h2><p id="packet">No packet.</p>
<p id="errors" class="warning"></p><p class="note">Unavailable or faulty values are hidden.
Stale values remain visible. __PATTERN_NOTE__</p></section>
<script>
const text = (id,value) => { document.getElementById(id).textContent=value; };
async function refresh() {
  try {
    const response=await fetch('/api/state',{cache:'no-store'});
    if (!response.ok) throw new Error('HTTP '+response.status);
    const s=await response.json(), f=s.telemetry;
    text('http-error','');
    text('broker',s.broker_connected ? (s.mqtt_subscribed?'connected · subscribed':'connected · subscribing'):'disconnected');
    text('robot',s.robot_state); text('freshness',s.freshness);
    text('update',s.last_update ? 'Last valid update (UTC): '+s.last_update+' · '+s.telemetry_age_seconds.toFixed(1)+' s ago':'No valid telemetry received.');
    const report=s.robot_report;
    text('ready','Bench command readiness report: '+(report?(report.ready?'ready':'not ready'):'unknown')+'. No robot execution is implied.');
    ['left','centre','right'].forEach((id,i)=>text(id,f&&f.line.adc!==null?f.line.adc[i]:'—'));
    text('pattern',f&&f.line.pattern!==null?f.line.pattern.toString(2).padStart(3,'0'):'—');
    text('barcode',f&&f.barcode.sample!==null?f.barcode.sample:'—');
    for (const name of ['line','barcode']) {
      const quality=f?f[name].quality:'unavailable';
      text(name+'-quality',name+' sensor quality: '+quality+(s.freshness==='stale'?' · reception stale':''));
    }
    text('packet',f?'Session: 0x'+f.session.toString(16).padStart(8,'0')+' · Sequence: '+f.sequence+' · Device uptime: '+f.timestamp_ms+' ms':'No packet.');
    text('errors','Invalid: '+s.invalid_messages+' · Ignored (retained/old/session): '+s.ignored_messages+(s.last_error?' · Last invalid: '+s.last_error:'')+(s.network_error?' · '+s.network_error:''));
  } catch(error) {
    text('http-error','Dashboard connection lost; displayed readings may be stale. '+error.message);
    text('broker','unknown'); text('robot','unknown'); text('freshness','stale');
  } finally { setTimeout(refresh,500); }
}
refresh();
</script></body></html>'''


def make_server(state, port=8081):
    if state.source == "sensors":
        notice = ("Real sensor demonstration — provisional 16-byte format. "
                  "Display mode is selected locally; use the sensor-demo publisher.")
        pattern_note = ("Pattern: L=1, C=2, R=4 (001 means left). "
                        "Unknown classification hides the line group. Barcode is raw GP7, not decoded text.")
    else:
        notice = "Simulated bench data — provisional 16-byte format. No actual robot measurements."
        pattern_note = "Raw pattern bits have no confirmed left/right mapping."
    page = HTML_PAGE.replace("__DATA_NOTICE__", notice).replace("__PATTERN_NOTE__", pattern_note)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body, kind = page.encode("utf-8"), "text/html; charset=utf-8"
            elif self.path == "/api/state":
                body, kind = json.dumps(state.snapshot()).encode("utf-8"), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--robot", help="Default: aai01, or aai01-sensors with --source sensors")
    parser.add_argument("--source", choices=("simulated", "sensors"), default="simulated",
                        help="Display label only; sensor demo uses --robot aai01-sensors")
    parser.add_argument("--http-port", type=int, default=8081)
    parser.add_argument("--stale-after", type=float, default=3.0, help="Reception timeout, seconds")
    args = parser.parse_args()
    args.robot = args.robot or ("aai01-sensors" if args.source == "sensors" else "aai01")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", args.robot):
        parser.error("robot must contain 1-32 letters, digits, underscores or hyphens")
    if not all(1 <= port <= 65535 for port in (args.port, args.http_port)):
        parser.error("ports must be between 1 and 65535")
    try:
        state = DashboardState(args.stale_after, source=args.source)
        receiver = MqttReceiver(state, args.host, args.port, args.robot)
        server = make_server(state, args.http_port)
    except (ValueError, OSError, ImportError) as error:
        parser.error(str(error))
    label = "Real sensor demonstration" if args.source == "sensors" else "Simulated bench data"
    print(f"{label}: http://127.0.0.1:{args.http_port}", flush=True)
    try:
        receiver.start()
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        server.server_close()


if __name__ == "__main__":
    main()
