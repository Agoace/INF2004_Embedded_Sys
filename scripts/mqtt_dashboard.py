#!/usr/bin/env python3
"""Live robot dashboard for the laptop: MQTT in, browser page and commands out.

Binds 127.0.0.1 only. The command token stays on the server and never reaches
the browser. Needs paho-mqtt 2.x (VERSION2) only when it runs for real.
"""
import argparse
import hmac
import http.server
import json
import os
import secrets
import sys
import threading
import time
import urllib.parse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_wire  # noqa: E402
from dashboard_model import DashboardModel  # noqa: E402
from dashboard_page import PAGE  # noqa: E402

BIND_ADDRESS = "127.0.0.1"
DEFAULT_HTTP_PORT = 8765
EVENT_INTERVAL_S = 0.2
MAX_BODY_BYTES = 1024
PUBACK_WAIT_S = 2.0
NOTE = ("Broker PUBACK only means the broker has the command. "
        "The robot decision appears in the vehicle panel.")
# A subscription caps delivery QoS: robot/log (DUMP refill), robot/status (retained last
# will), robot/summary (retained) and the reserved robot/barcode are QoS 1 on the robot side.
SUBSCRIPTIONS = [("robot/state", 0), ("robot/sonar", 0), ("robot/event", 0),
                 ("robot/heartbeat", 0), ("robot/log", 1), ("robot/obstacle", 0),
                 ("robot/status", 1), ("robot/barcode", 1), ("robot/imu", 0),
                 ("robot/detour", 0), ("robot/summary", 1)]


class Commander:
    """Build and publish robot commands. The token never leaves this object."""

    def __init__(self, link, model, token=None, request_id_source=None):
        self.link = link
        self.model = model
        self.token = token
        self.request_id_source = request_id_source or (lambda: secrets.randbits(32))

    def send(self, name, confirm):
        """Return (http_status, json_dict). Re-checks every gate before publishing."""
        if name not in robot_wire.COMMANDS:
            return 400, {"ok": False, "error": "unknown command"}
        gate = self.model.button_state(name)
        if name != "estop" and not gate["enabled"]:
            return 409, {"ok": False, "error": gate["reason"]}
        if gate["needs_confirm"] and confirm is not True:
            return 409, {"ok": False, "error": "confirmation required"}
        if not self.link.connected:
            return 503, {"ok": False, "error": "broker not connected"}
        request_id = self.request_id_source()
        token = self.token if self.token is not None else bytes(16)
        payload = robot_wire.pack_command(robot_wire.COMMANDS[name], request_id, token)
        try:
            puback = self.link.publish(payload, PUBACK_WAIT_S)
        except (ConnectionError, OSError):
            return 503, {"ok": False, "error": "publish failed"}
        self.model.note_command(name, request_id, puback)
        return 200, {"ok": True, "command": name, "request_id": request_id,
                     "puback": puback, "note": NOTE}


class DashboardApp:
    """Request rules, separate from sockets so tests can call them directly."""

    def __init__(self, model, commander, port, csrf=None, extra_origins=()):
        self.model = model
        self.commander = commander
        self.csrf = csrf or secrets.token_urlsafe(32)
        # Random per process and not secret: lets an open page detect a restart.
        self.instance = secrets.token_hex(8)
        self.extra_hosts = set()
        self.extra_origins = set()
        for origin in extra_origins:
            self.add_origin(origin)
        self.set_port(port)

    def add_origin(self, origin):
        """Also accept one exact reverse-proxy origin, for example a Tailscale Serve URL."""
        parts = urllib.parse.urlsplit(origin)
        if (parts.scheme not in ("http", "https") or not parts.netloc
                or parts.path not in ("", "/") or parts.query or parts.fragment
                or "@" in parts.netloc):
            raise ValueError(f"origin must be scheme://host[:port]: {origin!r}")
        netloc = parts.netloc.lower()
        self.extra_hosts.add(netloc)
        self.extra_origins.add(f"{parts.scheme}://{netloc}")

    def set_port(self, port):
        """Set the port the browser uses; call again if the server picked a free port."""
        local_hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        self.allowed_hosts = local_hosts | self.extra_hosts
        self.allowed_origins = {f"http://{host}" for host in local_hosts} | self.extra_origins

    def host_ok(self, headers):
        return headers.get("Host", "").lower() in self.allowed_hosts

    def check_post(self, headers):
        """Return an error text for a rejected command POST, or None when it may continue."""
        if not self.host_ok(headers):
            return "bad Host header"
        if headers.get("Origin") not in self.allowed_origins:
            return "bad or missing Origin"
        if headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            return "content type must be application/json"
        supplied = headers.get("X-CSRF-Token", "")
        if not hmac.compare_digest(supplied.encode("utf-8"), self.csrf.encode("utf-8")):
            return "bad CSRF token"
        return None

    def page(self):
        nonce = secrets.token_urlsafe(16)
        html = (PAGE.replace("__CSRF__", self.csrf).replace("__NONCE__", nonce)
                .replace("__INSTANCE__", self.instance))
        return html.encode("utf-8"), nonce

    def state_json(self):
        snapshot = dict(self.model.snapshot(), server_instance=self.instance)
        return json.dumps(snapshot, separators=(",", ":")).encode("utf-8")

    def command(self, body):
        """Handle an already authorised command body: JSON {"command": name, "confirm": bool}."""
        try:
            request = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return 400, {"ok": False, "error": "body must be JSON"}
        if not isinstance(request, dict) or not isinstance(request.get("command"), str):
            return 400, {"ok": False, "error": "missing command"}
        return self.commander.send(request["command"], request.get("confirm"))


def make_handler(app):
    class Handler(http.server.BaseHTTPRequestHandler):
        server_version = "RobotDashboard"

        def log_message(self, format, *args):  # noqa: A002
            # Only method, path and status; headers and bodies are never logged.
            del format
            status = args[1] if len(args) > 1 else ""
            print(f"http {self.command} {self.path.split('?')[0]} {status}", file=sys.stderr)

        def _send(self, status, body, content_type, extra=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            for name, value in (extra or {}).items():
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status, value):
            self._send(status, json.dumps(value).encode("utf-8"), "application/json")

        def do_GET(self):  # noqa: N802
            if not app.host_ok(self.headers):
                self._json(403, {"ok": False, "error": "bad Host header"})
                return
            path = self.path.split("?")[0]
            if path == "/":
                body, nonce = app.page()
                policy = (f"default-src 'none'; script-src 'nonce-{nonce}'; "
                          f"style-src 'nonce-{nonce}'; connect-src 'self'; "
                          "base-uri 'none'; form-action 'none'; frame-ancestors 'none'")
                self._send(200, body, "text/html; charset=utf-8",
                           {"Content-Security-Policy": policy})
            elif path == "/api/state":
                self._send(200, app.state_json(), "application/json")
            elif path == "/events":
                self._events()
            else:
                self._json(404, {"ok": False, "error": "not found"})

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            try:
                while not self.server.stopping.is_set():
                    self.wfile.write(b"data: " + app.state_json() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(EVENT_INTERVAL_S)
            except OSError:
                pass

        def do_POST(self):  # noqa: N802
            if self.path.split("?")[0] != "/api/command":
                self._json(404, {"ok": False, "error": "not found"})
                return
            error = app.check_post(self.headers)
            if error is not None:
                self._json(403, {"ok": False, "error": error})
                return
            try:
                length = int(self.headers.get("Content-Length", "-1"))
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_BODY_BYTES:
                self._json(400, {"ok": False, "error": "bad body length"})
                return
            status, result = app.command(self.rfile.read(length))
            self._json(status, result)

    return Handler


class DashboardServer(http.server.ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, app, port=DEFAULT_HTTP_PORT):
        self.stopping = threading.Event()
        self.serving = False
        super().__init__((BIND_ADDRESS, port), make_handler(app))

    def serve_forever(self, poll_interval=0.5):
        self.serving = True
        try:
            super().serve_forever(poll_interval)
        finally:
            self.serving = False

    def shutdown(self):
        self.stopping.set()  # ends every event stream
        if self.serving:  # shutdown() would wait forever if the loop never ran
            super().shutdown()


class PahoLink:
    """MQTT connection to the broker. Paho is imported only when started."""

    def __init__(self, host, port, model):
        self.host, self.port, self.model = host, port, model
        self.client = None
        self.connected = False

    def start(self):
        import paho.mqtt.client as mqtt
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        client.connect_async(self.host, self.port, 60)
        client.loop_start()
        self.client = client

    def stop(self):
        if self.client is not None:
            self.client.disconnect()
            self.client.loop_stop()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        del userdata, flags, properties
        if reason_code.is_failure:
            print("broker connection rejected", file=sys.stderr)
            return
        client.subscribe(SUBSCRIPTIONS)  # again after every reconnect
        self.connected = True
        self.model.set_broker_connected(True)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        del client, userdata, flags, reason_code, properties
        self.connected = False
        self.model.set_broker_connected(False)

    def _on_message(self, client, userdata, message):
        del client, userdata
        self.model.ingest(message.topic, message.payload, retained=bool(message.retain))

    def publish(self, payload, timeout):
        """Publish robot/cmd at QoS 1, not retained. True when PUBACK arrived in time."""
        if not self.connected:
            raise ConnectionError("not connected")
        info = self.client.publish("robot/cmd", payload, qos=1, retain=False)
        info.wait_for_publish(timeout=timeout)
        return bool(info.is_published())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="MQTT broker; default this laptop")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--http-port", type=int, default=DEFAULT_HTTP_PORT,
                        help="dashboard port on 127.0.0.1")
    parser.add_argument("--token-file", help="0600 file with 32 hex digits; "
                        "without it only ESTOP (zero token) is available")
    parser.add_argument("--allow-origin", action="append", default=[], metavar="URL",
                        help="also accept this exact proxy origin, e.g. "
                        "https://<name>.ts.net:8443 for Tailscale Serve; repeatable")
    args = parser.parse_args(argv)
    token = None
    if args.token_file:
        try:
            token = robot_wire.read_token(args.token_file)
        except (OSError, ValueError):
            parser.error("cannot read valid private token file")
    model = DashboardModel(token_configured=token is not None)
    link = PahoLink(args.host, args.port, model)
    try:
        app = DashboardApp(model, Commander(link, model, token), args.http_port,
                           extra_origins=args.allow_origin)
    except ValueError as error:
        parser.error(str(error))
    server = DashboardServer(app, args.http_port)
    link.start()
    print(f"dashboard: http://127.0.0.1:{args.http_port}/  broker: {args.host}:{args.port}",
          file=sys.stderr)
    for origin in sorted(app.extra_origins):
        print(f"dashboard: also accepting proxy origin {origin}", file=sys.stderr)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        link.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
