"""Standard-library checks for the environment-only API and loopback boundary."""
import copy
import http.client
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / "viewer"))
from serve import Editor, Handler, MAX_BODY, RequestError, Server
BUILD = Path(sys.argv[1]).resolve().parent


class QuietHandler(Handler):
    def log_message(self, *args):
        pass


class API(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=BUILD)
        self.addCleanup(self.tmp.cleanup)
        self.app = Editor(BUILD)
        self.app.output = Path(self.tmp.name) / "runs"
        self.app.scenarios = Path(self.tmp.name) / "scenarios"
        self.app.scenarios.mkdir()
        self.raw = json.loads((SIM / "scenarios/box_straight.json").read_text())
        (self.app.scenarios / "box_straight.json").write_text(json.dumps(self.raw))
        self.server = Server(("127.0.0.1", 0), QuietHandler)
        self.server.editor = self.app
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, path, body=None, headers=None, method=None):
        c = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=5)
        try:
            h = {"Content-Type": "application/json"}
            h.update(headers or {})
            c.request(method or ("POST" if body is not None else "GET"), path,
                      None if body is None else json.dumps(body), headers=h)
            r = c.getresponse()
            return r.status, r.read()
        finally:
            c.close()

    def test_environment_only(self):
        raw = copy.deepcopy(self.raw)
        raw["obstacles"][0]["size_mm"]["value"] = [480, 240]
        raw["obstacles"][0]["pose"]["value"]["x_mm"] = 980
        raw["robot"]["start_pose"]["value"]["y_mm"] = 100
        raw["track"]["tiles"][0]["type"] = "cross"
        raw["end_us"] = 50000000
        self.app.checked({"base": "box_straight", "scenario": raw})
        for field, value in (("clock_quantum_us", 50), ("seed", 2)):
            bad = copy.deepcopy(raw);bad[field] = value
            with self.assertRaisesRegex(RequestError, "read-only"):
                self.app.checked({"base": "box_straight", "scenario": bad})
        raw["robot"]["geometry"]["width_mm"] = 999
        with self.assertRaisesRegex(RequestError, "robot.geometry.width_mm.*read-only"):
            self.app.checked({"base": "box_straight", "scenario": raw})

    def test_numeric_json_roundtrip(self):
        base = copy.deepcopy(self.raw)
        base["robot"]["geometry"]["width_mm"] = 160.0
        (self.app.scenarios / "box_straight.json").write_text(json.dumps(base))
        self.app.checked({"base": "box_straight", "scenario": self.raw})
        bad = copy.deepcopy(self.raw)
        bad["robot"]["servo"]["instant_angle"]["value"] = 1
        with self.assertRaises(ValueError):
            self.app.checked({"base": "box_straight", "scenario": bad})

    def test_invalid_never_starts(self):
        raw = copy.deepcopy(self.raw)
        raw["obstacles"][0]["size_mm"]["value"][1] = -1
        status, body = self.request("/api/run", {"base": "box_straight", "scenario": raw})
        self.assertEqual(status, 400)
        self.assertIn(b"obstacles[0]", body)
        self.assertEqual(self.app.started_runs, 0)
        self.assertFalse(self.app.output.exists())

    def test_save_exclusive_reload_and_safe_names(self):
        payload = {"base": "box_straight", "scenario": self.raw, "name": "new_environment"}
        self.assertEqual(self.request("/api/save", payload)[0], 201)
        saved = self.app.base("new_environment")
        self.assertEqual(saved["name"], "new_environment")
        self.app.checked({"base": "new_environment", "scenario": saved})
        before = (self.app.scenarios / "new_environment.json").read_bytes()
        self.assertEqual(self.request("/api/save", payload)[0], 409)
        self.assertEqual((self.app.scenarios / "new_environment.json").read_bytes(), before)
        for name in ("../escape", "UPPER", "a.b", "", "/tmp/escape"):
            payload["name"] = name
            self.assertEqual(self.request("/api/save", payload)[0], 400)
        self.assertEqual(list(self.app.scenarios.glob("*.tmp")), [])

    def test_paths_origin_host_and_size_limit(self):
        for path in ("/etc/passwd", "/../AGENTS.md", "/%2e%2e/src/detour_test.c", "/runs/../../src/detour_test.c", "/api/scenario/../../secret", "/editor.js?path=/etc/passwd"):
            self.assertIn(self.request(path)[0], (400, 404))
        self.assertEqual(self.request("/api/status", headers={"Host": "evil.example"})[0], 403)
        self.assertEqual(self.request("/api/status", headers={"Origin": "http://evil.example"})[0], 403)
        self.assertEqual(self.request("/api/run", {}, {"Content-Length": str(MAX_BODY + 1)})[0], 413)
        self.assertEqual(self.request("/api/run", {}, {"Transfer-Encoding": "chunked"})[0], 400)

    def test_actual_non_loopback_peer_rejected(self):
        self.assertEqual(self.server.server_address[0], "127.0.0.1")
        with socket.socket() as s:
            s.settimeout(2)
            s.bind(("127.0.0.2", 0))
            s.connect(self.server.server_address)
            s.sendall(b"GET /api/status HTTP/1.0\r\n\r\n")
            try:
                self.assertEqual(s.recv(1024), b"")
            except ConnectionResetError:
                pass
        self.assertFalse(self.server.verify_request(None, ("192.0.2.1", 123)))

    def test_missing_and_stale_build_messages(self):
        with patch.object(self.app, "build", Path(self.tmp.name) / "missing"):
            self.assertIn("cmake --build", self.app.build_error())
            status, _ = self.request("/api/run", {"base": "box_straight", "scenario": self.raw})
            self.assertEqual(status, 503)
        native = BUILD / "firmware_sim"
        actual = native.stat()
        original = Path.stat
        def stat(path, *a, **kw):
            # Only replace the timestamp lookup after the is_file checks.
            result = original(path, *a, **kw)
            if path == native:
                return type("Stat", (), {"st_mode": actual.st_mode, "st_mtime_ns": 0})()
            return result
        with patch.object(Path, "stat", stat):
            self.assertIn("older", self.app.build_error())
        self.assertEqual(self.app.started_runs, 0)

    def test_busy_lock_and_session_history(self):
        self.app.run_lock.acquire()
        try:
            self.assertEqual(self.request("/api/run", {"base": "box_straight", "scenario": self.raw})[0], 409)
            self.assertEqual(self.request("/api/save", {"base": "box_straight", "scenario": self.raw, "name": "other"})[0], 409)
        finally:
            self.app.run_lock.release()
        self.assertEqual(self.app.started_runs, 0)
        self.assertEqual(json.loads(self.request("/api/history")[1]), [])
        self.assertEqual(self.request("/runs/not_a_session_run/run.html")[0], 404)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
