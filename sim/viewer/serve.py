"""Loopback environment editor; rebuilds native inputs, never edits firmware/model."""
import argparse
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import unquote, urlsplit

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
SIM = HERE.parent
sys.path.insert(0, str(SIM / "core"))
from scenario import fail, load, unique_object, unwrap, validate

MAX_BODY = 1024 * 1024
NAME = re.compile(r"[a-z0-9_-]{1,64}\Z")


class RequestError(ValueError):
    def __init__(self, message, field="$", status=400, build_output=None):
        super().__init__(message)
        self.field, self.status = field, status
        self.build_output = build_output


def difference(a, b, path=""):
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b)):
            p = f"{path}.{key}" if path else key
            if key not in a or key not in b:
                return p
            found = difference(a[key], b[key], p)
            if found:
                return found
        return None
    # JSON round trips may spell 160.0 as 160. Schema validation already
    # enforces booleans, strings and integer-only fields; compare numeric values.
    if a != b:
        return path or "$"
    return None


def locked_part(raw):
    result = copy.deepcopy(raw)
    for key in ("obstacles", "end_us"):
        result.pop(key, None)
    result["robot"].pop("start_pose", None)
    result["track"].pop("tiles", None)
    return result


class Editor:
    def __init__(self, build):
        self.build = Path(build).resolve()
        self.scenarios = SIM / "scenarios"
        self.output = SIM / "output/web"
        self.run_lock = threading.Lock()  # Save also holds this: no source-file race with a run.
        self.history_lock = threading.Lock()
        self.history = {}
        self.session = secrets.token_hex(4)
        self.started_runs = 0
        self.process = None
        self.build_message = "Native build not checked yet"

    @property
    def build_command(self):
        return f"cmake -S sim -B {shlex.quote(str(self.build))} -DSIM_FIRMWARE=detour_test && cmake --build {shlex.quote(str(self.build))}"

    def build_error(self):
        required = ["robot_sim", "target.json", "compile_commands.json", "compiler.txt", "CMakeCache.txt"]
        if any(not (self.build / p).is_file() for p in required):
            return "Built simulator is missing. Run: " + self.build_command
        return None

    @staticmethod
    def file_hash(path):
        return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None

    def build_inputs(self):
        # Compiler dependency files include headers outside this repository too.
        commands = json.loads((self.build / "compile_commands.json").read_text())
        sources = {Path(c["file"]) if Path(c["file"]).is_absolute() else Path(c["directory"]) / c["file"] for c in commands}
        if not sources:
            raise RequestError("No native compile commands. Run: " + self.build_command, "build", 503)
        for dep in (self.build / "CMakeFiles").rglob("*.o.d"):
            text = dep.read_text().replace("\\\n", " ").split(":", 1)[1]
            sources.update(Path(p) if Path(p).is_absolute() else self.build / p for p in shlex.split(text))
        # Also covers Ninja's consumed depfiles and newly included local headers.
        for directory in (SIM.parent / "src", SIM / "include", SIM / "targets", SIM / "core", SIM / "devices"):
            sources.update(directory.rglob("*.h"))
        sources.add(self.build / "compile_commands.json")
        return {str(p.resolve()): self.file_hash(p) for p in sorted(sources)}

    def ensure_build(self):
        """Called under run_lock. No successful build means no simulator launch."""
        error = self.build_error()
        if error:
            raise RequestError(error, "build", 503)
        state_file = self.build / "editor-build-state.json"
        try:
            previous = json.loads(state_file.read_text())
            if not isinstance(previous, dict) or not isinstance(previous.get("inputs"), dict):
                previous = {}
        except (OSError, ValueError):
            previous = {}
        before = self.build_inputs()
        native = self.build / "firmware_sim"
        binary = self.file_hash(native)
        changed = [p for p, digest in before.items() if previous.get("inputs", {}).get(p) != digest]
        native_time = native.stat().st_mtime_ns if native.is_file() else 0
        # Content changes with preserved/old timestamps must not fool Make/Ninja.
        clean = (not previous or binary != previous.get("binary") or
                 any(not Path(p).is_file() or Path(p).stat().st_mtime_ns <= native_time for p in changed))
        command = ["cmake", "--build", str(self.build)] + (["--clean-first"] if clean else [])
        self.build_message = "Building native firmware…" if clean or changed else "Checking native build…"
        start = time.perf_counter()
        log_path = self.build / "editor-build.log"
        timed_out = False
        with log_path.open("w") as log:
            self.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                            shell=False, start_new_session=True)
            try:
                self.process.communicate(timeout=120)
            except subprocess.TimeoutExpired:
                timed_out = True
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.communicate()
            finally:
                code = self.process.returncode
                self.process = None
        elapsed = time.perf_counter() - start
        if timed_out or code != 0:
            self.build_message = "Build timed out" if timed_out else "Build failed"
            state_file.unlink(missing_ok=True)
            with log_path.open("rb") as log:
                size = log_path.stat().st_size
                log.seek(max(0, size - 262144))
                output = ("[Earlier build output truncated]\n" if size > 262144 else "") + log.read().decode(errors="replace")
            raise RequestError("Build timed out; simulation not started" if timed_out else "Build failed; simulation not started",
                               "build", 504 if timed_out else 422, build_output=output)
        after = self.build_inputs()
        # CMake can discover new dependencies or regenerate compile commands.
        if any(after.get(p) != digest for p, digest in before.items() if p != str(self.build / "compile_commands.json")):
            state_file.unlink(missing_ok=True)
            raise RequestError("Sources changed during build; save and click Run again", "build", 409)
        built_binary = self.file_hash(native)
        if built_binary is None:
            state_file.unlink(missing_ok=True)
            raise RequestError("Build did not produce firmware_sim; simulation not started", "build", 503)
        rebuilt = bool(changed or binary != built_binary or clean)
        state_file.write_text(json.dumps({"inputs": after, "binary": built_binary}, sort_keys=True) + "\n")
        self.build_message = f"Rebuilt in {elapsed:.3f} s" if rebuilt else "Build up to date"
        return {"message": self.build_message,
                "elapsed_s": elapsed, "rebuilt": rebuilt,
                "firmware_hash": after[str(SIM.parent / "src/detour_test.c")][:12]}

    def base(self, name):
        if not isinstance(name, str) or not NAME.fullmatch(name):
            raise RequestError("Select a scenario name, not a file path", "base")
        path = self.scenarios / (name + ".json")
        if path.resolve().parent != self.scenarios.resolve() or not path.is_file():
            raise RequestError("Scenario does not exist inside sim/scenarios", "base", 404)
        return load(path)[0]

    def checked(self, payload, saving=False):
        expected = {"base", "scenario", "name"} if saving else {"base", "scenario"}
        if not isinstance(payload, dict) or set(payload) != expected:
            raise RequestError("Expected base and scenario" + (" and name" if saving else ""))
        raw = payload["scenario"]
        validate(unwrap(raw, "$", []))  # Exactly the existing executable schema.
        base = self.base(payload["base"])
        field = difference(locked_part(base), locked_part(raw))
        if field:
            raise RequestError(field + ": read-only; only environment fields can change", field)
        return copy.deepcopy(raw)

    def save(self, payload):
        raw = self.checked(payload, saving=True)
        name = payload["name"]
        if not isinstance(name, str) or not NAME.fullmatch(name):
            raise RequestError("Use 1–64 characters from [a-z0-9_-]", "name")
        raw["name"] = name
        validate(unwrap(raw, "$", []))
        if not self.run_lock.acquire(blocking=False):
            raise RequestError("Simulator/save is busy; try again after it finishes", status=409)
        temporary = None
        try:
            # Atomic publication without overwrite, including existing symlinks.
            with tempfile.NamedTemporaryFile(mode="w", dir=self.scenarios, prefix=".web-save-", suffix=".tmp", delete=False) as f:
                temporary = Path(f.name)
                json.dump(raw, f, indent=2, sort_keys=True, allow_nan=False)
                f.write("\n")
            try:
                os.link(temporary, self.scenarios / (name + ".json"))
            except FileExistsError:
                raise RequestError("That scenario name already exists; nothing was overwritten", "name", 409)
            return {"name": name, "scenario": raw}
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
            self.run_lock.release()

    def run(self, payload):
        raw = self.checked(payload)
        if not self.run_lock.acquire(blocking=False):
            raise RequestError("Simulator/save is busy; one simulation runs at a time", status=409)
        try:
            build = self.ensure_build()
            digest = hashlib.sha256(json.dumps(raw, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:10]
            self.started_runs += 1
            run_id = f"{self.session}-{self.started_runs:04d}-{digest}"
            folder = self.output / run_id
            folder.mkdir(parents=True, exist_ok=False)
            scenario = folder / "scenario.json"
            scenario.write_text(json.dumps(raw, indent=2, sort_keys=True, allow_nan=False) + "\n")
            with (folder / "console.txt").open("w") as console:
                self.process = subprocess.Popen([str(self.build / "robot_sim"), "--scenario", str(scenario), "--output", str(folder)],
                                                stdout=console, stderr=subprocess.PIPE, text=True, shell=False, start_new_session=True)
                try:
                    _, stderr = self.process.communicate(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.communicate()
                    raise RequestError("Simulator outer timeout; run files retained", status=504)
                finally:
                    self.process = None
            (folder / "stderr.txt").write_text(stderr)
            if not (folder / "trace.json").is_file():
                raise RequestError("No trace produced: " + stderr[-4000:], status=500)
            trace = json.loads((folder / "trace.json").read_text())
            steps = trace.get("firmware_observation", {}).get("detour_timeline", [])
            result = (steps[-1]["step_name"] + (" " + steps[-1]["fail_name"] if steps[-1]["fail"] else "")) if steps else "NO_DETOUR (see console)"
            console_text = "".join(e["text"] for e in trace["events"] if e["type"] == "text")
            stopped = re.search(r"\[run\] \d+ ms, stopped by ([^\n]+)", console_text)
            if stopped and (not steps or steps[-1]["step_name"] not in ("DONE", "FAILED")):
                result = "RUN STOP: " + stopped[1]
            contacts = [e for e in trace["events"] if e["type"] == "contact"]
            if contacts:
                result += f" · CONTACT at {contacts[0]['t_us']/1e6:.6f}s"
            if not trace["complete"]:
                result += " · INCOMPLETE " + trace.get("error", "")
            plain = unwrap(raw, "$", [])
            sizes = [f"{o['id']}: {o['size_mm'][1]}×{o['size_mm'][0]} mm (W×D)" if o["type"] == "box" else f"{o['id']}: bottle Ø{o['diameter_mm']} mm" for o in plain["obstacles"]]
            summary = re.findall(r"^\[run\] detour [^\n]+", console_text, re.MULTILINE)
            item = {"id": run_id, "base": payload["base"], "name": raw["name"], "sizes": sizes, "result": result,
                    "firmware_summary": summary[-1] if summary else "No firmware summary (short or calibration-only run)",
                    "firmware_hash": build["firmware_hash"], "build_message": build["message"]}
            with self.history_lock:
                self.history[run_id] = item
            return {"run": item, "trace": trace, "build": build}
        finally:
            self.run_lock.release()


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def verify_request(self, request, client_address):
        return client_address[0] == "127.0.0.1"


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def send(self, status, content, kind="application/json"):
        data = content if isinstance(content, bytes) else json.dumps(content, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", kind + ("; charset=utf-8" if kind.startswith("text/") else ""))
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "SAMEORIGIN")
        if kind == "text/html" and self.path == "/":
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; frame-src 'self'; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def check_origin(self):
        authority = "127.0.0.1" if self.server.server_port == 80 else f"127.0.0.1:{self.server.server_port}"
        if self.client_address[0] != "127.0.0.1" or self.headers.get("Host") != authority:
            raise RequestError("Only the loopback address and matching Host are accepted", status=403)
        origin = self.headers.get("Origin")
        if origin is not None and origin != "http://" + authority:
            raise RequestError("Cross-origin requests are not accepted", status=403)
        parsed = urlsplit(self.path)
        if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
            raise RequestError("Query strings and file paths are not accepted", status=404)
        return unquote(parsed.path)

    def dispatch(self, post=False):
        try:
            path = self.check_origin()
            app = self.server.editor
            if post:
                if self.headers.get("Transfer-Encoding"):
                    raise RequestError("Transfer-Encoding is not supported")
                length = self.headers.get("Content-Length", "")
                if not length.isdecimal():
                    raise RequestError("Content-Length is required", status=411)
                if int(length) > MAX_BODY:
                    raise RequestError("Request exceeds the 1 MiB size limit", status=413)
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise RequestError("Expected application/json", status=415)
                data = self.rfile.read(int(length))
                payload = json.loads(data, object_pairs_hook=unique_object, parse_constant=lambda s: fail("$", "non-finite JSON number " + s))
                if path == "/api/run":
                    return self.send(200, app.run(payload))
                if path == "/api/save":
                    return self.send(201, app.save(payload))
                raise RequestError("Unknown endpoint", status=404)
            assets = {"/": ("editor.html", "text/html"), "/editor.js": ("editor.js", "text/javascript"), "/editor.css": ("editor.css", "text/css")}
            if path in assets:
                file, kind = assets[path]
                return self.send(200, (HERE / file).read_bytes(), kind)
            if path == "/favicon.ico":
                return self.send(204, b"", "image/x-icon")
            if path == "/api/status":
                return self.send(200, {"build_error": app.build_error(), "build_message": app.build_message, "busy": app.run_lock.locked(), "started_runs": app.started_runs})
            if path == "/api/scenarios":
                names = [p.stem for p in sorted(app.scenarios.glob("*.json")) if NAME.fullmatch(p.stem) and p.resolve().parent == app.scenarios.resolve()]
                return self.send(200, names)
            if path.startswith("/api/scenario/"):
                return self.send(200, app.base(path[len("/api/scenario/"):]))
            if path == "/api/history":
                with app.history_lock:
                    history = list(app.history.values())
                return self.send(200, history)
            match = re.fullmatch(r"/runs/([a-z0-9_-]+)/(run\.html|trace\.json|scenario\.json)", path)
            if match:
                with app.history_lock:
                    known = match[1] in app.history
                if known:
                    file = app.output / match[1] / match[2]
                    if file.resolve().parent.parent == app.output.resolve() and file.is_file():
                        kind = "text/html" if match[2].endswith("html") else "application/json"
                        return self.send(200, file.read_bytes(), kind)
            raise RequestError("Not a permitted page or session run", status=404)
        except RequestError as exc:
            body = {"error": str(exc), "field": exc.field}
            if exc.build_output is not None:
                body["build_output"] = exc.build_output
            self.send(exc.status, body)
        except (ValueError, TypeError, KeyError, RecursionError) as exc:
            message = str(exc)
            self.send(400, {"error": message, "field": message.split(":", 1)[0]})
        except OSError as exc:
            self.send(500, {"error": str(exc), "field": "$"})

    def do_GET(self):
        self.dispatch()

    def do_POST(self):
        self.dispatch(post=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=SIM / "build")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("port must be in [0, 65535]")
    server = Server(("127.0.0.1", args.port), Handler)
    server.editor = Editor(args.build)
    print(f"http://127.0.0.1:{server.server_port}", flush=True)
    if error := server.editor.build_error():
        print(error, file=sys.stderr, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        process = server.editor.process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        server.server_close()


if __name__ == "__main__":
    main()
