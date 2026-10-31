"""Schema, device, time, and real-firmware integration checks."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / "core"))
from scenario import ScenarioError, load, validate
from runner import source_hashes

LAUNCHER = Path(sys.argv[1]).resolve()


class Milestone1(unittest.TestCase):
    def setUp(self):
        self.raw, self.scenario, self.assumptions = load(SIM / "scenarios/box_straight.json")

    def test_schema_and_assumptions(self):
        self.assertGreater(len(self.assumptions), 10)
        self.assertFalse(any(a["path"].endswith("line_width_mm") for a in self.assumptions))
        for path in ("end_us", "track", "robot", "events", "noise"):
            bad = copy.deepcopy(self.scenario)
            del bad[path]
            with self.assertRaises(ScenarioError):
                validate(bad)
        bad = copy.deepcopy(self.scenario)
        del bad["robot"]["motors"]["coast_tau_ms"]
        with self.assertRaises(ScenarioError):
            validate(bad)
        mutations = [
            lambda s: s.update(schema_version=2),
            lambda s: s.update(clock_quantum_us=True),
            lambda s: s.update(seed=-1),
            lambda s: s["events"].reverse(),
            lambda s: s["events"][0].update(pin=19),
            lambda s: s["track"]["tiles"][0].update(rotation_deg=False),
            lambda s: s["robot"]["ir"]["floor_adc"].__setitem__(0, 4096),
            lambda s: s["robot"]["geometry"].update(rear_x_mm=-50),
            lambda s: s["robot"]["ultrasonic"].update(trigger_gap_us=1),
            lambda s: s["noise"].update(enabled=True),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutate):
                bad = copy.deepcopy(self.scenario)
                mutate(bad)
                with self.assertRaises(ScenarioError):
                    validate(bad)

    def test_gap_consistency(self):
        validate(self.scenario)  # 22 mm succeeds.
        self.scenario["robot"]["ir"]["points_mm"] = [[119, 14], [119, 0], [119, -14]]
        with self.assertRaisesRegex(ScenarioError, "gap consistency"):
            validate(self.scenario)

    def test_duplicate_and_invalid_assumption(self):
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            path = Path(directory) / "invalid.json"
            path.write_text('{"schema_version":1,"schema_version":1}')
            with self.assertRaisesRegex(ScenarioError, "duplicate"):
                load(path)
            raw = copy.deepcopy(self.raw)
            raw["robot"]["ir"]["footprint_mm"].pop("note")
            path.write_text(json.dumps(raw))
            with self.assertRaises(ScenarioError):
                load(path)

    def test_native_contracts(self):
        normalised = "5000000000 100 202 185 191 2\n500 21 1\n4000 21 0\n0\n0\n"
        result = subprocess.run([str(LAUNCHER.parent / "sim_clock_probe")],
                                input=normalised, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        events = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(events[-1], {"t_us": 5000000000, "type": "end",
                                     "reason": "scenario_end", "complete": True})
        self.assertTrue(any(event["type"] == "probe_passed" for event in events))
        self.assertEqual([e["t_us"] for e in events if e["type"] == "button"], [500, 4000])

    def test_real_firmware_determinism_and_read_only(self):
        before = source_hashes(SIM.parent)
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            traces = []
            for run in ("a", "b"):
                output = Path(directory) / run
                result = subprocess.run([str(LAUNCHER), "--scenario", str(SIM / "scenarios/box_straight.json"),
                                         "--output", str(output)], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stderr)
                traces.append((output / "trace.json").read_bytes())
            self.assertEqual(traces[0], traces[1])
            trace = json.loads(traces[0])
            self.assertTrue(trace["complete"])
            self.assertEqual(trace["events"][-1]["t_us"], 40000000)
            text = "".join(e["text"] for e in trace["events"] if e["type"] == "text")
            self.assertIn("Obstacle detour test.", text)
            self.assertIn("[spin] step 1: left", text)
            self.assertIn("[calib] OK. Line reads HIGHER", text)
            self.assertNotIn("[run] Calibration failed:", text)
            self.assertIn("[run] start,", text)
            self.assertIn("[detour] FAILED", text)
            self.assertEqual(trace["firmware_observation"]["step_crosscheck"]["status"], "match")
            self.assertEqual(trace["header"]["source_hashes"], before)
            self.assertEqual([e["t_us"] for e in trace["events"] if e["type"] == "button"], [2100000, 2300000])
        self.assertEqual(source_hashes(SIM.parent), before)

    def test_gp20_and_short_scenario(self):
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            root = Path(directory)
            self.raw["events"][0]["pin"] = self.raw["events"][1]["pin"] = 20
            self.raw["end_us"] = 4000001
            path = root / "gp20.json"
            path.write_text(json.dumps(self.raw))
            result = subprocess.run([str(LAUNCHER), "--scenario", str(path), "--output", str(root / "out")],
                                    text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("[spin] step 1: left", result.stdout)
            self.assertNotIn("[run] Calibration failed:", result.stdout)
            trace = json.loads((root / "out/trace.json").read_text())
            self.assertEqual([e for e in trace["events"] if e["type"] == "world"][-1]["t_us"], 4000001)

    def test_cpu_watchdog(self):
        # A temporary native-process replacement deliberately never reads time.
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            root = Path(directory)
            for name in ("target.json", "compiler.txt", "compile_commands.json"):
                (root / name).write_bytes((LAUNCHER.parent / name).read_bytes())
            native = root / "firmware_sim"
            native.write_text(f"#!{sys.executable}\nwhile True: pass\n")
            native.chmod(0o755)
            code = (f"import sys;sys.dont_write_bytecode=True;sys.path.insert(0,{str(SIM / 'core')!r});"
                    f"from runner import main;main({str(SIM.parent)!r},{str(root)!r})")
            result = subprocess.run([sys.executable, "-c", code, "--scenario",
                                     str(SIM / "scenarios/box_straight.json"), "--output", str(root / "out")],
                                    text=True, capture_output=True, timeout=9)
            self.assertEqual(result.returncode, 1, result.stderr)
            trace = json.loads((root / "out/trace.json").read_text())
            self.assertFalse(trace["complete"])
            self.assertTrue((root / "out/run.html").is_file())
            self.assertRegex(trace["error"], r"native exit -(24|9)")


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
