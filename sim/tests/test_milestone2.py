"""Motion equations, production IR model, no tunnelling, and real following."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / "core"))
from protocol import normalise
from scenario import load
from runner import source_hashes, text_lines

LAUNCHER = Path(sys.argv[1]).resolve()
PROBE = LAUNCHER.parent / "sim_model_probe"


def polygon(pose, front, rear, width):
    angle = math.radians(pose["heading_deg"])
    c, s = math.cos(angle), math.sin(angle)
    return [(pose["x_mm"] + x*c - y*s, pose["y_mm"] + x*s + y*c)
            for x, y in ((front,width/2),(rear,width/2),(rear,-width/2),(front,-width/2))]


def overlaps(a, b):
    for points in (a, b):
        for p, q in zip(points, points[1:] + points[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            pa = [x*axis[0] + y*axis[1] for x, y in a]
            pb = [x*axis[0] + y*axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class MotionIR(unittest.TestCase):
    def setUp(self):
        self.raw, self.scenario, _ = load(SIM / "scenarios/no_box_straight.json")

    def probe_world(self, scenario, mode="ir"):
        # The model probe reads only the world section of the private protocol.
        data = normalise(scenario).splitlines()[1 + len(scenario["events"]):]
        return subprocess.run([str(PROBE), mode], input="\n".join(data) + "\n",
                              text=True, capture_output=True, timeout=5)

    def test_equations(self):
        result = subprocess.run([str(PROBE)], text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        records = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(records), 5)
        self.assertAlmostEqual(records[-1]["reference_coast_counts"][0], 27.5, places=5)
        self.assertAlmostEqual(records[-1]["reference_coast_counts"][1], 23, places=5)

    def test_real_sensor_gap_and_adc(self):
        poses = [(0, [202,2615,191]), (-11, [202,185,191]), (-22, [1645,185,191]), (9, [202,1400,191])]
        for y, expected in poses:
            with self.subTest(y=y):
                self.scenario["robot"]["start_pose"]["y_mm"] = y
                result = self.probe_world(self.scenario)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(result.stdout)["raw"], expected)
        bad = copy.deepcopy(self.scenario)
        bad["robot"]["ir"]["points_mm"] = [[119,14],[119,0],[119,-14]]
        result = self.probe_world(bad)
        self.assertEqual(result.returncode, 2)
        self.assertIn("real IR model failed", result.stderr)

    def test_tile_join_cross_and_curve(self):
        self.scenario["robot"]["start_pose"]["x_mm"] = 196
        result = self.probe_world(self.scenario)
        self.assertEqual(json.loads(result.stdout)["raw"][1], 2615)
        tile = self.scenario["track"]["tiles"][1]
        tile.update(type="cross")
        self.scenario["robot"]["start_pose"]["x_mm"] = 38.5
        result = self.probe_world(self.scenario)
        self.assertEqual(json.loads(result.stdout)["raw"], [1645,2615,3424])
        tile.update(type="curve")
        d = 157.5/math.sqrt(2)
        self.scenario["robot"]["start_pose"].update(x_mm=d-119,y_mm=d-157.5)
        result = self.probe_world(self.scenario)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["raw"][1], 2615)

    def test_thin_box_and_circle_cannot_tunnel(self):
        for obstacle in (
            {"id":"thin", "type":"box", "pose":{"x_mm":200,"y_mm":0,"heading_deg":30},
             "size_mm":[0.001,160], "contact":"freeze-both", "echo":"ideal-sector"},
            {"id":"bottle", "type":"bottle", "pose":{"x_mm":200,"y_mm":0,"heading_deg":0},
             "diameter_mm":66, "contact":"freeze-both", "echo":"ideal-sector"},
        ):
            self.scenario["obstacles"] = [obstacle]
            result = self.probe_world(self.scenario, "contact")
            self.assertEqual(result.returncode, 0, result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines()]
            self.assertEqual(records[0]["type"], "contact")
            self.assertGreaterEqual(records[0]["clearance_mm"], 0)
            self.assertLess(records[0]["clearance_mm"], 0.01)
            self.assertTrue(records[-1]["contact_latched"])

    def run_scenario(self, name, output):
        result = subprocess.run([str(LAUNCHER), "--scenario", str(SIM / "scenarios" / name),
                                 "--output", str(output)], text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads((output / "trace.json").read_text())

    def test_real_calibration_follow_and_repeatability(self):
        before = source_hashes(SIM.parent)
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            root = Path(directory)
            trace = self.run_scenario("no_box_straight.json", root / "a")
            self.run_scenario("no_box_straight.json", root / "b")
            self.assertEqual((root / "a/trace.json").read_bytes(), (root / "b/trace.json").read_bytes())
            lines = list(text_lines(trace["events"]))
            text = "\n".join(line for _, line in lines)
            self.assertIn("[calib] OK. Line reads HIGHER", text)
            self.assertIn("[run] start, run 1", text)
            self.assertIn("stopped by time limit", text)
            self.assertIn("searches 0", text)
            self.assertFalse(any(e["type"] == "contact" for e in trace["events"]))
            start = next(t for t, line in lines if line.startswith("[run] start"))
            end = next(t for t, line in lines if "stopped by time limit" in line)
            snapshots = [e for e in trace["events"] if e["type"] == "world"]
            self.assertEqual([e["t_us"] for e in snapshots], list(range(0,40000001,10000)))
            run = [e for e in snapshots if start <= e["t_us"] <= end]
            self.assertLess(max(abs(e["y_mm"]) for e in run), 30)
            self.assertGreater(run[-1]["x_mm"]-run[0]["x_mm"], 5000)
        self.assertEqual(source_hashes(SIM.parent), before)

    def test_real_box_contact_and_no_penetration(self):
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            trace = self.run_scenario("box_straight.json", Path(directory) / "box")
            contacts = [e for e in trace["events"] if e["type"] == "contact"]
            self.assertEqual(len(contacts), 1)
            contact = contacts[0]
            self.assertEqual(contact["obstacle_id"], "box")
            geometry = self.scenario["robot"]["geometry"]
            box = polygon({"x_mm":800,"y_mm":0,"heading_deg":0},60,-60,120)
            stopped = [e for e in trace["events"] if e["type"] == "world" and e["t_us"] >= contact["t_us"]]
            self.assertTrue(stopped)
            for snapshot in stopped:
                self.assertTrue(snapshot["contact"])
                self.assertEqual(snapshot["x_mm"], stopped[0]["x_mm"])
                self.assertEqual(snapshot["counts"], stopped[0]["counts"])
                outline = polygon(snapshot, geometry["front_x_mm"], geometry["rear_x_mm"], geometry["width_mm"])
                self.assertFalse(overlaps(outline, box))
            text = "".join(e["text"] for e in trace["events"] if e["type"] == "text")
            self.assertIn("stopped by left wheel stalled", text)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]], verbosity=2)
