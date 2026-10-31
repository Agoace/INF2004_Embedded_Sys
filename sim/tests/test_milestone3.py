"""Timed echo/servo contracts, real scan, real detour, observational equivalence."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM / "core"), str(SIM / "targets")]
from scenario import load, validate, ScenarioError
from protocol import normalise
from runner import cpu_limit, source_hashes
from detour_decode import decode
LAUNCHER = Path(sys.argv[1]).resolve()
PROBE = LAUNCHER.parent / "sim_ultrasonic_probe"


def box(distance):
    return {"id":"target", "type":"box", "pose":{"x_mm":65+distance+0.5,"y_mm":0,"heading_deg":0},
            "size_mm":[1,120], "contact":"freeze-both", "echo":"ideal-sector"}


class EchoDetour(unittest.TestCase):
    def setUp(self):
        _, self.s, _ = load(SIM / "scenarios/no_box_straight.json")
        self.s["end_us"] = 100000
        self.s["events"] = []

    def probe(self, mode="single", active=True):
        result = subprocess.run([str(PROBE)], input=normalise(self.s,active), capture_output=True,
                                text=True, env={**os.environ,"SIM_PROBE_MODE":mode}, timeout=10, preexec_fn=cpu_limit)
        self.assertEqual(result.returncode,0,result.stderr)
        events = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertTrue(any(e["type"]=="device_probe_passed" for e in events))
        return events

    def test_range_boundaries_and_circle(self):
        for distance in (59,60,511,512):
            for kind in ("box","bottle"):
                with self.subTest(distance=distance,kind=kind):
                    target = box(distance)
                    if kind == "bottle":
                        target.update(type="bottle",diameter_mm=10)
                        target.pop("size_mm")
                        target["pose"]["x_mm"] = 65+distance+5
                    self.s["obstacles"] = [target]
                    events = self.probe()
                    trigger = next(e for e in events if e["type"]=="ultrasonic_trigger")
                    result = next(e for e in events if e["type"]=="probe_result")
                    self.assertAlmostEqual(trigger["distance_mm"],distance,places=6)
                    self.assertEqual(result["latency_us"],215)
                    self.assertEqual(result["distance_mm"],distance)
                    self.assertEqual(result["word"],1536 if distance==512 else (0 if distance==59 else distance))
                    self.assertEqual(result["word"] & 0xC000,0)
                    self.assertEqual(result["falls"],1)
                    self.assertEqual(trigger["ready_us"],215+result["pulse_us"])

    def test_unrounded_surface_limit(self):
        self.s["obstacles"] = [box(511.1)]
        events = self.probe()
        result = next(e for e in events if e["type"]=="probe_result")
        self.assertEqual(result["word"],1536)
        self.assertEqual(result["distance_mm"],511)
        self.assertEqual(result["falls"],1)
        self.s["obstacles"] = [box(59.9)]
        result = next(e for e in self.probe() if e["type"]=="probe_result")
        self.assertEqual(result["word"],0)
        self.assertEqual(result["distance_mm"],60)

    def test_sector_not_centre_ray_and_no_return(self):
        for angle in (12,16):
            target = box(300)
            target["size_mm"] = [1,1]
            target["pose"].update(x_mm=65+300*math.cos(math.radians(angle)),y_mm=300*math.sin(math.radians(angle)))
            self.s["obstacles"] = [target]
            events = self.probe()
            trigger = next(e for e in events if e["type"]=="ultrasonic_trigger")
            self.assertEqual(trigger["hit"],angle==12)
            result = next(e for e in events if e["type"]=="probe_result")
            if angle==12:
                self.assertGreater(trigger["point_mm"][1],50) # The centre ray cannot hit this box.
                self.assertEqual(result["word"] >> 9,0)
            else:
                self.assertEqual(result["word"],1536)
        self.s["obstacles"] = []
        events = self.probe()
        result = next(e for e in events if e["type"]=="probe_result")
        self.assertEqual(result["word"],1536)
        self.assertEqual(result["pulse_us"],0)
        self.assertEqual(result["falls"],0)

    def test_faults_streak_reset_and_immediate_stuck(self):
        self.s["end_us"] = 700000
        self.s["obstacles"] = [box(100)]
        self.s["events"] = [{"at_us":at,"ultrasonic_fault":mode} for at,mode in
                            ((0,"NO_RISE"),(420000,"STUCK_HIGH"),(480000,"NORMAL"),
                             (540000,"NO_RISE"),(600000,"NO_RETURN"))]
        validate(self.s)
        events = self.probe("faults")
        results = [e for e in events if e["type"]=="probe_result"]
        self.assertEqual([(e["word"]>>11)&7 for e in results],[1,2,3,4,5,5,5,5,0,1,0])
        stuck = results[7]
        self.assertEqual(stuck["t_us"],420000)
        self.assertEqual((stuck["word"]>>9)&3,2)

    def test_32bit_wrap_gap_and_busy(self):
        self.s["end_us"] = 4295067296
        self.probe("wrap",active=False)

    def test_real_scan_hand_geometry(self):
        self.s["obstacles"] = [box(300)]
        self.s["obstacles"][0]["size_mm"] = [120,120]
        self.s["obstacles"][0]["pose"]["x_mm"] = 425
        self.s["end_us"] = 3000000
        events = self.probe("scan")
        points = [e["word"] for e in events if e["type"]=="scan_point"]
        self.assertEqual(points,[1536,305,301,300,300,300,300,300,300,300,301,305,1536])
        triggers = [e for e in events if e["type"]=="ultrasonic_trigger"]
        self.assertEqual(len(triggers),26)
        for index in (1,2,6):
            echo = triggers[2*index]
            # Nearest point on front x=365, constrained by sector boundary.
            pulse = [1167,1222,1278,1333,1389,1444,1500,1556,1611,1667,1722,1778,1833][index]
            nearest_angle = max(0,abs((pulse-1500)*0.09)-15)
            expected = 300/math.cos(math.radians(nearest_angle))
            self.assertAlmostEqual(echo["distance_mm"],expected,places=6)
        self.assertFalse(triggers[0]["hit"])

    def test_real_full_loop_and_hook_equivalence(self):
        before = source_hashes(SIM.parent)
        with tempfile.TemporaryDirectory(dir=LAUNCHER.parent) as directory:
            root = Path(directory)
            # Preserve the accepted ideal trajectory; specular has separate checks.
            raw = json.loads((SIM / "scenarios/box_straight.json").read_text())
            raw["obstacles"][0]["echo"] = "ideal"
            ideal = root / "ideal.json"
            ideal.write_text(json.dumps(raw))
            args = ["--scenario",str(ideal)]
            subprocess.run([str(LAUNCHER),*args,"--output",str(root/"hooked")],check=True,capture_output=True,timeout=10)
            trace = json.loads((root/"hooked/trace.json").read_text())
            obs = trace["firmware_observation"]
            self.assertEqual([e["phase"] for e in obs["phase_timeline"]],
                ["NOT_RUNNING","RUN_START_UNRESOLVED","FOLLOW","STOP","SCAN","DETOUR","NOT_RUNNING"])
            self.assertEqual(obs["step_crosscheck"]["status"],"match")
            self.assertEqual(obs["detour_timeline"][-1]["step_name"],"FAILED")
            self.assertEqual(obs["detour_timeline"][-1]["fail_name"],"TIME")
            self.assertFalse(any(e["type"]=="contact" for e in trace["events"]))
            damaged = copy.deepcopy(trace["events"])
            line = next(e for e in damaged if e["type"]=="text" and "TURN_AWAY at" in e["text"])
            line["text"] = line["text"].replace("TURN_AWAY at","BACK at")
            self.assertEqual(decode(damaged)["step_crosscheck"]["status"],"mismatch")
            # Build the same real sources without the caller renames.
            build = root/"unhooked"
            subprocess.run(["cmake","-S",str(SIM),"-B",str(build),"-DSIM_OBSERVE_HOOKS=OFF"],check=True,capture_output=True,timeout=30)
            subprocess.run(["cmake","--build",str(build),"--target","firmware_sim"],check=True,capture_output=True,timeout=30)
            subprocess.run([str(build/"robot_sim"),*args,"--output",str(root/"plain")],check=True,capture_output=True,timeout=10)
            plain = json.loads((root/"plain/trace.json").read_text())
            self.assertEqual([e for e in trace["events"] if e["type"]!="hook"],plain["events"])
        self.assertEqual(source_hashes(SIM.parent),before)

    def test_phase_search_pending_and_idle_boundaries(self):
        events = [
            {"t_us":0,"type":"text","text":"[run] start, run 1\n"},
            {"t_us":1,"type":"hook","call":"steer_reset"},
            {"t_us":2,"type":"hook","call":"lp_update","result":5},
            {"t_us":3,"type":"hook","call":"lp_update","result":6},
            {"t_us":4,"type":"hook","call":"steer_reset"},
            {"t_us":5,"type":"text","text":"[run] 1 ms, stopped by GP21 pressed\n"},
            {"t_us":6,"type":"hook","call":"lp_update","result":5},
        ]
        observed = decode(events)
        self.assertEqual([e["phase"] for e in observed["phase_timeline"]],
                         ["NOT_RUNNING","RUN_START_UNRESOLVED","FOLLOW","SEARCH","FOLLOW","NOT_RUNNING"])
        self.assertEqual(events[2]["phase_last_confirmed"],"FOLLOW")
        self.assertEqual(events[3]["transition_pending"],"FOLLOW reset")
        self.assertEqual(events[-1]["phase_inferred"],"NOT_RUNNING")

    def test_no_silent_servo_slew(self):
        self.s["robot"]["servo"]["instant_angle"] = False
        with self.assertRaises(ScenarioError):
            validate(self.s)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]],verbosity=2)
