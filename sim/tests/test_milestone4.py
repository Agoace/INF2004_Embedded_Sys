"""Offline display fidelity, safe embedding, regeneration, and optional browser checks."""
import copy
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
sys.dont_write_bytecode = True
SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM / "core"), str(SIM / "viewer")]
from render import display_data, write_html
from runner import source_hashes
LAUNCHER = Path(sys.argv[1]).resolve()


class Viewer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(dir=LAUNCHER.parent)
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        cls.before = source_hashes(SIM.parent)
        subprocess.run([str(LAUNCHER), "--scenario", str(SIM / "scenarios/box_straight.json"),
                        "--output", str(cls.root / "box")], check=True, capture_output=True, timeout=15)
        cls.trace_path = cls.root / "box/trace.json"
        cls.trace = json.loads(cls.trace_path.read_text())
        cls.html = cls.root / "box/run.html"
        cls.data = display_data(cls.trace)

    def test_saved_data_fidelity(self):
        data, trace = self.data, self.trace
        embedded = re.search(r'<script id="viewer-data" type="application/json">(.*?)</script>',
                             self.html.read_text(), re.S)[1]
        self.assertEqual(json.loads(embedded), data)
        self.assertEqual(data["steps"], trace["firmware_observation"]["detour_timeline"])
        self.assertEqual(data["phases"], trace["firmware_observation"]["phase_timeline"])
        self.assertEqual(data["console"], [[e["t_us"],e["text"]] for e in trace["events"] if e["type"]=="text"])
        expected = [[e["t_us"],"duty",e["wheel"],e["pct"]] if e["type"]=="duty" else
                    [e["t_us"],"servo",e["pulse_us"]] for e in trace["events"] if e["type"] in ("servo","duty")]
        self.assertEqual(data["commands"], expected)
        self.assertEqual([e["word"] for e in data["dt"]], [e["word"] for e in trace["events"] if e["type"]=="hook" and e["call"] in ("dt_init","dt_update","dt_reading")])
        worlds = [e for e in trace["events"] if e["type"]=="world"]
        self.assertEqual(len(data["world"]),len(worlds))
        for row, original in zip(data["world"],worlds):
            self.assertEqual(row[0],original["t_us"])
            self.assertEqual(row[1],[original["x_mm"],original["y_mm"],original["heading_deg"]])
            self.assertEqual(row[4],original["encoder_counts"])
            self.assertEqual(row[6],original["ir_raw"])
        self.assertLess(self.html.stat().st_size,3000000)

    def test_regeneration_and_read_only_source(self):
        out = self.root / "regenerated.html"
        subprocess.run([sys.executable,str(SIM / "viewer/render.py"),str(self.trace_path),"--output",str(out)],check=True,capture_output=True)
        self.assertEqual(self.html.read_bytes(),out.read_bytes())
        saved = self.trace_path.read_bytes()
        reject = subprocess.run([sys.executable,str(SIM / "viewer/render.py"),str(self.trace_path),"--output",str(self.trace_path)],capture_output=True)
        self.assertNotEqual(reject.returncode,0)
        self.assertEqual(self.trace_path.read_bytes(),saved)
        self.assertEqual(source_hashes(SIM.parent),self.before)

    def test_ghost_frame_and_missing_origin(self):
        for side,expected in ((1,[80,210,120]),(-1,[120,210,60])):
            trace=copy.deepcopy(self.trace)
            base={"type":"hook","call":"dt_init","t_us":1,"step":1,"fail":0,"heading":0,"x_mm":0,"y_mm":0,"side":side,"world_pose":[100,200,90]}
            moved={**base,"call":"dt_update","t_us":2,"x_mm":10,"y_mm":20,"heading":30}
            trace["events"]=[base,moved]
            estimate=display_data(trace)["dt"][-1]["world_estimate"]
            for actual,wanted in zip(estimate,expected):self.assertAlmostEqual(actual,wanted)
            del base["world_pose"]
            self.assertIsNone(display_data(trace)["dt"][-1]["world_estimate"])

    def test_escaping_and_incomplete_trace(self):
        trace=copy.deepcopy(self.trace)
        trace["events"].insert(-1,{"t_us":37000000,"type":"text","text":"</script><b>literal</b><script>window.viewerInjected=true</script>\n"})
        path=self.root/"escaping.html";write_html(trace,path)
        self.assertNotIn('</script><b>literal',path.read_text())
        self.assertIn('\\u003c/script\\u003e\\u003cb\\u003eliteral',path.read_text())
        trace["complete"]=False;trace["error"]="watchdog";trace["events"]=[]
        write_html(trace,self.root/"empty.html")
        data=display_data(trace)
        self.assertFalse(data["meta"]["complete"])
        self.assertEqual(data["world"],[])
        self.assertEqual(data["meta"]["recorded_end_us"],0)

    def test_browser_offline(self):
        if not shutil.which("node"):
            self.skipTest("Node not available; use viewer_browser.cjs with Playwright for browser verification")
        if not os.environ.get("SIM_PLAYWRIGHT_MODULE"):
            result=subprocess.run(["node","-e","require.resolve('playwright')"],capture_output=True)
            if result.returncode:self.skipTest("Set SIM_PLAYWRIGHT_MODULE to an installed Playwright module for browser verification")
        escaped=copy.deepcopy(self.trace)
        escaped["events"].insert(-1,{"t_us":37000000,"type":"text","text":"</script><b>literal</b><script>window.viewerInjected=true</script>\n"})
        escape_path=self.root/"escaped.html";write_html(escaped,escape_path)
        gap=copy.deepcopy(self.trace)
        gap["events"]=[e for e in gap["events"] if not(e["type"]=="world" and 21540000<=e["t_us"]<=21560000)]
        gap_path=self.root/"gap.html";write_html(gap,gap_path)
        subprocess.run(["node",str(SIM/"tests/viewer_browser.cjs"),str(self.html),str(self.trace_path),
                        str(self.root/"screenshots"),str(escape_path),str(gap_path)],check=True,capture_output=True,text=True,timeout=45)
        report=json.loads((self.root/"screenshots/browser_report.json").read_text())
        self.assertEqual(report["errors"],[])
        self.assertEqual(report["network_requests"],[])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]],verbosity=2)
