"""Use the real Make options; do not invoke a firmware compiler."""
import subprocess
import unittest
from test_line_p_near_make import NearOption


class BaseOption(NearOption):
    def test_base_defaults(self):
        a, default = self.probe()
        b, explicit = self.probe(LINE_BASE="120", LINE_P_NEAR="85")
        self.assertEqual(a.returncode, 0, a.stderr)
        self.assertEqual(b.returncode, 0, b.stderr)
        self.assertEqual(default, explicit)
        self.assertNotIn("-DLINE_BASE", default["flags"])
        self.assertNotIn("_base", default["image"])

    def test_base_candidates_and_bounds(self):
        for base in (83, 100, 146, 378):
            result, out = self.probe(LINE_BASE=str(base))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(out["image"].endswith(f"_base{base}"))
            self.assertIn(f"-base{base}", out["profile"])
            self.assertNotIn("LINE_BASE", out["flags"])
            result = subprocess.run(
                ["make", "-s", "-f", "probe.mk",
                 "mtkernel_3/app_program/vehicle/vehicle_task.o",
                 "SMP=0", "WIFI=none", f"LINE_BASE={base}"],
                cwd=self.build, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(f"-DLINE_BASE={base}", result.stdout)
            self.assertIn("-DLINE_P_NEAR=85", result.stdout)

    def test_base_rejects(self):
        for base in ("", "0", "62", "82", "379", "1000", "0100", "-1",
                     "120.5", "x", "100 120"):
            result, _ = self.probe(LINE_BASE=base)
            self.assertNotEqual(result.returncode, 0, base)
            self.assertIn("LINE_BASE", result.stderr)

    def test_base_combination(self):
        result, out = self.probe(LINE_BASE="100", LINE_P_NEAR="60")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(out["image"].endswith("_pnear60_base100"))
        self.assertIn("-pnear60-base100", out["profile"])
        result, out = self.probe(LINE_P_NEAR="60")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(out["image"].endswith("_pnear60"))

    def test_base_invalidates(self):
        self.probe(LINE_BASE="146")
        obj = self.build / "mtkernel_3/app_program/vehicle/sentinel.o"
        obj.parent.mkdir(parents=True, exist_ok=True)
        obj.write_bytes(b"stale")
        self.probe(LINE_BASE="100")
        self.assertFalse(obj.exists())
        obj.write_bytes(b"stale")
        self.probe(LINE_BASE="120")
        self.assertFalse(obj.exists())


if __name__ == "__main__":
    unittest.main()
