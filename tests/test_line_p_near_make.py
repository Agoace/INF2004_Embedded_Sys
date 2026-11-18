"""Exercise the real Make option in an isolated tree, no compiler/SDK calls."""
import subprocess
import unittest
from test_barcode_map import MakeOption


class NearOption(MakeOption):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        with (cls.build / "probe.mk").open("a") as out:
            out.write('mtkernel_3/app_program/vehicle/vehicle_task.o:\n'
                      '\t@echo "vehicle_flags=$(CFLAGS)"\n')

    def test_near_defaults(self):
        a, default = self.probe()
        b, explicit = self.probe(LINE_P_NEAR="85")
        self.assertEqual(a.returncode, 0, a.stderr)
        self.assertEqual(b.returncode, 0, b.stderr)
        self.assertEqual(default, explicit)
        self.assertNotIn("pnear", default["image"] + default["profile"])
        self.assertNotIn("LINE_P_NEAR", default["flags"])

    def test_near_candidates(self):
        for gain in (60, 40):
            result, out = self.probe(LINE_P_NEAR=str(gain))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(out["image"].endswith(f"_pnear{gain}"))
            self.assertIn(f"-pnear{gain}", out["profile"])
            self.assertNotIn("LINE_P_NEAR", out["flags"])
            result = subprocess.run(
                ["make", "-s", "-f", "probe.mk",
                 "mtkernel_3/app_program/vehicle/vehicle_task.o",
                 "SMP=0", "WIFI=none", f"LINE_P_NEAR={gain}"],
                cwd=self.build, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(f"-DLINE_P_NEAR={gain}", result.stdout)
            self.assertIn("-include ../app_program/vehicle/line_p_near.h",
                          result.stdout)

    def test_near_rejects(self):
        for value in ("", "-1", "86", "256", "060", "1.5", "x", "60 40"):
            result, _ = self.probe(LINE_P_NEAR=value)
            self.assertNotEqual(result.returncode, 0, value)
            self.assertIn("LINE_P_NEAR", result.stderr)

    def test_near_profile_invalidates(self):
        self.probe(LINE_P_NEAR="60")
        obj = self.build / "mtkernel_3/app_program/vehicle/sentinel.o"
        obj.parent.mkdir(parents=True, exist_ok=True)
        obj.write_bytes(b"stale")
        self.probe(LINE_P_NEAR="40")
        self.assertFalse(obj.exists())
        obj.write_bytes(b"stale")
        self.probe()
        self.assertFalse(obj.exists())


if __name__ == "__main__":
    unittest.main()
