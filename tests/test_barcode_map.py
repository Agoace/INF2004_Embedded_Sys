"""BARCODE_MAP: converter unit tests and the real make option block."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/barcode_map/barcode_map.py"
sys.path.insert(0, str(TOOL.parent))


def none26():
    return [0] * 26


class Convert(unittest.TestCase):
    def conv(self, text):
        import barcode_map
        return barcode_map.convert(text)

    def test_readable_form(self):
        want = none26()
        want[ord("F") - 65] = 1
        want[ord("K") - 65] = 2
        self.assertEqual(self.conv("F=LEFT,K=RIGHT"), want)

    def test_all_actions_case_and_spaces(self):
        want = none26()
        want[0:5] = [0, 1, 2, 3, 4]
        self.assertEqual(
            self.conv(" a=none, B=Left ,c=RIGHT,D=straight,e=UTURN "), want)

    def test_rejects(self):
        import barcode_map
        for bad in ("", ",", "F=LEFT,", "F", "F=", "=LEFT", "1=LEFT",
                    "AA=LEFT", "F=JUMP", "F=LEFT,F=RIGHT", "f=LEFT,F=LEFT",
                    "F=LEFT=RIGHT"):
            with self.assertRaises(barcode_map.MapError, msg=bad):
                self.conv(bad)

    def test_cli(self):
        ok = subprocess.run([sys.executable, str(TOOL), "A=UTURN"],
                            text=True, capture_output=True)
        self.assertEqual(ok.returncode, 0, ok.stderr)
        self.assertEqual(ok.stdout.strip(), ",".join(["4"] + ["0"] * 25))
        bad = subprocess.run([sys.executable, str(TOOL), "F=JUMP"],
                             text=True, capture_output=True)
        self.assertEqual(bad.returncode, 1)
        self.assertIn("JUMP", bad.stderr)
        self.assertEqual(bad.stdout, "")


class MakeOption(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (ROOT / "build/wip").mkdir(parents=True, exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="bmap-make-",
                                              dir=ROOT / "build/wip")
        tree = Path(cls.tmp.name)
        names = subprocess.check_output(
            ["git", "ls-files", "build_make", "tools/barcode_map",
             "tools/kernel_ram"], cwd=ROOT, text=True).splitlines()
        for name in names:
            if (name.endswith((".mk", "/makefile", ".py", ".list"))):
                target = tree / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / name, target)
        cls.build = tree / "build_make"
        (cls.build / "probe.mk").write_text(
            'include pico_rp2040.mk\n.PHONY: show\nshow:\n'
            '\t@echo "image=$(EXE_FILE)"\n'
            '\t@echo "profile=$(PROFILE_ID)"\n'
            '\t@echo "flags=$(CFLAGS)"\n'
            '\t@echo "bwflags=$(BARCODE_MAP_CFLAGS)"\n')

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def probe(self, **options):
        args = ["make", "-s", "-f", "probe.mk", "show", "SMP=0", "WIFI=none"]
        args += [f"{k}={v}" for k, v in options.items()]
        r = subprocess.run(args, cwd=self.build, text=True,
                           capture_output=True)
        out = dict(x.split("=", 1) for x in r.stdout.splitlines()
                   if "=" in x and x.split("=", 1)[0] in
                   ("image", "profile", "flags", "bwflags"))
        return r, out

    def test_default_adds_nothing(self):
        r, out = self.probe()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("bmap", out["image"] + out["profile"])
        self.assertNotIn("BW_MAP", out["flags"])
        self.assertEqual(out["bwflags"], "")
        r2, out2 = self.probe(BARCODE_MAP="")
        self.assertEqual(out, out2)

    def test_valid_map(self):
        r, out = self.probe(BARCODE_MAP="F=LEFT,K=RIGHT")
        self.assertEqual(r.returncode, 0, r.stderr)
        vals = none26()
        vals[5], vals[10] = 1, 2
        text = ",".join(map(str, vals))
        self.assertIn(f"-DBW_MAP_VALUES={text}", out["bwflags"])
        self.assertIn("-DBW_MAP_POLICY_ID=2u", out["bwflags"])
        self.assertNotIn("BW_MAP", out["flags"])
        self.assertTrue(out["image"].endswith("_bmap2"), out["image"])
        self.assertIn("-bmap" + "".join(map(str, vals)) + "-id2",
                      out["profile"])
        r, out = self.probe(BARCODE_MAP="F=LEFT", BARCODE_MAP_ID="7")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("-DBW_MAP_POLICY_ID=7u", out["bwflags"])
        self.assertTrue(out["image"].endswith("_bmap7"))

    def test_with_kernel_ram(self):
        r, out = self.probe(BARCODE_MAP="F=LEFT", KERNEL_RAM="1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue(out["image"].endswith("_kernel_ram_bmap2"))

    def test_errors(self):
        cases = [dict(BARCODE_MAP="Q=JUMP"),
                 dict(BARCODE_MAP="1=LEFT"),
                 dict(BARCODE_MAP="F=LEFT,F=RIGHT"),
                 dict(BARCODE_MAP="F=LEFT", BARCODE_MAP_ID="0"),
                 dict(BARCODE_MAP="F=LEFT", BARCODE_MAP_ID="1"),
                 dict(BARCODE_MAP="F=LEFT", BARCODE_MAP_ID="256"),
                 dict(BARCODE_MAP="F=LEFT", BARCODE_MAP_ID="x"),
                 dict(BARCODE_MAP_ID="3")]
        for case in cases:
            r, _ = self.probe(**case)
            self.assertNotEqual(r.returncode, 0, case)
            self.assertIn("BARCODE_MAP", r.stderr, case)


if __name__ == "__main__":
    unittest.main()
