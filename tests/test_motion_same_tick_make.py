"""MOTION_SAME_TICK make option: on by default for robot images, off for
BENCH_KERNEL=1; 0 gives the old timing with a suffix. No compiler calls."""
import subprocess
import unittest
from test_barcode_map import MakeOption

HEADER = "-include ../app_program/subsystem2/motion_same_tick.h"
IMPL = "-include ../app_program/subsystem2/motor_ctl_same_tick_impl.h"


class SameTickOption(MakeOption):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        with (cls.build / "probe.mk").open("a") as out:
            out.write('mtkernel_3/app_program/subsystem2/motion.o:\n'
                      '\t@echo "motion_flags=$(CFLAGS)"\n'
                      'mtkernel_3/app_program/subsystem2/motor_ctl.o:\n'
                      '\t@echo "motor_flags=$(CFLAGS)"\n')

    def objects(self, **options):
        args = ["make", "-s", "-f", "probe.mk",
                "mtkernel_3/app_program/subsystem2/motion.o",
                "mtkernel_3/app_program/subsystem2/motor_ctl.o",
                "SMP=0", "WIFI=none"]
        args += [f"{k}={v}" for k, v in options.items()]
        return subprocess.run(args, cwd=self.build, text=True,
                              capture_output=True)

    def test_default_on(self):
        a, default = self.probe()
        b, explicit = self.probe(MOTION_SAME_TICK="1")
        self.assertEqual(a.returncode, 0, a.stderr)
        self.assertEqual(default, explicit)
        self.assertNotIn("sametick", default["image"] + default["profile"])
        objs = self.objects()
        self.assertEqual(objs.returncode, 0, objs.stderr)
        self.assertIn(HEADER, objs.stdout)
        self.assertIn(IMPL, objs.stdout)

    def test_off_is_old_timing(self):
        a, default = self.probe()
        r, off = self.probe(MOTION_SAME_TICK="0")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(off["image"], default["image"] + "_sametick0")
        self.assertIn("-sametick0", off["profile"])
        objs = self.objects(MOTION_SAME_TICK="0")
        self.assertEqual(objs.returncode, 0, objs.stderr)
        self.assertNotIn("same_tick", objs.stdout)

    def test_bench_kernel(self):
        r, out = self.probe(BENCH_KERNEL="1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("sametick", out["image"] + out["profile"])
        objs = self.objects(BENCH_KERNEL="1")
        self.assertNotIn("same_tick", objs.stdout)
        r, _ = self.probe(BENCH_KERNEL="1", MOTION_SAME_TICK="1")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("MOTION_SAME_TICK", r.stderr)

    def test_rejects(self):
        for value in ("", "2", "x", "0 1"):
            r, _ = self.probe(MOTION_SAME_TICK=value)
            self.assertNotEqual(r.returncode, 0, value)
            self.assertIn("MOTION_SAME_TICK", r.stderr)

    def test_profile_invalidates(self):
        self.probe()
        obj = self.build / "mtkernel_3/app_program/subsystem2/sentinel.o"
        obj.parent.mkdir(parents=True, exist_ok=True)
        obj.write_bytes(b"stale")
        self.probe(MOTION_SAME_TICK="0")
        self.assertFalse(obj.exists())


if __name__ == "__main__":
    unittest.main()
