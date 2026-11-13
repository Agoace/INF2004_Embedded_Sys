#!/usr/bin/env python3
"""Run the real SIM guard/source fragment in isolated host-only Make trees."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BUILD = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else ROOT / "build"
HW = {
    "subsystem2/drv_motor_hw.c", "subsystem3/drv_line_hw.c",
    "subsystem3/barcode_hw.c", "subsystem4/imu_hw.c",
    "subsystem5/drv_ultrasonic_hw.c", "platform/drv_gpio_irq_hw.c",
}
SIM = {name.replace("_hw.c", "_sim.c") for name in HW}


class MakeSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        BUILD.mkdir(parents=True, exist_ok=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="pil-make-", dir=BUILD)
        cls.tree = Path(cls.tmp.name)
        cls.cwd = cls.tree / "build_make"
        cls.cwd.mkdir()
        (cls.cwd / "app_sdk_hw.mk").write_text("# no SDK in this host test\n")
        for path in (ROOT / "app_program").glob("*.c"):
            dest = cls.tree / "app_program" / path.name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.touch()
        nested = list((ROOT / "app_program").glob("*/*.c"))
        for path in nested:
            if path.parent.name == "sim" or path.name.endswith("_sim.c"):
                continue
            dest = cls.tree / "app_program" / path.relative_to(ROOT / "app_program")
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.touch()
        # Let GNU Make evaluate the ORIGINAL wildcard before adding new files.
        # Its glob order is locale-dependent; Python lexical order is not proof.
        original = subprocess.run(["make", "--no-print-directory", "-f", "-"],
            input="print:\n\t@echo $(wildcard ../app_program/*.c) "
                  "$(wildcard ../app_program/*/*.c)\n",
            cwd=cls.cwd, capture_output=True, text=True, check=True)
        cls.original_sources = original.stdout.split()
        for path in nested:
            dest = cls.tree / "app_program" / path.relative_to(ROOT / "app_program")
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.touch()
        mk = (ROOT / "build_make/pico_rp2040.mk").read_text()
        guard = "SIM ?= 0" + mk.split("SIM ?= 0", 1)[1].split("# RT-0075", 1)[0]
        flags = "ifeq ($(SIM),1)\nCFLAGS  += -DAPP_SIM=1\nendif\n"
        profile = "ifeq ($(SIM),1)\nPROFILE_ID := $(PROFILE_ID)-sim1\nendif\n"
        assert flags in mk and profile in mk
        source = ROOT / "build_make/mtkernel_3/app_program/subdir.mk"
        cls.program = (".DEFAULT_GOAL := print\nCONSOLE ?= usb_cdc\n"
                       "CRASH_REPORT ?= 0\nPROFILE_ID := original\n" + guard +
                       flags + profile + f"include {source}\n" +
                       "print:\n\t@echo SOURCES=$(TEMP_SRCS)\n"
                       "\t@echo FLAGS=$(CFLAGS)\n"
                       "\t@echo PROFILE=$(PROFILE_ID)\n"
                       "\t@echo SUFFIX=$(SIM_SUFFIX)\n")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_make(self, *args):
        return subprocess.run(["make", "--no-print-directory", "-f", "-", *args],
                              input=self.program, text=True, cwd=self.cwd,
                              capture_output=True, check=False)

    def test_default_equals_explicit_zero(self):
        base = self.run_make()
        zero = self.run_make("SIM=0")
        self.assertEqual(base.returncode, 0, base.stderr)
        self.assertEqual(base.stdout, zero.stdout)
        sources = base.stdout.split("SOURCES=", 1)[1].splitlines()[0].split()
        actual = [p.removeprefix("../app_program/") for p in sources]
        self.assertEqual(sources, self.original_sources)
        self.assertTrue(HW.issubset(actual))
        self.assertIn("PROFILE=original\n", base.stdout)
        self.assertNotIn("APP_SIM", base.stdout)

    def test_sim_selection(self):
        result = self.run_make("SIM=1")
        self.assertEqual(result.returncode, 0, result.stderr)
        sources = result.stdout.split("SOURCES=", 1)[1].splitlines()[0].split()
        names = {p.removeprefix("../app_program/") for p in sources}
        self.assertTrue(SIM.issubset(names))
        self.assertFalse(HW & names)
        self.assertIn("platform/drv_time_hw.c", names)
        self.assertIn("platform/drv_watchdog_hw.c", names)
        self.assertIn("sim/pil_wire.c", names)
        self.assertIn("FLAGS=-DAPP_SIM=1", result.stdout)
        self.assertIn("PROFILE=original-sim1", result.stdout)
        self.assertIn("SUFFIX=_sim", result.stdout)

    def test_forbidden(self):
        for args in [("SIM=1", "CRASH_REPORT=1"),
                     ("SIM=1", "CONSOLE=uart"),
                     ("SIM=",), ("SIM=2",), ("SIM=0 1",)]:
            with self.subTest(args=args):
                result = self.run_make(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("SIM", result.stderr)


if __name__ == "__main__":
    unittest.main()
