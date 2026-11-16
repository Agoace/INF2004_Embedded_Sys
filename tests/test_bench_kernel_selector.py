#!/usr/bin/env python3
"""Keep every bench object out of ordinary links, including empty objects."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BenchSelector(unittest.TestCase):
    def sources(self, mode):
        # Evaluate the real source selector, not a duplicated Python filter.
        text = (ROOT / "build_make/mtkernel_3/app_program/subdir.mk").read_text()
        selector = text[text.index("TEMP_SRCS ="):text.index("TEMP_OBJS =")]
        # Inject a report object even before SW7's report package is present.
        fixtures = ("TEMP_SRCS = ../app_program/app_main.c "
                    "../app_program/bench_kernel/bench_report.c "
                    "../app_program/platform/drv_gpio_irq_hw.c "
                    "../app_program/subsystem2/drv_motor_hw.c "
                    "../app_program/subsystem3/drv_line_hw.c "
                    "../app_program/subsystem4/imu_hw.c "
                    "../app_program/subsystem5/drv_ultrasonic_hw.c\n")
        selector = fixtures + selector.split("\n", 1)[1]
        makefile = selector + "\nall:\n\t@echo $(TEMP_SRCS)\n"
        with tempfile.NamedTemporaryFile(mode="w", suffix=".mk") as out:
            out.write(makefile)
            out.flush()
            return subprocess.check_output(
                ["make", "--no-print-directory", "-f", out.name,
                 f"BENCH_KERNEL={mode}"], cwd=ROOT / "build_make",
                text=True).split()

    def test_ordinary_order_and_objects_unchanged(self):
        expected = ["../app_program/app_main.c",
                    "../app_program/platform/drv_gpio_irq_hw.c",
                    "../app_program/subsystem2/drv_motor_hw.c",
                    "../app_program/subsystem3/drv_line_hw.c",
                    "../app_program/subsystem4/imu_hw.c",
                    "../app_program/subsystem5/drv_ultrasonic_hw.c"]
        self.assertEqual(self.sources(""), expected)
        self.assertEqual(self.sources("0"), expected)

    def test_bench_includes_owned_report_and_providers(self):
        sources = self.sources("1")
        self.assertEqual(sources, ["../app_program/app_main.c",
                         "../app_program/bench_kernel/bench_report.c"])


class BenchMode(unittest.TestCase):
    def preflight(self, mode, **changes):
        text = (ROOT / "build_make/pico_rp2040.mk").read_text()
        block = text[text.index("# Explicit spare-Pico"):
                     text.index("#\n# Images are named")]
        args = dict(WIFI="none", NET="none", MQTT="0", CONSOLE="usb_cdc",
                    CRASH_REPORT="0", BENCH="0", TEST="0", DIAG="0",
                    IMU_STREAM="0", BARCODE_CAPTURE="0",
                    BARCODE_LIVE_ENABLE="0", JUNCTION_TEST_CMD="NONE")
        args.update(changes)
        if mode is not None:
            args["BENCH_KERNEL"] = mode
        with tempfile.NamedTemporaryFile(mode="w", suffix=".mk") as out:
            out.write(block + "\nall:\n\t@echo $(BENCH_KERNEL)\n")
            out.flush()
            return subprocess.run(
                ["make", "--no-print-directory", "-f", out.name] +
                [f"{k}={v}" for k, v in args.items()],
                capture_output=True, text=True)

    def test_default_zero_and_explicit_one(self):
        self.assertEqual(self.preflight(None).stdout.strip(), "0")
        self.assertEqual(self.preflight("0").returncode, 0)
        self.assertEqual(self.preflight("1").returncode, 0)

    def test_invalid_mode(self):
        for mode in ("", "2", "0 1", "yes"):
            with self.subTest(mode=mode):
                self.assertNotEqual(self.preflight(mode).returncode, 0)

    def test_conflicting_profiles_rejected(self):
        for key, value in dict(WIFI="cyw43", NET="lwip", MQTT="1",
                CONSOLE="uart", CRASH_REPORT="1", BENCH="1", TEST="1",
                DIAG="1", SIM="1", PIL_GATE1="1", IMU_STREAM="1",
                BARCODE_CAPTURE="1", BARCODE_LIVE_ENABLE="1",
                JUNCTION_TEST_CMD="FAKE").items():
            with self.subTest(key=key):
                self.assertNotEqual(
                    self.preflight("1", **{key: value}).returncode, 0)


    def test_ram_variant_is_up_bench_only(self):
        self.assertEqual(self.preflight("1", SMP="0").returncode, 0)
        ok = self.preflight("1", SMP="0", BENCH_KERNEL_RAM="1")
        self.assertEqual(ok.returncode, 0, ok.stderr)
        for mode, smp, ram in (("0", "0", "1"), ("1", "1", "1"),
                               ("1", "0", "2"), ("1", "0", "")):
            with self.subTest(mode=mode, smp=smp, ram=ram):
                self.assertNotEqual(self.preflight(
                    mode, SMP=smp, BENCH_KERNEL_RAM=ram).returncode, 0)


class RamHotPath(unittest.TestCase):
    """The RAM list names real objects and every approved hot function."""

    def entries(self):
        rows = []
        for line in (ROOT / "app_program/bench_kernel/ram_hot_path.list"
                     ).read_text().splitlines():
            if line.strip() and not line.startswith("#"):
                rows.append(line.split())
        return rows

    def test_objects_have_sources(self):
        for row in self.entries():
            with self.subTest(row=row):
                obj = row[0]
                self.assertTrue(obj.endswith(".o"))
                base = ROOT / obj[:-2]
                self.assertTrue(base.with_suffix(".c").exists() or
                                base.with_suffix(".S").exists(), obj)
                self.assertTrue(row[1] == ".text" or
                                row[1].startswith(".text."))

    def test_approved_functions_listed(self):
        symbols = set()
        for row in self.entries():
            symbols.update(row[2:] or [row[1][len(".text."):]])
        for name in ("knl_dispatch_entry", "knl_dispatch", "disint",
                     "knl_hll_inthdr", "knl_systim_inthdr",
                     "knl_timer_handler", "tk_wup_tsk", "tk_slp_tsk",
                     "tk_can_wup", "tk_loc_mtx", "tk_unl_mtx",
                     "knl_make_ready", "knl_make_non_ready",
                     "knl_wait_release_ok", "alarm_irq", "gpio_dispatch",
                     "encoder_irq", "drv_line_hw_read", "bk_gpio_ack",
                     "drv_time_us", "drv_time_hw_us", "ClearInt",
                     "bk_line_timer_ack", "bk_line_timer_alarm",
                     "drv_motor_hw_ab", "bk_gpio_ab",
                     "knl_timer_insert_reltim"):
            with self.subTest(name=name):
                self.assertIn(name, symbols)


if __name__ == "__main__":
    unittest.main()
