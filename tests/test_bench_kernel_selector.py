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


    def test_kernel_ram_is_up_only_opt_in(self):
        for mode in ("0", "1"):
            with self.subTest(mode=mode):
                ok = self.preflight(mode, SMP="0", KERNEL_RAM="1")
                self.assertEqual(ok.returncode, 0, ok.stderr)
        for mode, smp, ram in (("1", "1", "1"), ("0", "1", "1"),
                               ("1", "0", "2"), ("1", "0", "")):
            with self.subTest(mode=mode, smp=smp, ram=ram):
                self.assertNotEqual(self.preflight(
                    mode, SMP=smp, KERNEL_RAM=ram).returncode, 0)

    def test_old_bench_kernel_ram_name_rejected(self):
        self.assertNotEqual(self.preflight(
            "1", SMP="0", BENCH_KERNEL_RAM="1").returncode, 0)


LISTS = ROOT / "tools/kernel_ram"


def entries(name):
    rows = []
    for line in (LISTS / name).read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            rows.append(line.split())
    return rows


def symbols(*names):
    out = set()
    for name in names:
        for row in entries(name):
            out.update(row[2:] or [row[1][len(".text."):]])
    return out


class RamHotPath(unittest.TestCase):
    """The lists name real objects and every approved hot function."""

    def test_objects_have_sources(self):
        for name in ("common.list", "robot.list", "bench.list"):
            for row in entries(name):
                with self.subTest(list=name, row=row):
                    obj, section = row[0], row[1]
                    self.assertTrue(obj.endswith(".o"))
                    self.assertTrue(section == ".text" or
                                    section.startswith(".text.") or
                                    section.startswith(".time_critical."))
                    if section.startswith(".time_critical.") or \
                            section == ".text":
                        self.assertGreater(len(row), 2, "symbols needed")
                    if obj.startswith("tinyusb/"):
                        continue  # external TINYUSB_PATH sources
                    base = ROOT / obj[:-2]
                    self.assertTrue(base.with_suffix(".c").exists() or
                                    base.with_suffix(".S").exists(), obj)

    def test_common_functions(self):
        names = symbols("common.list")
        for name in ("knl_dispatch_entry", "knl_dispatch", "disint",
                     "ClearInt", "knl_hll_inthdr", "knl_systim_inthdr",
                     "knl_timer_handler", "knl_timer_insert_reltim",
                     "tk_wup_tsk", "tk_slp_tsk", "tk_can_wup", "tk_loc_mtx",
                     "tk_unl_mtx", "knl_make_ready", "knl_make_non_ready",
                     "knl_wait_release_ok", "alarm_irq", "gpio_dispatch",
                     "encoder_irq", "drv_time_us", "drv_time_hw_us"):
            with self.subTest(name=name):
                self.assertIn(name, names)

    def test_robot_and_bench_providers(self):
        robot = symbols("robot.list")
        bench = symbols("bench.list")
        for name in ("drv_line_hw_alarm", "drv_line_hw_ack",
                     "drv_line_hw_read", "drv_gpio_irq_hw_ack",
                     "drv_gpio_irq_hw_pending", "drv_motor_hw_ab",
                     "echo_irq_handler", "dcd_rp2040_irq",
                     "dcd_event_handler", "hw_endpoint_xfer_continue",
                     "_hw_endpoint_buffer_control_update32",
                     "hw_endpoint_reset_transfer", "tu_fifo_write",
                     "tud_event_hook_cb", "sync_ep_buffer",
                     "hw_endpoint_start_next_buffer", "prepare_ep_buffer"):
            with self.subTest(name=name):
                self.assertIn(name, robot)
        for name in ("bk_gpio_ack", "bk_gpio_pending", "bk_gpio_ab",
                     "bk_line_timer_ack", "bk_line_timer_alarm",
                     "drv_line_hw_read", "drv_motor_hw_ab"):
            with self.subTest(name=name):
                self.assertIn(name, bench)
        robot_objs = {r[0] for r in entries("robot.list")}
        self.assertFalse(any("bench_kernel" in o for o in robot_objs))
        bench_objs = {r[0] for r in entries("bench.list")}
        self.assertTrue(all("bench_kernel" in o for o in bench_objs))


class KernelRamRule(unittest.TestCase):
    """Evaluate the real make rule block: rows per profile, none by default."""

    def rows(self, **args):
        text = (ROOT / "build_make/pico_rp2040.mk").read_text()
        block = text[text.index("# Kernel hot path in RAM: rule"):
                     text.index("# End of kernel hot path in RAM")]
        with tempfile.NamedTemporaryFile(mode="w", suffix=".mk") as out:
            out.write(block + "\nprint:\n\t@echo $(KERNEL_RAM_ROWS)\n")
            out.flush()
            return subprocess.check_output(
                ["make", "--no-print-directory", "-f", out.name, "print"] +
                [f"{k}={v}" for k, v in args.items()],
                cwd=ROOT / "build_make", text=True).split()

    def test_default_has_no_rows(self):
        self.assertEqual(self.rows(), [])
        self.assertEqual(self.rows(KERNEL_RAM="0", BENCH_KERNEL="1"), [])

    def test_profile_lists(self):
        robot = " ".join(self.rows(KERNEL_RAM="1", BENCH_KERNEL="0"))
        bench = " ".join(self.rows(KERNEL_RAM="1", BENCH_KERNEL="1"))
        for text in (robot, bench):
            self.assertIn("kernel/tkernel/task_sync.o:.text.tk_wup_tsk", text)
        self.assertIn("subsystem3/drv_line_hw.o", robot)
        self.assertNotIn("bench_kernel", robot)
        self.assertIn("bench_kernel/bench_gpio.o", bench)
        self.assertNotIn("subsystem3/drv_line_hw.o", bench)


if __name__ == "__main__":
    unittest.main()
