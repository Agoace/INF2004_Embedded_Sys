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
                    "../app_program/platform/drv_gpio_irq_hw.c\n")
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
                    "../app_program/platform/drv_gpio_irq_hw.c"]
        self.assertEqual(self.sources(""), expected)
        self.assertEqual(self.sources("0"), expected)

    def test_bench_includes_owned_report_and_providers(self):
        sources = self.sources("1")
        self.assertEqual(sources, ["../app_program/app_main.c",
                         "../app_program/bench_kernel/bench_report.c",
                         "../app_program/platform/drv_gpio_irq_hw.c"])


if __name__ == "__main__":
    unittest.main()
