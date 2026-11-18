#!/usr/bin/env python3
"""Host checks for the CR1 GNU map parser."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/ram_report.py"
SPEC = importlib.util.spec_from_file_location("ram_report", SCRIPT)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class MapTests(unittest.TestCase):
    def test_wrapped_sections_padding_and_ram_vector(self):
        text = """Discarded input sections
 .bss.dead 0x20000000 0x999 ./mtkernel_3/app_program/subsystem2/dead.o
Linker script and memory map
vector          0x20000000 0xc0
 .data.vector   0x20000000 0xc0 ./mtkernel_3/kernel/vector.o
.data           0x200000c0 0x10 load address 0x10000000
 .data.cfg      0x200000c0 0xc ./mtkernel_3/app_program/subsystem2/a.o
 *fill*         0x200000cc 0x4
.bss            0x200000d0 0x30
 .bss.long_symbol
                0x200000d0 0x10 ./mtkernel_3/lwip/core/mem.o
                0x200000d0 ignored_symbol
 .noinit        0x200000e0 0x10 ./mtkernel_3/kernel/tkernel/task.o
 .bss.__sf      0x200000f0 0x8 /opt/libg.a(libc_a-findfp.o)
 .bss.empty     0x200000f8 0x0 ./empty.o
 .text.code     0x10000000 0x999 ./mtkernel_3/flash.o
 .bss.discarded 0x00000000 0x999 ./mtkernel_3/zero.o
"""
        result = REPORT.parse_map(text)
        self.assertEqual(result["static_ram"], 256)
        self.assertEqual(result["alignment_bytes"], 12)
        self.assertEqual(len(result["contributions"]), 5)
        self.assertEqual(result["groups"], {
            "kernel": 208, "subsystem2": 12, "lwIP": 16, "C library": 8})

    def test_module_names(self):
        cases = {"app_program/platform/a.o": "platform",
                 "app_program/vehicle/a.o": "vehicle",
                 "mtkernel_3/app_sdk_hw/adc.o": "SDK",
                 "mtkernel_3/cyw43/pio.o": "radio SDK",
                 "mtkernel_3/tinyusb/device/usbd.o": "USB SDK",
                 "mtkernel_3/lib/liblwip/sys_arch.o": "lwIP",
                 "mtkernel_3/lib/libtm/x.o": "other support"}
        for obj, expected in cases.items():
            with self.subTest(obj=obj):
                self.assertEqual(REPORT.module_for(obj), expected)

    def test_missing_map_body_has_no_measurement(self):
        self.assertEqual(REPORT.parse_map("Discarded input sections\n")
                         ["contributions"], [])


if __name__ == "__main__":
    unittest.main()
