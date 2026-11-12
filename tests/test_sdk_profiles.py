"""M6: full CLI profile gate; compiler/nm are isolated doubles, never firmware."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "sdk_check", ROOT / "scripts/check-sdk-hw-link.py")
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)
BASE = ("smp{smp}-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0-bench0-diag0"
        "-imu_stream0-hump_source1-junction{junction}")
SYMBOLS = ("app_sdk_hw_link_probe adc_init gpio_set_function tick_start "
           "drv_line_hw_init drv_line_hw_read drv_ultrasonic_hw_pulse "
           "drv_watchdog_hw_start watchdog_enable watchdog_update "
           "watchdog_caused_reboot watchdog_enable_caused_reboot").split()


def flags(smp="1", force="0", barcode=None, junction="0"):
    result = [f"-DCNF_SMP={smp}", "-DTM_CONSOLE_USB_CDC=1",
              "-DTM_WIFI_CYW43=1", "-DTM_NET_LWIP=1", "-DTM_MQTT=1",
              f"-DJUNCTION_TEST_CMD={junction}"]
    if force is not None:
        result.append(f"-DAPP_FORCE_CORE0={force}")
    if barcode is not None:
        result.extend(f"-D{name}={value}" for name, value in zip(
            ("APP_BARCODE_CAPTURE", "BARCODE_LIVE_ENABLE",
             "BARCODE_MEASURED_CLEAR_MM"), barcode))
    return result


def marker(smp="1", force="0", barcode=None, junction="0"):
    value = BASE.format(smp=smp, junction=junction)
    if force is not None:
        value += f"-forcecore{force}"
    if barcode is not None:
        value += (f"-barcode_capture{barcode[0]}-bclive{barcode[1]}"
                  f"-bcclear{barcode[2]}")
    return value


class ProfileGate(unittest.TestCase):
    def gate(self, definitions, current, accepted, complete=True, linked=True):
        parent = ROOT / "build/sdk-profile-tests"
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as folder:
            root = Path(folder) / "source"
            (root / "build_make").mkdir(parents=True)
            (root / "build_make/.build_profile").write_text(current)
            log = Path(folder) / "full.log"
            compile_line = ("arm-none-eabi-gcc " + " ".join(definitions) +
                            " -c /sdk/hardware_adc/adc.c -o adc.o\n")
            log.write_text((compile_line if complete else "") +
                           ("arm-none-eabi-gcc -o firmware.elf adc.o\n"
                            if linked else ""))
            out = Path(folder) / "probe"
            argv = ["check-sdk-hw-link.py", str(log), "--output-dir", str(out)]

            def nm(command, **_kwargs):
                return "" if "-u" in command else "\n".join(
                    f"00000000 T {name}" for name in SYMBOLS)

            with mock.patch.object(CHECK, "__file__", str(root / "scripts/check.py")), \
                    mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(CHECK.subprocess, "run") as compiler, \
                    mock.patch.object(CHECK.subprocess, "check_output", side_effect=nm), \
                    contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                if accepted:
                    CHECK.main()
                    self.assertEqual(compiler.call_count, 2)
                else:
                    with self.assertRaises(SystemExit) as error:
                        CHECK.main()
                    self.assertNotEqual(error.exception.code, 0)
                    compiler.assert_not_called()

    def test_migration_placements(self):
        for smp, force in (("0", "0"), ("1", "0"), ("1", "1")):
            with self.subTest(smp=smp, force=force):
                self.gate(flags(smp, force), marker(smp, force), True)

    def test_staging_and_combined(self):
        for force in (None, "0", "1"):
            for barcode in (("0", "0", "0"), ("1", "0", "0"),
                            ("0", "1", "150")):
                with self.subTest(force=force, barcode=barcode):
                    self.gate(flags(force=force, barcode=barcode),
                              marker(force=force, barcode=barcode), True)

    def test_mismatched_keys(self):
        current = marker(barcode=("0", "0", "0"))
        for old, new in (("forcecore0", "forcecore1"), ("smp1", "smp0"),
                         ("junction0", "junction1"),
                         ("barcode_capture0", "barcode_capture1"),
                         ("bclive0", "bclive1"), ("bcclear0", "bcclear150"),
                         ("hump_source1", "hump_source2"),
                         ("imu_stream0", "imu_stream1"), ("test0", "test1"),
                         ("bench0", "bench1"), ("diag0", "diag1"),
                         ("mqtt1", "mqtt0"), ("crash0", "crash1"),
                         ("usb_cdc", "uart"), ("wificyw43", "wifinone"),
                         ("netlwip", "netnone")):
            with self.subTest(key=old):
                self.gate(flags(barcode=("0", "0", "0")),
                          current.replace(old, new), False)

    def test_missing_or_conflicting_definitions(self):
        self.gate(flags(force=None), marker(), False)
        self.gate(flags() + ["-DAPP_FORCE_CORE0=1"], marker(), False)
        self.gate(flags() + ["-DAPP_BARCODE_CAPTURE=0"], marker(), False)
        self.gate(flags() + ["-DBARCODE_LIVE_ENABLE=0"], marker(), False)
        self.gate(flags() + ["-DBARCODE_MEASURED_CLEAR_MM=0"], marker(), False)
        self.gate(flags(force="2"), marker(force="2"), False)

    def test_bad_barcode_permission(self):
        cases = (("2", "0", "0"), ("0", "2", "0"), ("0", "1", "0"),
                 ("0", "0", "01"), ("0", "0", "-1"), ("0", "0", "+1"),
                 ("0", "0", "65536"), ("0", "0", "١"))
        for barcode in cases:
            with self.subTest(barcode=barcode):
                self.gate(flags(barcode=barcode), marker(barcode=barcode), False)
        self.gate(flags(barcode=("0", "1", "1"), junction="1"),
                  marker(barcode=("0", "1", "1"), junction="1"), False)

    def test_full_log_required(self):
        self.gate(flags(), marker(), False, complete=False)
        self.gate(flags(), marker(), False, linked=False)


if __name__ == "__main__":
    unittest.main()
