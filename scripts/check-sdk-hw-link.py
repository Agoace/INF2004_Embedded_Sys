#!/usr/bin/env python3
"""Retain an SDK hardware probe in a separate ELF. Never run or flash it.

Shared template deviations: docs/architecture/TEMPLATE_CHANGES.md; checked separately by
scripts/check-template-changes.py. This probe checks SDK runtime boundaries.
"""
import argparse
from pathlib import Path
import shlex
import subprocess
import re


def profile_from_compile_command(command):
    """Reconstruct the exact schema from flags, never from the current marker.

    Migration supplies FORCE_CORE0; staging supplies the complete barcode
    family. A merged build supplies both, with placement before barcode keys.
    """
    binary = {"CNF_SMP", "TM_CONSOLE_USB_CDC", "TM_WIFI_CYW43", "TM_NET_LWIP",
              "TM_MQTT", "TM_CONSOLE_UART", "APP_TEST", "APP_BENCH", "APP_DIAG",
              "APP_IMU_STREAM", "APP_FORCE_CORE0", "APP_BARCODE_CAPTURE",
              "BARCODE_LIVE_ENABLE"}
    relevant = binary | {"IMU_HUMP_SOURCE", "JUNCTION_TEST_CMD",
                         "BARCODE_MEASURED_CLEAR_MM"}
    definitions = {}
    for flag in command:
        if not flag.startswith("-D"):
            continue
        name, separator, value = flag[2:].partition("=")
        if name not in relevant:
            continue
        value = value if separator else "1"
        if name in definitions and definitions[name] != value:
            raise ValueError(f"log has conflicting {name} definitions")
        definitions[name] = value
    for name in binary:
        if name in definitions and definitions[name] not in ("0", "1"):
            raise ValueError(f"log has invalid {name}")
    smp = definitions.get("CNF_SMP", "0")
    console = "usb_cdc" if definitions.get("TM_CONSOLE_USB_CDC") == "1" else "uart"
    wifi = "cyw43" if definitions.get("TM_WIFI_CYW43") == "1" else "none"
    net = "lwip" if definitions.get("TM_NET_LWIP") == "1" else "none"
    mqtt = definitions.get("TM_MQTT", "0")
    crash = definitions.get("TM_CONSOLE_UART", "0")
    test = definitions.get("APP_TEST", "0")
    bench = definitions.get("APP_BENCH", "0")
    diag = definitions.get("APP_DIAG", "0")
    stream = definitions.get("APP_IMU_STREAM", "0")
    source = definitions.get("IMU_HUMP_SOURCE", "1")
    junction = definitions.get("JUNCTION_TEST_CMD", "0")
    if source not in ("1", "2"):
        raise ValueError("log has invalid IMU_HUMP_SOURCE")
    if junction not in ("0", "1", "2", "3", "4"):
        raise ValueError("log has invalid JUNCTION_TEST_CMD")
    expected = (f"smp{smp}-{console}-wifi{wifi}-net{net}-mqtt{mqtt}-crash{crash}"
                f"-test{test}-bench{bench}-diag{diag}-imu_stream{stream}"
                f"-hump_source{source}-junction{junction}")
    if "APP_FORCE_CORE0" in definitions:
        expected += f"-forcecore{definitions['APP_FORCE_CORE0']}"
    barcode = {"APP_BARCODE_CAPTURE", "BARCODE_LIVE_ENABLE",
               "BARCODE_MEASURED_CLEAR_MM"}
    present = barcode & definitions.keys()
    if present:
        if present != barcode:
            raise ValueError("log has an incomplete barcode profile family")
        capture = definitions["APP_BARCODE_CAPTURE"]
        live = definitions["BARCODE_LIVE_ENABLE"]
        clear = definitions["BARCODE_MEASURED_CLEAR_MM"]
        if (len(clear) > 5 or re.fullmatch(r"0|[1-9][0-9]*", clear) is None or
                int(clear) > 65535):
            raise ValueError("log has an invalid canonical barcode bound")
        if live == "1" and (clear == "0" or junction != "0"):
            raise ValueError("log combines live actions with no bound or fake source")
        expected += f"-barcode_capture{capture}-bclive{live}-bcclear{clear}"
    return expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_log", type=Path, help="full make log for the current object profile")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="directory outside the repository")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = args.output_dir.resolve()
    if out == root or root in out.parents:
        parser.error("keep probe artifacts outside the repository")
    out.mkdir(parents=True, exist_ok=True)
    commands = [shlex.split(line) for line in args.build_log.read_text().splitlines()
                if line.startswith("arm-none-eabi-gcc ")]
    compile_cmd = next((cmd for cmd in commands if "-c" in cmd and
                        any("hardware_adc/adc.c" in part for part in cmd)), None)
    link_cmd = next((cmd for cmd in commands if "-c" not in cmd and
                     any(part.endswith(".elf") for part in cmd)), None)
    if compile_cmd is None or link_cmd is None:
        parser.error("log needs the adc.c compile command and final ELF link; "
                     "clean and rebuild first")
    try:
        expected = profile_from_compile_command(compile_cmd)
    except ValueError as error:
        parser.error(str(error))
    current = (root / "build_make/.build_profile").read_text().strip()
    if current != expected:
        parser.error(f"object profile is {current}, but log is {expected}; "
                     "rebuild the logged profile first")
    cmd = list(compile_cmd)
    for option in ("-MF", "-MT", "-o"):
        while option in cmd:
            index = cmd.index(option)
            del cmd[index:index + 2]
    cmd = [part for part in cmd if not part.endswith(".c")]
    obj = out / "sdk-hw-probe.o"
    cmd += ["-Wall", "-Wextra", "-Wconversion", "-Wshadow", "-Wformat=2", "-Werror",
            "-o", str(obj), str(root / "tests/probes/sdk_hw_link_probe.c")]
    subprocess.run(cmd, cwd=root / "build_make", check=True)
    elf = out / "sdk-hw-probe.elf"
    link_cmd[link_cmd.index("-o") + 1] = str(elf)
    link_cmd = [("-Wl,-Map," + str(out / "sdk-hw-probe.map"))
                if part.startswith("-Wl,-Map,") else part for part in link_cmd]
    link_cmd += [str(obj), "-Wl,--undefined=app_sdk_hw_link_probe"]
    subprocess.run(link_cmd, cwd=root / "build_make", check=True)
    symbols = subprocess.check_output(["arm-none-eabi-nm", str(elf)], text=True)
    (out / "symbols.txt").write_text(symbols)
    names = {line.split()[-1] for line in symbols.splitlines() if line.split()}
    for name in ("app_sdk_hw_link_probe", "adc_init", "gpio_set_function", "tick_start",
                 "drv_line_hw_init", "drv_line_hw_read", "drv_ultrasonic_hw_pulse",
                 "drv_watchdog_hw_start", "watchdog_enable", "watchdog_update",
                 "watchdog_caused_reboot", "watchdog_enable_caused_reboot"):
        if name not in names:
            raise SystemExit(f"missing retained symbol: {name}")
    undefined = subprocess.check_output(["arm-none-eabi-nm", "-u", str(elf)], text=True)
    if undefined.strip():
        raise SystemExit("unresolved symbols: " + undefined)
    probe_refs = subprocess.check_output(["arm-none-eabi-nm", "-u", str(obj)], text=True)
    forbidden = {"clock_get_hz", "irq_set_exclusive_handler", "timer_time_us_64", "runtime_init"}
    if forbidden & {line.split()[-1] for line in probe_refs.splitlines() if line.split()}:
        raise SystemExit("probe requires a forbidden SDK runtime function")
    print(f"PASS {expected}: retained PWM/GPIO/ADC/alarm/ultrasonic/reset/"
          f"timer32/tick/watchdog probe links: {elf}")
    print("Link-only result. Hardware tests remain NOT RUN. Do not flash this ELF.")


if __name__ == "__main__":
    main()
