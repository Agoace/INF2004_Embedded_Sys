#!/usr/bin/env python3
"""Retain an SDK hardware probe in a separate ELF. Never run or flash it.

Shared template deviations: docs/architecture/TEMPLATE_CHANGES.md; checked separately by
scripts/check-template-changes.py. This probe checks SDK runtime boundaries.
"""
import argparse
from pathlib import Path
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_log", type=Path, help="full make log for the current object profile")
    parser.add_argument("--output-dir", type=Path, required=True, help="directory outside the repository")
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
        parser.error("log needs the adc.c compile command and final ELF link; clean and rebuild first")
    flags = set(compile_cmd)
    smp = int("-DCNF_SMP=1" in flags)
    console = "usb_cdc" if "-DTM_CONSOLE_USB_CDC=1" in flags else "uart"
    wifi = "cyw43" if "-DTM_WIFI_CYW43=1" in flags else "none"
    net = "lwip" if "-DTM_NET_LWIP=1" in flags else "none"
    mqtt = int("-DTM_MQTT=1" in flags)
    crash = int("-DTM_CONSOLE_UART=1" in flags)
    test = int("-DAPP_TEST=1" in flags)
    bench = int("-DAPP_BENCH=1" in flags)
    diag = int("-DAPP_DIAG=1" in flags)
    expected = f"smp{smp}-{console}-wifi{wifi}-net{net}-mqtt{mqtt}-crash{crash}-test{test}-bench{bench}-diag{diag}"
    stream = int("-DAPP_IMU_STREAM=1" in flags)
    source = next((flag.split("=", 1)[1] for flag in flags
                   if flag.startswith("-DIMU_HUMP_SOURCE=")), "1")
    if source not in ("1", "2"):
        parser.error("log has an invalid IMU_HUMP_SOURCE")
    junction = next((flag.split("=", 1)[1] for flag in flags
                     if flag.startswith("-DJUNCTION_TEST_CMD=")), "0")
    if junction not in ("0", "1", "2", "3", "4"):
        parser.error("log has an invalid JUNCTION_TEST_CMD")
    expected += f"-imu_stream{stream}-hump_source{source}-junction{junction}"
    capture = int("-DAPP_BARCODE_CAPTURE=1" in flags)
    expected += f"-barcode_capture{capture}"
    current = (root / "build_make/.build_profile").read_text().strip()
    if current != expected:
        parser.error(f"object profile is {current}, but log is {expected}; rebuild the logged profile first")
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
    print(f"PASS {expected}: retained PWM/GPIO/ADC/alarm/ultrasonic/reset/timer32/tick/watchdog probe links: {elf}")
    print("Link-only result. Hardware tests remain NOT RUN. Do not flash this ELF.")


if __name__ == "__main__":
    main()
