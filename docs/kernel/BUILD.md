# Build, flash and host checks

## What it is

The current robot uses the Make build in `build_make/`, not the root CMake
probe project. Build files select the image and include the app-owned SDK
hardware subset. [KERNEL.md](KERNEL.md) describes runtime ownership.
Only the operator authorizes a flash or powered motor test.

## Tools and settings

| Tool/setting | Use |
| --- | --- |
| `arm-none-eabi-gcc`, GNU Make, host C/C++ compiler | Firmware and UF2 converter |
| `PICO_SDK_PATH` | SDK outside the repository, including TinyUSB and CYW43 submodules; current adapter evidence uses SDK 2.3.0 |
| Python 3, CMake | Host tests and documentation checks |
| picotool | Load and verify a selected UF2 in ROM BOOTSEL mode |
| Node, Firefox (optional) | Real page-script host tests; offline screenshots |
| `paho-mqtt` 2.x (operator tools only) | Recorder/dashboard connection to an existing broker |

Check `arm-none-eabi-gcc --version` and the SDK path before building. The
[template setup guide](TEMPLATE_SETUP.md) has installation steps. The exhaustive
current SDK source/header list is in
[app_sdk_hw.mk](../../build_make/app_sdk_hw.mk); do not add SDK startup,
IRQ registration, time initialization or a second runtime. See
[SDK hardware subset](../s2-motion/MOTION.md#sdk-hardware-subset).

## Build profiles

Run from the repository root. Use `-j4` for parallel builds.

```sh
cd build_make
make SMP=0 WIFI=none -j4 --output-sync=target
make SMP=0 WIFI=none TEST=1 -j4 --output-sync=target
make SMP=0 WIFI=none BENCH=1 -j4 --output-sync=target
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 -j4 --output-sync=target
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=1 -j4 --output-sync=target
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 DIAG=1 -j4 --output-sync=target
make -j4 --output-sync=target  # default SMP=1, radio on, no lwIP/MQTT
```

Current demo kernel: `demo-kernel-v2`, UP robot with live barcode and the
kernel hot path in RAM. Submission-v4 changes documents only. The live
build has RAM_PROOF PASS 84/84, 23906 B; the fallback without live barcode
has 80/80, 23018 B. See [KERNEL_RAM](KERNEL.md#ram-placement-kernel_ram) and
[demo-day build steps](../release/DEMO_KERNEL_V2.md).

```sh
make -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 \
  BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="M=LEFT,O=RIGHT,P=UTURN" \
  JUNCTION_TEST_CMD=NONE
```

| Option | Default / effect |
| --- | --- |
| `SMP` | 1; use 0 for the single-core robot acceptance profile |
| `CONSOLE` | `usb_cdc`; `uart` reserves GP0/GP1 and forces crash reporting |
| `WIFI` | `cyw43`; `none` leaves radio off |
| `NET` | `none`; `lwip` requires `WIFI=cyw43` |
| `MQTT` | 0; 1 requires `NET=lwip` |
| `TEST` | 0; 1 runs subsystem tests, not the normal vehicle |
| `BENCH` | 0; 1 selects the exclusive motion bench |
| `DIAG` | 0; 1 adds fixed motion timing captures, printed at GP20 |
| `CRASH_REPORT` | 0; 1 mirrors console to UART0, occupying GP0/GP1 |
| `IMU_STREAM` | 0; 1 requires TEST=1, not BENCH; S4 stream only, no other tests |
| `BARCODE_CAPTURE` | 0; 1 selects B1b driven TEST capture; VIN required |
| `JUNCTION_TEST_CMD` | NONE (default) selects source2 (SHADOW unless live enable1); A/B/C/D select exclusive fake START commands with `_jtestA`..`_jtestD` suffixes |
| `BARCODE_LIVE_ENABLE` | 0 default; 1 requires NONE, a non-empty `BARCODE_MAP` and a positive explicit measured bound; opt-in adds `_bclive`; the demo uses 1 |
| `BARCODE_MEASURED_CLEAR_MM` | 0 default; decimal integer0..65535 without leading zeros;150 mm SHADOW label is not a measured default |
| `BARCODE_MAP` | Empty default = built-in table, image unchanged. `"F=LEFT,K=RIGHT"` sets the barcode_window action table (unlisted letters NONE; NONE/LEFT/RIGHT/STRAIGHT/UTURN) via [`barcode_map.py`](../../tools/barcode_map/barcode_map.py); bad letter, action or a repeat stops make. Adds `_bmap<id>`. Check the boot line `[BARCODE_TABLE] id <id> confirmed 1 A=.. Z=..` (0..4 = N/L/R/S/U). |
| `BARCODE_MAP_ID` | 2 default, 2..255; only with `BARCODE_MAP`. Id 1 is the built-in table. |
| `LINE_KD` | 30 default (config value, since 2026-11-17; was 85). 0..85 replaces only the LINE D gain ([D gain trial](../s3-line-barcode/LINE.md#d-gain-trial-line_kd-default-30-since-2026-11-17)). A value other than 30 adds `_kdN`. Not with `BENCH_KERNEL=1`. |
| `MOTION_SAME_TICK` | 1 default for robot images (since 2026-11-17): a new wheel speed applies in the same 10 ms period ([same-tick retarget](../s2-motion/MOTION.md#same-tick-retarget-default-on)). 0 gives the old timing and adds `_sametick0`. `BENCH_KERNEL=1`: default 0, no suffix; 1 stops make. |
| `BARCODE_FAIL_CAPTURE` |0 default, images unchanged.1 is ROBOT only and keeps raw edge/decimated ADC evidence for the first two failed attempts per run. Adds `_bcfail2`. See [failed capture](../s3-line-barcode/FAILED_ATTEMPT_CAPTURE.md). |
| `BARCODE_FAIL_SLOTS` |2 default;1 is the smaller fallback. Used only when failed capture is enabled; changes image/profile suffix and invalidates old objects. Recheck combined RAM/headroom before use. |
| `IMU_HUMP_SOURCE` | 1 corrected (default); 2 filtered roll; other values rejected; 2 adds `_humpfilt` to the image name |

Robot/TEST IMU images require USB console and `CRASH_REPORT=0` because
GP0/GP1 belong to I2C0. BENCH has no IMU task. Follow the current
[50 Hz support/GP21 procedure](../s4-imu/IMU.md#step-2d-current-procedure).
The GP21 drive trial requires VIN and a lifted-wheel stop check first.
It is not the full TEST suite or a navigation feature.

### Barcode B1b driven TEST image

```sh
make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

The filename is `mtk3pico_smp0_usb_cdc_test_barcode_capture.uf2`. Use USB
console and CRASH_REPORT=0 on the connected robot. This profile requires
TEST=1, SMP=0, WIFI=none, NET=none and MQTT=0; BENCH and IMU_STREAM are
rejected. It skips ordinary subsystem tests and the automatic S2 steps,
but **GP21 now drives in this image**. VIN is required. Lift both wheels
for the first check. GP20 records floor/white/black ranges; after those,
GP21 starts a 430 mm forward pass at 100 mm/s, then a fresh press after
the complete dump starts a backward pass. Use only one sheet per run;
swap the 2x F and 141 mm F sheets while stopped. Settings are in
[`barcode_settings.h`](../../app_program/subsystem3/barcode_settings.h);
the target is clamped to <=450 mm for the operator's shorter map. A press
while moving requests stop.
Faults/deadline/capacity stop and inhibit restart. No dump/rearm occurs
until stop API success and two fresh zero-duty/low-speed snapshots.
Busy snapshot reads use the last coherent board for at most two failed
10 ms polls; three failures or source age >30 ms stops. BC_FAULT records
cause bits, read/age values, motor faults/watchdog and actual wake rc.
A false watchdog flag has no startup grace; it must be armed to command.
Capture uses 6144 four-byte decimated records, about 12.3 s at default
2 ms, plus 512 motion records. The TEST runner reports each frame and
WIDTH and B1c PEAK on independent TIME/DISTANCE axes after freeze.
All accepted payloads must agree; conflicts give AMBIGUOUS. B1c PEAK has
host replay evidence, not powered acceptance. Those captures used the
historical 5.5 mm mount. Since 2026-11-19 the operator
uses 2.5 mm and has fixed hump clearance mechanically; see [BARCODE.md](../s3-line-barcode/BARCODE.md).
Full dumps can take tens of seconds:
wait for READY before GP21; presses during the dump are ignored with one
notice. Partial-contrast optical rules and the approved real-capture host
fixtures are documented in the barcode contract. Read the
[B1c procedure, stop limits and RAM](../s3-line-barcode/BARCODE.md) first.

### Profile identity

The profile marker prevents reuse of objects with different options. For a
fully clean validation, use `make clean` with the same profile options,
then build it. A file left from an old profile is not a new build result.
The image name includes its options:

| Image | File under `build_make/` |
| --- | --- |
| Radio-off robot | `mtk3pico_smp0_usb_cdc.uf2` |
| Radio-off TEST | `mtk3pico_smp0_usb_cdc_test.uf2` |
| Radio-off bench | `mtk3pico_smp0_usb_cdc_bench.uf2` |
| MQTT robot | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| MQTT TEST | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test.uf2` |
| MQTT DIAG | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_diag.uf2` |
| Default | `mtk3pico_smp1_usb_cdc_wifi.uf2` |

### H3 source comparison images

The operator accepts uncorrected height 11.5/12.4 mm as good enough, not
the corrected +4..6 mm over-read. The default remains corrected. Compare
both sources for FALSE humps on a fuller circuit (curves + SEARCH):

```sh
cd build_make
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 IMU_HUMP_SOURCE=1 JUNCTION_TEST_CMD=NONE -j4 --output-sync=target
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 IMU_HUMP_SOURCE=2 JUNCTION_TEST_CMD=NONE -j4 --output-sync=target
```

| Source | MQTT robot image under `build_make/` |
| --- | --- |
| 1, corrected (default) | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| 2, filtered roll | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_humpfilt.uf2` |

Both files can coexist. The source is in `PROFILE_ID`, so switching it
invalidates old objects even without `make clean`. It is passed as
`-DIMU_HUMP_SOURCE=1` or `=2`; use numeric values, not C macro names in
the Make option. The non-default suffix applies to other profiles too.
Boot prints `[S4] hump_source 1 corrected` or `2 filtered` once (not
BENCH). Run-end, GP20 DUMP and `[S4_HUMP]` lines end with
`source 1 corrected` or `source 2 filtered`. Record the boot line and
filename for every H3 session. MQTT payloads do not identify the source;
do not mix sessions/images in a source comparison.
See [H3 plan](../../testing/subsystem4/TEST_LOG.md#h3-two-image-comparison-plan).

### B2a fake junction images

Use `JUNCTION_TEST_CMD=NONE|A|B|C|D`. A/B force SEARCH left/right to the
first new arm; D skips the first LEFT arm and accepts the second. C follows
normal LINE/SEARCH routes, with only its detector suppressed for160 mm
from the first edge across natural SEARCH handoffs. The fake command is latched once at normal vehicle
START; TEST/BENCH have no such START. NONE leaves LINE decisions unchanged.
The option is included in the object-profile marker and passed to C as0..4.
Unconsumed pending survives successful natural SEARCH back to LINE, with a
new clear detector baseline. Other routes, failure and STOP cancel it.
No pending or course-gap expiry is added.

```sh
cd build_make
for cmd in NONE A B C D; do
    make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 JUNCTION_TEST_CMD=$cmd \
        -j4 --output-sync=target
done
```

NONE retains the normal MQTT filename. The other images end in
`_jtestA.uf2` through `_jtestD.uf2`. Fake boot prints its command and clearance;
a kind7 phase event and telemetry-owned `[JUNCTION]` USB line report the job.
NONE without live enable prints BARCODE SHADOW and the 26-entry table at boot.
The demo uses NONE with live enable1; it can offer mapped letters to Vehicle.
No summary version is added. Use NONE for H3 IMU circuit
comparisons. Read the [design](../architecture/JUNCTION_ACTION.md) and
[validation/J0–J6 procedure](JUNCTION_VALIDATION.md) before an authorized test.
NONE crossed `+` on `eb18004` (operator-supplied J1 PASS). Operator J2–J5 acceptance on `7648081`/`75a95f7` is recorded with its
capture/measurement limits. Main closed B2a after the `50426ae` A regression.
Pending-command barcode rejection and full named-task stack gates remain open.

### B2b first SHADOW image

```sh
make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 JUNCTION_TEST_CMD=NONE \
    BARCODE_LIVE_ENABLE=0 BARCODE_MEASURED_CLEAR_MM=0 -j4 --output-sync=target
```

In default builds without live enable, the decoder and adapter report
evidence only, and S3 forces valid0 even in mode2. The frozen demo enables
live commands; the command above is a SHADOW diagnostic profile, not the demo.
A live opt-in build ([BARCODE_LIVE_COMMAND.md](../s3-line-barcode/BARCODE_LIVE_COMMAND.md))
can set valid1 only with mode2, a custom `BARCODE_MAP` and a mapped letter.
The150 mm SHADOW label is unverified and cannot authorize actions. Enable,
bound and source choice are object-profile keys. Invalid bounds, fake+live,
or enable1 with bound0 fail in Make and C. Do not build/flash an action image
without a separate approval and measured clearance. Public-example proof
images contain no real credentials and are not operator images.
Read the [live adapter](../architecture/JUNCTION_LIVE_ADAPTER.md) for50 ms
record/control limits,10 ms LINE age, terminal ACK and stopped diagnostics.

## Private settings

| Path | Policy |
| --- | --- |
| `config/wifi_secrets.h` | Ignored Wi-Fi settings; use mode 0600 |
| `app_program/subsystem1/robot_mqtt_secrets.h` | Ignored application broker/client/token settings; mode 0600 |
| `~/.config/robot-mqtt.token` | Laptop command token; owned regular file, 0600, no symbolic link |
| `src/wifi_config.h` | Ignored settings for the separate old CMake/UDP benches only |

Use the tracked `.example` headers as shapes, not credentials. In a fresh
worktree, copy private settings privately and apply `chmod 600`; do not
print or commit their values. The broker default `0.0.0.0` builds but does
not connect. [TELEMETRY.md operation](../s1-telemetry/TELEMETRY.md#private-settings)
has first-time token setup. UF2/ELF files embed credentials; do not publish
them or attach them to public evidence. Do not replace existing secrets.

## Flash

Lift both wheels, clear the mechanism and save any RAM log first. Reset
loses that log. Check the full image filename and profile before loading.

1. Disconnect Pico USB. Remove other board power if a cold start is needed.
2. **Hold BOOTSEL while plugging USB into the Pico.**
3. Release BOOTSEL after the ROM device / `RPI-RP2` drive appears.
4. Load and verify the selected image. Example, from the repository root:

   ```sh
   picotool load -v build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2
   picotool verify build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2
   picotool reboot
   ```

Copying that UF2 onto `RPI-RP2` is the alternative ROM load method.
`picotool load -f` is not a promise of application-to-ROM reboot support:
the robot has CDC only, no vendor-reset or 1200-baud reset interface.
Physical BOOTSEL works independently of application task priorities.

After flashing, perform the relevant S0/S1/S2 checks on the new image.
A cold-power test means remove **all** power before applying it again;
a warm flash/reboot can hide a TIMER setup fault. Full TEST moves motors
briefly and ends with the one-time S2-U05 watchdog reset. MQTT/GP21 do not
control those TEST motor steps. Keep wheels lifted throughout.

## Read output

The USB CDC device is normally `/dev/ttyACM0` on Linux. A terminal needs a
baud value, but CDC ignores that value. Example:

```sh
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

Save terminal output under ignored `build/` when needed. Reconnect after a
reset if the terminal closes. Check the boot profile, reset reason, complete
TEST lines and dump loss footer; an enumerated port is not a timing pass.

## Kernel bench image (BENCH_KERNEL)

Runs the real radio-free robot tasks on a **spare** Pico, not the robot.
Coding workers do not flash hardware.

```sh
make -C build_make SMP=0 WIFI=none NET=none MQTT=0 BENCH_KERNEL=1 -j8
make -C build_make SMP=1 WIFI=none NET=none MQTT=0 BENCH_KERNEL=1 -j8
make -C build_make SMP=0 FORCE_CORE0=0 WIFI=none BENCH_KERNEL=1 KERNEL_RAM=1
```

Images end in `_bench_kernel_step0.uf2` (`_kernel_ram` with the third line).

| Item | Rule |
| --- | --- |
| Stimulus | Alarm1 gives 3800 encoder edges/s, alternating wheels, for 600 s, through a virtual GPIO provider into the unchanged dispatcher and motion callback. A missed slot or a pending-edge collision stops the generator (`GEN_STOPPED`); it never replays edges |
| Other inputs | Line samples constant 1800; IMU NACK; sonar no echo; motor/servo writes kept in RAM, no PWM |
| Run | Record USB from boot. After the stop, a virtual GP20 press dumps about 500 ms later. Do not press GP21 |
| Report | `[timeline]`, `[load]`, `[bench_load]` (`WRAP_RISK` above 7146368/s), P1 lines, `[bench_top]` (count >= 100 us and top 20 per P1 counter), then `[bench] window_us W sched S delivered D coalesced C overflow O gen_calls N gen_us_sum U gen_us_max M cost_bp B V` and a per-wheel `COUNT_MATCH` line |
| Verdict V | `GEN_STOPPED` (O > 0, overrides), `OK` (B < 300), `COST_HIGH`, `BAD_WINDOW` (W = 0 or > 60 min) |
| Pass | No reset in 10 min; window 600000000 us; 2280000 edges scheduled = delivered; 1140000 per wheel with `diff 0` and `COUNT_MATCH`; coalesced and overflow 0; `cost_bp < 300` |
| Limit | Generator time excludes IRQ entry/exit (about 1-2 us per call). Encoder/interrupt load test only, not sensor or robot acceptance |
| Gate | Bench objects are not in ordinary links: 76 ordinary ELF/UF2 hashes (hash76) must match the base |

Code: `app_program/bench_kernel/` (`bench_stats`, `bench_report`,
`bench_gpio`, `bench_top`); tests `tests/test_bench_*.c`. Results:
[split parked](KERNEL.md#split-placement-parked-2026-11-15) and
[KERNEL_RAM](KERNEL.md#ram-placement-kernel_ram).

## SWD debug probe

Use it to see both cores, read memory while the firmware runs, or load
without BOOTSEL. Connect **only SWDIO, SWCLK and GND** (no probe power).

| Probe | Where | OpenOCD | Wiring SWCLK / SWDIO / GND |
| --- | --- | --- | --- |
| Raspberry Pi Debug Probe | school, laptop | `-f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000"` | `D` port SC / SD / GND |
| Raspberry Pi 400 header | home | start script in its Pico folder (loopback only); forward with `ssh -N -L 3333:127.0.0.1:3333 <pi-user>@<pi-host>` | pin 22 (GPIO25) / 18 (GPIO24) / 20 |

| Task | Command |
| --- | --- |
| Read memory, no halt (safe on track) | `arm-none-eabi-nm -S build_make/<image>.elf \| grep <symbol>`, then `mdw <address> [n]` on Telnet port 4444 |
| GDB | `arm-none-eabi-gdb build_make/<image>.elf -ex "target extended-remote :3333"`; `info threads`, `thread 1/2`, `bt`, `monitor halt`, `monitor reset halt` |
| Load | add `-c "program build_make/<image>.elf verify reset exit"` |

Safety and limits:

- PWM keeps the last duty while the cores are halted: halt only with wheels
  lifted or motor power off. The watchdog pauses on debug halt.
- A halt stops kernel time; deadline counters after a halt are not valid.
- Use the ELF of the image on the robot; addresses change each build.
- Do not change other services on the Raspberry Pi 400.
- Tested 2026-11-13: Raspberry Pi 400, OpenOCD 0.12.0, both cores examined
  without a halt. The laptop with the Debug Probe is not tested.

## Host tests and link proof

These tests do not need a robot, broker or secret files:

```sh
cmake -S tests -B /tmp/robot-host-tests
cmake --build /tmp/robot-host-tests -j4
ctest --test-dir /tmp/robot-host-tests --output-on-failure
python3 tests/test_mqtt_robot.py /tmp/robot-host-tests/test_mqtt_wire
python3 tests/test_mqtt_dashboard.py /tmp/robot-host-tests/test_mqtt_wire
python3 scripts/check-docs.py
python3 scripts/check-template-changes.py
git diff --check
```

Node page tests report a skip if Node is absent. The C fixture emitter
checks Python decoding against C-packed bytes. Neither replaces browser,
radio, motor, stack or cold-power measurements.

For an SDK dependency change, save the full build log and run
`python3 scripts/check-sdk-hw-link.py LOG --output-dir /tmp/sdk-proof`
before changing profiles. It builds a retained, never-called adapter probe
and rejects profile mismatch or unwanted runtime linkage. Do not flash the
probe ELF. Source: [sdk_hw_link_probe.c](../../tests/probes/sdk_hw_link_probe.c).

## Results, limits and evidence

| Check | Recorded result / limit |
| --- | --- |
| Seven robot/profile builds at USB-priority change | Warning-free; 39 strict ARM invocations and seven SDK proofs passed; revision-scoped, not a new build claim |
| USB21 hardware enumeration/dump | Passed on `94015fd`; see [kernel results](KERNEL.md#results-and-limits) |
| Cold-power timing | Required for every new firmware image; warm reboot alone is insufficient |
| Host tests | Validate logic and contracts with doubles, not physical timing or safety |

Use [S1 TEST_LOG](../../testing/subsystem1/TEST_LOG.md),
[S2 TEST_LOG](../../testing/subsystem2/TEST_LOG.md) and
[integration TEST_LOG](../../testing/integration/TEST_LOG.md) for operator records.
The original development record (old CMake probes, UDP benches,
calibration commands and observations) is in the
[kernel history](../archive/kernel/HISTORY.md); those targets are not the
current robot Make build.
