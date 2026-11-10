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
[repository README](../../README.md) has installation steps. The exhaustive
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
| `JUNCTION_TEST_CMD` | NONE (default), A/B/C/D; fake command latched at vehicle START; other values rejected; fake images add `_jtestA`..`_jtestD` |
| `IMU_HUMP_SOURCE` | 1 corrected (default); 2 filtered roll; other values rejected; 2 adds `_humpfilt` to the image name |

Robot/TEST IMU images require USB console and `CRASH_REPORT=0` because
GP0/GP1 belong to I2C0. BENCH has no IMU task. Follow the current
[50 Hz support/GP21 procedure](../s4-imu/IMU.md#step-2d-current-procedure).
The GP21 drive trial requires VIN and a lifted-wheel stop check first.
It is not the full TEST suite or a navigation feature.

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
normal LINE across, with only its detector suppressed for160 mm. The fake command is latched once at normal vehicle
START; TEST/BENCH have no such START. NONE leaves LINE decisions unchanged.
The option is included in the object-profile marker and passed to C as0..4.

```sh
cd build_make
for cmd in NONE A B C D; do
    make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 JUNCTION_TEST_CMD=$cmd \
        -j4 --output-sync=target
done
```

NONE retains the normal MQTT filename. The other images end in
`_jtestA.uf2` through `_jtestD.uf2`. Boot prints a FAKE command/forced-corner clearance label;
a kind7 phase event and telemetry-owned `[JUNCTION]` USB line report the job.
No barcode decoder or summary version is added. Use NONE for H3 IMU circuit
comparisons. Read the [design](../architecture/JUNCTION_ACTION.md) and
[validation/J0–J6 procedure](JUNCTION_VALIDATION.md) before an authorized test.
NONE crossed `+` on `eb18004` (operator-supplied J1 PASS). Redesigned
command accuracy and pending-command barcode rejection remain **Not run**.

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
not connect. [OPERATION.md](../s1-telemetry/OPERATION.md#private-settings)
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
The [original development record](../archive/kernel/DEVELOPMENT.md) keeps
all old CMake probes, UDP benches, calibration commands and observations;
those targets are not the current robot Make build.
