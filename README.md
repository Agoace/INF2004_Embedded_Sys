# INF2004 Embedded Systems: autonomous line-following robot

A Robo Pico car with a Raspberry Pi Pico W that runs micro T-Kernel 3.0. The
robot follows a black line, reads Code 39 barcodes beside the line, turns at
junctions as the barcodes tell it, goes around obstacles, reports humps, and
sends live data to a laptop dashboard over Wi-Fi (MQTT).

| Item | Value |
| --- | --- |
| Submission | branch `main`, tag `submission-v3.1` |
| Demo kernel | tag `demo-kernel-v2` ([record](docs/release/DEMO_KERNEL_V2.md)) |
| State per area | [PROJECT_STATUS.md](docs/architecture/PROJECT_STATUS.md) |
| Course requirements (summary) | [project-requirements/README.md](docs/project-requirements/README.md) |
| Change Requirement 1 (minimum-bit data) | [CR1.md](docs/architecture/CR1.md), [CR1_MIN_BIT_PROOF.md](docs/architecture/CR1_MIN_BIT_PROOF.md) |

## What the robot does

```
            +-------------------- vehicle task (one module at a time) --------------------+
 sensors -> |  LINE  --line lost-->  SEARCH  --found-->  LINE                                |
            |  LINE  --obstacle-->   SCAN    -->  DETOUR  --line found-->  LINE             |
            |  LINE  --barcode-->    command held  --next junction-->  LEFT/RIGHT/STRAIGHT/U-TURN |
            +----------------------------------------------------------------------------------+
                 | motion commands (mm/s)                 | rows, events, summaries
                 v                                        v
            motion task -> motors, servo            telemetry task -> RAM log (GP20 dump), MQTT
```

| Subsystem | Does | Code | Record |
| --- | --- | --- | --- |
| S1 Telemetry | RAM run log, GP20 USB dump, MQTT over Wi-Fi, laptop dashboard | [`app_program/subsystem1/`](app_program/subsystem1/), [`scripts/`](scripts/) | [TELEMETRY.md](docs/s1-telemetry/TELEMETRY.md), [MQTT_WIRE.md](docs/s1-telemetry/MQTT_WIRE.md) |
| S2 Motion | Motor speed control (mm/s, encoders), servo, hardware watchdog | [`app_program/subsystem2/`](app_program/subsystem2/) | [MOTION.md](docs/s2-motion/MOTION.md) |
| S3 Line and barcode | Two-sensor line tracing, line search, Code 39 reader on GP28 | [`app_program/subsystem3/`](app_program/subsystem3/) | [LINE.md](docs/s3-line-barcode/LINE.md), [LIVE_WINDOW_READER.md](docs/s3-line-barcode/LIVE_WINDOW_READER.md) |
| S4 IMU | GY-511 hump report, lift and overturn safety stop | [`app_program/subsystem4/`](app_program/subsystem4/) | [IMU.md](docs/s4-imu/IMU.md) |
| S5 Obstacle | Ultrasonic service, servo scan, obstacle detour | [`app_program/subsystem5/`](app_program/subsystem5/) | [OBSTACLE.md](docs/s5-obstacle/OBSTACLE.md) |
| Vehicle | State machine shell, junction actions, live barcode commands | [`app_program/vehicle/`](app_program/vehicle/), [`app_program/barcode_live_vehicle/`](app_program/barcode_live_vehicle/) | [ARCHITECTURE.md](docs/architecture/ARCHITECTURE.md), [JUNCTION_ACTION.md](docs/architecture/JUNCTION_ACTION.md) |

Owners, pins, task priorities and interfaces are in [TEAM.md](TEAM.md).

## Repository layout

| Path | Contents |
| --- | --- |
| `app_program/` | Robot application: one folder per subsystem, `vehicle/`, `platform/` (kernel support) |
| `build_make/` | Firmware build (Make). Output `.uf2` files go here. |
| `kernel/`, `include/`, `lib/`, `device/`, `config/`, `etc/` | Course template (micro T-Kernel 3.0). Local changes are listed in [TEMPLATE_CHANGES.md](docs/architecture/TEMPLATE_CHANGES.md). |
| `tests/`, `host-tests/` | Host (PC) unit and integration tests |
| `sim/` | Host simulator for the obstacle detour |
| `testing/` | Robot test procedures and results per subsystem |
| `tools/`, `scripts/` | Build helpers (letter map, RAM proof), log decoder, MQTT recorder, dashboard, doc checks |
| `src/` | Earlier bare-metal programs, kept for benches and calibration |
| `docs/` | All records; start at the [documentation index](docs/README.md) |
| `design-review/` | State chart and task diagrams |

## Branches

| Branch | Role |
| --- | --- |
| `main` | Submission. Receives `mtk3-staging` with one `--no-ff` merge at each freeze. Tags `submission-v*`, `demo-kernel-v*`. |
| `mtk3-staging` | Integration. Every module branch merges here. |
| `mtk3-testing` | Test environments and tools: Webots SIL/HIL/PIL simulation, bench kernels. Receives `mtk3-staging`; never merges back without approval. |
| `mtk3-telemetry`, `mtk3-dashboard`, `mtk3-motion`, `mtk3-line`, `mtk3-imu`, `mtk3-search`, `mtk3-scan`, `mtk3-detour`, `mtk3-vehicle` | One branch per module |
| `mtk3` | Course template base |

Finished work branches are kept as `archive/<name>` tags. The full rules are
in [PROJECT_STATUS.md (branch layout)](docs/architecture/PROJECT_STATUS.md#branch-layout-and-integration-rules).

## Build and flash the demo image

Tools: Pico SDK 2.2.0 or 2.3.0 (`PICO_SDK_PATH`), Arm GNU toolchain
(`arm-none-eabi-gcc`), `make`, `picotool`. First-time setup is in the
[template setup guide](docs/kernel/TEMPLATE_SETUP.md).

1. Copy the private settings (they are not in Git):
   `config/wifi_secrets.example.h` to `config/wifi_secrets.h`, and
   `app_program/subsystem1/robot_mqtt_secrets.h.example` to
   `app_program/subsystem1/robot_mqtt_secrets.h`. Fill in your values.
2. Build:

   ```sh
   cd build_make
   make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 \
     KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
     BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="A=LEFT,F=UTURN" \
     JUNCTION_TEST_CMD=NONE
   ```

   Check for `RAM_PROOF PASS` and 0 warnings.
3. Flash: hold BOOTSEL, connect USB, then `picotool load -v -x <file>.uf2`.
4. Put the robot flat on the floor and do not touch it for 2 s (IMU level).
   Press GP21 to calibrate and start. Press GP21 again to stop. GP20 dumps
   the RAM log over USB.

`BARCODE_MAP` sets the letter actions (`LEFT`, `RIGHT`, `STRAIGHT`,
`UTURN`). On the demo day only this value changes; see
[DEMO_KERNEL_V2.md](docs/release/DEMO_KERNEL_V2.md#demo-day-letter-map-to-be-announced).
Other build profiles (TEST, BENCH, captures) are in [BUILD.md](docs/kernel/BUILD.md).

## Dashboard

Start a broker, then the recorder and the dashboard, and open
`http://127.0.0.1:8765/`. Steps: [TELEMETRY.md (operation)](docs/s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard).

## Tests

```sh
cmake -S tests -B /tmp/host-tests && cmake --build /tmp/host-tests -j
ctest --test-dir /tmp/host-tests          # host tests (735 on submission-v3.1)
cmake -S sim -B /tmp/sim -DSIM_FIRMWARE=detour_test && cmake --build /tmp/sim
ctest --test-dir /tmp/sim                 # detour simulator
python3 scripts/check-docs.py             # links and anchors in all records
python3 scripts/check-template-changes.py # template change list
```

Robot test records are in [testing/](testing/README.md). Measured and
assumed numbers, each with its source, are in
[MEASUREMENTS.md](docs/architecture/MEASUREMENTS.md).

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
