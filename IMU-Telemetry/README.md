# IMU-Telemetry — standalone IMU producer

A `pico_w` firmware image that drives a GY-511 breakout carrying an LSM303DLHC
and publishes IMU state for the future RoboCar Manager.

It is an **IMU producer only**. It is not the Manager, not the motor controller,
and not the Wi-Fi transmitter. It implements no `mbf_ctrl_evt`, claims no
`mtx_snapshot` mutex, reads no encoder GPIO, and links no CYW43 code.

The contract it implements is [`IMU_TELEMETRY_PLAN.md`](IMU_TELEMETRY_PLAN.md);
the state behaviour is [`IMU_States.md`](IMU_States.md).

## Hardware and wiring

| Signal | Pico W pin | Notes |
| --- | --- | --- |
| I2C0 SDA | GP0 | GY-511 board pull-ups, 400 kHz |
| I2C0 SCL | GP1 | same |
| Data ready | GP6 | input, active high, sampled as a hint only |
| Accelerometer | address `0x19` | SA0 high |
| Magnetometer | address `0x1E` | on the same bus |

Device configuration: accelerometer 100 Hz, high resolution, **±8 g**;
magnetometer continuous 75 Hz, ±1.3 gauss. The identity and configuration
readback are verified at start-up.

The accelerometer full scale is set by one value, `IMU_CFG_ACCEL_FS_G` in
[`src/imu_config.h`](src/imu_config.h), which re-derives both the mg-per-LSB
scale factor and the `CTRL_REG4_A` full-scale bits, so the driver, the
conversion, and the tests cannot drift apart. ±8 g was chosen because the
collision detector needs to measure about 2.5 g, and a ±2 g part saturates at
2.047 g. The cost is resolution: 4 mg per LSB instead of 1 mg.

**Deviation from the GY-511 norm:** the plan requires external board pull-ups,
and this driver relies on the module's own. `imu_hal_bus_init()` enables the
weak internal pull-ups as well, because a bare module without them holds the
bus low and no device answers.

## Coordinate convention

Body axes are **+X right, +Y forward, +Z up**, and pitch is positive nose-up.
The module's own +Y is robot forward by default.

The signed axis remap lives in exactly one place, `IMU_CFG_ACCEL_AXIS_MAP` and
`IMU_CFG_MAG_AXIS_MAP` in [`src/imu_config.h`](src/imu_config.h). Remount the
module and only those two structures change — no filter, state-machine, or
threshold edit.

## Public contract

```c
void imu_init(uint32_t now_ms);                                  /* self-test  */
void imu_step(uint32_t now_ms);                                  /* 13 ms      */
void imu_publish_encoder_input(const imu_encoder_input_t * in);  /* latest     */
void imu_begin_run(void);                                        /* new run    */
bool imu_snapshot_read(imu_snapshot_t * out);                    /* 2 words    */
bool imu_event_pop(imu_event_t * out);                           /* 16-deep    */
void imu_diag_read(imu_diag_t * out);                            /* USB only   */
```

The two snapshot words are packed and unpacked by named helpers with explicit
shifts and masks; see [`src/imu_snapshot.h`](src/imu_snapshot.h) for the bit
table. No packed struct and no compiler bit-field is used.

Events are `IMU_EVT_SELFTEST`, `IMU_EVT_CALIBRATED`, `IMU_EVT_HUMP_START`,
`IMU_EVT_HUMP_END`, `IMU_EVT_COLLISION` and `IMU_EVT_SENSOR_FAULT`.

## Behaviour

| Concern | Implementation |
| --- | --- |
| Cadence | 13 ms, absolute-time scheduled in `main`, resynchronised after a stall |
| Filtering | exponential low-pass, alpha 0.20, on both vectors |
| Pitch | `atan2(forward, horizontal)` of the filtered acceleration |
| Heading | tilt-compensated from up/east/north basis vectors, clockwise from magnetic north |
| Turn rate | wrap-safe heading delta divided by the measured interval; no gyroscope exists |
| Calibration | 2 s stationary window: magnitude inside 0.90..1.10 g and magnitude standard deviation at most 0.03 g |
| Terrain | LEVEL at \|pitch\| ≤ 4°, CLIMBING at ≥ +7°, DESCENDING at ≤ −7°, three samples each; CREST needs a fresh dwell |
| Hump | `h += distance_delta_mm * sin(pitch)` in CLIMBING and DESCENDING; peaks below 15 mm are rejected |
| Collision | filtered magnitude residual, two consecutive samples, latch and re-arm |
| Faults | bus failure, implausible or non-finite data, failed self-test: latch `SENSOR_FAULT`, one event, retry every second, then repeat self-test and calibration |
| Events | 16-entry FIFO; a full queue drops the newest non-fault event, and a fault evicts the newest non-fault entry instead |

Every numeric value above lives in
[`src/imu_config.h`](src/imu_config.h).

## Build

The Pico SDK is installed under the user-managed `.pico-sdk` tree and is not
exported in a plain shell. Use the exact commands that produced the shipped
image:

```sh
SDK="C:/Users/Jaison Tan/.pico-sdk"
export PATH="$SDK/cmake/v3.29.9/bin:$SDK/ninja/v1.13.2:$SDK/picotool/2.3.0/picotool:$PATH"

cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w \
      -DPICO_SDK_PATH="$SDK/sdk/2.3.1" \
      -DPICO_TOOLCHAIN_PATH="$SDK/toolchain/15_2_Rel1" \
      -DCMAKE_MAKE_PROGRAM="$SDK/ninja/v1.13.2/ninja.exe"
ninja -C build
```

Products:

```text
build/imu_telemetry.uf2
build/imu_telemetry.elf
build/imu_telemetry.bin
```

Flash the UF2 by copying it to the `RPI-RP2` device, or
`picotool load -f -v -x build/imu_telemetry.uf2`.

## Host tests

The tests compile the production core, driver, and codec, and supply the
hardware seam from `imu_hal.h` over a fake LSM303DLHC register file:

```sh
cmake -S tests -B build-host
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

Or directly:

```sh
cd tests
cc -std=c11 -Wall -Wextra -Wshadow -I../src -DIMU_TEST_SNAPSHOT_HOOK=1 \
   test_imu.c ../src/imu_core.c ../src/imu_lsm303.c ../src/imu_snapshot.c \
   -lm -o test_imu && ./test_imu
```

`IMU_TEST_SNAPSHOT_HOOK` lets the interleaving test force a publish between the
two word loads of `imu_snapshot_read()` and prove that the reader retries
instead of returning a torn pair. Production builds compile the hook away.

## USB diagnostics

USB CDC only; UART stdio is disabled. One line at 10 Hz, all values integer:

```text
t=<ms> acc=<raw|filtered x,y,z mg> mag=<raw|filtered x,y,z mG> pitch=<ddg>
head=<ddg> turn=<dps> terr=<name> h=<mm> hi=<mm> dd=<mm> enc=<l>,<r> x=<agree|disagree|unknown>
seq=<n> w0=<word1> w1=<word2> ev=<posted>/<dropped>/<high-water> i2c=<n> inv=<n>
st=<n> cal=<n> rec=<n> drdy=<n> snap=<n> dist=<mm>
```

An event prints its own line as it is drained. Nothing prints from an
interrupt: this image installs no interrupt handler.

## Snapshot coherence

The two words are published and read through a **seqlock**, so a reader in
another task can never pair word 1 from one cycle with word 2 from another.

- `publish()` raises a sequence counter to odd, stores both words, then raises
  it to even. The counter and both words are `volatile`, so the compiler keeps
  that order.
- `imu_snapshot_read()` reads the counter, copies both words into locals, reads
  the counter again, and returns the pair only when it did not change. An
  interrupted attempt is retried, up to eight times; after that it returns
  `false` and increments `snapshot_failures`, leaving the caller's buffer
  untouched so it can keep its previous copy.

This needs no RTOS type in the core. A future Manager that also takes its
`mtx_snapshot` simply makes the retry unnecessary rather than incorrect. The
cost is bounded: the reader spins only while a publish is mid-flight, which is
a handful of instructions on a single core.

`imu_diag_read()` copies a whole structure and is **not** coherent; it is
USB-diagnostic-only, which is why it is not part of the Manager contract.

## Known limitations

1. **Collision detection costs accelerometer resolution.** The plan's 1.5 g
   residual needs about 2.5 g of measurable acceleration, which a ±2 g part
   cannot provide (it saturates at 2.047 g). The full scale is therefore set to
   ±8 g, which makes the threshold reachable and costs resolution: 4 mg per LSB
   instead of 1 mg. Retune `IMU_CFG_ACCEL_FS_G` if that trade is wrong for the
   future filter tuning.
2. **The low-pass also hides short impacts.** At alpha 0.20 a single-sample
   spike is attenuated heavily, so an impact must persist for several 13 ms
   samples to move the filtered magnitude.
3. **Stationary calibration assumes a level robot.** The per-axis estimate
   absorbs the zero offset plus any static mounting tilt, so a robot calibrated
   on a slope learns that slope as level. Calibrate on a flat surface.
4. **No magnetic hard or soft iron calibration.** Identity and zero correction
   only, as the plan requires: stationary data is not a complete magnetic
   calibration, and heading accuracy near large steel objects is unverified.
5. **Small packet fields.** Pitch and turn rate are 10-bit signed deci-degrees
   and degrees/s, so they saturate at ±51.2° and ±512 °/s. The internal values
   are wider; only the compact snapshot saturates.
6. **The encoder cross-check is qualitative.** It never raises a fault, because
   track width and counts-to-angle are not calibrated.

## Out of scope

No Manager, no `mbf_ctrl_evt`, no `mtx_snapshot`, no encoder GPIO interrupt, no
motor command, no Wi-Fi, no dashboard, and no merge into the motor-line branch.

## Bench validation still required

Before this is hardware-validated: both I2C addresses answer, DRDY transitions
appear on GP6, the coordinate signs match the mounted module, stationary
calibration is accepted on a flat surface, heading behaves when crossing north,
the console holds 10 lines per second, and a disconnect and reconnect of the
module recovers through `SENSOR_FAULT` back to LEVEL.
