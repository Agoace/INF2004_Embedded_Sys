# IMU-Telemetry implementation contract

## Goal and boundary

Implement a standalone `pico_w` firmware target in this directory for a
GY-511 carrying an LSM303DLHC.  It is an IMU producer for the future RoboCar
Manager, not the Manager, motor controller, or Wi-Fi transmitter.

The producer samples and derives IMU state every 13 ms.  It publishes a
latest-value, two-word binary snapshot and a bounded pull queue of discrete
events.  The future Manager interprets that information, owns `mbf_ctrl_evt`,
and produces any Wi-Fi outbound packet.  USB CDC output is diagnostic only.

## Hardware and coordinates

* Target board: Pico W (`pico_w`).
* I2C: I2C0, SDA GP0 and SCL GP1.  Use external board pull-ups; configure the
  pins for I2C and do not enable conflicting GPIO functions.
* Data ready: GP6 input.  Treat it as a sampling hint; the 13 ms periodic
  step remains the scheduling authority.  Record a missed-DRDY diagnostic,
  but never block forever waiting for an edge.
* Accelerometer address `0x19`; magnetometer address `0x1e`.
* Configure the accelerometer for 100 Hz, high-resolution, +/-2 g operation;
  configure the magnetometer for continuous 75 Hz operation at +/-1.3 gauss.
* Robot body axes are right `+X`, forward `+Y`, up `+Z`; module `+Y` is robot
  forward.  Pitch is positive nose-up.  Isolate the signed axis remap in one
  configuration structure so a remount is not a filter/state-machine change.

## Public producer contract

Keep the types and core independent of Pico SDK/RTOS headers so host tests
use exactly the production state machine.

* `imu_init(now_ms)`: initialise state, I2C driver, counters, and begin
  self-test/calibration.
* `imu_step(now_ms)`: do one non-blocking 13 ms IMU cycle; callers invoke it
  at that cadence.
* `imu_publish_encoder_input(const imu_encoder_input_t *)`: future Manager or
  encoder owner supplies the most recent signed forward `distance_delta_mm`
  and left/right encoder deltas.  The IMU never owns GPIO encoder interrupts
  or counts-to-millimetres calibration.
* `imu_begin_run()`: the only operation that clears `highest_hump_peak_mm` and
  marks the IMU's reader-owned odometer for a new run.
* `imu_snapshot_read(imu_snapshot_t *)`: copies the current two words.  The
  IMU task is the only writer.  Future Manager code obtains matching words
  under its `mtx_snapshot`; this project must not implement that mutex.
* `imu_event_pop(imu_event_t *)`: drains a 16-entry FIFO.  Overflow drops the
  newest non-fault event and increments a diagnostic counter; latched-fault
  state remains visible in the snapshot even if its event cannot be queued.

Use event IDs `IMU_EVT_SELFTEST`, `IMU_EVT_CALIBRATED`, `IMU_EVT_HUMP_START`,
`IMU_EVT_HUMP_END`, `IMU_EVT_COLLISION`, and `IMU_EVT_SENSOR_FAULT`.  Event
payloads include the event ID, an 8-bit sequence, a status/detail byte, and a
signed 16-bit measured peak or diagnostic value.  The eventual Manager maps
the last two safety events to its controller event buffer and fault bits.

## Snapshot packet

`imu_snapshot_t` consists of two aligned `uint32_t` words.  Provide named
pack/unpack helpers; do not use packed structs or compiler bit-fields.

| Word | Bits | Field | Encoding |
| --- | --- | --- | --- |
| 1 | 0..11 | heading | unsigned deci-degrees, 0..3599 |
| 1 | 12..21 | pitch | signed two's-complement deci-degrees, -512..511 |
| 1 | 22..31 | turn rate | signed two's-complement degrees/s, -512..511 |
| 2 | 0..15 | highest hump peak | unsigned millimetres |
| 2 | 16..18 | terrain state | INIT, LEVEL, CLIMBING, CREST, DESCENDING, SENSOR_FAULT |
| 2 | 19 | accelerometer valid | 1 when the current reading passed validation |
| 2 | 20 | magnetometer valid | 1 when the current reading passed validation |
| 2 | 21 | calibrated | 1 after stationary bias calibration succeeds |
| 2 | 22 | self-test passed | 1 after both sensor self-tests succeed |
| 2 | 23 | fault latched | 1 in SENSOR_FAULT |
| 2 | 24..31 | sequence | increment once per completed 13 ms publish |

Full raw/filtered acceleration and magnetic axes, current hump height,
received distance delta, and encoder-difference diagnostics remain in an
internal diagnostic structure and USB output.  They are intentionally not in
the compact Manager snapshot.

## Required state and calculation behaviour

Implement `IMU_States.md` exactly with the following initial tunable defaults.
Put every numeric threshold in a documented configuration header.

* Stationary calibration: 2 s of samples; acceleration magnitude must remain
  0.90..1.10 g with standard deviation at most 0.03 g.  Estimate the
  accelerometer zero/bias relative to gravity.  Magnetometer hard/soft-iron
  correction defaults to identity/zero: stationary data alone must not be
  misrepresented as a complete magnetic calibration.
* Use an exponential low-pass filter with alpha 0.20 for acceleration and
  magnetic vector inputs before derived values.  Derive tilt-compensated
  heading and wrap-safe heading delta.  Turn rate is heading delta divided by
  the actual 13 ms interval; no gyroscope is assumed.
* Preserve left/right encoder deltas as a qualitative turn cross-check
  diagnostic only.  Do not declare an IMU fault from their disagreement until
  vehicle track width and counts-to-angle calibration are supplied.
* LEVEL means absolute pitch at most 4.0 degrees for three consecutive
  samples.  CLIMB starts at pitch at least +7.0 degrees for three samples;
  DESCENDING starts at pitch at most -7.0 degrees for three samples.  CREST
  is confirmed by LEVEL criteria after CLIMBING.  A valid hump must reach 15
  mm before it is retained; otherwise discard it as a false start.
* While CLIMBING or DESCENDING, integrate `h_mm += distance_delta_mm *
  sin(pitch)`, preserve the current peak, and retain the run's largest peak.
  Post exactly one HUMP_START/HUMP_END per episode.
* Detect collision from a filtered acceleration-magnitude residual of at
  least 1.5 g above the 1 g baseline for two consecutive samples.  Latch it
  until the residual stays below 1.2 g for ten samples; post one COLLISION
  per episode.
* On I2C transfer failure, invalid/non-finite derived data, or failed
  self-test, stop publishing valid motion data, latch SENSOR_FAULT, and post
  one `IMU_EVT_SENSOR_FAULT`.  Retry bus identity/configuration every second.
  A recovered device must repeat self-test and stationary calibration before
  returning to LEVEL.  Only the future Vehicle Controller clears global
  faults.

## Firmware, diagnostics, and tests

* Add a CMake target named `imu_telemetry`, enable USB stdio, disable UART
  stdio, and generate UF2/ELF/BIN outputs.  Do not link CYW43/Wi-Fi code.
* Print one human-readable USB CDC diagnostic line at 10 Hz: raw and filtered
  acceleration/magnetic axes, pitch, heading, turn rate, terrain state,
  current/highest hump height, received distance delta, encoder cross-check,
  packet words, and fault/queue counters.  Never print from an interrupt.
* Add host tests for packet packing boundaries/sign extension, heading wrap,
  calibration acceptance/rejection, each terrain transition, false-start
  rejection, run reset, distance integration, collision debounce/re-arm,
  queue overflow plus retained fault, I2C fault/recovery, and sequence
  progression.
* Build `imu_telemetry` using the installed Pico SDK and report the absolute
  UF2 path.  Bench-test the two I2C addresses, DRDY transitions, coordinate
  signs, stationary calibration, north-wrap behaviour, USB output rate, and
  disconnect/reconnect recovery before calling it hardware-validated.

## Out of scope

No Manager implementation, `mbf_ctrl_evt` implementation, encoder GPIO ISR,
motor command, Wi-Fi, dashboard, or live branch merge belongs in this slice.
