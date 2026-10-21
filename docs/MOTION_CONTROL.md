# Motion control (Buddy 2)

Status: Implemented as a library and bench target. The μT-Kernel port is not in the repository yet. The motion core does not depend on it.

This record describes the Motion Control Task code, the interface that the other buddies use, and the tests that produce the Week 6 and Tribunal 1 evidence.

## Requirements covered

| Requirement | Where |
| --- | --- |
| FR5 PWM on both motors, NFR10 pins | `src/motion/motion_hal_pico.c` |
| FR6, NFR5 A/B encoder ISR, 2535 counts/revolution | `src/motion/encoder.c` |
| NFR4 20 ms PID, new command applied at once | `motion_tick()`, `motion_poll_commands()` |
| Section 6.2 statechart | `src/motion/motion_control.c` |
| FR14 stall and ESTOP fault | `MOTION_EVT_STALL`, `FAULT_*` bits |
| FR17 encoder self-test | `MOTION_CMD_SELFTEST` |
| NFR13 1 m and 90° accuracy | `MOTION_CMD_FWD`, `MOTION_CMD_ROTATE` |
| NFR16 PWM 0 within 20 ms | `motion_request_stop()` |

## Tribunal 1 decisions implemented

| Decision | Implementation |
| --- | --- |
| Wrapping encoder counter, ISR is the only writer, readers use `now - last` | 16-bit counters. `enc_delta16()` and `odometer_t` are in `src/common/robot_shared.h`. |
| STEER is a latest-value variable, so STOP never queues behind it | `motion_set_steer()` does one atomic 32-bit store. |
| STOP flag checked before every PWM write (R2) | `apply_pwm()` checks the stop latch and the latched faults inside the same critical section as the write. |
| STOP and ESTOP skip SETTLE | `motion_request_stop()` sets PWM to 0 in the caller's context and flushes queued commands. |
| SETTLE timeout | SETTLE ramps PWM to 0 over 60 ms. It reports `MOTION_DONE_SETTLE_TIMEOUT` if the wheels still move after 200 ms. |
| Stale-STEER timeout | In FOLLOW, if STEER is older than 100 ms, the wheels are held and `MOTION_EVT_STEER_STALE` is posted once. |
| Measure the Motion WCET | `motion_get_stats()` reports `tick_wcet_us`. GP22 pulses on every stop request. |
| One shared header for packed messages, fault bits and the delta helper | `src/common/robot_shared.h` (proposal for team agreement) |
| 9-bit signed speed field | Snapshot word W1. |

## Files

| File | Purpose |
| --- | --- |
| `src/common/robot_shared.h` | Team-wide types: encoder delta helper, odometer, fault bits, motion command and event structs, packed snapshot layout. |
| `src/motion/motion_config.h` | Pins, geometry, timing, speeds and default gains. Values marked MEASURE are placeholders. |
| `src/motion/motion_units.h` | Integer conversions between counts, mm, mm/s and degrees. |
| `src/motion/motion_control.h`, `.c` | RTOS-independent core: statechart, PI speed loops, sync, kick, stall, queues, snapshot. |
| `src/motion/motion_hal.h` | Hardware boundary. The Pico version is `motion_hal_pico.c`. The host tests use a simulated robot. |
| `src/motion/encoder.h`, `.c` | Encoder ISR and read-only count API for every task. |
| `src/motion/line_motion_bridge.h`, `.c` | Interface to the Line & Barcode Task (see below). |
| `src/motion/motion_task_utkernel.c` | μT-Kernel task, 20 ms cyclic handler and `mbf_ctrl_evt` forwarding. Template: it compiles only with `MOTION_USE_UTKERNEL`. |
| `src/motion_bench.c` | Bench target with a USB console. |
| `src/robocar_line_follow.c` | Buddy 2 and Buddy 3 integration target. CMake builds it only when `src/line_detector.c` exists. |
| `tests/motion/test_motion.c` | Host unit tests against a simulated robot. |

## Interface for the other buddies

### Vehicle Controller (Buddy 5)

- Send commands with `motion_command(&cmd)`. `motion_cmd_t` is 8 bytes. Commands are STOP, ESTOP, RESET, FOLLOW, FWD(mm), REV(mm), TURN(deg), ROTATE(deg), SELFTEST, and SPEED (test only). A positive angle turns left.
- STOP and ESTOP take effect at once. ESTOP latches `FAULT_ESTOP` until RESET.
- FWD, REV, TURN and ROTATE run to completion. A command sent during a move waits in the queue (depth 3) and starts when the move ends. A command sent during FOLLOW starts at once, for example a ROTATE at a junction. Send STOP to cancel a move.
- Read events in the Motion task and forward them to `mbf_ctrl_evt`: DONE, STOPPED, STALL, STEER_STALE, SELFTEST and REJECTED. The field meanings are in `robot_shared.h`.
- `motion_set_slow()` mirrors the `flg_mode` SLOW bit. `motion_odometer_reset()` is called at RUN start.
- The ESTOP ISR may call `motion_request_stop(true)` directly. It is ISR-safe and gives the shortest NFR16 path. The Week 6 path (ISR, then `sem_estop`, then the VC) also works.

### Line & Barcode (Buddy 3, `ziven-line_tracing_algo`)

- Each line cycle, call `line_bridge_update(black_pattern, unknown_mask, fault_mask)` with the values that `robocar_ir_sensors.c` already computes. The bridge uses the same truth table (L = 1, C = 2, R = 4) and converts the -2..+2 error to STEER through a gain table. Tune it with `line_bridge_set_steer_table()`.
- When the line is lost, the last steer is kept, which biases the robot toward the side where the line left. Junction, split and unknown patterns give steer 0. A sensor fault stops STEER refreshes, so Motion holds the wheels after 100 ms.
- The encoder inputs that Line & Barcode needs:
  - `encoder_count(MOTOR_LEFT/RIGHT)` gives the raw wrapping counts.
  - `line_bridge_mark()` and `line_bridge_travel_mm()` give the distance since a mark. Use them to bound SEARCH_LINE and to measure barcode length.
  - `line_bridge_speed_mm_s()` gives the current speed, to check assumption A6 (constant speed while crossing a barcode).
- STEER must be refreshed at least every 100 ms during FOLLOW.

### IMU (Buddy 4) and Telemetry (Buddy 1)

- IMU: use your own `odometer_t` per wheel with `encoder_count()`. Never reset the counter. The left-minus-right difference gives the turn cross-check: in-place rotation degrees = `counts × 360 × D / (W × 2535)`.
- Telemetry: copy the two snapshot words with `motion_get_snapshot()`.
  - W1 holds the 9-bit left and right speeds (counts per 20 ms), the state, and the fault, slow and stale bits.
  - W2 holds the odometer in cm and a sequence number.

### Encoder ISR and other GPIO interrupts

The encoder ISR uses `gpio_add_raw_irq_handler_masked()`, not `gpio_set_irq_enabled_with_callback()`. The Pico SDK allows only one shared GPIO callback per core, so the ESTOP (GP20), Echo (GP2) and DRDY (GP6) ISRs must also use raw handlers (as `motion_bench.c` does for GP20), or one of them can silently replace another.

## Build and test

Host unit tests (no Pico):

```sh
cc -std=c11 -Wall -Wextra -Isrc/common -Isrc/motion tests/motion/test_motion.c \
   src/motion/motion_control.c src/motion/line_motion_bridge.c -lm -o /tmp/tm && /tmp/tm
```

The tests cover:
- delta and odometer wrap-around
- snapshot packing
- FWD, REV, ROTATE and pivot accuracy against the simulated robot
- STOP: queue flush, and a STOP arriving mid-tick
- ESTOP and RESET
- stall while running and at start
- FOLLOW steering, SLOW and stale STEER
- encoder self-test pass and fail
- queue limits
- the line bridge

Firmware:

```sh
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build --target motion_bench
```

## Bench procedures (`motion_bench`)

Power the motors from VIN with USB disconnected during runs (see [USB power safety](USB_POWER_SAFETY.md)). Type `h` for the command list.

| Evidence | Procedure |
| --- | --- |
| Wheel geometry (needed first) | Measure the loaded wheel diameter and the track width. Set `MOTION_WHEEL_DIAMETER_MM` and `MOTION_WHEELBASE_MM`. |
| Left/right and turn sign | `r 90` must turn the robot left. If it turns right, set `MOTION_MOTOR_1_IS_LEFT` to 0. |
| A4 PID step response | Wheels lifted. Run `a 0` (step), `l` (log on), `v 150 150`, wait 2 s, `x`, `l`. Repeat at three speeds. Plot `spd_l`/`spd_r` against `sp_*_q8 / 256`, record rise time and overshoot, and tune with `g`. |
| NFR13 straight | 10 × `f 1000`. Measure the floor distance against the 1 m target (±3 cm). Record DONE `achieved` and the floor value. |
| NFR13 turn | 10 × `r 90`. Measure the heading change against 90° (±5°). |
| NFR16 stop | Logic analyser on GP20, GP8–GP11 and GP22. During `f 1000`, press GP20. Measure from the GP20 falling edge to all PWM pins low. Repeat with all tasks running after integration. |
| Stall | During `f 1000`, hold a wheel. Expect a `STALL` event and PWM 0. Send `z` to reset. |
| Self-test | Run `k`. Expect `SELFTEST arg=1 detail=0x3f`. |
| WCET | `p` prints `wcet`: the longest `motion_tick()` time in µs. |

## Open measurements

- Wheel diameter and wheelbase: placeholders of 65 mm and 120 mm. Every distance, turn and mm/s value depends on them. The tribunal needs them for the cruise speed.
- Cruise speed: the placeholder is 200 mm/s (about 49 counts per tick). Buddy 5 needs the measured value for the stop distance.
- Gains: the feed-forward comes from the unloaded 5 V VIN table. Kp, Ki and Ksync are starting values until the A4 report.
- Coast under load: `MOTION_COAST_LEAD_COUNTS` is 0 until it is measured.
- The NFR16 and WCET numbers must be measured again under full RTOS load.
