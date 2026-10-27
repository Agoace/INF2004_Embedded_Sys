# Line-following Manager checkpoint

Status: Implemented for the `motor-line-manager` checkpoint. The μT-Kernel adapter is a template because the port is not in the repository.

This record describes the Manager that joins the Motion Control library (Buddy 2, branch `Joseph`) and the line-sensing library (Buddy 3, branch `ziven-line_tracing_algo`). The Manager owns the vehicle state and the communication between the two libraries.

## Safety limitation

**There is no physical emergency stop in this checkpoint.** GP20 and GP21 are both short-press toggles for line following. To stop the robot, remove motor power.

The Manager stops the wheels in these cases:

- A button toggle ends line following.
- The line stays lost for the whole recovery distance.
- A line sensor reports a fault.
- The encoder self-test fails.
- Motion reports a stall or a latched fault.

## Statechart

```
                    boot
                      |
                      v
                 +-----------+
                 | SELFTEST  |  MOTION_CMD_SELFTEST (FR17)
                 +-----------+
                   |       |
          pass     |       |  fail
                   v       +------------------------> +---------+
          +-----------------+                         |  FAULT  |
          | LINE_FOLLOW_OFF |<-------- reset needed --+---------+
          +-----------------+                              ^
            |            ^                                 |
   toggle   |            |  toggle                         |
            v            |                                 |
          +-----------------+                              |
          | LINE_FOLLOW_ON  |------------------------------+
          +-----------------+   line sensor fault         |
            |            ^      Motion stall              |
   100 ms   |            |      Motion ESTOP               |
   LOST     v            |      recovery budget exhausted  |
          +-----------------+                              |
          |   RECOVERY      |------------------------------+
          +-----------------+
            3 tracking samples -> LINE_FOLLOW_ON
```

Toggles are ignored in `SELFTEST` and in `FAULT`. A fault is latched: the Manager sends no drive command again, and only a Motion `MOTION_CMD_RESET` (or a power cycle) clears the state.

## Mailbox and event-queue boundary

The Manager has two inputs with different rules.

| Input | Rule | Why |
| --- | --- | --- |
| Line observation | One-element overwrite mailbox. A new 1 ms sample replaces the old one. | Only the newest sensor state matters. A queue would add latency to every steering decision. |
| Button toggle, `motion_evt_t` | Bounded first-in-first-out queue, 16 entries. | Events are discrete. Each one changes the state, so none may be lost by overwriting. A full queue drops the newest event and increments `events_dropped`. |

The mailbox is one aligned 32-bit word: bits 2:0 black pattern, 5:3 unknown mask, 6:8 fault mask. An aligned 32-bit store is atomic on the Cortex-M0+, so the Line task or an ISR can overwrite it while the 1 ms step reads it. A separate publish counter lets the Manager count samples that were overwritten before it used them.

STEER is not queued at any level. The Manager calls [the line bridge](../src/motion/line_motion_bridge.h) once per sample, and the bridge alone maps the L/C/R pattern to the latest-value STEER. This preserves the Motion property that a STOP never waits behind a burst of steering updates.

### Step order

Each Manager step drains the discrete event queue first, and only then consumes the latest line observation. A terminal Motion outcome (the recovery limit) and the third tracking sample can arrive in the same millisecond, and the terminal outcome must win. In the other order, a reacquired line would restart `FOLLOW` after the recovery budget was already used up, and the fault would never latch.

`finish_recovery()` also refuses to resume when Motion has already left `MOTION_STATE_RECOVERY`. That covers the narrower case where a Motion tick lands between two Manager steps, so its report is still in flight when the third tracking sample arrives. Both cases latch `FAULT_RECOVERY_TIMEOUT` and hold PWM at zero.

## Ownership

- Motion is the only writer of motor PWM. It is also the only writer of encoder counts.
- The Manager is the only writer of the vehicle state, and the only caller of `motion_command()` and `motion_request_stop()`.
- The line bridge is the only writer of STEER.
- The Line task writes line observations and nothing else.

## Recovery

Recovery is a bounded reverse-follow manoeuvre. Motion enforces the limits, so no caller can request an unbounded reverse.

| Limit | Value | Where |
| --- | --- | --- |
| Continuous loss before recovery | 100 ms | `MANAGER_LINE_LOST_TIMEOUT_MS` |
| Reverse speed | 100 mm/s | `MOTION_RECOVERY_SPEED_MM_S` |
| Reverse distance budget | 100 mm | `MOTION_RECOVERY_MAX_MM` |
| Wall-clock guard | 3 s | `MOTION_RECOVERY_TIMEOUT_MS` |
| Tracking samples that end recovery | 3 | `MANAGER_RECOVERY_TRACKING_SAMPLES` |

The Manager keeps publishing the retained forward STEER during recovery. Motion negates both wheel set-points while reversing, so the stored left/right bias unwinds instead of accumulating: a line lost to the left is retraced back to the left. Motion does not flip the STEER sign, because that would curve away from the line instead of back to it.

The manoeuvre ends in one of two ways:

1. Three consecutive tracking samples arrive. The Manager stops, marks the travel origin again, and sends `MOTION_CMD_FOLLOW`. The bridge already holds the forward direction for the new sample, so nothing has to be restored.
2. The distance budget or the wall-clock guard is used up. Motion stops and reports `MOTION_EVT_DONE` with `MOTION_DONE_RECOVERY_LIMIT`. The Manager latches `FAULT_RECOVERY_TIMEOUT` and enters `FAULT`.

Repeated recovery attempts are out of scope. One attempt per loss is the whole policy in this checkpoint.

## GPIO and task behaviour

### Bare-metal target (`robocar_line_manager`)

| Work | Mechanism |
| --- | --- |
| Line sampling | 1 ms main loop, 3 ADC channels, 8 samples per channel |
| Manager step | 1 ms main loop, after the line sample and the event forwarding |
| Motion control | 20 ms repeating timer interrupt, `motion_tick()` |
| Button edges | Raw GPIO interrupt handlers, one per button |

The two button handlers use `gpio_add_raw_irq_handler()`, not `gpio_set_irq_enabled_with_callback()`. The Pico SDK has one shared GPIO callback per core, and the encoder ISR already uses a raw handler. A shared callback would let one handler replace the other. See [Interrupt and polling conventions](INTERRUPT_CONVENTIONS.md).

An interrupt handler only captures the edge level and the edge time, then acknowledges the interrupt. The 1 ms loop debounces the edge with `manager_button_edge()`. All Manager calls therefore happen in one context, and the event queue needs no lock.

### μT-Kernel adapter (disabled by default)

Build it with `-DUSE_UTKERNEL_ADAPTER=ON` after the external port and the startup integration are available.

| Task | Priority | Period | Work |
| --- | --- | --- | --- |
| Motion | 10 | 20 ms cyclic | `motion_tick()`, then forward each `motion_evt_t` to the Manager |
| Line | 11 | 1 ms cyclic | Sample the ADC channels, publish one observation |
| Manager | 12 | 1 ms cyclic | Debounce the buttons, run `manager_step()` |

Create the Motion task first. The Manager requests the encoder self-test from its first step, so the Motion command queue must already exist.

## Button filter

Both buttons are active low with the internal pull-up. The filter accepts a release only when the press lasted between 20 ms and 800 ms. A shorter press is contact bounce. A longer press is a hold, not a toggle.

## Files

| File | Purpose |
| --- | --- |
| `src/manager/manager_types.h` | Shared types: `line_observation_t`, `button_event_t`, the tagged `manager_evt_t`, and the limits above. |
| `src/manager/manager.h`, `.c` | Platform-independent Manager core: statechart, mailbox, event queue, recovery policy. |
| `src/manager/manager_button.h`, `.c` | Short-press filter shared by both adapters. |
| `src/manager/manager_task_utkernel.h`, `.c` | μT-Kernel Line and Manager tasks. Compiles only with `MANAGER_USE_UTKERNEL`. |
| `src/robocar_line_manager.c` | Runnable bare-metal target. |
| `src/motion/line_motion_bridge.h`, `.c` | The only L/C/R to STEER mapping. |
| `src/motion/motion_control.c` | Motion, including the bounded `MOTION_CMD_RECOVER` state. |
| `tests/manager/test_manager.c` | Host unit tests with a simulated Motion HAL. |

## Build and test

Host tests (no Pico needed):

```sh
cc -std=c11 -Wall -Wextra -Isrc/common -Isrc/motion -Isrc/manager \
   tests/manager/test_manager.c src/manager/manager.c \
   src/manager/manager_button.c src/motion/motion_control.c \
   src/motion/line_motion_bridge.c -lm -o /tmp/tmgr && /tmp/tmgr
```

Firmware:

```sh
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build --target robocar_line_manager
```

The host tests cover the boot self-test, both button toggles, the 100 ms loss threshold, the recovery steering inversion for both directions, the three tracking samples, the recovery timeout, the line-sensor fault, and the fact that PWM does not restart after a STOP or a fault. They also cover both recovery-limit races: the terminal event arriving in the same step as the third tracking sample, and the terminal event still in flight when that sample arrives.

## Open items

- The wheel diameter and the wheelbase are still placeholders. The 100 mm recovery budget and the 100 mm/s reverse speed depend on them, like every other distance in the Motion library.
- `MOTION_STEER_STALE_MS` and the 100 ms loss timeout are the same value today. Motion holds the wheels when STEER is older than 100 ms, so a Manager step that stops publishing STEER for 100 ms also stops the robot.
- The GPIO pin assignment GP20/GP21 as toggles is a checkpoint decision. A future checkpoint must allocate a dedicated emergency-stop input before the robot leaves the bench.

## Deferred work

These items are explicitly out of scope for this checkpoint:

- A physical emergency-stop input.
- Barcode-driven navigation, and the barcode capture loop.
- Telemetry integration and the Wi-Fi transport.
- Ultrasonic obstacle avoidance.
- IMU integration.
- Junction and mission policy. The bridge keeps its current behaviour: unknown, split, and junction patterns steer straight, and the Manager treats them as not lost.
- Repeated recovery attempts after a timeout.
- Loaded-wheel geometry, PID tuning, cruise-speed, and coast measurements.
