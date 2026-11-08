# ADR: one hardware heartbeat; short work in the interrupt, decisions in the main loop

| Item | Value |
| --- | --- |
| Status | Accepted, 2026-11-04 (operator: option B); revised 2026-11-04: owner is a separate platform module (decision 7) |
| Scope | Timing of all modules on core 0 |
| Related | [ADR: motor controller](ADR_MOTOR_CONTROLLER.md), [motor controller](../s2-motion/MOTION.md), [interrupt conventions](ARCHITECTURE.md), [control architecture](../archive/architecture/CONTROL_ARCHITECTURE.md) |

## Context

- Each test program (`detour_test`, `steer_test`, …) has its own main
  loop with its own timing. All read the same clock, but nothing gives a
  common rhythm.
- `mc_tick()` is the motor controller's heartbeat: commands only store
  values; the tick computes and gives out the duty. If the tick stops,
  the PWM keeps its last duty and the stale stop (inside the tick) cannot
  work.
- Encoder edges and ultrasonic echo timing use interrupts. A long
  interrupt handler makes them wait.

## Decision

1. **One heartbeat:** a hardware timer interrupt every **1 ms** on core 0.
   Every periodic module runs on a multiple of it.
2. **Short, exact work runs inside the heartbeat interrupt:**
   - line sensor ADC sample (1 ms);
   - `mc_tick()` (every 10 ms) and the PWM write of its output;
   - restart of the hardware watchdog;
   - "due" flags for the main loop.
3. **Decisions run in the main loop**, started by the flags: line
   position, steering, `mc_set_speed()`, detour, obstacle watch, buttons,
   Wi-Fi log, `printf`.
4. **Order in each 10 ms step: sense → decide → act.** The decision for a
   step is ready before the next `mc_tick()`; the delay from a reading to
   the motors is fixed.
5. **Safety:**
   - Main loop hangs: no new `mc_set_speed()`; the heartbeat still runs
     `mc_tick()`; stale stop after 100 ms.
   - Heartbeat stops (chip hangs): the hardware watchdog, restarted only
     by the heartbeat, resets the chip; the PWM pins go to 0. Watchdog
     time: set in step 2 (about 50 ms).
6. **Rules for the interrupt:** no `printf`, Wi-Fi, sleep, or blocking
   call; measure the worst-case handler time; count overruns. Shared
   data between the main loop and the handler (for example
   `mc_set_speed()` against `mc_tick()`) uses a short interrupt-off
   section ([interrupt conventions](ARCHITECTURE.md)).

| Every | Where | Work |
| --- | --- | --- |
| 1 ms | Interrupt | Line sensor ADC sample; watchdog restart |
| 10 ms | Interrupt | `mc_tick()`, PWM write |
| 10 ms | Main loop | Line position → steering → `mc_set_speed()`; detour step |
| 20 ms | Main loop | Buttons |
| 50 ms | Main loop | Wi-Fi log row |
| 60 ms | Main loop | Ultrasonic ping start (echo timing stays on its own interrupt) |
| — | Hardware | Servo pulse (50 Hz PWM) |

7. **Owner: a separate platform module, `heartbeat`** (revised
   2026-11-04, operator). It owns the 1 ms timer, the hardware watchdog
   and the "due" flags, and it is not part of any module that it drives.
   The users hook into it: the motor controller's Pico layer registers
   `mc_tick()` for every 10th tick, the line sensors register the ADC
   sample for every tick. Reason: the heartbeat serves every layer
   ([ADR: Vehicle Controller](ADR_VEHICLE_CONTROLLER.md)); if one of
   the driven modules owned it, the others would depend on that module.
8. **Not on the heartbeat:** encoder edges and the ultrasonic echo
   (events, their own interrupts) and the motor and servo pulses
   (hardware PWM).

```text
  heartbeat (1 ms timer, watchdog, flags)          ← platform module
     │ interrupt hooks            │ flags
     ▼                            ▼
  ADC sample, mc_tick       detectors → running module → Vehicle Controller
                                                           → mc_set_speed (used at the next mc_tick)
```

Delay from a reading to the motors: at most one 10 ms frame, and always
the same.

## Alternatives considered

| Option | Why not chosen |
| --- | --- |
| A. All module work inside the heartbeat interrupt | Exact timing, but encoder and echo interrupts wait during the long handler; no logging; one slow module makes the motors late too |
| No common heartbeat (each program keeps its own loop) | `mc_tick()` stops when the loop hangs; the stale stop cannot work |

## Consequences

- A new platform module `heartbeat` owns the timer, the watchdog and the
  flags (decision 7). The motor controller's Pico layer (step 2) hooks
  `mc_tick()` into it; it does not own the timer.
- With μT-Kernel (team firmware, `motion_task_utkernel.c`), the RTOS tick
  takes the place of the heartbeat; the split stays the same.
- Core 1 (planned in [control architecture](../archive/architecture/CONTROL_ARCHITECTURE.md))
  keeps the peripherals that tolerate delay.
