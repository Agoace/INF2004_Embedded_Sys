# Motor controller

The only owner of the motor PWM, the servo PWM and the wheel encoders. The
decision and its reasons: [ADR: motor controller](ADR_MOTOR_CONTROLLER.md).
Branch: `mtk3-motion`.

## Status

| Step | What | State |
| --- | --- | --- |
| 1 | Pure core [`app_program/subsystem2/motor_ctl.c`](../app_program/subsystem2/motor_ctl.c), [`app_program/subsystem2/motor_ctl.h`](../app_program/subsystem2/motor_ctl.h); robot settings [`app_program/subsystem2/motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h); host test [`tests/test_motor_ctl.c`](../tests/test_motor_ctl.c) | Done 2026-11-04 |
| 2 | Pico layer [`src/motor_ctl_pico.c`](../src/motor_ctl_pico.c), [`src/motor_ctl_pico.h`](../src/motor_ctl_pico.h): PWM, encoder interrupt, servo; hooks `mc_tick()` into the [heartbeat](HEARTBEAT.md) every 10 ms (the heartbeat owns the timer and the watchdog: [ADR: heartbeat](ADR_HEARTBEAT.md)). CMake library `motor_ctl_pico` | Done 2026-11-04: strict ARM compile clean; not yet run on hardware (step 3) |
| 3 | Floor bench target `motor_ctl_bench` ([`src/motor_ctl_bench.c`](../src/motor_ctl_bench.c)): step response, both directions, slow turn; tune P, I and feed-forward | Done 2026-11-04: runs B1 and B2; settings of B2 kept (`1cf08a1`). Run B3: rise limit 8 (after line run P5) |
| 4 | Callers move to wheel speeds (line steering, detour, scan, search) | Not started; one branch each |

## Interface

| Call | Use |
| --- | --- |
| `mc_init(mc, cfg, counts, now)` | Start; travel and heading count from here |
| `mc_set_speed(mc, left_mm_s, right_mm_s, now)` | Wheel speeds, signed, forward positive. Send again within 100 ms, or the wheels stop (stale) |
| `mc_set_servo(mc, deg, now)` | Servo angle, positive = left; clamped to the safe range; returns the angle set |
| `mc_stop(mc)` | Duty 0 at once, not latched |
| `mc_estop(mc)` / `mc_clear_faults(mc)` | Latched stop; clear (the wheels stay stopped until a new speed) |
| `mc_tick(mc, counts, now, &out)` | Every 10 ms: encoder counts in; motor duty (permille) and servo pulse (µs) out |
| `mc_travel_mm`, `mc_speed_mm_s`, `mc_heading_cdeg` | Measurements: travel, speed, heading change (0.01°, left positive) |
| `mc_servo_settled(mc, now)`, `mc_servo_deg(mc)` | Servo settle estimate and angle |
| `mc_faults(mc)` | Bits: stall left 0x01, stall right 0x02, emergency stop 0x04 (latched); stale 0x08 |

## Speed loop

Adapted from Motion (Buddy 2), `origin/motor-line-manager`,
`src/motion/motion_control.c`, `wheel_control()`. Intent parts not copied:
steer mixing, calm mode, FOLLOW state, move completion, command queue.

| Part | Rule | Setting ([`motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h)) |
| --- | --- | --- |
| Feed-forward | offset + slope × set point, **per wheel and per direction** | Offset / slope (‰, ‰ per count/tick): L fwd 48 / 9.66, L rev 59 / 8.89, R fwd 56 / 9.39, R rev 62 / 8.67. Fit to floor bench run B1 (2026-11-04). Step 1 used a rough fit to the track logs (offset 85–90, slope 9.0–12.0) |
| P | kp × (set point − measured) | 6.0 ‰ per count/tick (Motion 3.0 at 20 ms, scaled) |
| I | ki × error each tick; limit ±400 ‰. **Held** (not integrated) while the output is at the limit in the error's direction, **while the set point still rises, and while the start kick runs** (added after run B1: the wheel lags then, and I wound up and overshot) | 0.5 per tick |
| Rise limit | Set point rises at most 8 counts/tick per tick (about 6400 mm/s²); falls at once. Was 2 (1600 mm/s²) until 2026-11-04: line run P5 showed the wheels followed the steering 50 ms later than with duty, and the steering swung | — |
| Start kick | From rest or on a direction change: at least 12 % until 10 counts; none in 250 ms = stall | Start policy in [calibration results](CALIBRATION_RESULTS.md) |
| Stall | Duty ≥ 15 % and no count for 500 ms | Latched |
| Duty limit | 80 % | — |
| Speed limit | `mc_set_speed()` limits each wheel to ±`max_speed_mm_s` (0 = none) | 460 mm/s: the 60 % point of the [duty map](DUTY_MAP.md) (run DM1); the robot did wheelies at 80 % open loop |
| Units | 12414 counts per metre (2535 counts, 65 mm wheel); 24.05 counts per degree (track 111 mm) | [Robot geometry](CALIBRATION_RESULTS.md#robot-geometry) |

Note: Motion's `motion_config.h` has wheelbase 120 mm (MEASURE); this
repository measured 111 mm.

Servo: 1500 µs centre, 1100–1900 µs (±36°), 11.1 µs per degree. Settle
estimate 42 ms + 3.6 ms per degree, fitted to the scan settle times
(150 ms for 30°, 60 ms for 5°; [servo obstacle scan](OBSTACLE_SCAN.md)).

## Pico layer

```text
 main loop                         heartbeat interrupt, every 10 ms (hook)
 ─────────                         ───────────────────────────────────────
 mcp_set_speed(l, r) ─┐            lock
 mcp_set_servo(deg)  ─┤ lock        read encoder counts (sign per wheel)
 mcp_stop / estop    ─┤ store       mc_tick(counts, ms) → duty, servo pulse
 mcp_status(&s)      ─┘ copy        write motor PWM and servo PWM
                                   unlock
 encoder edges: own GPIO interrupt (quadrature table, from drive.c)
```

| Part | Rule |
| --- | --- |
| Pins | Left PWM GP8/GP9, encoder GP16/GP17 (+1); right PWM GP10/GP11, encoder GP4/GP5 (−1); servo GP15. Copied from `drive.c` and `servo.c`; do not link those files in the same target |
| Motor PWM | 20 kHz, wrap 6249; level = 6250 × \|duty‰\| / 1000 on the forward or the reverse pin; the other pin 0 |
| Servo PWM | 50 Hz, 1 count = 1 µs; written only when the pulse changes |
| Hook | Added by `mcp_init()` with period `tick_ms` (10 heartbeat ticks), phase 0. `mcp_init()` refuses a `tick_ms` that is not a whole number of heartbeat ticks |
| Lock | `critical_section_t` (interrupts off on this core and a hardware spin lock), so calls are safe from both cores. The hook also holds it while it writes the PWM, so a duty computed before an `mcp_estop()` cannot overwrite it |
| `mcp_stop()`, `mcp_estop()` | Also write PWM 0 at once; they do not wait for the next tick |
| Clock | `mcp_now_ms()` = `time_us_64() / 1000`, used by the hook and the main-loop calls. (`time_us_32()` in ms would wrap after 71 minutes.) |
| `mcp_status()` | One consistent copy: counts, travel, speed, duty, heading, servo, faults, tick count, longest hook time |
| Size | 1480 bytes of code (`-Os`, Cortex-M0+), core and heartbeat not included |

The servo pulse changes at the next tick, so up to 10 ms after
`mcp_set_servo()`. The settle estimate starts at the call; the error is
small against the 42 ms base.

## Floor bench (step 3)

`motor_ctl_bench` drives a fixed list of wheel speed steps on the floor
and records one row per 10 ms tick in RAM (up to 24 s). After the run it
prints a summary and all rows over USB and Wi-Fi (`text_<boot>.txt` from
`scripts/steer_log_receiver.py`).

| Step | Speed L/R (mm/s) | Time | Checks |
| --- | --- | --- | --- |
| fwd 150, rev 150 | ±150 | 1.5 s each | Rise, overshoot, steady error, reverse against forward |
| fwd 300, rev 300 | ±300 | 1.0 s each | Higher speed, duty limit |
| spin L 20, spin R 20 | ∓20 / ±20 | 3.0 s each (about 60°) | The slow-turn case (detour sweep, 20°/s): does the I term hold a speed below the dead zone? |
| fwd 40, rev 40 | ±40 | 2.0 s each | The slowest steady speed seen in the track logs |

Each step is followed by 0.7 s at speed 0 (coast). The list is 21.4 s; the
net travel is about 0.

| Control | Action |
| --- | --- |
| GP21 | Start after 1 s; a second press is an emergency stop |
| GP20 | Print the last run again |

The run stops early on a latched fault (stall or emergency stop).
Space: about 40 cm clear ahead and behind, and room to turn.

Row format: `MB i step tgtL tgtR spdL spdR dutyL dutyR cntL cntR faults`
(mm/s, permille, encoder counts). The summary gives, per step, the mean
speed and duty over the second half and the heading change from the
counts, plus the longest hook time and the heartbeat counters.

### Run B1 (2026-11-04, boot 5bf1, `ad2404c`, floor, VIN)

Log `build/steer_logs/text_5bf1.txt`; report
`python3 scripts/motor_bench_report.py build/steer_logs/text_5bf1.txt`.
All 17 steps done, no fault, 2101 of 2110 rows received (rows 7–15, in
the first rest, lost over Wi-Fi). Settings: `MC_ROBOT_CONFIG` as in
step 1 (untuned).

| Step | Wheel | Time to 90 % (ms) | Overshoot (50 ms avg) | Mean, 2nd half (mm/s) | Ripple sd (mm/s) | Duty, 2nd half (‰) |
| --- | --- | --- | --- | --- | --- | --- |
| fwd 150 | L / R | 150 / 180 | 18 % / 17 % | +151 / +149 | 2.2 / 2.8 | 219..242 / 223..241 |
| rev 150 | L / R | 150 / 170 | 34 % / 24 % | −152 / −150 | 4.1 / 2.5 | 214..233 / 211..230 |
| fwd 300 | L / R | 230 / 230 | 11 % / 12 % | +310 / +310 | 4.0 / 5.2 | 406..432 / 402..427 |
| rev 300 | L / R | 220 / 240 | 26 % / 17 % | −325 / −317 | 18.1 / 12.2 | 403..441 / 390..417 |
| spin L 20 | L / R | 60 / 60 | 100 % / 60 % | −20 / +19 | 3.8 / 3.3 | 82..115 / 75..103 |
| spin R 20 | L / R | 50 / 50 | 124 % / 100 % | +21 / −20 | 2.6 / 2.7 | 69..97 / 85..108 |
| fwd 40 | L / R | 90 / 60 | 52 % / 40 % | +40 / +40 | 1.1 / 1.6 | 81..95 / 86..99 |
| rev 40 | L / R | 70 / 70 | 56 % / 48 % | −41 / −40 | 1.4 / 2.0 | 83..99 / 84..98 |

| Item | Result |
| --- | --- |
| Steady speed | Within 2 mm/s of the target at 20, 40 and 150 mm/s, both wheels and both directions. **20 mm/s holds** (the detour sweep case), at 75–115 ‰ duty |
| Spin | 65.3° and −66.1° for a 3 s step at 20 mm/s (62° from the target alone; the extra is the start overshoot) |
| Straight drives | Heading change −0.3° (fwd 150), +3.2° (rev 150), +0.2° (fwd 300), +6.7° (rev 300): the left wheel overshoots more in reverse |
| Overshoot | 11–34 % at 150 and 300 mm/s; 40–124 % at 20 and 40 mm/s (start kick and quantisation: 1 count per tick = 8 mm/s). At 300 mm/s the 1 s step ends before the overshoot is gone |
| Cause of the overshoot (fwd 150, left) | The wheel lags the set point by about 100 ms. During the lag P and I rise; the duty peaks at 320 ‰ against 230 ‰ steady, then the I term unwinds over about 300 ms |
| Feed-forward (steady points) | Forward and reverse are now alike (the weak left reverse from the track logs is not seen). Straight-line fit, counts per tick: L fwd 48 + 9.66·v, L rev 59 + 8.89·v, R fwd 56 + 9.39·v, R rev 62 + 8.67·v. The step-1 values are too high (L rev slope 12.0: 308 ‰ against 223 ‰ needed at 150 mm/s). Below 5 counts per tick the duty is flat at about 80–97 ‰ (dead zone) |
| Timing | Hook at most 88 µs; heartbeat interrupt at most 102 µs (10 % of a tick); no overruns. Missed 424 frames: the main loop stopped during the row print after the run (motors at 0) |

### Run B2 (2026-11-04, boot 8460, `1cf08a1`, floor, VIN)

Changes from B1: feed-forward refit to B1, and the I term held while the
set point rises and while the kick runs. Same step list. Log
`build/steer_logs/text_8460.txt`. All 17 steps done, no fault; the same
9 rows (7–15) lost over Wi-Fi.

| Step | Overshoot B1 → B2 (L / R) | Time to 90 % B1 → B2 (ms, L / R) | Mean, 2nd half B2 (mm/s) | Heading change B1 → B2 |
| --- | --- | --- | --- | --- |
| fwd 150 | 18/17 % → **7/8 %** | 150/180 → 170/180 | +149 / +150 | −0.3° → +0.5° |
| rev 150 | 34/24 % → **7/8 %** | 150/170 → 200/200 | −152 / −151 | +3.2° → −0.7° |
| fwd 300 | 11/12 % → **5/6 %** | 230/230 → 260/260 | +304 / +304 | +0.2° → +0.5° |
| rev 300 | 26/17 % → **4/4 %** | 220/240 → 290/290 | −304 / −306 | +6.7° → −0.5° |
| spin L 20 | 100/60 % → 116/28 % | 60/60 → 50/60 | −20 / +20 | 65.3° → 59.8° |
| spin R 20 | 124/100 % → 116/116 % | 50/50 → 40/50 | +21 / −19 | −66.1° → −59.1° |
| fwd 40 | 52/40 % → **12/24 %** | 90/60 → 160/70 | +39 / +39 | −0.6° → +0.5° |
| rev 40 | 56/48 % → **20/28 %** | 70/70 → 130/120 | −39 / −39 | +0.9° → +0.1° |

| Item | Result |
| --- | --- |
| 150 and 300 mm/s | Overshoot 4–8 % (was 11–34 %); ripple sd 1.8–2.7 mm/s (was up to 18); the 300 mm/s mean is now within 6 mm/s in the 1 s step. Rise is 20–70 ms slower: the I term no longer helps during the rise |
| Straight drives | Heading change at most 0.7° in every straight step (was up to 6.7°): the two wheels now match |
| Slow spin (20 mm/s) | Still about 2× the target speed in the first 50 ms. Cause: the start kick (120 ‰ until 10 counts) drives the wheel to 40–56 mm/s in 3 ticks; after the kick the speed drops to 8–16 mm/s for about 200 ms while I builds from 0. The turn is 59–60° against 62° from the target |
| Feed-forward refit to B2 | Offset 52–60 ‰, slope 8.7–9.3: close to the values in use, so no change |
| Timing | Hook at most 85 µs, heartbeat interrupt at most 99 µs, no overruns |

Decision (2026-11-04, operator): keep the B2 settings; the slow-start
kick is accepted (about 100 ms at the start of a slow turn; the spin is
within 3° of the target). Options not taken are under *Ideas*.

### Run B3 (2026-11-04, boot ede2, rise limit 8 counts/tick per tick, floor, VIN)

Change from B2: the rise limit 2 → 8 counts/tick per tick (line run P5,
[Vehicle Controller line run](VC_LINE_RUN.md)). Same step list. Log
`build/steer_logs/text_ede2.txt`. All 17 steps done, no fault; the same
rows 7–15 lost over Wi-Fi.

| Step | Overshoot B2 → B3 (L / R) | Time to 90 % B2 → B3 (ms, L / R) | Ripple sd B2 → B3 (mm/s) | Heading change B3 |
| --- | --- | --- | --- | --- |
| fwd 150 | 7/8 % → 6/11 % | 170/180 → **140/130** | 2.1/2.5 → 2.5/2.4 | +0.5° |
| rev 150 | 7/8 % → 9/12 % | 200/200 → **180/170** | 1.8/2.6 → 2.6/3.1 | −1.3° |
| fwd 300 | 5/6 % → 10/9 % | 260/260 → **160/150** | 2.4/2.7 → 5.0/4.5 | +0.2° |
| rev 300 | 4/4 % → 8/9 % | 290/290 → **200/180** | 2.0/2.2 → 7.5/7.7 | −2.2° |
| spin L 20 | 116/28 % → 44/36 % | 50/60 → 70/50 | about 3 | 59.0° |
| spin R 20 | 116/116 % → 100/116 % | 40/50 → 50/50 | about 3 | −58.9° |
| fwd 40 | 12/24 % → 8/24 % | 160/70 → 210/70 | 1.4 → 1.4/1.8 | +1.3° |
| rev 40 | 20/28 % → 12/24 % | 130/120 → 190/60 | about 2 | −1.6° |

| Item | Result |
| --- | --- |
| Rise | 20–50 ms faster at 150 mm/s, 90–110 ms faster at 300 mm/s |
| Overshoot | 6–12 % at 150 and 300 mm/s (B2: 4–8 %; B1: 11–34 %) |
| Ripple at 300 mm/s | sd 4.5–7.7 mm/s (B2: 2–2.7); about one count per tick (8 mm/s) |
| Straight heading | Up to 2.2° in rev 300 (B2: 0.7°) |
| Slow steps | Same as B2: the start kick sets them |
| Feed-forward refit | Offset 53–68 ‰, slope 8.4–9.2: close to the values in use |
| Timing | Hook at most 85 µs, heartbeat interrupt at most 99 µs, no overruns |

## Ideas (not implemented)

**Burst drive for the dead zone** (operator idea, 2026-11-04). At 20 kHz the
motor sees the average voltage, so a low duty is a steady weak push below
the static friction. Short strong pushes with rests between them can
break the friction and give a lower average speed. Reason: the slowest
steady speed is about 40–50 mm/s (14 %), but a 20°/s detour sweep needs
about 19 mm/s per wheel; the sweeps ran at 40°/s.

| Way | How | Cost |
| --- | --- | --- |
| Lower PWM frequency | About 100–500 Hz | Audible whine |
| Bursts | 20 kHz at about 25 % for N ms, then off for M ms | Step-like motion; counts come in bunches |
| Dither | A small fast change added to the duty | Smaller effect |
| Brake mode (driver chip) | Brake instead of coast between pulses (`drive.c` coasts now) | Needs the Robo Pico driver chip datasheet |

If built: the speed loop must average over a full burst cycle. Bench
test first: the slowest steady speed on the floor for each way.


Slow-start kick (from run B2, not taken):

| Option | Effect | Cost |
| --- | --- | --- |
| Shorter kick (`kick_counts` 10 → about 4) | Smaller burst at the start of a slow turn | A weak start may not break the static friction; needs a bench run |
| Start I at the kick duty, not 0 | Removes the 200 ms slow patch after the kick | One more rule in the loop; needs a bench run |

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_motor_ctl.c app_program/subsystem2/motor_ctl.c -o build/test_motor_ctl
./build/test_motor_ctl
```

The test drives a simulated wheel pair (dead zone about 9 %, first-order
lag) and checks: 150 mm/s reached within 5 %; the same with a 30 % battery
drop, a 20 % weaker left motor, a weak left reverse, and a slow turn on the
spot (±40 mm/s, the detour sweep case) with no stall; rise limit; start
kick and kick on a direction change; stale stop at exactly 100 ms; stop;
emergency stop and clear; stall at exactly 500 ms (both duties 0); speed
sent while latched has no effect; late tick; exact feed-forward, P, clamp
and anti-windup values; I held while the set point rises and while the
kick runs, and I grows at the target; travel, heading; servo pulse, clamp
and settle time.

Mutation check 2026-11-04: 35 of 36 caught. The survivor is equivalent
(`<=` against `<` in the rise limit: both give the asked set point). Four
redundant lines found by the check were removed.

Mutation check after the speed limit (2026-11-04): 42 of 43 caught (4
new mutants), the same equivalent survivor.

Mutation check after the I hold (2026-11-04): 38 of 39 caught (3 new
mutants for the hold rule), the same equivalent survivor. The check first
showed that the anti-windup test had become empty (the kick held I); the
test now ends the kick after one tick.

Pico compile check: `arm-none-eabi-gcc` with `-Wall -Wextra -Wconversion
-Wshadow -Wformat=2 -Werror`: clean; 1.9 KB of code.

## μT-Kernel layer

Step 1 of the kernel bring-up uses `mtk3-motion`, merged only into
`mtk3-staging`. Code: [`motion.c`](../app_program/subsystem2/motion.c),
[`motion.h`](../app_program/subsystem2/motion.h), hardware adapter
[`drv_motor_hw.c`](../app_program/subsystem2/drv_motor_hw.c), floor bench
[`motion_bench.c`](../app_program/subsystem2/motion_bench.c). Robot settings
are in [`motor_ctl_robot.h`](../app_program/subsystem2/motor_ctl_robot.h).
The pure `motor_ctl.[ch]` and its tuning values are unchanged.

### Decision: T first, H only if needed

- **T (accepted):** the 10 ms `cyc_motion` handler only calls
  `tk_wup_tsk()`. `motion_task`, priority 10 and pinned to processor 1
  (`TP_PRC1`, physical core 0), copies encoder counts with DI/EI, calls
  `mc_tick()` and writes motor/servo PWM. It calls `tk_can_wup()` before
  the work; queued requests are discarded and counted, not replayed as
  a burst of short controller intervals.
- **Why T:** controller work stays in task context. The same inherited
  mutex protects task work and API calls; there is no interrupt handler
  that must take a task mutex or duplicate controller state.
- **H (fallback, not implemented):** move the bounded controller update
  to a hardware timer interrupt only if measured task-gap jitter exceeds
  1 ms (outside 9000–11000 us), or U01/B4 loses wakes. Review the ISR/API
  sharing design first; an ISR must not acquire `mtx_motion`. Do not
  claim a hard timing guarantee before hardware measurement.

The kernel tick remains 10 ms. The `motion_task` priority is above every
other application task and above the kernel radio (20) and USB console
(21). Only the initial task (1) during startup is higher. No print call or sleep is inside the controller mutex.
`tick_max_us` includes mutex wait and work through the PWM writes.
`gap_min_us` and `gap_max_us` measure successive task-start times;
`late_wakes` counts gaps above 11000 us. `missed_wakes` counts queued
requests discarded by `tk_can_wup()`. These are lifetime counters.

### API and ownership

A task function never returns; end it with tk_ext_tsk().

All `motion_*` control/status calls are for task context. They return
`E_OK` or a negative kernel error. `motion_set_servo(deg, &accepted)` also
returns the accepted angle through the optional pointer. Calls use
`mtx_motion` with `TA_INHERIT`, including calls from the other core.

| Call | Rule |
| --- | --- |
| `motion_set_speed(l, r)` | Refresh within `stale_ms` (100 ms) |
| `motion_stop()` | Clear requested motion and write PWM 0 inside the mutex |
| `motion_estop()` | Latch ESTOP and write PWM 0 inside the mutex; a previous computation cannot overwrite it |
| `motion_clear_faults()` | Clear latched faults, keep wheels stopped and write PWM 0 |
| `motion_set_servo(deg, &accepted)` | Clamp through the unchanged core; the next task tick writes the pulse |
| `motion_status(&s)` | One mutex-protected copy of counts, travel, speed, duty, heading, servo state, faults and timing |
| `motion_notify_after_tick(tid)` | Wake one client task after each completed tick and mutex release; positive ID registers, 0 disables; keep the client alive until disabled |

Status counts are the coherent **last-tick** sample. Only the core-0
motion task copies the raw encoder counters under DI/EI; disabling local
interrupts on core 1 would not protect a live core-0 encoder read.
Stop/ESTOP writes do not wait for the next 10 ms task tick. PWM compare
registers take effect at the next 20 kHz wrap (at most 50 us), as in the
bare-metal controller. Stop is coast, not active braking.

### Hardware and interrupt boundary

Kernel-side `motion.c` and `drv_gpio_irq.c` include no SDK headers.
SDK/register-only `*_hw.c` files include no kernel headers, avoiding the
kernel/SDK `size_t` conflict. `drv_time_us()` reads TIMERAWL;
`drv_time_ms()` reads a stable high/low hardware timer pair before division,
so milliseconds do not jump back after the 32-bit microsecond wrap.

The kernel `cpu_clock.c` configures PLL SYS to 125 MHz (1500 / 6 / 2).
Motor slices 4/5 use div 1, wrap 6249: 20 kHz. Servo GP15, slice 7 channel
B, uses div 125 and wrap 19999: 50 Hz and 1 us per count. With
`USE_PTMR=0` the kernel does not release PWM reset; the hardware owner
uses the SDK reset/unreset helper and waits for `reset_done` before setup.

Pins, signs and the quadrature table match `src/motor_ctl_pico.c`: left
GP8 forward/GP9 reverse and GP16/17 encoder sign +1; right GP10/11 and
GP4/5 sign -1. Encoder inputs have no pulls and use both edges.
The shared core-0 dispatcher owns IRQ 13 (`IO_IRQ_BANK0`), installed with
`tk_def_int(TA_HLNG)`. It dispatches registered pin masks and acknowledges
edge bits in IO_BANK0 INTR before callbacks. GPIO uses unshifted priority
1. The template USB IRQ5 bridge does not write NVIC priority, so it keeps
reset level 0; GPIO is below USB and above SysTick/PendSV level 3.

### Bench and tests

The bench task has priority 11 on core 0 and now requires `BENCH=1`.
The default normal image selects the [kernel robot program](KERNEL_ROBOT.md)
instead; only one task owns the buttons and tick notification slot. GP21 arms a run after 1 s;
press again to ESTOP. GP20 prints the saved log. Buttons use pull-ups,
active low, polled every 20 ms. The B3 step list is unchanged and the RAM
log has 2400 rows. The bench registers for a wake after each completed
motion tick and waits with `tk_slp_tsk(20)`, then drains extra queued wakes
with `tk_can_wup()`. It records one row per motion tick (10 ms), as in B3.
`skipped_rows` counts lost rows from extra queued wakes or tick gaps without
double counting; it must be 0 in a normal run. `tk_dly_tsk(10)` is not used
for sampling: this kernel adds `TIMER_PERIOD` (10 ms) to the delay, which
would sample about every 20 ms and lose alternate rows. Step durations
and button polling still use `drv_time_ms()`. No output occurs during a run. The log is not dumped
automatically: run on VIN with USB unplugged, then connect USB and press
GP20 without resetting. The dump pauses 20 ms after each eight rows and
ends with `tm_usb_dropped` total and dump delta. A new GP21 start clears
old faults; a completed/stopped run keeps its rows and fault state.

TEST builds start neither the button bench nor the robot task. They run S0-U01 (the
platform clock comparison), U01 (5 s of ticks) and U02 (a brief
lifted-wheel stale-command test). Before the clock fix, S2-U01/U02/U03
passed and B4-lifted passed its functional checks on USB after flashing.
U03 passed after a left motor/encoder wire fault was repaired. With the
clock fix, S0-U01 passed after a cold power-on, and floor B4 (cold VIN
start) passed functionally: 2110 rows, 0 faults, task gap 9952..10046 us,
speeds and duty close to B3. Its strict B3 limits were missed on slow
speeds and on heading, within run-to-run spread; straight steps drifted less
than B3. U04 remains **not run**. See exact measurements and the B3 means in
[`testing/subsystem2/TEST_LOG.md`](../testing/subsystem2/TEST_LOG.md).
Host doubles check the dispatcher, core affinity, mutex ownership, wake
accounting, encoder signs, stale stop and immediate ESTOP ordering; they
do not measure hardware timing or physical safety.

### Hardware watchdog

The [watchdog record](KERNEL_WATCHDOG.md) defines the motion-only 100 ms
hardware watchdog, startup prerequisites, reset classification and pending
S2-U05 two-boot test. It supplements the stale-command stop. A failed
motion initialization does not start it; motor PWM remains zero.

`motion_start()` starts the task, then cyclic handler, then watchdog.
Speed commands are rejected until the watchdog is armed. Each completed
tick feeds only after PWM writes and mutex release, before notifying the
bench/client. A stuck task or failed mutex acquisition does not feed.
Bench dumps print the cached last reset reason. S2-U05 is a separate final
TEST hook after S1..S5; the next boot reports its scratch-carried elapsed
observation and does not repeat the automatic reset.

### Cold-power timer source

Test every kernel image after a cold power-on, not only after a flash.
The RP2040 TIMER uses the watchdog tick generator. Kernel startup selects
12 MHz XOSC for `clk_ref`, but does not set `WATCHDOG_TICK`. At cold power-on
its reset value is `0x200`: enabled, CYCLES 0 (divide by 512). TIMER then
runs about 42.7 times too slowly. A picotool reboot can leave CYCLES 12
from boot ROM setup, hiding the defect. SysTick kernel time is independent.

`usermain()` calls `drv_time_init()` before any subsystem. The SDK-only
adapter calls runtime-free `tick_start(TICK_WATCHDOG, 12)`, where 12 is the
kernel `XOSC_MHz` value, and polls `WATCHDOG_TICK_RUNNING_BITS` with a bounded
CPU-iteration wait. SDK 2.3.0 `tick_is_running()` checks ENABLE only on
RP2040, so it is not sufficient here. Repeated initialization leaves an
already correct generator unchanged. No TIMER count is reset. Time
initialization alone does not enable the watchdog reset counter;
`motion_start()` enables it later. Failure returns `E_SYS` and the
startup `CHECK` prints an error.

`drv_time_check()` compares TIMER microseconds with elapsed `tk_get_otm()`
kernel milliseconds over a nominal 100 ms delay and rejects error greater
than 2%. It first waits for a kernel tick boundary to reduce 10 ms time
quantization. It measures the actual kernel interval; `tk_dly_tsk()` adds
one `TIMER_PERIOD`, so its argument is not the measured duration.
S0-U01 uses a nominal 1 s window and a 1% limit, printing both measurements;
see [the platform test log](../testing/subsystem1/TEST_LOG.md). The updated
image has not yet passed this cold-power hardware test.

The template Wi-Fi `time_us_64()` in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c` reads the same TIMER
and has the same cold-power defect. The early application initialization
covers later Wi-Fi calls without changes to template kernel or library
files. It cannot correct any time read before `usermain()`.

`motion_init()` first sets up zero motor PWM, then calls `drv_time_init()`
and `drv_time_check()`. A failure prints one clear motion error and returns
before a mutex, cyclic handler or motion task is created. `motion_start()`
then fails, so `subsystem2_start()` does not start the bench. The servo may
be centered; all motor PWM remains zero. Host doubles check both failure
paths and confirm that no bench task starts.

The operator found the defect on 2026-11-06 after cold VIN power-on:
11165 motion ticks represented about 112 s of real time, but TIMER gaps
were only 233..235 us. The nominal 500 ms rest lasted about 21 s, followed
by an unexpected forward step; the 2400-row log filled early. The ignored
local log is `build/mtk_logs/usb_console.txt`. Earlier post-flash success
is not cold-power validation. See the [S2 hardware record](../testing/subsystem2/TEST_LOG.md).

### SDK hardware subset

`drv_motor_hw.c` uses `gpio_set_function()`, `pwm_get_default_config()`,
`pwm_config_set_clkdiv()`, `pwm_config_set_wrap()`, `pwm_init()`,
`pwm_set_gpio_level()` and `pwm_gpio_to_slice_num()`, close to the
bare-metal adapter. It also uses GPIO input/pull/read helpers. No hand-written
PWM/GPIO register setup remains in this adapter. The opposite motor pin
is cleared before applying duty, as in the bare-metal source.

The app-owned [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk)
is included by [`the application build hook`](../build_make/mtkernel_3/app_program/subdir.mk).
No kernel source changes. Robot selection adds `BENCH` to the profile,
image name and C flags in `build_make/pico_rp2040.mk`; the SDK source hook
stays app-owned. `PICO_SDK_PATH` is
required, also for UART/radio-off builds. SDK version: 2.3.0.

Every added include path, relative to `PICO_SDK_PATH`:

```text
src/common/pico_base_headers/include
src/common/pico_binary_info/include
src/common/pico_util/include
src/rp2040/pico_platform/include
src/rp2_common/pico_platform_compiler/include
src/rp2_common/pico_platform_sections/include
src/rp2_common/pico_platform_panic/include
src/rp2_common/pico_platform_common/include
src/rp2_common/hardware_base/include
src/rp2_common/hardware_gpio/include
src/rp2_common/hardware_pwm/include
src/rp2_common/hardware_adc/include
src/rp2_common/hardware_timer/include
src/rp2_common/hardware_ticks/include
src/rp2_common/hardware_watchdog/include
src/rp2_common/pico_bootrom/include
src/rp2_common/boot_bootrom_headers/include
src/rp2_common/pico_flash/include
src/rp2_common/hardware_boot_lock/include
src/rp2_common/hardware_resets/include
src/rp2_common/hardware_clocks/include
src/rp2_common/hardware_irq/include
src/rp2_common/hardware_sync/include
src/rp2_common/hardware_sync_spin_lock/include
src/rp2040/hardware_regs/include
src/rp2040/hardware_structs/include
```

Source added: `src/rp2_common/hardware_adc/adc.c`. The S3 line adapter now
uses it as well as the retained SDK link proof. Source added for
`WIFI=none`: `src/rp2_common/hardware_gpio/gpio.c`. For `WIFI=cyw43` the
fragment reuses the template's existing `mtkernel_3/cyw43/gpio.o`, not a
duplicate copy. No PWM/resets .c files exist: the selected APIs are inline.
Timer and clocks .c files are not added. Source added:
`src/rp2_common/hardware_ticks/ticks.c`; its RP2040 `tick_start()` only
writes the tick register and needs no SDK runtime. Also added:
`src/rp2_common/hardware_watchdog/watchdog.c`; only its runtime-free
RP2040 enable/update/reason paths are retained. No bootrom/flash/boot-lock
.c source is compiled.

Per-function/data sections and linker garbage collection are enabled in
all profiles, also radio-off, so unused GPIO SDK interrupt helpers do not
pull SDK IRQ ownership or runtime into the image. PWM setup and duty
writes, GPIO setup/read, reset release and the inline
`timer_time_us_32()` need no SDK runtime. `adc_init()` calls only inline
reset/poll operations. A separate, never-called hardware probe retained at
link time can check these dependencies without running on hardware.

Do not add SDK `pico_runtime`, crt0, stdlib, IRQ, sync, multicore or time
initialization sources. IRQ/sync include paths provide declarations only.
Do not call GPIO SDK IRQ registration, timer alarm helpers,
`irq_set_exclusive_handler()`, or spinlock APIs: the kernel owns these.
`clock_get_hz()` needs SDK clocks runtime state and is not used. Kernel
PLL SYS is fixed at 125 MHz; the servo divider is `125.0f`, 1 us/count.
`reset_unreset_block_num_wait_blocking(RESET_PWM)` is inline and needs no
runtime; it releases PWM even with kernel `USE_PTMR=0`. Reset all PWM
only during the one S2 hardware initialization, before any PWM client
runs. The free-running hardware timer is read, not reinitialized;
64-bit time uses the stable high/low register pair, not SDK alarm/time
initialization. SDK-only adapters must not include `tk/` headers.

SDK directories use `-isystem`, not application `-I` paths. SDK 2.3.0 has
conversion warnings in unused GPIO 64-bit helpers and conversion/unused
parameter warnings in timer inline helpers with our strict flags. System
headers keep vendor diagnostics separate; the application still passes
`-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror` unchanged.

#### Repeat the link-only proof

The source is [`tests/probes/sdk_hw_link_probe.c`](../tests/probes/sdk_hw_link_probe.c).
It is not in the application wildcard and is never called. After a full
build, use [`scripts/check-sdk-hw-link.py`](../scripts/check-sdk-hw-link.py)
before changing the object profile:

```sh
cd build_make
make clean SMP=0 WIFI=none
make SMP=0 WIFI=none -j8 > /tmp/motion-up.log 2>&1
cd ..
python3 scripts/check-sdk-hw-link.py /tmp/motion-up.log --output-dir /tmp/sdk-proof-up
```

Repeat for `SMP=0 WIFI=none TEST=1` and the default SMP/radio profile.
The script rejects a log/profile mismatch, compiles the SDK-only probe
with strict flags, and retains it with `--undefined=app_sdk_hw_link_probe`
in a separate ELF. `adc_init`, GPIO setup, `tick_start` and the probe must be retained,
with no undefined symbols. No probe UF2 is made. Do not call or flash it.

The template radio profile already has a `clock_get_hz()` compatibility
function in `cyw43_utk_compat.c` that returns constant 125000000. This is
not SDK clocks runtime state. Neither the motor adapter nor the link
probe calls it; radio-off builds must not depend on that conditional shim.
The kernel already configures CLK_ADC from PLL USB to 48 MHz in
`startup_clock(CLKATR_USB | CLKATR_ADC | CLKATR_RTC | CLKATR_PREI)`.
This source check is not a measured ADC result. The new S3 driver checks
its source/divider and owns ADC0/1; S3-U01 on the robot image is not run.
