# Subsystem 2: motion bring-up

Hardware results below were supplied by the operator on 2026-11-06.
A build or a host test is not an on-target result. Keep both wheels lifted
before powering a TEST image. Earlier passes used USB after flashing;
they do not validate a cold power-on. The fixed image passed
[S0-U01](../subsystem1/TEST_LOG.md) after a cold power-on, and B4 ran
from a cold VIN power-on.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S2-U01 | TEST priority-20 runner: begin/end a separate 5 s timing window; print buffered PROBE A and lifetime INFO afterwards | 500 ± 1 window ticks; maximum complete in-window task gap ≤ 11000 us; no discarded wakes in window; report window work maximum, gaps and late count; retain lifetime extrema/counters | PASS 2026-11-06 (3f889a1), both boots: window gap 9984..10016 and 9791..10210 us, late 0, missed 0. The earlier 445084c FAIL was a test artefact. [Results](../integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06) |
| S2-U02 | TEST image: read the warning first; wheels lifted. After a 2 s delay, send 40/40 mm/s once, then no refresh for `stale_ms + 30` ms | `MC_FAULT_STALE` set; both duty values 0; wheels stop | PASS |
| S2-U03 | Normal bench image, idle, wheels lifted: turn each wheel forward by hand, then press GP20 for status | Left and right counts both increase for forward rotation; pins/signs match the table in TEAM.md; reverse rotation decreases counts | PASS after left wire fault repair; left motor and encoder were dead before repair; bare-metal bench confirmed the wire fault |
| S2-U04 | Normal bench image, wheels lifted: press GP21, wait for a drive step, press GP21 again, then press GP20 | ESTOP remains latched; both motor PWM channels become 0 without waiting for the next 10 ms task tick; later ticks cannot restore an old duty. Confirm physical stop and, where available, PWM with a scope | PASS 2026-11-06 (BENCH, mtk3-staging 8df29d6): ESTOP during fwd 300 (about 330 mm/s, duty about 410), wheels stopped at once and stayed stopped (operator); faults 0x04. Scope not used. See the S2-U04 run below |
| S2-U05 | TEST=1 LAST, after S1..S5: wheels lifted; warning, stop feeds after one final tick, expect reboot; leave power connected and capture next boot's later previous-boot line | Own watchdog TIMER reset; previous elapsed sample 90..110 ms (last observation, not exact reset instant); motor pins reset and wheels remain stopped; no endless reset loop | PASS 2026-11-06: second boot reported `watchdog reset after 99 ms`, no repeat reset; wheels stopped (log `build/mtk_logs/s2u05_20261106-201116.txt`) |
| B4-lifted | USB after flash, wheels lifted, same 17-step list | Functional check: all steps complete, 0 faults/skipped rows, maximum gap ≤11000 us; not a floor speed/heading match | PASS: 2110 rows, 0 skipped/faults, gap 9958..10041 us, max 119 us, 0 late/missed |
| B4 | Floor/VIN run, same 17-step B3 list. Leave USB unplugged during the run. GP21 starts after 1 s; GP21 again stops. After the run, connect USB, open the terminal and press GP20. Save the summary and MB rows | Second-half speed within ±5% of each signed B3 mean below; step heading change within ±100 cdeg of B3; no faults; maximum task gap ≤11000 us. Dump delta of `tm_usb_dropped` 0; no skipped log rows | Functional PASS; strict speed and heading limits missed (see the B4 record). Run-to-run spread, not a defect |

## STEP 7 S2-U01 failure and approved window/probe correction

Operator log (read-only main checkout):
`build/mtk_logs/s7_test_20261106-215149.txt`, TEST image `445084c`.
Both boots reported 501 ticks and one lifetime late wake, no discarded
wakes. Gap ranges were 8991..11024 us and 8535..11475 us; work maxima were
113 and 198 us. S2-U02 and the last two-boot S2-U05 passed (99 ms).
These results do not pass the 11000 us gap gate.

Analysis found that only tick count used the 5 s delta. Gap extrema and
late/missed fields were lifetime values, including S0/S1 beforehand.
S1 filled its MBF from the initial task at priority 1: BEGIN plus 100 sends
without yielding. This task outranked motion10 and USB3. The 34 accepted /
66 lost rows match the 4096-byte MBF and 116-byte messages plus headers.
That one-off burst, possibly followed by USB service, is the leading cause
of the late/early pair. It is not yet confirmed by phase snapshots; the old
log cannot locate the event inside or outside S2's own window. No long
sensor-lock or interrupt-off section was found that explains 1.5 ms.

The approved correction changes TEST only:

- After startup, the initial TEST runner changes to priority 20. Motion10,
  line11 and sonar12 can preempt it; telemetry30 still cannot drain the
  saturation burst while the runner is ready. A failed priority change
  aborts TEST, not the firmware startup path.
- The new motion API counts starts inside the window and only complete
  gaps between those starts. The crossing gap from before the window is
  excluded. Late count starts at zero; discarded wakes use an unsigned
  lifetime end-minus-begin delta. Lifetime counters are never reset.
- Tick count remains 499..501, gap limit remains 11000 us and discarded
  wakes must remain zero. No ADC/sample/settings/production priority change.
- Eight small PROBE A snapshots record TIMER us, ticks, lifetime gap/work
  maxima and late/missed counts around S0 printing, S1 burst, its yield and
  print, and S2 entry/exit. They are copied under the motion mutex without
  output. The one PROBE block and lifetime INFO print only after the window.
  See [expected format](../../docs/KERNEL_ROBOT.md#fix-1-and-probe-a-test-only).

**New-image hardware status: PASS 2026-11-06** (see the S2-U01 row). Record both boots and the exact
revision before changing this row. If demotion removes the startup delay,
all probe late counts can be zero. If a remaining S0/S1/USB delay occurs,
its lifetime count must stay visible while a clean S2 window reports zero
late/missed events. Do not fabricate a late event to match that hypothesis,
or widen the timing limit. Preserve any failing window for review.

## S2-U05: two-boot watchdog procedure

Earlier images passed this destructive test, including 445084c at 99 ms.
The new window/probe TEST image passed (99 ms) on 2026-11-06. It is a separate last hook called
by `usermain()` after all other subsystem tests, not inside S2-U01/U02.
Lift both wheels before TEST power. Keep USB connected for this check.

1. Save earlier tests and the S2-U05 warning. Motor PWM is zero before the
   warning. After a 1 s USB delivery pause, motion feeds once more after
   a completed tick and acknowledges feed suppression. No other task feeds.
   Expect reset about 100 ms after the final feed. Do not press RUN or
   remove power; doing so discards or invalidates the result.
2. USB can disconnect during reset. Reopen it and wait for the rebooted
   image's later TEST output: `S2-U05 previous boot: watchdog reset after
   N ms (last observed before reset; target 100 ms)`, then PASS/FAIL.
   Scratch0/1 carry the last elapsed observation; scratch4 remains the SDK
   enable/reboot marker. The result is consumed once, so this second boot
   does not repeat the reset. Save both boot logs and the tested revision.

The elapsed value is the last live CPU observation before reset, not a
hardware-latched reset timestamp. Use a scope/independent timer to verify
strict reset/physical stop latency. Debug halt pauses the watchdog; do not
halt during the test. See [watchdog design](../../docs/KERNEL_WATCHDOG.md).
Failure to initialize the motion clock or start the task/cyclic prevents
watchdog startup and driving; S2-U05 reports that it is not armed.

## Hardware run records: 2026-11-06

S2-U01 and S2-U02 log: `build/mtk_logs/s2_test_20261106-160716.txt`.
S2 reported 2 passed, 0 failed. S2-U03 repair and pass are operator-reported;
no separate encoder log was supplied. These ignored local logs are not
committed.

### B4-lifted: USB, after flash

Source: `build/mtk_logs/b4_lifted_20261106-162058.txt`. Completed all 17
steps: 2110 rows, skipped 0, faults 0x00; final heading +153 cdeg, counts
L/R -37/0. Lifetime 6211 ticks, maximum 119 us, gap 9958..10041 us,
late 0, missed 0. **PASS is functional only**, not the floor B4 criterion.

| Step | Target L/R (mm/s) | Speed second half L/R (mm/s) | Duty second half L/R (permille) | Heading change (cdeg) | Largest speed difference from floor B3 |
| --- | --- | --- | --- | --- | --- |
| rest | 0/0 | 0/0 | 0/0 | 0 | — |
| fwd 150 | +150/+150 | +150/+150 | +184/+187 | -16 | 0.67% |
| rest | 0/0 | 0/0 | 0/0 | -16 | — |
| rev 150 | -150/-150 | -150/-150 | -184/-189 | +29 | 1.35% |
| rest | 0/0 | 0/0 | 0/0 | +33 | — |
| fwd 300 | +300/+300 | +305/+304 | +376/+373 | +16 | 0.33% |
| rest | 0/0 | 0/0 | 0/0 | -74 | — |
| rev 300 | -300/-300 | -304/-303 | -376/-373 | +24 | 1.30% |
| rest | 0/0 | 0/0 | 0/0 | +70 | — |
| spin L 20 | -20/+20 | -19/+19 | -29/+38 | +6931 | 0% |
| rest | 0/0 | 0/0 | 0/0 | +37 | — |
| spin R 20 | +20/-20 | +19/-19 | +29/-38 | -6889 | 5.00% |
| rest | 0/0 | 0/0 | 0/0 | -37 | — |
| fwd 40 | +40/+40 | +39/+39 | +50/+58 | 0 | 2.63% |
| rest | 0/0 | 0/0 | 0/0 | -4 | — |
| rev 40 | -40/-40 | -39/-39 | -51/-59 | +41 | 2.50% |
| rest | 0/0 | 0/0 | 0/0 | +4 | — |

The last column is `max(abs(lifted - B3) / abs(B3))` over the two wheels,
not error from the target. Rounded integer speeds differ by 1 mm/s at the
slow steps, which can mean 2.6–5%; do not claim all speeds are within 2%.
Spin heading from counts is about 69° lifted versus about 59° on the floor:
lifted wheels have no floor grip losses. Encoder heading in this test is
not physical robot rotation and is not a floor heading pass.

### Cold-power TIMER defect: floor B4 blocked

After a cold VIN power-on, `build/mtk_logs/usb_console.txt` recorded
11165 motion ticks (about 112 s), but TIMER gaps of only 233..235 us and
a maximum tick work time of 3 us. The rest intended for 500 ms took about
21 s, then forward 150 began unexpectedly. The 2400-row buffer filled;
this was not a completed B4 run. The operator stopped floor testing.

The watchdog tick generator was enabled with CYCLES 0, dividing 12 MHz
by 512 instead of 12; TIMER ran about 42.7 times too slowly. Post-flash
watchdog reboot retained boot ROM tick setup and hid the defect.
Application initialization now sets 12 cycles, waits for RUNNING, and
checks the TIMER/kernel ratio before enabling motion. The fixed image
passed S0-U01 after a cold power-on. Test every kernel image after a cold
power-on, not only after a flash. See the
[motor record](../../docs/MOTOR_CONTROLLER.md#cold-power-timer-source).

### B4: floor, cold VIN power-on

Image: committed `mtk3-motion` f0b4194 bench (`make SMP=0 WIFI=none`).
USB was unplugged before VIN power-on, so the start was cold. After the run,
USB was connected with VIN still on, and GP20 printed the RAM log. Log:
`build/mtk_logs/b4_floor_20261106-170154.txt` (ignored, not committed).

Result: list done, 2110 rows, 0 skipped, faults 0x00, 2110 MB rows
received, `tm_usb_dropped` 0. Motion ticks 4862, longest tick 112 us, task
gap 9952..10046 us, 0 late, 0 missed. End heading +216 cdeg.

| Step | Speed L / R, B4 (mm/s) | B3 | Largest speed difference | Heading, B4 (cdeg) | B3 | Heading difference | Duty L / R, B4 (permille) | B3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| fwd 150 | +150 / +150 | +149 / +150 | 1 mm/s (0.7%) | +70 | +45 | 25 | +221 / +218 | +225 / +222 |
| rev 150 | -151 / -150 | -149 / -148 | 2 mm/s (1.4%) | -12 | -133 | **121** | -217 / -214 | -227 / -211 |
| fwd 300 | +305 / +305 | +304 / +304 | 1 mm/s (0.3%) | +58 | +16 | 42 | +405 / +398 | +406 / +402 |
| rev 300 | -307 / -307 | -308 / -306 | 1 mm/s (0.3%) | -33 | -216 | **183** | -402 / -394 | -404 / -388 |
| spin L 20 | -21 / +20 | -19 / +19 | **2 mm/s (10.5%)** | +6120 | +5900 | **220** | -85 / +82 | -97 / +77 |
| spin R 20 | +19 / -20 | +20 / -20 | 1 mm/s (5.0%) | -6058 | -5887 | **171** | +79 / -91 | +93 / -99 |
| fwd 40 | +40 / +40 | +40 / +38 | **2 mm/s (5.3%)** | +20 | +128 | **108** | +91 / +97 | +98 / +95 |
| rev 40 | -40 / -40 | -39 / -40 | 1 mm/s (2.6%) | +12 | -158 | **170** | -86 / -87 | -103 / -92 |

Bold: outside the written limit (speed ±5 %, heading ±100 cdeg of B3).

Assessment, agreed with the operator on 2026-11-06:

- **Functional PASS.** All steps ran, with no faults, skipped rows or late
  ticks. Timing is better than the bare-metal heartbeat.
- The strict limits are **missed** on 2 speed rows and 6 heading rows. The
  misses are not worse driving. On all six straight steps, B4 drifted less
  than B3 (at most 70 cdeg against up to 216). The slow-step misses are
  1–2 mm/s at 20–40 mm/s. Both spins turned about 2° more, in both
  directions.
- B3 is one run, so its own run-to-run spread is unknown. The limits were
  set tighter than that spread. No extra reference run was made.
- Duty matches B3 within normal spread. This confirms that the PWM output
  and the controller are unchanged; floor load sets the duty.
- For later comparisons: speed within ±5 % **or** ±2 mm/s, whichever is
  larger; heading on straight steps at most 100 cdeg worse than the
  reference.

## B4 reference: B3 measured values

Source: saved B3 console summary `build/steer_logs/text_ede2.txt`, boot
`ede2`, 2026-11-04, floor, VIN. This ignored local log is not committed.
Only its measured values are copied here. The design record is
[Motor controller](../../docs/MOTOR_CONTROLLER.md#run-b3-2026-11-04-boot-ede2-rise-limit-8-countstick-per-tick-floor-vin).
The numeric heading below uses the exact console cdeg, not the rounded
values in the design table. Compare `abs(B4 - B3) <= 0.05 * abs(B3)` for
each nonzero mean speed, including the slow spin steps.

| B3 step | Duration | Speed, second half L / R (mm/s) | Heading change (cdeg) |
| --- | --- | --- | --- |
| fwd 150 | 1500 ms | +149 / +150 | +45 |
| rev 150 | 1500 ms | -149 / -148 | -133 |
| fwd 300 | 1000 ms | +304 / +304 | +16 |
| rev 300 | 1000 ms | -308 / -306 | -216 |
| spin L 20 | 3000 ms | -19 / +19 | +5900 |
| spin R 20 | 3000 ms | +20 / -20 | -5887 |
| fwd 40 | 2000 ms | +40 / +38 | +128 |
| rev 40 | 2000 ms | -39 / -40 | -158 |

The list starts with 500 ms rest and has 700 ms rest after each drive
step. B3 completed all 17 steps, faults 0x00, final heading -307 cdeg,
controller hook maximum 85 us, heartbeat interrupt maximum 99 us, no gap
or ISR overruns. Its console summary had 2110 rows; rows 7–15 were lost
in the Wi-Fi text transfer. Do not require B4 to repeat that loss.

## Operator procedure

1. First use `make SMP=0 WIFI=none TEST=1 -j8`. Lift both wheels before
   power is applied. S2-U02 sends a real speed command automatically after
   its warning; no floor run is permitted with this image. The button
   bench task is not started in TEST builds.
2. Record S2-U01 and S2-U02 output. If gap jitter exceeds 1 ms, do not run
   on the floor. Reconsider Option H as described in the motor record.
3. Use the normal single-core radio-off **BENCH=1** image for U03, U04
   and B4. The default image now runs the robot task, not this bench. Clear
   the space in front of and behind the robot. Keep access to GP21.
4. Each GP21 start clears previous latched faults after the 1 s delay.
   A stop/fault does not print or erase the RAM log. Do not reset or power
   off before saving it. GP20 prints status even when there are no rows.
5. Dump output pauses for 20 ms after about eight rows. It ends with total
   `tm_usb_dropped` and the increase during the dump. Boot messages can
   already have increased the total while USB was unplugged; require
   zero increase during the saved dump, not a zero lifetime total.
6. Record the image profile, supply, surface, log, all pass conditions and
   any motion fault. Preserve the distinction between post-flash/lifted
   results and cold-power/floor results. Before floor B4, require S0-U01
   to pass after cold power-on on the fixed image.

## S2-U05 result: 2026-11-06

Image `mtk3-staging-next` dd75b28, `make SMP=0 WIFI=none TEST=1`, wheels
lifted, VIN on and then USB. First boot: S2-U01 PASS (gap 9627..10376 us),
S2-U02 PASS, S3-U01 PASS (1009 samples in 1009603 us, 0 missed), S5-U01 no
object (manual check, not done), then the S2-U05 warning and the reset. The
S0-U01 line of the first boot was lost before USB connected. Second boot
(after the watchdog reset): S0-U01 PASS (TIMER 1009990 us, kernel 1010 ms),
S2-U01 PASS (gap 9361..10643 us), S2-U02 PASS, then
`S2-U05 previous boot: watchdog reset after 99 ms` and PASS, with no second
reset. Result: **PASS**.

## S2-U04 run: 2026-11-06

BENCH image (`make SMP=0 WIFI=none BENCH=1`) from mtk3-staging 8df29d6,
wheels lifted, USB and VIN connected. Logs (ignored, not committed):
`build/mtk_logs/s2u04_*.txt` and `build/mtk_logs/s2u04b_*.txt`.

| Try | ESTOP time | Result |
| --- | --- | --- |
| 1 | 80 ms after the bench itself set rest (duty already 0) | Not valid: no active drive to stop. ESTOP latched (0x04) |
| 2 | During fwd 300: speed 330 mm/s, duty 408/403 in the last row | **PASS**: the operator saw the wheels stop at once and stay stopped for about 3 s. ESTOP latched (0x04). Gap 9961..10031 us, late 0, missed 0 |

The bench stops its RAM log at ESTOP, so the log has no rows after the
stop. The duty 0 and "no later tick restores an old duty" result comes from
the operator observation. Possible improvement: log about 1 s after ESTOP.
