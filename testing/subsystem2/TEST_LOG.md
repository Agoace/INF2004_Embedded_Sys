# Subsystem 2: motion bring-up

Hardware results below were supplied by the operator on 2026-11-06.
A build or a host test is not an on-target result. Keep both wheels lifted
before powering a TEST image. Earlier passes used USB after flashing;
they do not validate a cold power-on. The fixed image still needs
[S0-U01](../subsystem1/TEST_LOG.md), currently **Not run**.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S2-U01 | TEST image: compare motion status before and after 5 s | 500 ± 1 ticks; maximum task gap ≤ 11000 us; no discarded wakes; report `tick_max_us` and min/max gap | PASS: 499 ticks, gap 9900..10093 us, max 105 us, 0 late/missed |
| S2-U02 | TEST image: read the warning first; wheels lifted. After a 2 s delay, send 40/40 mm/s once, then no refresh for `stale_ms + 30` ms | `MC_FAULT_STALE` set; both duty values 0; wheels stop | PASS |
| S2-U03 | Normal bench image, idle, wheels lifted: turn each wheel forward by hand, then press GP20 for status | Left and right counts both increase for forward rotation; pins/signs match the table in TEAM.md; reverse rotation decreases counts | PASS after left wire fault repair; left motor and encoder were dead before repair; bare-metal bench confirmed the wire fault |
| S2-U04 | Normal bench image, wheels lifted: press GP21, wait for a drive step, press GP21 again, then press GP20 | ESTOP remains latched; both motor PWM channels become 0 without waiting for the next 10 ms task tick; later ticks cannot restore an old duty. Confirm physical stop and, where available, PWM with a scope | Not run |
| B4-lifted | USB after flash, wheels lifted, same 17-step list | Functional check: all steps complete, 0 faults/skipped rows, maximum gap ≤11000 us; not a floor speed/heading match | PASS: 2110 rows, 0 skipped/faults, gap 9958..10041 us, max 119 us, 0 late/missed |
| B4 | Floor/VIN run, same 17-step B3 list. Leave USB unplugged during the run. GP21 starts after 1 s; GP21 again stops. After the run, connect USB, open the terminal and press GP20. Save the summary and MB rows | Second-half speed within ±5% of each signed B3 mean below; step heading change within ±100 cdeg of B3; no faults; maximum task gap ≤11000 us. Dump delta of `tm_usb_dropped` 0; no skipped log rows | Not run; blocked by cold-power TIMER defect until S0-U01 passes |

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
checks the TIMER/kernel ratio before enabling motion. The fixed image's
cold-power S0-U01 is **Not run**. Test every kernel image after a cold
power-on, not only after a flash. See the
[motor record](../../docs/MOTOR_CONTROLLER.md#cold-power-timer-source).

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
3. Use the normal single-core radio-off image for U03, U04 and B4. Clear
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
