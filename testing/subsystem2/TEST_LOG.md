# Subsystem 2: motion bring-up

No test below has run on the robot. A build or a host test is not an
on-target result. Keep both wheels lifted before powering a TEST image.

| ID | Check and procedure | Pass condition | Status |
| --- | --- | --- | --- |
| S2-U01 | TEST image: compare motion status before and after 5 s | 500 ± 1 ticks; maximum task gap ≤ 11000 us; no discarded wakes; report `tick_max_us` and min/max gap | Not run |
| S2-U02 | TEST image: read the warning first; wheels lifted. After a 2 s delay, send 40/40 mm/s once, then no refresh for `stale_ms + 30` ms | `MC_FAULT_STALE` set; both duty values 0; wheels stop | Not run |
| S2-U03 | Normal bench image, idle, wheels lifted: turn each wheel forward by hand, then press GP20 for status | Left and right counts both increase for forward rotation; pins/signs match the table in TEAM.md; reverse rotation decreases counts | Not run |
| S2-U04 | Normal bench image, wheels lifted: press GP21, wait for a drive step, press GP21 again, then press GP20 | ESTOP remains latched; both motor PWM channels become 0 without waiting for the next 10 ms task tick; later ticks cannot restore an old duty. Confirm physical stop and, where available, PWM with a scope | Not run |
| B4 | Floor/VIN run, same 17-step B3 list. Leave USB unplugged during the run. GP21 starts after 1 s; GP21 again stops. After the run, connect USB, open the terminal and press GP20. Save the summary and MB rows | Second-half speed within ±5% of each signed B3 mean below; step heading change within ±100 cdeg of B3; no faults; maximum task gap ≤11000 us. Dump delta of `tm_usb_dropped` 0; no skipped log rows | Not run |

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
   any motion fault. All current status cells remain **Not run** until
   the operator supplies real results.
