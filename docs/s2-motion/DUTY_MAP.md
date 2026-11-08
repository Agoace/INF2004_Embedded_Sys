# Duty map (loaded, floor)

Maps open-loop motor duty (%) to steady wheel speed (mm/s) on the floor,
with the robot's weight on the wheels. Use: move the locked duty-based
settings (line steering, detour) to wheel speeds for the
[motor controller](MOTOR_CONTROLLER.md), one to one, so the first track
run compares with run P4 ([two-sensor line tracing](../s3-line-barcode/TWO_SENSOR_LINE_TRACING.md)).
Branch: `yx-line-steer`.

- Bench target: `duty_map_bench` ([`src/duty_map_bench.c`](../../src/duty_map_bench.c))
- Unloaded speeds (wheels lifted, 2026-10-27): [calibration results](CALIBRATION_RESULTS.md#unloaded-motor-speed-at-5-v-vin). Not used here: the load changes the low end and the slope.

## Method

| Part | Rule |
| --- | --- |
| Drive | Both wheels the same duty; forward, then reverse; 23 levels: 5–12 % in 1 % steps, 14–20 % in 2 % steps, 25–50 % in 5 % steps, 60–100 % in 10 % steps |
| Start | From rest: kick 12 % until both wheels have 10 counts (at most 250 ms), then the level. Marked `*` if the kick did not reach 10 counts |
| Settle, measure | 5–20 %: 500 ms + 500 ms; 25–40 %: 400 + 400; 45–50 %: 300 + 400; 60–100 %: 300 + 300 |
| Speed | Encoder counts in the measure window ÷ window time; 12414 counts per metre |
| Rest | 500 ms at 0 between drives |
| Safety | The heartbeat hook is the only PWM writer and stops the wheels if the main loop has not refreshed the command for 100 ms; hardware watchdog 100 ms; stall stop (15 % or more, no count for 500 ms); GP21 stops |
| Log | Summary table and one row per 10 ms (`DM i level phase dL dR`) over USB and Wi-Fi (`text_<boot>.txt`) |
| Space | About 70 cm clear ahead and behind |

## Results

### Run DM1 (2026-11-04, boot 5d51, `7a65c0e`, floor, VIN)

Log `build/steer_logs/text_5d51.txt`. All 23 levels done, no stall;
6524 of 6541 rows received. The summary printed on the robot is wrong
from 80 % (32-bit overflow in the mm/s sum, fixed after the run); the
table below is from the 10 ms rows (mean count per tick in the measure
window × 8.055 mm/s).

| Duty % | Fwd L | Fwd R | Rev L | Rev R | Mean \|speed\| (mm/s) |
| --- | --- | --- | --- | --- | --- |
| 5 | 0 | 0 | 0 | 0 | 0 |
| 6 | 0 | 0 | 0 | 0 | 0 |
| 7 | 0 | 0 | −4 | −1 | 1 |
| 8 | +24 | +24 | −29 | −26 | 26 |
| 9 | +34 | +33 | −39 | −36 | 36 |
| 10 | +43 | +42 | −48 | −45 | 45 |
| 11 | +52 | +51 | −56 | −53 | 53 |
| 12 | +60 | +59 | −65 | −63 | 62 |
| 14 | +78 | +77 | −82 | −80 | 79 |
| 16 | +96 | +95 | −98 | −99 | 97 |
| 18 | +114 | +114 | −116 | −117 | 115 |
| 20 | +129 | +129 | −131 | −134 | 131 |
| 25 | +168 | +171 | −171 | −175 | 171 |
| 30 | +211 | +214 | −214 | −217 | 214 |
| 35 | +253 | +257 | −256 | −262 | 257 |
| 40 | +296 | +299 | −299 | −304 | 300 |
| 45 | +332 | +334 | −335 | −342 | 336 |
| 50 | +376 | +378 | −379 | −380 | 378 |
| 60 | +464 | +463 | −463 | −462 | 463 |
| 70 | +561 | +554 | −548 | −543 | 552 |
| 80 † | +654 | +637 | −617 | −618 | 632 |
| 90 † | +730 | +706 | −690 | −697 | 706 |
| 100 † | +807 | +781 | −759 | −734 | 770 |

† The robot did wheelies at high duty (operator), and at 90–100 % the
speed still rose in the 300 ms window. Do not use these rows for tuning.

| Item | Result |
| --- | --- |
| Running minimum | 8 % (after the start kick). 5–6 %: the wheel stops; 7 %: almost stopped. The 12 % in the start policy is the duty to **start** from rest, not to keep running |
| Shape | Close to a straight line from 8 % to 70 %: about 8.5 mm/s per % (12 % → 62, 25 % → 171, 50 % → 378) |
| Wheels and directions | Within about 3 % of each other up to 70 % (reverse a little faster at the low end) |
| Against the closed-loop bench (B2) | B2 held 150 mm/s at 220 ‰; this map gives 22 % → about 150 mm/s |
| Against the unloaded speeds | Loaded speed is lower and the running minimum is lower than the unloaded start minimum; see [calibration results](CALIBRATION_RESULTS.md#unloaded-motor-speed-at-5-v-vin) |
| Timing | Heartbeat interrupt at most 56 µs; no overruns |
