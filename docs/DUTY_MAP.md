# Duty map (loaded, floor)

Maps open-loop motor duty (%) to steady wheel speed (mm/s) on the floor,
with the robot's weight on the wheels. Use: move the locked duty-based
settings (line steering, detour) to wheel speeds for the
[motor controller](MOTOR_CONTROLLER.md), one to one, so the first track
run compares with run P4 ([two-sensor line tracing](TWO_SENSOR_LINE_TRACING.md)).
Branch: `yx-line-steer`.

- Bench target: `duty_map_bench` ([`src/duty_map_bench.c`](../src/duty_map_bench.c))
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

Not run yet.
