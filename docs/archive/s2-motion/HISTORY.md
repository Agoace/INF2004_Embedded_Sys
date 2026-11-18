# Motion history

The motion records below were removed from the tree on 2026-11-16.
The current controller, hardware, calibration and results are in
[MOTION.md](../../s2-motion/MOTION.md). Each removed file is still in Git,
unchanged.

Read one file:

```sh
git show 569672a:docs/archive/s2-motion/MOTOR_CONTROLLER.md
```

Commit `569672a` is on `origin/mtk3-staging`. Use the path from the table.
Relative links inside an old file can point to files that have moved.

## Files

| Path at `569672a` | What it holds | Now in |
| --- | --- | --- |
| `docs/s2-motion/HARDWARE.md` | Pins, geometry, 50-turn encoder ratio, servo and sonar, unloaded speed tables, power status | [MOTION.md](../../s2-motion/MOTION.md#hardware-wiring-and-calibration) (full text) |
| `docs/archive/s2-motion/MOTOR_CONTROLLER.md` | First controller: interface, speed loop, Pico layer, floor bench step 3 (B1/B2/B3 comparisons), ideas, kernel layer | History only |
| `docs/archive/s2-motion/CALIBRATION_RESULTS.md` | 2026-10-30 operator geometry, wiring, GY-511, HC-SR04, old three-sensor IR values, motor sides, servo, ten-cycle drift | History only |
| `docs/archive/s2-motion/MOTOR_ROTATION_DRIFT.md` | 50-turn method, superseded 2540/2539, coast and jog data, speed observations | History only |
| `docs/archive/s2-motion/DUTY_MAP.md` | DM1 method and settle windows | History only; table in [MOTION.md](../../s2-motion/MOTION.md#duty-map) |
| `docs/archive/s2-motion/HEARTBEAT.md` | Bare-metal 1 ms heartbeat, health counters, 2026-11-04 bench | History only; summary in [MOTION.md](../../s2-motion/MOTION.md#heartbeat) |
| `docs/archive/MOTION_TIMING_INVESTIGATION.md` | STEP 8 motion gap investigation: 11000 us gate, dispatch delay 4488 us, DIAG=1 capture | History only |

[USB_POWER_SAFETY.md](USB_POWER_SAFETY.md) stays in the tree: the old
bare-metal firmware in `src/` cites it.
