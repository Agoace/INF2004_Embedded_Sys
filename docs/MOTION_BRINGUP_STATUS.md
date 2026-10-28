# Motion bring-up status (Buddy 2)

Status: Current at 2026-10-28. Branch `joseph`.

This record gives the state of the motor and motion tests on the real robot, the measured values, and the open items. Read it before you continue the motion work.

## Ownership

- Buddy 2 (this branch) owns motion: `src/motion/`, the motor test programs, and the values in `src/motion/motion_config.h`.
- Buddy 5 owns the Vehicle Controller (mission state machine). Do not write a Vehicle Controller in this branch.
- The team statechart sends motion commands through `motion_command()`, `motion_set_steer()` and `motion_set_slow()`. See [Motion control](MOTION_CONTROL.md).

## Test programs

| Target | File | Purpose | Buttons |
| --- | --- | --- | --- |
| `max_speed` | `src/max_speed.c` | Open loop. 3 s at 100% duty. Measured about 17 cm/s. | GP20 start, GP21 emergency stop |
| `fixed_speed` | `src/fixed_speed.c` | Open loop. 5 s at a fixed duty with one trim for each motor. | GP20 start, GP21 emergency stop |
| `motion_sequence` | `src/motion_sequence.c` | Closed loop. Runs a list of moves. After each move, it corrects the distance and the heading. | GP21 start/stop, GP20 emergency stop |
| `motion_bench` | `src/motion_bench.c` | USB console for PID, accuracy, stop and stall tests. | GP20 emergency stop |

The buttons of `max_speed` and `fixed_speed` are the opposite of the team pin table. `motion_sequence` and `motion_bench` use the team pin table (GP20 = ESTOP).

## Measured values and results

| Item | Value | Source |
| --- | --- | --- |
| Encoder counts per wheel revolution | 2535 | [Motor rotation drift](MOTOR_ROTATION_DRIFT.md) |
| Wheel diameter | 64.5–65 mm. The code uses 65 (integer). | Ruler |
| Tyre width | 14 mm | Ruler |
| Chassis width | 91 mm | Ruler |
| Track width (`MOTION_WHEELBASE_MM`) | **Not measured.** Placeholder 120 mm. | — |
| `fixed_speed` straight run | 50% duty, left trim 0, right trim 1%. The robot goes straight. | Floor test |
| `motion_sequence` turns | The robot turns a little too far. It returns to a straight line, because the left and right errors cancel. | Floor test |

## Open items

1. **Track width.** The turns are too large, so the true track width is less than 120 mm. The chassis and tyre widths suggest about 105 mm. To tune it:
   - Set the `motion_sequence` table to one `{ STEP_LEFT, 360 }` line and `MAX_CORRECTIONS` to 0.
   - Run it. Then set the new width to 120 × (angle turned ÷ 360).
2. **Turn check cannot see a track-width error.** The heading correction in `motion_sequence` uses the encoder heading, which uses the same track width, so it reports a wrong turn as correct. When the IMU is ready, change `measured_heading_cdeg()` in `src/motion_sequence.c` to return the IMU heading in 0.01 degree, with left positive. No other change is necessary.
3. **PID gains.** `MOTION_KP_Q8`, `MOTION_KI_Q8` and `MOTION_KSYNC_Q8` are starting values. Tune them with the `motion_bench` step logs (`l`, `v`, `g`).
4. **Integration.** Wait for the line, IMU, ultrasonic and Vehicle Controller code from the other buddies.

## Speed control summary

- `fixed_speed` is open loop. It does not read the encoders while it drives.
- The motion library is closed loop. Every 20 ms, `wheel_control()` in `src/motion/motion_control.c` measures each wheel speed and sets the duty. It uses a feed-forward, a proportional term and an integral term.
- During straight moves and rotations, a sync term equalises the travel of the two wheels. This replaces the open-loop trim.
- The robot slows near the end of each move to decrease coasting.

## Build and flash

The build uses the Pico SDK 1.5.1 installation. In Git Bash:

```sh
C="/c/Program Files/Raspberry Pi/Pico SDK v1.5.1"
export PATH="$C/cmake/bin:$C/ninja:$C/gcc-arm-none-eabi/bin:$PATH"
cmake -S . -B build -DPICO_BOARD=pico_w      # once, or after you delete build/
cmake --build build --target motion_sequence # or fixed_speed, motion_bench
```

To flash:

1. Hold BOOTSEL and connect the USB cable.
2. Copy `build/<target>.uf2` to the `RPI-RP2` drive.

Supply the motor power from VIN, not from USB. See [USB power safety](USB_POWER_SAFETY.md).
