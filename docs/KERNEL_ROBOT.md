# Kernel robot program

## Scope and safety

The default normal kernel image connects the reference program
[`src/vc_line_test.c`](../src/vc_line_test.c) to the kernel motion task,
paired ADC acquisition and ultrasonic driver. It uses the unchanged
Vehicle Controller shell and LINE, SEARCH, SCAN and DETOUR modules.
No hardware was flashed by this port. S3-U01, S5-U01 and R5 are **Not run**.
Build success does not approve floor driving.

Before power, lift both wheels. First run the TEST image after a cold
power-on; verify S0-U01 and the subsystem tests. TEST includes a real short
S2 motor command. A normal GP21 press starts physical spin calibration,
then waits 1 s and drives for at most 90 s. GP21 during calibration or a
run latches ESTOP. GP20, idle only, dumps RAM; do not reset before saving.
Use VIN with USB disconnected for track work; see
[USB power safety](USB_POWER_SAFETY.md).

## Selection and ownership

```sh
cd build_make
make SMP=0 WIFI=none -j4            # robot, default
make SMP=0 WIFI=none BENCH=1 -j4    # motion bench only
make SMP=0 WIFI=none TEST=1 -j4     # lifted-wheel subsystem tests only
make -j4                           # robot, default SMP/radio profile
```

`APP_BENCH` is part of the build profile and `_bench` image name. Switching
profiles invalidates objects. The robot and bench never share the buttons
or the single `motion_notify_after_tick()` client slot. BENCH leaves S3/S5
hardware idle; TEST initializes acquisition but starts neither button task.
`vehicle_task_start()` runs only after all subsystem start calls and
requires successful motion status and driver initialization.

| Work | Rate / location |
| --- | --- |
| TIMER source | Early `usermain()` tick initialization; independent kernel-time check before motion |
| Encoder/echo edges | Shared GPIO IRQ13, priority 1, physical core 0 |
| ADC means | TIMER alarm 0, IRQ0, priority 2, core 0, 1 ms |
| Motor controller | Kernel cyclic wake, task priority 10, core 0, 10 ms |
| Vehicle shell | After completed motion tick, task priority 12, core 0, 10 ms |
| Buttons | 20 ms time check in the same task; calibration and run keep GP21 active |
| Run rows | 20 ms, fixed 16-byte rows, 4501-row capacity (at least 90 s) |
| Dump | Idle only, pauses after eight rows/events; USB drop total and delta at end |

USB IRQ5 retains priority 0; SysTick/PendSV priority 3. No kernel tick,
PWM physical timer, SDK alarm pool or SDK IRQ registration is used for ADC.
Alarm targets advance by 1000 us; late periods count as missed, not replayed.
An already-passed hardware target forces an IRQ so acquisition cannot stall
until TIMER wrap. ADC failure has a bounded ISR wait and increments missed.
The 128-slot line queue has 127 usable entries. Its producer is the core-0
ISR, consumer the core-0 vehicle task; DI/EI only copies one sample or stats.
The task drains every timestamped 1 ms sample, not just the last one.
New dropped/missed line samples or queued/lost motion wakes stop an active
calibration/run. No task function returns.

## Reference match

[`vehicle_robot_config.h`](../app_program/vehicle/vehicle_robot_config.h)
is extracted from the reference configuration block, not retyped. The
host contract test compares that block exactly (only NULL is spelled 0).
It keeps line base 146, gain 85, D85, robot motor KI128, SEARCH 131 mm/s,
SCAN stop 300 ms, detour straight/turn 131, derived sweep 19 mm/s, pause
250 ms, fixed back-off 25 degrees, vehicle/seen 400 mm and all other fields.
Pure module source and motor tuning remain unchanged.

Calibration keeps the reference quarter-wheel goals, independent wheel
stops, 131 mm/s speed, 6 s limit, 200 ms pauses, range learning, return-home
and centre peak/drop stopping. Eight ADC reads per channel are averaged
with integer division, L GP27/ADC1 then C GP26/ADC0; right is always zero.
LP pair mode and all line filtering stay in the unchanged pure module.
ADC reset is released by the app. CLK_ADC enable/source/divider are checked
against the kernel's locked 48 MHz PLL_USB source. The template ADC device
`adca` stays unopened; GP29 is never selected. No template configuration,
kernel or device source is changed.

The ultrasonic phase machine, result conversion and fault streak are
ported from `src/ultrasonic.c`: GP3 trigger at least 10 us; GP2 edges use
TIMER timestamps; rise timeout 2 ms, echo timeout 3 ms, minimum trigger
interval 60 ms. The callback rejects late rises even before task service,
so the 10 ms task interval cannot turn a timed-out ping into a valid echo. No echo is unknown, not clear. The vehicle retains the
module-requested ping interval and never sends a new ping while an echo
waits for consumption. A STUCK_HIGH result is consumed in the same step
as its ping, in reference order. Servo commands go only through motion.

### Intentional differences and limits

- GP20 dumps instead of cycling steering D; D stays at the approved 85.
- No Wi-Fi run transfer, live print, automatic replay or calibration print.
  RAM rows and explicit outcome/state/fault events replace them.
- The kernel vehicle runs **after** motion, rather than reference heartbeat
  phase 9 before motion. A command normally takes effect on the next
  10 ms motor tick. Sampling timestamps are preserved; decisions/calibration
  wheel stops still run on 10 ms boundaries. Hardware must validate this
  phase change, jitter and stopping distance on R5; do not claim identical
  interrupt latency or track behaviour from host tests.
- The bare-metal heartbeat hardware watchdog is not ported here. The
  existing motion stale-command stop remains; it is not a guarantee against
  a stalled CPU. No SDK watchdog/clock/time runtime is linked.
- Extra GP21 safety handling during calibration and lost-sample stop checks
  are deliberate safety additions. All hardware results for this robot
  image remain pending.

## KR1 rows and VE events

`KR index hex` contains 32 hex digits (16 bytes). Bit numbering is
little-endian, least significant bit first. All fields use explicit masks
and shifts, no C bit-fields. Signed fields use two's complement.

| Bits | Value |
| --- | --- |
| 0..15 | Elapsed time in 20 ms units (round down) |
| 16..17 / 18..20 | VSM state / module ID |
| 21..32 / 33..44 | L / C raw ADC means, 12 bits each |
| 45..52 | Signed line position |
| 53..62 / 63..72 | Asked L/R mm/s, signed 10 bits each |
| 73..82 / 83..92 | Measured L/R mm/s, signed 10 bits each |
| 93..108 | Complete 16-bit ultrasonic word: distance/status/streak |
| 109..115 | Signed servo degrees |
| 116..123 | Event bits: LOST=1, FOUND=2, echo=4, ping=8; last outcome type in bits 4..6; motor fault=128 |
| 124..127 | Flags: bit 0 means a speed saturated to -512..511; other bits zero |

Each row holds events since the preceding row. The separate VE list saves
every non-RUNNING module outcome, including intermediate same-tick handoffs
that a single row cannot represent. It has 512 fixed 12-byte entries.
`VE time_ms kind from to word` uses full millisecond times. Kinds:
1 = module outcome (from=module, word=packed outcome), 2 = state/module
change (from/to modules, word low byte=state, high byte=fault),
3 = motor faults, 4 = end of shell run (word=fault detail),
5 = end of calibration/run session (word=cancelled flag). The initial
kind-2 entry is NONE to LINE, state RUNNING. Overflow counts lost events
and stops the session. Rows are not a rolling log: a new GP21 clears the saved
session; a stop preserves it. Row times begin at shell start, so calibration
is not part of the 90 s row budget. Calibration failures keep event/status
information but have no driving rows.

```sh
python3 scripts/robot_log_decode.py capture.txt > rows.csv
python3 scripts/robot_log_decode.py --events capture.txt > events.csv
```

The CSV decoder preserves unknown echo status and leaves unknown distance
blank. A code 0 VALID echo is TOO_NEAR, not an exact 0 mm range. Inspect the
final `tm_usb_dropped` delta and row indexes for transfer loss.

## Checks and open hardware work

Host doubles check queue timing/overflow, ultrasonic edges/timeouts/faults,
exclusive bench selection, calibration/start/ESTOP and no run printing.
A C-to-Python row test checks packing, signed fields and saturation. The
reference settings contract and existing module tests protect pure logic.
These tests are not sensor, scheduling or physical stop measurements.

- [S3-U01](../testing/subsystem3/TEST_LOG.md): paired ADC rate/range/spread.
- [S5-U01](../testing/subsystem5/TEST_LOG.md): 20 fixed-object pings.
- [R5 integration procedure](../testing/integration/KERNEL_ROBOT.md): track
  comparison with [R4](VC_DETOUR_RUN.md), still **Not run**.

Final build sizes, check counts and branch revisions are recorded below
when checks complete. RAM includes 72016 row bytes and 6144 event bytes;
the bench buffer is absent from a robot image. Dynamic kernel/task stack
usage and hardware ISR timing must still be measured on target.
