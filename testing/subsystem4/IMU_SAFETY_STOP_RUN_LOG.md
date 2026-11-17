# IMU safety stop hardware run log, session 2

## What this is

This log records the hardware run of the repaired IMU safety stop path. The
repairs are in commit `48b5f9b` and are described in
[IMU_SAFETY_STOP_PATH_REPAIR.md](../../docs/s4-imu/IMU_SAFETY_STOP_PATH_REPAIR.md).

Session 1 is [IMU_SAFETY_RUN_LOG.md](IMU_SAFETY_RUN_LOG.md). Session 1 proved
the cold-boot fix and a partial calibration spin. It could not test the stop
path because the base branch had the defects that RT-0092, RT-0093 and
RT-0094 found.

The goal of this session is the gate that session 1 could not reach:

| Gate | Meaning |
| --- | --- |
| Cold boot latches no fault | The repaired image must boot IDLE with no IMU stop |
| Calibration spin does not false stop | Wheels turning on a stand must not latch |
| Lift produces STOP | The robot stops on a real lift and stays stopped |
| Overturn produces STOP | The robot stops on a real overturn and stays stopped |
| GP21-only recovery | Two presses: the first requests the clear, the second starts |
| Upright gate refuses a start | A tilted or inverted boot refuses every start |

## Environment

| Item | Value |
| --- | --- |
| Image | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| Image size | 860160 bytes |
| Image SHA-256 | `75B0F16A9BFA635DB5ECF91176A39EDCD534200086FDDDB97F31A5046C5FE854` |
| Build options | `SMP=0 WIFI=cyw43 NET=lwip MQTT=1 IMU_HUMP_SOURCE=1 JUNCTION_TEST_CMD=NONE` |
| Firmware commit | `48b5f9b` |
| Host tests at that commit | 370 tests, 3 pre-existing environment failures |
| Broker | `192.168.72.229:1883`, robot configuration, no persistence |
| Hotspot | `space8`, WPA2-Personal |
| Robot address | `192.168.72.191` |
| Console | USB CDC, 115200, 8N1 |

## Step 1: flash the repaired image

The operator put the robot in BOOTSEL. The `RPI-RP2` volume appeared as drive
`E:`. The file copy replaced the image and the volume then disappeared, so the
device rebooted into the new image.

| Check | Observation | Result |
| --- | --- | --- |
| BOOTSEL volume present | `E:` with label `RPI-RP2`, FAT | RECORDED |
| Image hash before the copy | `75B0F16A...C5FE854` | RECORDED |
| Volume after the copy | gone; the device rebooted | RECORDED |
| USB ports after the reboot | `COM4` and `COM6` enumerated | RECORDED |
| New operator messages in the linked ELF | `IMU safety clear requested`, `press GP21 again`, `IMU safety start refused` all present | PASS |

The last row checks the bytes in the linked image, not the source. A message
can be compiled out of one profile and still pass a host test.

## Step 2: cold boot with no fault

The recorder subscribed to the broker. The claim below is bounded to one
observation window, not to the growing total: **93 `robot/state` rows and 18
`robot/heartbeat` rows, covering robot uptime 78.2 s to 170.4 s**, which is a
92 s stationary period. The recorder kept running after that window.

| Check | Observation | Result |
| --- | --- | --- |
| Robot reached the broker | `192.168.72.191` connected, `robot/state` about every 1 s | PASS |
| `boot_id` | `4211540474` on every heartbeat row, one boot only | RECORDED |
| `state` | `0` (IDLE) on all 93 state rows in the window | PASS |
| `module` | `0` (none) on all 93 state rows in the window | PASS |
| `motion_faults` | `0` on all 93 state rows in the window | PASS |
| `active` | `0` on all 93 state rows in the window | PASS |
| Stationary duration | 92 s with no spurious stop or fault | PASS |
| Freshness | `motion_age_ms` 4 to 6, `vehicle_age_ms` 3 to 5 | PASS |
| IMU safety event | 0 event rows of any kind | PASS |
| `stack_min_free` | 476 bytes on every heartbeat row | PASS, above the 256 B gate |
| `cpu_load_permille` | 534 to 539 | RECORDED |
| `wifi_up` | `1` | PASS |

This is the repair target of RT-0094/F2 in the harmless direction: the IMU
service has not published a board on the first Vehicle wakes, and the repaired
image must not latch that as a fault. It did not latch, and the robot stayed
IDLE.

`stack_min_free` is 476 B here against 500 to 532 B in session 1. The repaired
path adds locals and one more board read. The figure is still above the 256 B
gate, but the IMU-specific high-water gate is still NOT RUN.

## Step 3: physical actions by the operator

The operator performed these actions while the recorder and the USB console
capture were running:

1. A quick short lift.
2. A slow lift of about 3 s.
3. An overturn to +X for about 2 s, then to -X for about 2 s.
4. A GP21 press to clear, then a second GP21 press to start.

The actions were not instrumented. They are the operator's description, not a
measured sequence.

## Step 4: overturn produces STOP

PASS. The USB console captured:

```
[robot] IMU safety IMU overturn detected g_mg 979 tilt_ddeg 757 vertical_mg -7 clear_ms 0 age_ms 9
```

| Check | Observation | Result |
| --- | --- | --- |
| Reason | `IMU overturn detected` | PASS |
| Tilt at the latch | 757 deci-degrees, which is 75.7 deg, just over the 75 deg limit | PASS |
| Gravity magnitude at the latch | 979 mg, inside the near-1 g band | RECORDED |
| Vertical change | -7 mg, so the trigger was the tilt, not a vertical rule | PASS |
| Sample age | 9 ms, inside the 50 ms limit | PASS |
| Shell FAULT published | `state = 2` on 29 state rows, uptime 279765 to 307765 ms | PASS |
| Motor E-Stop | `motion_faults = 4` (`MC_FAULT_ESTOP`) throughout the FAULT window | PASS |
| Event record | `robot/event` kind 3, word 1 (`IMU_SAFETY_OVERTURN`) | PASS |

The latch held for about 28 s until the operator cleared it. The robot did not
resume on its own.

This is the first measured evidence for the overturn bound. 757 deci-degrees
confirms that the 75 deg limit is reached by a real hand overturn. The value is
only 7 deci-degrees above the limit, so the bound is close to the real motion
and is not a wide-margin threshold.

The second overturn (-X) produced no separate line. That is correct: the latch
was already set, so the detector does not report a second reason until a safe
restart.

## Step 5: lift produces STOP

NOT DETECTED. Both lifts produced no console line and no event. The robot did
not latch, so `state` stayed IDLE and `motion_faults` stayed 0 through both
lifts.

This is a real result, not a logging loss. The console capture was active, the
overturn line in step 4 proves the capture works, and the event sequence has no
gap.

Cause, from the code: the lift path needs one of two conditions.

- Free fall: `|a| < 400 mg` for 2 consecutive observations. A hand that holds
  the robot keeps it near 1 g, so this path needs a real release.
- Vertical change: `|(a - gravity) . gravity_unit| > 500 mg` for 3
  consecutive observations, and `|a|` must be more than 200 mg away from 1 g.
  A gentle hand lift stays inside the near-1 g band, so the vertical rule is
  separated out; a lift that leaves the band still needs about 5 m/s2 of
  sustained upward acceleration for 30 ms to pass the 500 mg rule.

A slow lift of 3 s cannot satisfy either condition. A quick short lift by hand
usually cannot either, because the hand supports the robot and the
acceleration is small.

The lift stop therefore is NOT achieved with the current candidate bounds on a
hand lift. RT-0091 already asked for recorded lift traces before these bounds
become device thresholds. This step shows why that is necessary.

## Step 6: GP21-only recovery

PASS for the two-press design.

| Check | Observation | Result |
| --- | --- | --- |
| First press requests the clear | console `IMU safety clear requested, hold upright; press GP21 again` | PASS |
| First press does not start | no motor command in the interval; `active` stayed 0 | PASS |
| Refusal event on the first press | `robot/event` kind 3, word 1 (the latched reason) at uptime 308411 ms | PASS |
| Latch released only after the writer | Last FAULT sample 307765 ms, first IDLE sample 308765 ms, request at 308411 ms is between them, and the refusal event has `from = 2`, so the shell was still FAULT at the request | PASS |
| Second press starts | `active = 1` at uptime 311885 ms, `motion_faults` 4 to 0 | PASS |
| Calibration ran | `[BC_CAL] gen 1 samples 2100 ... mode ABORTED`; expected with no line under the wheels | RECORDED |

The motor E-Stop stayed set from the latch until the start cleared it. No motor
command was possible between the clear and the start.

## Step 7: upright gate refuses a start

NOT RUN. The operator did not boot the robot tilted or inverted.

## Step 8: final capture window and session end

The operator stopped the MQTT recorder and the USB console capture when the
robot was disconnected from USB and from the hotspot. No further evidence can
be added to this session.

Final captured totals, from the whole `repair-boot-20261117-1502.csv` file:

| Check | Observation | Result |
| --- | --- | --- |
| Total rows | 792 | RECORDED |
| `robot/state` rows | 653 | RECORDED |
| Uptime window | 78.2 s to 712.6 s, about 11 minutes, one boot | RECORDED |
| `boot_id` | `4211540474` on every row that carries one | PASS: no reboot |
| `state` values | 0 (IDLE) and 2 (FAULT) only | PASS: no RUNNING state without a start |
| `motion_faults` values | 0 and 4 (`MC_FAULT_ESTOP`) only | PASS: no other fault bit |
| `stack_min_free` | 476 bytes on every heartbeat | PASS, above the 256 B gate |
| `cpu_load_permille` | 526 to 611 | RECORDED; the maximum is at the calibration attempt |
| Console capture | 438 bytes: one overturn line, one clear-request line, one aborted calibration line, one Wi-Fi window line | RECORDED |

The `cpu_load_permille` maximum of 611 is higher than the 539 measured at
idle. It occurs at the calibration attempt, when the motors and the Wi-Fi
window run together, so it is not an IMU safety cost.

## What this session adds to the candidate bounds

| Bound | Candidate value | Measured this session |
| --- | --- | --- |
| Overturn tilt | more than 75 deg for 5 observations | 75.7 deg at the latch, 979 mg, 9 ms old |
| Lift free fall | below 400 mg for 2 observations | no measurement; the operator did not release the robot |
| Lift vertical change | more than 500 mg for 3 observations, away from 1 g | no measurement; the hand lift stayed near 1 g |

The next step for the lift stop is a recorded raw trace during a real lift, not
another threshold guess. Use the procedure in
[IMU_SAFETY_TRACE_PROCEDURE.md](../../docs/s4-imu/IMU_SAFETY_TRACE_PROCEDURE.md)
with the IMU stream image, then replay the capture through the analyser.

## Not claimed

- The lift stop is NOT achieved with the current candidate bounds on a hand
  lift. No lift trigger was recorded in this session.
- Stop latency, FOLLOW, the upright-gate refusal and the IMU-specific stack
  high-water are still NOT RUN.
- The overturn bound has one measured latch. One sample is not a distribution
  and does not prove a margin.
- The lifted-action sequence is the operator's description. No instrumented
  timeline of the hand motion exists.
- Hump accuracy and the H3 comparison are not part of this log.

## Recorded evidence files

All under the ignored 	esting/logs/ directory:

- 
epair-boot-20261117-1502.csv - MQTT recorder output, boot_id 4211540474.
- console-session2.txt - USB console capture with the overturn line, the
  clear-request line and the calibration lines.

Analysis must filter on oot_id 4211540474 for this session. Session 1
evidence uses different boot IDs and must not be combined with this one.
