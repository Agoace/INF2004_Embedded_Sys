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

## Step 3: calibration spin with no false stop

NOT RUN. The operator must press GP21 with the robot on the stand. The spin is
the first motor-enabled movement, so it also proves that the new acknowledged
clear path does not block a normal start that has no latch.

## Step 4: lift produces STOP

NOT RUN. Needs the operator to lift the robot.

What must be recorded:

- The stop is immediate and the wheels do not turn again.
- The published `state` becomes `2` (shell FAULT), not IDLE.
- The run summary, if a run was active, carries `shell_fault = 6`
  (`VSM_FAULT_IMU`) with detail `2` (`IMU_SAFETY_LIFT`).
- The console prints one `[robot] IMU safety IMU lift detected ...` line.
- `g_mg`, `tilt_ddeg`, `vertical_mg` in that line become the first measured
  evidence for the candidate bounds, which RT-0091 asked for.

## Step 5: overturn produces STOP

NOT RUN. Needs the operator to turn the robot on its side.

The expected detail is `1` (`IMU_SAFETY_OVERTURN`), and the console line is
`IMU overturn detected`.

## Step 6: GP21-only recovery

NOT RUN. This is the new two-press behaviour:

1. The first press requests the clear and prints
   `IMU safety clear requested, hold upright; press GP21 again`. It must not
   start the robot and must not command the wheels.
2. The second press starts the run only if the pose was upright, still and
   fresh for the full 1 s window.

A refused or late clear must keep the latch and the FAULT state. This is the
F1 repair in the direction that matters.

## Step 7: upright gate refuses a start

NOT RUN. Needs the robot physically tilted or inverted at boot.

## Not claimed

- No lift, overturn, restart, stop-latency or FOLLOW result exists yet.
- The candidate bounds are still candidates until step 4 and step 5 give
  measured gravity values.
- The IMU-specific stack high-water is still NOT RUN.
- Hump accuracy and the H3 comparison are not part of this log.

## Recorded evidence files

All under the ignored `testing/logs/` directory:

- `repair-boot-20261117-1502.csv` — the step 2 recorder output.

Analysis must filter on `boot_id` `4211540474` for this session. Session 1
evidence uses different boot IDs and must not be combined with this one.
