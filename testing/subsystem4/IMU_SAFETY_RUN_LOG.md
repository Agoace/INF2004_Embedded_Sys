# IMU safety stop: hardware run log

Run started 2026-11-16, Asia/Singapore. Source: branch `mtk3-imu-safety`,
commit recorded in step 1. Plan and acceptance gates:
`IMU_SAFETY_HUMP_PLAN.md` and `IMU_SAFETY_HUMP_CHECKLIST.md` (both still
untracked in the `mtk3-staging` worktree at the time of this run).

This record is written during the run, one step at a time, so that a loss of
network or of the operator's terminal does not lose the evidence. A step is
recorded before the next step starts.

**No result in this file is a PASS unless the step says PASS and shows the
observation that establishes it.** A step without an observation is NOT RUN.

## Safety constraints for this run

- The robot is restrained or its wheels are clear of the floor for every
  step that can move the wheels. There is no physical emergency stop: GP21
  is a control button, so removing motor power is the only hard stop.
- A regulated 5 V supply is on VIN before any motor command. USB alone must
  not drive the motors.
- GP21 starts line calibration and then the run. Expect motor motion.

## Steps

| # | Step | Result |
| --- | --- | --- |
| 1 | Record source revision, image and network | see below |

### Step 1: source, image and network (recorded before flashing)

| Item | Value |
| --- | --- |
| Branch | `mtk3-imu-safety` |
| Commit | `479572f46ec64e77911acbaaf2b51a121dd70c6c` |
| Image | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2` |
| Image bytes | 859136 |
| Image SHA256 | `C1CAC96035A08082249A67EDF3CCAFD4D3285A977CFC87A4D0D277837CB7EDB1` |
| Build flags | `SMP=0 WIFI=cyw43 NET=lwip MQTT=1` (normal robot image) |
| Hotspot | `space8`, WPA2-Personal, 2.4 GHz |
| Laptop (broker) address | `192.168.72.229` |
| Broker | mosquitto 2.1.2, bound to `192.168.72.229:1883` |
| Board USB ID before flash | `VID_2E8A&PID_0003` (BOOTSEL), `RPI-RP2` on `E:` |

Result: RECORDED. The broker address matches the value compiled into the
image, so no rebuild is needed for the network.

Note: the TEST image cannot be used for the safety gates. It contains no
Vehicle task, so it has no safety gate, latch or restart. This run uses the
normal robot image.

### Step 2: broker and recorder

| Check | Observation | Result |
| --- | --- | --- |
| Broker listener | `192.168.72.229:1883` owned by the started process | PASS |
| Publish/subscribe probe | `mosquitto_pub` on `run/probe` returned `ok` to a subscriber on the hotspot address | PASS |

Result: PASS. The broker serves the hotspot address the image targets.

### Step 3: flash and boot

| Check | Observation | Result |
| --- | --- | --- |
| Flash | `RPI-RP2` on `E:`; image copied; board re-enumerated as `MTK3PICO` on `COM6` (`PID_000A`) | PASS |
| Wi-Fi join | `wifi_up = 1`, `connect_attempts = 1` on the first boot | PASS |
| MQTT | `mqtt_connected = 1`; broker logged `inf2004-robot` publishing `robot/state` ~1 Hz and `robot/heartbeat` every 5 s | PASS |
| Recorder | CSV written, 30 rows within the first minutes | PASS |
| USB console | silent when attached after boot; the banner and any boot-time line were missed | INCONCLUSIVE |

### Step 4: DEFECT FOUND - the safety gate faults at every cold boot

| Item | Value |
| --- | --- |
| Observation | `robot/state` on the first boot reports `motion_faults = 4` |
| Decoding | `MC_FAULT_ESTOP` (`motor_ctl.h:59`), a latched fault |
| Robot position | `active = 0`, `state = 0` (IDLE), `count_l = count_r = 0`, `duty_* = 0` |

Result: **FAIL.** The robot latched a motor ESTOP at boot without any operator
input or motion.

Cause, from the code: `imu_safety_check()` runs on the first Vehicle wake,
before the IMU service has published a safety board. `imu_safety_try_read()`
returns `E_OBJ`, `imu_safety_evaluate(NULL, false, ...)` returns
`IMU_UNAVAILABLE`, and `imu_safety_check()` treated that as a confirmed
reason: it latched the fault and called `vehicle_estop()`.

This is my defect, not a pre-existing one. The plan says an unavailable IMU
"blocks start and stops active motion". It does **not** say to latch a
persistent fault; the latch is reserved for a confirmed overturn or lift
("An IMU fault remains latched across run cleanup" refers to that fault).
Latching unavailability makes every cold boot fault, and only a GP21 press
can clear it, so the robot cannot be started at all.

Consequence for this run: the robot is latched and refuses every start. The
one automatic USB report line was printed before the console could be
attached, so it was not captured. Recovering this boot needs a GP21 press;
reflashing needs the board in BOOTSEL.

Next: fix the code so unavailability blocks and stops without latching, add a
regression test for the boot race, rebuild, and reflash.

### Step 5: fix and source-level verification

Change in `vehicle_task.c`: an unavailable IMU no longer latches a fault.
It blocks a start and stops active motion, which is what the plan requires,
and the persistent latch is reserved for a confirmed overturn or lift. A
refused start now prints one bounded USB line, because unavailability no
longer produces a latch report:

`[robot] IMU safety start refused: <reason> g_mg ... tilt_ddeg ... age_ms ...`

Regression test added: `test_vehicle_imu_boot_race` (scenario
`imubootrace`). The safety board fails its first three reads, as at a cold
boot. The test requires no stop request, no latch, and a normal start once
the board appears. Before the fix this scenario refused the start, which is
what the robot did on hardware in step 4.

| Check | Result |
| --- | --- |
| 8 Vehicle scenarios, including the new boot race | PASS |
| Host suite | 367 tests, the same 5 failures as `a03f9e5`; all 8 IMU tests pass |
| Firmware profiles rebuilt | 7 profiles, warning-free |
| New image | `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`, 859136 bytes, SHA256 `C5C896B32A5007119698CE50F7571DC5C7E66245D645E03915DC85D31EDC46DB` |

Not yet verified: the fix on the robot. The running image is the pre-fix
build, and reflashing needs the board in BOOTSEL.
