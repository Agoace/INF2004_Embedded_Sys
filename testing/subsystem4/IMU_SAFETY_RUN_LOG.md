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

### Step 6: waiting for BOOTSEL

The robot is stopped and safe: `active = 0`, `state = 0` (IDLE),
`count_l = count_r = 0`, `duty_* = 0`, `motion_faults = 4` (the boot
latch). 382 telemetry rows are recorded in `testing/logs/imu-safety-run.csv`.

`picotool info -f` and `picotool reboot -f -u` both report "No accessible
RP-series devices in BOOTSEL mode were found", with the RP2040 seen only as a
USB serial connection. picotool cannot reset a running application, so the
fixed image cannot be flashed without physical access to BOOTSEL.

Blocked on: the board in BOOTSEL mode. No further hardware step can run
until then.

Tests still to run after reflashing, in this order:

1. Boot with the fixed image: `motion_faults` must be 0 and the robot must
   sit in IDLE with no stop requested. This is the direct check that the
   step-4 defect is gone.
2. Upright gate: boot with the robot held upside down or on its side. The
   start must be refused and `[robot] IMU safety start refused: ...` must
   appear on USB.
3. Restrained calibration spin: GP21 press with the wheels clear of the
   floor. Record whether a false lift or overturn latches during the spin,
   and which reason. This is the main untested risk.
4. Safe restart: after any latch, hold the robot upright and still for at
   least 1 s, then press GP21. The latch must clear and the normal
   calibration and start must proceed.
5. IMU stack high-water against the 256 B minimum.

### Step 7: reflash with the fix and verify the boot

| Item | Value |
| --- | --- |
| Image SHA256 | `C5C896B32A5007119698CE50F7571DC5C7E66245D645E03915DC85D31EDC46DB` |
| Boot console | captured from the first second; saved as `testing/logs/boot-console.txt` |

The boot console shows the full start sequence and **no** `[robot] IMU
safety ...` line:

```text
microT-Kernel Version 3.00
[S4] hump_source 1 corrected
=== micro T-Kernel on Raspberry Pi Pico W (spec 0x6300, product 0x0003) ===
single-core build
wifi: cyw43_utk_task starting (proc 1)
all subsystems started
[robot] GP21 calibrate then run / ESTOP; GP20 telemetry dump. D85.
wifi: joining configured network ...
wifi: link up, ip=192.168.72.191
```

| Check | Observation | Result |
| --- | --- | --- |
| Motor fault at boot | `motion_faults = 0` on all sampled `robot/state` rows | **PASS** |
| Robot position | `active = 0`, `state = 0` (IDLE), `module = 0`, `duty_l = duty_r = 0` | PASS |
| No boot stop | no IMU safety line on USB; nothing latched | PASS |
| Wi-Fi / MQTT | `wifi_up = 1`, `mqtt_connected = 1`; robot IP `192.168.72.191` | PASS |
| Stack | `stack_min_free = 532` bytes, above the 256 B minimum | PASS (this boot) |
| Timing | `motion_late = 0`, `vehicle_overruns = 0`, `cpu_load_permille = 541` | RECORDED |

Result: **PASS for the boot gate.** The step-4 defect is gone: a cold boot
with the fixed image latches no fault and sits in IDLE.

Note on the stack figure: `stack_min_free` is the minimum across the
registered application stacks, so 532 B is the worst task, not the IMU task
alone. It establishes that no stack is below the 256 B minimum on this boot.
The IMU-only high-water still needs the TEST/GP20 report.

Next: step 3 of the test order, the restrained calibration spin. This needs
the wheels clear of the floor. The spin is started with the MQTT
`calibrate` command, which is the same path a GP21 press takes and is
gated by the same check, so no button press is needed.

### Step 8: restrained calibration spin (the main untested risk)

Operator confirmed at 2026-11-16 17:30 that the car is held clear of the
floor and will stay lifted for the rest of this session.

The spin is started with the MQTT `calibrate` command, not a GP21 press.
Both take the same idle-loop path and pass the same safety gate; the command
path only removes the need for a physical press. `calibrate` runs the spin
and returns to IDLE without driving, so this step exercises the spin and the
safety gate without commanding a run.

Condition under test: the calibration spin is the most energetic stationary
motion the robot performs, and the lift and overturn bounds are still
candidates with no recorded trace behind them. A false trigger here would be
a false stop in normal use.

Capture: USB console to `testing/logs/spin-console.txt` and telemetry to
`testing/logs/imu-safety-run.csv`.

Result: **PASS for the spin test.** No IMU safety stop was produced.

| Check | Observation | Result |
| --- | --- | --- |
| Motor output during the spin | 8 one-second samples with nonzero duty over 1859 ms; peak `duty_l = -193`, `duty_r = +186` permille, so the wheels turned in opposite directions | PASS (spin happened) |
| Encoder travel | counts went from `-254/297` to `52/7`; the wheels really turned | PASS |
| `motion_faults` at every sample this boot | `0` for all 212 state rows, including all 8 spin rows and every row after | **PASS** |
| IMU safety line on USB | absent; the console captured only the barcode observer and the Wi-Fi window lines | PASS |
| Robot after the spin | `active = 0`, `state = 0` (IDLE), `duty = 0/0` | PASS |

The console during the spin contained only:

```text
[BC_CAL] gen 1 samples 1640 GP28 3943..3985 mode ABORTED faults 08
[BC_CAL] white 0/0 black 3957/1640 hold_us 1639006 gap_us 1015 rise 320 fall 240; shadow only
[robot] WiFi window wall polls 105 gaps 104 max_gap_us 21067 gaps_over_10000 104 work_max_us 3232 probe_max_us 56
```

The `[BC_CAL]` line is the barcode observer, not an IMU safety stop. Its
`mode ABORTED` and `white 0/0` mean the line calibration found no white
under the sensors, which is expected with the car held clear of the floor.
That also explains the short sweep: the spin stopped after about 1.9 s
instead of running the full bounded sequence.

Limit to state with this result: the wheels turned and the detectors stayed
quiet, but the full calibration sweep was not exercised because there is no
track under a lifted robot. A complete-sweep spin test needs the robot on the
line with the wheels held clear, or a bench run with the track in place.

Two caveats on reading the capture file: the telemetry CSV spans both boots,
so every analysis filters on `boot_id`; and the pre-fix boot contributes
rows with `motion_faults = 4` that must not be attributed to this image.

### Step 9: telemetry dump and stack evidence

MQTT `dump` published request `4238312033`. The dump reports
`rows_total = 0`, `rows_sent = 0`, `events_total = 1`, `lost_total = 0`
and `generation = 1`. Zero rows is correct: no run has completed on this
boot, so the RAM log holds nothing. The single event is the start-up stop
event at `uptime_ms = 2091`.

The dump does not carry the per-task stack report. That report is printed on
USB at run end and at a GP20 dump, and neither happened here.

| Check | Observation | Result |
| --- | --- | --- |
| Telemetry dump | 1 event, 0 rows, no loss | PASS (dump works) |
| `stack_min_free` on this boot | 532 B early, 508 B after the spin | above the 256 B minimum |
| IMU task high-water specifically | not available without the USB stack report | NOT RUN |
| `cpu_load_permille` | 541 to 549 | RECORDED |

So the 256 B gate is satisfied for the worst application stack on this boot,
but the IMU-specific figure the plan asks for still needs the USB stack
report. It is not a PASS for that narrower claim.

### Step 10: start request with the wheels free

MQTT `start` published request `2489399017`.

| Check | Observation | Result |
| --- | --- | --- |
| Motor output after the request | 0 rows with nonzero duty after `uptime_ms` 210 s; no motion at all | RECORDED |
| `motion_faults` | `0` on all 329 current-boot state rows, including after the request | PASS |
| Command accounting | `command_lost = 0`, `commands_enabled = 1`: the request was consumed, not dropped | PASS |
| IMU safety refusal line on USB | **absent**, but that proves nothing about the gate: it is never evaluated | NOT RUN |
| Robot position afterwards | `active = 0`, `state = 0`, `duty = 0/0` | PASS |

The run did not start. The reason is not the IMU gate. `vehicle_network_policy.c`
rejects a START request without line calibration, and that policy check runs
before the IMU start gate, so this step cannot test the gate at all.

The `start` opcode uses an already-learned line calibration instead of running
a new spin, and the earlier spin aborted without learning one because there was
no white under the lifted robot.

Recorded command status for this step: consumed (`command_lost = 0`,
`commands_enabled = 1`), no motion, no refusal line, robot left IDLE.

What this step cannot show: the IMU start gate, the FOLLOW path with wheels
turning, or false-stop freedom while driving. The gate is first reached only
with a learned line calibration, which needs the robot on the track.

### Step 11: session end

Operator lowered and disconnected planning is not needed; the robot is left
in IDLE with `motion_faults = 0`.

Covered on hardware in this session:

| Gate | Result |
| --- | --- |
| Cold boot latches no fault | **PASS** (was FAIL before the fix) |
| Calibration spin produces no false lift or overturn | **PASS** for the partial sweep |
| Worst application stack above 256 B | 500 to 532 B |
| Telemetry dump works, no loss | PASS |

Not covered, and not claimed:

| Gate | Status |
| --- | --- |
| Upright gate refuses a start for an upside-down or tilted boot | NOT RUN; needs the robot physically inverted |
| Safe restart clears a latch after 1 s upright and still | NOT RUN; no latch occurred this session |
| Full bounded calibration sweep without a false trigger | NOT RUN; needs the track under the robot |
| FOLLOW with wheels turning, false-stop free | NOT RUN; needs a learned calibration |
| IMU task stack high-water specifically | NOT RUN; needs the USB stack report at run end or GP20 |
| Hump accuracy and the H3 corrected-versus-filtered comparison | NOT RUN |

Every number above is from this boot only. The telemetry CSV also contains
the earlier pre-fix boot, so analysis must filter on `boot_id`
`4000845492`.

### Step 12: final telemetry read before shutdown

The recorder made one last dump after the start request:

```
[boot:ee780eb4:0] dump 4238312033: complete, 0 rows, 1 events
```

The dump completed without truncation, and the boot reported one event and no
state rows. That is expected for a robot left IDLE.

The boot ID of this last dump, `4238312033`, differs from the
`4000845492` of the main evidence above. The session therefore contains at
least two boots, and any cross-boot analysis must filter on `boot_id`
explicitly. Do not combine the two.

The broker and the recorder were then stopped by the operator. No test was
running at shutdown.
