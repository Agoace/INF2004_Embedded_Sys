# IMU safety latch capture (`IMU_SAFETY_CAPTURE=1`)

## Purpose

This profile records the IMU safety detector input and output at each
observation around the first latch edge. Use it to find which rule fired
for a stop and why. The K10_1 school run (`dd26a38`) stopped a sideways
push-over as LIFT, not OVERTURN. The capture shows if the freefall rule or
the vertical-change rule fired, and how long the fall, bounce and settle
take. These values set X and T for the proposed settled-pose event. A
gentle lift does not latch (K10_1 boot 347926102: no stop; a later, firmer
lift latched). A GP21 stop captures it, to show how far below the
vertical-change rule (500 mg, 3 samples) it stays, and if the raw Z trace
could support a secondary lift cue (the hump estimate read -21.1 mm). No
threshold changes.

The capture does not change the detector, its thresholds, the latch time,
the Vehicle or MQTT. It is a ROBOT image with a real IMU.

## Build

The profile is opt-in. Off (the default) adds no flag, object or name.
The 76 default images are unchanged. It combines with
`BARCODE_FAIL_CAPTURE=1`.

```sh
make -C build_make clean SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 \
  BARCODE_FAIL_CAPTURE=1 IMU_SAFETY_CAPTURE=1
make -C build_make --output-sync=target -j12 SMP=0 WIFI=cyw43 NET=lwip \
  MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 IMU_SAFETY_CAPTURE=1
```

Output: `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_kernel_ram_bcfail2_imucap.uf2`.
The profile ID gets `-imucap`. Without `BARCODE_FAIL_CAPTURE=1` the suffix
is `_kernel_ram_imucap`.

Guards (`tests/test_imu_safety_capture_make.py`): the value must be 0 or
1; it requires `TEST=0 BENCH=0`; it excludes `IMU_SAFETY_BENCH` and
`IMU_STREAM`.

| Image (build at `9dd835d` + capture, no secrets) | text | bss | static RAM | free RAM | RAM_PROOF |
| --- | ---: | ---: | ---: | ---: | --- |
| Demo (no capture) | 447656 | 201476 | 222052 | 48284 | 77/77, 19544 B |
| `IMU_SAFETY_CAPTURE=1` | 449880 | 206636 | 227212 | 43124 | 77/77, 19544 B |
| `BARCODE_FAIL_CAPTURE=1` | 451272 | 218964 | 239884 | 30452 | 77/77, 19772 B |
| Both | 453496 | 224124 | 245044 | 25292 | 77/77, 19772 B |

Free RAM is 270336 B minus the RAM sections of the ELF. The capture ring
is 5156 B (`capture`, 256 samples of 20 B plus the header). The IMU task
stack gets one 16 B wrapper frame on the safety branch. That branch is not
the deepest IMU path.

## Mechanism

[`app_program/imu_safety_capture/`](../../app_program/imu_safety_capture):

- `capture.{h,c}`: pure ring. While ARMED it keeps the newest 256
  samples. Two sources start COLLECTING:
  - LATCH: the first latch edge (latched trigger NONE to any). It keeps
    up to 64 samples before the edge (0.64 s), the edge sample and 191
    more (1.92 s at 100 Hz).
  - ESTOP: a Vehicle ESTOP with no latch (GP21 press while driving, MQTT
    ESTOP, line stop). It keeps up to 192 samples before (1.92 s) and 64
    from the ESTOP (0.64 s). An IMU stop also calls ESTOP, but its latch
    capture is already running and wins.

  Then the ring is FROZEN. A later latch edge only increments `missed`;
  a later ESTOP is ignored. One writer (IMU task), one requester
  (Vehicle task), one reader (telemetry task). The reader reads samples
  only while FROZEN.
- `service_impl.h`: forced include on `imu_service.o`. It replaces the
  `imu_safety_step` call with `isc_safety_step`. The wrapper calls the
  real detector, then stores one sample. It prints nothing.
- `telemetry_impl.h`: forced include on `telemetry.o`. The GP20 dump
  prints the capture just before the run timeline, paced with the dump
  pause (8 lines, then 20 ms). After a frozen capture is printed, the ring
  re-arms for the next latch.
- `vehicle_impl.h`: forced include on `vehicle_task.o`. It replaces the
  `motion_estop` call with `isc_motion_estop`, which requests a capture
  and then calls the real ESTOP.
- `hook.c`: the static ring, the wrappers and the printer.

Tests: [`test_imu_safety_capture.c`](../../tests/test_imu_safety_capture.c)
(pre window, freeze, missed edges, re-arm, 16-bit time wrap, clamps,
non-finite input), [`test_imu_safety_capture_make.py`](../../tests/test_imu_safety_capture_make.py).

## USB lines

Each GP20 dump prints:

```text
[IMU_CAP] state FROZEN latches 1 missed 0 captures 1 pre 64 post 192
[IMU_CAP] source LATCH trigger 2 latch_ms 244279 samples 256 pre 64
[IMU_CAP] cols i rel_ms ax ay az g_mg tilt_ddeg vert_mg flags ff lift ot trig in
[IMU_CAP_S] 0 -640 12 -8 1003 1003 0 3 17 0 0 0 00 5
...
[IMU_CAP] end
```

If no capture is frozen, only the first line is printed (state ARMED or
COLLECTING).

| Column | Meaning |
| --- | --- |
| `i` | Sample index, oldest first. Index `pre` is the edge (or first ESTOP) sample. |
| `rel_ms` | Observation time minus the edge (or ESTOP) time, ms. |
| `ax ay az` | Input acceleration, mg, X right, Y forward, Z up (bias corrected). |
| `g_mg` | Detector magnitude, mg. |
| `tilt_ddeg` | Tilt from +Z, 0.1°. It is 0 inside the upright bounds (not computed there). |
| `vert_mg` | Vertical change against the quiet gravity reference, mg. |
| `flags` | `IMU_SAFETY_FLAG_*`, hex: 01 valid, 02 upright, 04 gravity, 08 freefall, 10 near 1 g. |
| `ff lift ot` | Freefall, vertical-change and overturn runs (consecutive samples). LIFT fires at ff 2 or lift 3; OVERTURN at ot 5. |
| `trig` | Hex, current trigger × 16 + latched trigger (1 OVERTURN, 2 LIFT, 3 IMU_UNAVAILABLE). |
| `in` | Input bits, hex: 1 valid, 2 calibrated, 4 wheels still, 8 read failed. |

## Operator procedure (lift and overturn only)

Scope: lift and overturn only. Do not drop or bump the robot.

Before each GP20 press, connect USB and start the USB log. Do not
power-cycle the robot: the capture is in RAM only. One capture is kept
until a GP20 dump prints it. So do one event, then one dump.

For each run:

1. Start a run (or keep idle, as the list says).
2. Do the event once. Then do not touch the robot for 3 s.
3. Press GP20 once. Wait for `[IMU_CAP] end` and
   `[robot] end tm_usb_dropped ... (dump delta 0)`.
4. Only if the robot latched (shell FAULT, kind 10 STOP): put it upright
   on the floor, hands off, wait more than 1 s, then press GP21 once to
   clear (expect kind 10 PENDING, then CLEARED). After a GP21 ESTOP with
   no latch, do not press GP21 again: that starts a new run.

| Run | Event | Expect |
| --- | --- | --- |
| P1–P3 | While driving LINE, push the robot over to its LEFT side | Today: LIFT. Capture shows which rule fired. |
| P4–P6 | While driving LINE, push the robot over to its RIGHT side | Same |
| T1–T2 | Idle: tip the robot slowly by hand to its side (about 2–3 s to 90°), then leave it on its side | OVERTURN (tilt > 75° at near 1 g for 5 samples) |
| T3 | Idle: tip slowly forward onto its nose, leave it | OVERTURN |
| L1–L3 | While driving LINE, lift the robot firmly straight up about 10 cm and hold it level | LIFT, source LATCH |
| G1–G3 | While driving LINE, lift the robot GENTLY and slowly straight up about 5 cm and hold it level. Within 1 s of the lift, press GP21 once (ESTOP). If it latches LIFT before the press, record that and do not press | Normally no stop; source ESTOP, the lift is in the 1.92 s before the press |

Record for each run: run name, MQTT kind 10 STOP word, and the USB dump.

## Limits

- One capture per dump. A latch edge while COLLECTING or FROZEN is
  counted in `missed`, not recorded. An ESTOP then is ignored.
- Any Vehicle ESTOP starts an ESTOP capture, also a line or MQTT stop.
  Dump after each event, so a stray stop does not hide the next event.
- Samples before the edge: at most 64 (0.64 s) for LATCH, 192 (1.92 s)
  for ESTOP. After a re-arm, fewer exist until enough samples have
  passed.
- The time column uses the low 16 bits of the observation time. It is
  correct for the 2.56 s window.
- The dump is USB only. MQTT does not carry the capture.
- The profile is for diagnosis. Do not use it for timing claims: the
  wrapper adds one store per observation.
