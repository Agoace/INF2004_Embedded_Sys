# B2a validation and junction trials

## Status

Software checks below passed on the B2a candidate based on `168d705`.
**J0–J6 hardware checks are Not run.** No image was pushed or flashed by the
worker. Build artifacts used public example settings, not device credentials.
The [design](../architecture/JUNCTION_ACTION.md) defines ownership, bounds,
source-record draft and the reserved event activated by B2a.

## Software evidence

| Check | Result |
| --- | --- |
| Strict host C/Python suite | 179/179 passed |
| Clang address/undefined/float-cast-overflow sanitizers | 179/179 passed |
| Recorder protocol tests | 69 passed, included above |
| Dashboard tests | 186 passed, included above |
| Firmware | Nine clean profile builds, no warnings/errors |
| Strict ARM application compilation | 675/675 (75 application units × 9 profiles) |
| Retained SDK hardware link proof | 9/9 passed; never-called probe, not for flashing |
| ELF/helper/legacy-source checks | Nine images: no `__atomic_*` helpers; no legacy drive/servo/spin/scan sources |
| Matching baseline | `168d705`, same public settings/toolchain; 71 strict ARM units for frame comparison |
| Documentation/template | Documentation links checked; template deviations remain 8 against `c3b0954` |

The five requested profiles use `SMP=0 WIFI=cyw43 NET=lwip MQTT=1`, default
corrected IMU source, and `JUNCTION_TEST_CMD=NONE/A/B/C/D`. Four additional
profiles use NONE: single-core radio-off robot, radio-off TEST, radio-off
BENCH, and dual-core MQTT robot. Profile changes discarded stale objects.

Build from `build_make/` with **public examples in an isolated copy** for
validation. An operator must rebuild with their private settings before an
authorized hardware session. Do not copy a credential-bearing image or log.

```sh
for cmd in NONE A B C D; do
    make SMP=0 WIFI=cyw43 NET=lwip MQTT=1 JUNCTION_TEST_CMD=$cmd \
        -j4 --output-sync=target
done
```

The NONE filename is `mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
A/B/C/D append `_jtestA`, `_jtestB`, `_jtestC`, `_jtestD` before `.uf2`.
The option is part of the object-profile marker. The boot line identifies the
FAKE command and geometry. These options do not start junction jobs in TEST
or BENCH roles, which have no normal vehicle START.

Strict ARM used the logged target flags plus `-Wall -Wextra -Wconversion
-Wshadow -Wformat=2 -Werror -fstack-usage`; SDK headers were system headers.
The SDK proof was run before changing each object profile. Forbidden source
paths checked: `src/drive.c`, `servo.c`, `line_spin.c`, `obstacle_scan.c`,
`motor_ctl_pico.c`. Pure application `ls_*` and `ss_*` modules are allowed.
No GPIO IRQ or template source changed.

### Fixtures and limits

- No-command wrapper and original LINE produce the same commands, state and
  outcomes across 1000 calibrated samples, including junctions, loss and return.
  The locked LINE/SEARCH/SCAN/DETOUR source and reference settings are unchanged.
- Pure fixtures cover already-on-cross START, clear baseline, the global
  1250-sum threshold ramp before both reach 700, exact 700 qualification, short black
  spans, one-sided/weak channels, stop/reverse/duplicate samples, stale data,
  epoch/sequence/slot rejection, sequence/time/encoder wrap, centre origin,
  all A/B/C/D targets, coast/travel bounds, stalls, timeouts, obstacle,
  cancellation, missing line and the bounded two-direction sweep.
- Outgoing evidence needs both normalized readings at least 300, plus existing
  SEEN/CENTRED and no junction, for 30 ms of fresh observations. A weak pair
  cannot be mistaken for a line during LP's LOST grace period.
- Real vehicle-task doubles execute all four actions with LP LOST/FOUND during
  spins; D passes an intermediate arm without finishing early. They verify
  command acknowledgement/busy state and the unchanged existing stop path.
  Busy/stale START motion-board reads fail without waiting or starting a run.
- Event-v2 fixtures cover every command/phase and malformed fields. The old
  event-v1 reserved kind7 remains readable. USB results repeat at GP20 without
  adding RAM event records, KR1 bytes or summary fields.

These are synthetic tests. Barcode-only false-positive rejection, wheel slip,
stop overshoot, speed choice, sensor footprint and physical turn accuracy are
not established by them.

## RAM and stack

Matching public-config single-core MQTT builds, GNU Arm 14.3, SDK 2.3.0:

| Image | Text B | Data B | BSS B | Static RAM B |
| --- | ---: | ---: | ---: | ---: |
| `168d705` baseline | 403988 | 904 | 189768 | 190672 |
| B2a NONE | 410244 | 904 | 190048 | 190952 |
| B2a A | 410308 | 904 | 190048 | 190952 |
| B2a B | 410304 | 904 | 190048 | 190952 |
| B2a C | 410308 | 904 | 190048 | 190952 |
| B2a D | 410304 | 904 | 190048 | 190952 |

B2a adds **280 B static RAM**, below the 512 B estimate. New named storage:
LINE adapter 188 B, telemetry result 48 B, source board 28 B, source sequence/ACK/
busy 12 B, reported phase 1 B; alignment accounts for the remaining delta.
The source value is 24 B, result 48 B. Telemetry INFO stays 108 B and the value
message stays 116 B. The existing 4096 B message buffer and task stacks do not
grow. There is no dynamic allocation.

| Compiler frame | Matching baseline | B2a MQTT |
| --- | ---: | ---: |
| Vehicle entry (inlined run job) | 880 B | NONE 1000 B; A/B/C/D 992 B |
| Telemetry entry | 360 B | 360 B |
| Junction observer | — | 144 B |
| Pure action step | — | 128 B |
| LINE wrapper step | — | 40 B |
| Junction USB formatter | — | 96 B |
| IMU sample | 1000 B | 1000 B |

Frames are **not** whole-stack bounds. Existing vehicle/telemetry stacks are
4096 B; IMU is 2048 B. Operator-supplied pre-B2a evidence reported IMU 480 B
free on `29d57de` and vehicle 2592 B free on `d51d27b`; these are not new-image
measurements. Preserve at least 256 B free in every affected task, including IMU. Collect named GP20
stack marks after the longest action and repeated runs. Timing, response peaks,
missed samples, radio loss and actual stack margins need hardware evidence.

## J0–J6 hardware procedure

Only the operator authorizes power/flash. Keep a real GP21 ESTOP available.
Lift both wheels and verify stop paths before a floor run. Start each floor
trial on the incoming line at least 200 mm before the cross. Use the same 18 mm
`+` on two flat tiles. Mark the incoming axle centre, cross centre and outgoing
90/180-degree references. Record encoder counts and video/ruler/protractor
measurements. Do not use a wall collision or an untested hard wheel restraint
as a stall test.

| Check | Procedure / required evidence | Status |
| --- | --- | --- |
| J0 | Measure axle-to-line sensors and pair spacing/height. Verify normalized contrast on floor, plain line, cross and barcode-only. Lifted wheel signs and A/B/D spins; stop/cancel first. Ruler-check the132 mm pivot target before accepting powered turns. | Not run |
| J1 | NONE baseline on plain line, `+` and beside barcode: no new turn, hold or action event. Then barcode-only with pending fake C (shared detector): zero junction claims. Keep raw/normalized pair evidence. | Not run |
| J2 | C straight bridge; verify equal commands through side arms,160 mm suppression, outgoing line and no SEARCH. Measure centre passage and exit overshoot. | Not run |
| J3 | A left90, at least3 repeats. Measure actual pivot offset and final heading; outgoing line resumed without an early stem match. | Not run |
| J4 | B right90, at least3 repeats; same measurements as J3. | Not run |
| J5 | D LEFT180, at least3 repeats; ignore intermediate90-degree arm, reacquire incoming stem in reverse direction. | Not run |
| J6 | Controlled missing branch/line, cancellation/ESTOP and bounded timeout; repeat at different approach speeds and with barcode+cross. Verify stops, no automatic turn retry, no false curve/barcode claims, named stack margins and zero timing/sample faults. | Not run |

Approved initial tolerances are **at most 10 degrees final heading error** and
**at most 15 mm axle-pivot offset**. Report actual values on every powered
repeat, not only PASS/FAIL. Encoder angle alone cannot prove physical heading
or pivot position because of slip. A report of DONE is not physical acceptance.
Do not tune the locked LINE controller to hide a pivot/turn error.

For every trial preserve command image/boot line, tested revision, bootID/runID,
source record sequence, kind7 phases, USB result/marks, measured pivot/angle,
stop reason, missed/overrun/loss counters and named task stacks. Keep operator
supplied observations distinct from direct log inspection. Do not open or
publish private logs. Leave a test Not run until evidence is supplied.
