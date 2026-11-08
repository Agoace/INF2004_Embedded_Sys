# Analog barcode B1: TEST-only observation

## Scope

B1 implements a pure ordinary Code 39 decoder and an exclusive hand-push
capture profile. It does not send commands to the Vehicle Controller or
change normal LINE behaviour. Physical decoding is **Not run**.

The [requirements summary](../project-requirements/README.md) points to the
local course originals. They remain ignored; do not publish their pages.
The reference branch `origin/ziven-barcode-algo` is reference only. This
implementation uses the standard Code 39 table, not the old GP7 reader.

## Input and ownership

| Item | Contract |
| --- | --- |
| Analog input | GP28/ADC2, existing 1 kHz acquisition, eight conversions |
| `raw_bc` | Third ADC value, never a third line-control input |
| VBAT sense jumper | Keep open; otherwise VIN/2 enters GP28 |
| GP7 | GY-511 INT1 wire now; barcode code never configures GP7 |
| GP20 | Arm/freeze button in this profile only; release then press |
| GP21 | No barcode handling; do not use it to start a capture |
| Line task | Priority 11; O(1) copy after releasing its mutex, outside ISR |
| TEST runner | Priority 20; coherent motion observations every 10 ms |
| Decoder | TEST runner, after freeze, bounded steps with 1 ms sleeps |
| ADC ownership | Unchanged; no new setup or second `drv_line_take()` reader |

At about 5.5 mm or more above the floor, the view can cover multiple narrow
bars. Slower motion does not remove optical blur. Keep the fixed left-front
mount clear of ramps. Measure its position before any later sensor-point
turn correction; B1 does not implement that correction.

## Pure decoder contract

Sources: [`barcode.h`](../../app_program/subsystem3/barcode.h) and
[`barcode.c`](../../app_program/subsystem3/barcode.c).

- Learn low/high from the complete frozen analog profile. Require at least
  64 ADC counts of contrast by default; this is a software setting, not a
  measured optical acceptance threshold. Hysteresis switches at 40%/60%
  of that span. Both analog polarities are tried.
- Measure every alternating run in microseconds and encoder millimetres.
  Decode each method separately and report both; never substitute one
  silently for the other. Motion uses bracketed source timestamps, wheel
  mean counts and 12414 counts/m. No extrapolation is permitted.
- Distance rejects missing/gapped motion or a movement reversal. The
  straight-wheel estimate has no sensor-offset or turn correction. A
  hand-push result is not distance calibration: back-driven wheels slip.
- Recognize all 43 standard data symbols plus framing `*`, in either read
  direction. This is not extended ASCII Code 39 and has no checksum.
- Each character has nine elements and exactly three wide elements.
  Require wide/narrow mean ratio 1.6..3.4, separated classes (at least 1.4),
  bounded spread, a narrow inter-character gap and white quiet zones of
  at least ten narrow widths before and after a complete frame.
- Report a string of up to 16 data symbols. Only exactly one A/B/C/D maps
  to LEFT/RIGHT/STRAIGHT/UTURN. F, Z and all other valid strings produce
  `BC_CMD_NONE`. Conflicting complete frames reject as ambiguous.
- Quality is an enum: REJECTED or DECODED. Separation is the smallest
  wide / largest narrow ratio in per mille, not a confidence probability.
  Reject reasons distinguish contrast, loss, width, pattern, quiet zone,
  truncation, capacity, ambiguity and unavailable motion.
- Sample flags, non-increasing time or an ADC gap above the configured
  3 ms limit reject both methods. Capture flags gaps above 1.5 ms, so
  that stricter profile rule also rejects. No glitch is repaired by guess.
- `barcode_begin()` / `barcode_step()` process at most 32 input records or
  one possible frame per step. The synchronous helper is for host tools;
  the TEST task yields between steps. Inputs stay immutable until done.

No result authorizes a turn. Future robot routing needs separate physical
validation, copy agreement and explicit Vehicle Controller integration.
Time widths are useful for straight hand-push contrast/geometry checks,
not proof of correct widths during weaving or changing speed. Distance
and time can disagree; both results remain visible, with no combined vote.

## Exclusive capture profile

```sh
make -C build_make SMP=0 WIFI=none NET=none TEST=1 BARCODE_CAPTURE=1 -j4
```

The profile rejects BENCH, IMU_STREAM, SMP, Wi-Fi, network and MQTT
combinations. Use USB console and CRASH_REPORT=0 for the connected robot.
It skips all ordinary subsystem tests, including the S2 driving steps and
watchdog-reset test. It requests stop before capture and never requests
speed. Zero duty is coast, not a physical wheel brake. First pass is
**HAND PUSH STRAIGHT**, not a powered drive test.

1. Start terminal recording. Verify the full `_test_barcode_capture` image
   name and the hand-push banner. Check the VBAT jumper and mount clearance.
2. Place FLOOR, WHITE_PAPER, then BLACK_BAR under the installed sensor.
   At each prompt, press GP20 to record the **next** one-second range while
   stationary. Save each `[BC_RANGE]` sample count and raw minimum/maximum.
   No range is an automatic contrast PASS.
3. Place the robot before the 2x F print, with enough white margin for a
   quiet zone. Press GP20 and push straight across the full frame within
   about 4.1 s. Release GP20; another press freezes early. Capacity also
   freezes automatically. Leave white visible after the last bar.
4. Wait for both frozen results and the complete USB dump. Save BEGIN/END
   row counts, ADC rows, motion rows and run widths. A new GP20 press after
   the dump starts a new pass and replaces the old data.
5. Repeat in reverse print direction, then use the course-size F copy.
   If contrast or width separation fails, retain the raw profile and report
   unreadable. Do not lower thresholds until a letter appears.

| Storage | Bytes / limit |
| --- | --- |
| ADC | 4096 x 8 = 32768; 1 kHz, about 4.1 s |
| Motion | 512 x 12 = 6144; source-stamped 10 ms observations |
| Decoder workspace | 1728, including incremental state; ARM report adds 64 |
| Capture arrays | 38912 = 38 KiB; static, exclusive profile only |
| Whole capture image static RAM | 159964 = data 152 + BSS 159812 |

Workspace plus report uses 1792 ARM bytes (1816 on the host); an assert
checks the 2 KiB limit.
At most 192 alternating runs are retained (enough for 16 payload symbols).
Static RAM is from the software-validation image based on `b569adc`, not a
runtime free-heap measurement. Dynamic kernel objects/stacks still need
hardware checks. Normal images do not allocate the capture arrays/workspace.

`BC_STATS` reports acquisition missed/dropped deltas, ADC gaps, ADC full,
motion overflow/missing reads, observer missed wakes, line-copy maximum
elapsed microseconds, decoder step maximum and decoder total. ADC full is
an intentional freeze, not overwrite. Motion overflow is separately counted.
`BC_RUN` lists time_us and signed distance_um; INT32_MIN means unavailable
or out of output range. USB dump output is paced after capture freeze.
Check row counts: transport loss is not repaired. Elapsed costs include
preemption and are upper bounds, not measured CPU-only time.

## Fixtures and evidence

| Item | Known value / expected result |
| --- | --- |
| Course samples | Standard `*A*` and `*Z*`; not the operator's F print |
| Operator print | Photo plus ruler confirmed standard `*F*`, no command |
| Course-size F | About 141 mm total; narrow 2.9 mm, wide 8.7 mm |
| 2x F, first physical test | About 285 mm; narrow 5.8 mm, wide 17.5 mm |
| `*` elements | `nwnnwnwnn` |
| F / Z elements | `nnwnwwnnn` / `nwwnwnnnn`; two gaps differ |
| A elements | `wnnnnwnnw` |
| Track reference | Barcode-to-line gap 20 mm; line width 18 mm |
| Earlier GP28 floor evidence | `3f889a1`: 178..230 and 196..217 only |
| New host tests | Standard table, both methods/directions/polarities pass |
| Contrast and physical F passes | Not run; no optical or timing PASS |

Host F fixtures use ideal widths near the supplied ruler values, not a
captured sensor trace. At 100 mm/s, 2.9/5.8 mm narrow bars last 29/58 ms;
these calculations do not prove optical resolution. Quiet-zone margins
must be checked on the actual print. A nearby line can truncate a margin.
Voting across course copies, steering changes and powered trials remain
outside B1. See [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md),
[LINE](LINE.md) and [build profiles](../kernel/BUILD.md).
