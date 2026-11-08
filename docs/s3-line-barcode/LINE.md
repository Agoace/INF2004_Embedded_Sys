# Current line detection, steering and search

## What it is

The kernel robot uses two analog line sensors and the P7 wheel-speed
settings. The older duty design remains the locked physical reference
(tag `line-tracing-2sensor-v1`, e4ba9ec). P7 is the retained port setting,
not a new lock or proof that the kernel matches the duty run everywhere.

The [architecture](../architecture/ARCHITECTURE.md) owns scheduling and
routing. [Barcode](BARCODE.md) uses a separate analog channel; it does not
supply line decisions or decoded robot commands.

## How it works

### Acquisition and calibration

| Item | Current value / rule |
| --- | --- |
| Centre / left / barcode | GP26 ADC0 / GP27 ADC1 / GP28 ADC2 |
| Acquisition | 1 ms TIMER alarm; eight ADC conversions per channel; timestamped 128-slot queue |
| Line task | Priority 11, consumes each queued sample and publishes a snapshot |
| Pair selection | `lp_set_pair(..., true, 0)`; right line channel is unused/zero |
| Spin calibration | 131 mm/s; 633 counts each side of the start, 200 ms pause; 6 s step timeout |
| Return to line | Centre reaches 900 per mille, or falls 50 below a peak of at least 500 |
| Calibration range | At least 200 ADC counts of contrast on L and C; failed calibration retains the old one |
| Learned scale | 0 = floor, 1000 = line; polarity learned from the sweep |
| Slow adaptation | Floor below 20%, line above 80%; move each end 1/256 per sample |

The acquisition interrupt records data; the line task runs `lp_update()`.
`raw_bc` carries GP28 unchanged for future barcode work. The vehicle reads a
service copy; telemetry uses a separate bounded, non-consuming board read.
Sampling continues while the robot is idle.

### Pair-mode position

Let L and C be normalized readings in 0–1000, with target 0. Negative
position means line left, positive means right. Position is clamped to
−100..+100. The sensor peak is memory of the strongest single-sensor read,
not a new fixed ADC threshold.

| Observation | Position / action |
| --- | --- |
| Both see the line (at least 5%) | `(C - L - target) / 20` |
| Only one sees it, reading rises | Same formula; line approaches that sensor from the middle |
| Only C sees it, below its peak | `(2 * peak - C - L - target) / 20` |
| Only L sees it, below its peak | `(C - (2 * peak - L) - target) / 20` |
| Both below 30% | Keep position/peak; LOST after 50 ms if one sensor was last alone, else 300 ms |
| Return after LOST | FOUND when a sensor reaches 50% |
| L + C at least 1250, each at least 250 | Junction: retain position/peak, not centred |
| Both see it, no junction, absolute C−L−target at most 200 | Centred |

Pair mode does not set the old three-sensor junction-hold flag. The generic
LINE module retains its 450-count straight-across-hold setting, but that
is not evidence of a separate 450-count hold in this pair-mode detector.

### Steering and search

The shared steering rule adds P times position, D times the change in
position (limited to ±25 per update), and extra far gain beyond ±50.
It scales the turn by `(base - min) / 10000`, rounds, and applies opposite
changes to the two wheels. A max clamp reduces the other wheel by the
same excess. An inner wheel below min stops. `steer_mm_straight()` clears
D memory. There is no right trim in the mm/s path.

LINE resets steering/watch/heading on start. Encoder heading hints use
left-minus-right movement: beyond +20 counts hints LEFT, below −20 hints
RIGHT; centred clears the heading. Only a new echo advances its monitor;
STOP returns OBSTACLE, a sensor fault returns FAILED. NEAR does not select
a separate slow speed in current LINE.

SEARCH turns at 131 mm/s toward the last side (RIGHT first only for a RIGHT
payload, otherwise LEFT). It turns 316 counts, then reverses through the
start to 316 counts on the other side. FOUND returns to LINE; exhaustion
fails the shell. SEARCH does not run its own forward obstacle watch.

## Interface

| API/data | Contract |
| --- | --- |
| `lp_calib_*`, `lp_update()` | Pure learned detector; LOST/FOUND include the retained side where applicable |
| `line_snapshot_read()` | Inherited-mutex service copy; explicit event-consumption choice |
| `line_snapshot_try_read()` | Bounded observation board; does not consume an event |
| `lt_start/step`, `ls_start/step` | Pure module interfaces; request mm/s and return shell outcomes |
| Line word | Position bits 0–7, seen bit 8, junction bit 9, side bits 10–11, bits 12–15 zero |
| Steering word | Left/right 9-bit 0–511 mm/s fields; bits 18–31 spare; controller limit 460 mm/s |

Sources: [`line_position.h`](../../app_program/subsystem3/line_position.h),
[`line_steer.c`](../../app_program/subsystem3/line_steer.c),
[`line_service.c`](../../app_program/subsystem3/line_service.c),
[`line_trace.c`](../../app_program/subsystem3/line_trace.c),
[`line_search.c`](../../app_program/subsystem3/line_search.c) and
[robot settings](../../app_program/vehicle/vehicle_robot_config.h).

## Settings

| Setting | Locked duty reference P4 | Current kernel/P7 |
| --- | --- | --- |
| Base / min / max | 25 / 12 / 50% | 146 / 62 / 378 mm/s |
| Gain / D / far gain | 100 / 100 / 400 | 85 / 85 / 340 |
| Far threshold | 50 | 50 |
| Right trim | +1% | None; motion owns wheel matching |
| Search and calibration | 20% | 131 mm/s |
| Target | Between L and C | Same, target 0 |
| Sensor spacing / height | 14.5 mm measured / about 9 mm | Physical reference unchanged |
| Line width | 18 mm | Same course reference |

The original duty-map port used 171/62/378 mm/s and 100/100/400, then P7
matched measured P4 speed/turn response. Do not replace current 146 with
171 by reading an older mapping table. The robot wheel loop has I enabled
(`ki_q8` 128) and rise limit 8. Old GP20 D-step experiments belong to the
bare-metal test target, not the current kernel operator interface.

## Results

| Evidence | Result and scope |
| --- | --- |
| Spin, boot 5209 | Midway L/C 38–40% each; plain-line sum at most 102%; about 21 mm optical spacing inferred, not a replacement for measured 14.5 mm |
| P4 duty lock | 57 s; wheel stopped 10%; 2.6 turn swaps/s; 35 searches of 50–100 ms, correct first direction |
| P5 → P6 | Wheel-response delay improved 100 → 50 ms; P6 remained wobbly and faster than P4 |
| P7 | Base 146, gains 85/85/340; mean speed 142 mm/s; 33 searches/min; straight position spread 37 versus P4 26 |
| P8 / P9 experiments | D 130 and wheel I off did not remove straight-after-curve swing; P9 spread 35, mean speed 136 mm/s; retained setting is P7/I on |
| Locked duty comparison | 800000 updates, zero differences after shared-rule/mm/s work |
| Host mutation checks | Position 35/36 (equivalent survivor), steering 11/11, mm/s/shared 17/17, LINE/SEARCH 35/35 caught |
| Kernel S3-U01/U02, 3f889a1 | 1009 samples, no missed; age 993/983 us; task max 84/152 us; ISR max 110/108 us |
| Kernel M1-S7 | Basic line/search passed; three initial losses recovered in 20–110 ms; no missed/dropped samples |

## Limits and links

Sharp right curves can need a short search. Midway 38% is only modestly
above the 30% dark limit. A falling single-sensor value can briefly look
outward when the line actually returns inward. Straight-after-curve swing
remains unresolved. Sensor spacing and optical spread are not the same
measurement; simulator success does not settle that question. Keep the
barcode mount clear of ramps: contact caused the observed jerk, not a
confirmed steering fault.

- [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md).
- [Kernel integration results](../../testing/integration/KERNEL_ROBOT.md#step-7-hardware-results-2026-11-06).
- [Original position record](../archive/s3-line-barcode/LINE_POSITION.md) retains every calibration, gap, simulator and track table.
- [Original steering record](../archive/s3-line-barcode/LINE_STEERING.md) retains all earlier tuning runs and limits.
- [Original mm/s record](../archive/s3-line-barcode/LINE_STEER_MM.md) retains mapping calculations and host results.
- [Locked duty record](../archive/s3-line-barcode/TWO_SENSOR_LINE_TRACING.md) retains the complete P1–P4 evidence.
- [P5–P9 line-run record](../archive/s3-line-barcode/VC_LINE_RUN.md) retains full metrics, stop reasons and local log names.
