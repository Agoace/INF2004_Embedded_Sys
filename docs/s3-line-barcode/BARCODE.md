# Barcode input and decoder ideas

## What it is

This is an input contract and an ideas record, not an implemented robot
decoder. The kernel samples the barcode sensor's analog output on GP28
(ADC2) and publishes `raw_bc`. It does not decode a command or change the
Vehicle Controller route from a barcode.

The team's [requirements summary](../project-requirements/README.md) is the
public navigation source. Course originals remain local. Do not copy their
pages into this repository.

## How it works now

| Input | Current handling |
| --- | --- |
| Sensor analog output | GP28/ADC2, sampled with L/C every 1 ms, eight conversions per channel |
| Line service | Stores the third acquisition value as `raw_bc`, not a third line sensor |
| GP7 digital input | Earlier reference decoder input; not the current robot barcode path |
| VBAT sense jumper | Must stay open; otherwise VIN/2 enters GP28 |
| Mount | Left front, fixed; keep it clear of the ramp surface |

At roughly 5.5 mm or more above the floor, the sensor view can overlap
several narrow bars. A fixed digital threshold can miss a small dip. The
planned analog approach learns a switching level from measured low/high
values. Slowing down does not remove optical blur.

The robot also weaves. Sensor speed and angle vary more at the front than
at the axle; a sensor can cross a bar twice during a turn. Time alone is
therefore not a reliable width measurement.

## Proposed interface — not implemented

| Stage | Proposed value / check |
| --- | --- |
| Acquire | Analog profile with sample time and encoder movement |
| Locate sensor | Measure mm ahead of axle and left of centre; do not treat the 123/80 mm body estimate as a calibration |
| Estimate width | Distance moved by the sensor point, with turn/angle correction |
| Decode | Code 39 candidate; nine elements, three wide, framed by start/stop `*` |
| Validate | Only an allowed command; check copy direction and agreement |
| Reject | Report unreadable and keep following the line; never turn on a guess |

Ideas retained for later work: calmer steering in the barcode zone, use the
first copy as a low-trust trigger, compare widths within each character,
and accept two agreeing copies out of three only after checking alternating
direction. Copy count/content/direction are still to be confirmed against
the actual sample before implementation. These are not accepted command
semantics or evidence of a working decoder.

Do not hold encoder heading without line feedback across the whole code.
Do not move the fixed front sensor without operator approval. The older
270–680 mm estimates used assumed character/bar sizes; they are not a
replacement for the course geometry below.

## Settings and dimensions

| Quantity | Value | Status |
| --- | --- | --- |
| Barcode length | Approximately 141 mm | Course-spec summary; verify the original drawing |
| Gap from barcode to line | 20 mm | Course-spec summary |
| Line width | 18 mm | Course-spec summary |
| Sensor height | About 5.5 mm or more | Mount-clearance observation, not an optical calibration |
| Narrow-bar candidates | About 2–3 mm small / 5 mm large | Earlier operator observations; confirm actual sample |
| Sampling | 1 ms | Existing acquisition, not a demonstrated decode rate |
| Minimum sample idea | At least four samples per narrow bar | Design check only; blur can still prevent a read |

At 146 mm/s, 2/3/5 mm bars last about 14/21/34 ms. At the 460 mm/s speed
limit they last about 4.3/6.5/11 ms. These calculated durations do not
establish whether the sensor can resolve the bars at its installed height.

## Results

| Evidence | Result / limit |
| --- | --- |
| Kernel TEST, 3f889a1, sensor over floor | GP28 raw ranges 178–230 and 196–217; acquisition evidence only |
| Ramp observation, 2026-11-04 | Robot jerk stopped when the touching barcode sensor was removed; mount clearance is required |
| Analog decode / two-of-three vote | Not implemented or hardware-validated |
| 2 mm / 5 mm bars at 5.5 mm height | Planned optical check; no PASS recorded |

## Limits and links

Measure the installed sensor position, narrow/wide ratio, quiet zones and
actual copy content. Record analog passes in both travel directions and
while weaving. Do not import the old GP7 digital reader as if it satisfies
this interface. Barcode logic, command validation and route integration
remain future work.

- [S3 TEST_LOG](../../testing/subsystem3/TEST_LOG.md).
- [Current line record](LINE.md).
- [Complete original ideas and assumptions](../archive/s3-line-barcode/BARCODE_READING_IDEAS.md).
- [Line acquisition/service](../../app_program/subsystem3/line_service.c).
- [Reference reader source](../../src/barcode_reader.c) — not a robot analog decoder.
