# Host firmware simulator — milestone 2

This host build reads the real `src/` files. It does not change or link to
firmware CMake targets. See the [simulation record](../docs/SIMULATION.md)
for the design, model equations, assumptions, and remaining work.

## Build and run

Requirements: Linux, a host C compiler (GCC or Clang), CMake 3.20 or later,
and Python 3.9 or later with its standard library. No Pico SDK, network,
credentials, Python packages, or browser is required for this milestone.

From the repository root:

```sh
cmake -S sim -B sim/build -DSIM_FIRMWARE=detour_test
cmake --build sim/build
sim/build/robot_sim --scenario sim/scenarios/no_box_straight.json --output sim/output/follow
sim/build/robot_sim --scenario sim/scenarios/box_straight.json --output sim/output/box
ctest --test-dir sim/build --output-on-failure
```

`robot_sim` is a generated Python launcher. It validates JSON, sends a
numeric configuration to the native `firmware_sim` process, and writes
`trace.json`. This uses Python's JSON parser instead of a vendored C parser.
The C process runs the unchanged firmware entry point.

The selected source list and per-source renames are in
[`targets/detour_test.cmake`](targets/detour_test.cmake).
Only `detour_test.c` gets `main=firmware_main` and module call renames.
All retained firmware sources get `printf=sim_printf`. Simulator source
files have strict warnings with `-Werror`; firmware has warnings enabled
without changes to its sources. Unsupported targets or SDK symbols fail.

## Milestone boundary

- The real boot wait, buttons, spin calibration, line processing, steering,
  and run limits execute. Calibration is not injected or bypassed.
- [`core/world.c`](core/world.c) supplies duty/speed response, signed
  fractional encoders, differential drive, the robot rectangle, tile lines,
  sensor overlap, and conservative contact detection.
- [`scenarios/no_box_straight.json`](scenarios/no_box_straight.json) has 30
  straight tiles and no box. Calibration succeeds; following ends at the
  real 30 s run limit, with no searches in the reference run.
- [`scenarios/box_straight.json`](scenarios/box_straight.json) keeps the
  120 × 120 mm box on the line at x = 800 mm. Contact is recorded; a safety
  latch stops motion and encoder progress. The real wheel-stall check then
  ends the run. The robot does not pass through the box.
- [`scenarios/unloaded_reference.json`](scenarios/unloaded_reference.json)
  selects the unloaded curve and the explicit synthetic coast fit. The
  loaded scenarios use marked 50 ms acceleration/coast constants per wheel.
- Servo pulses are clamped and recorded. Ultrasound still gives only
  NO_RETURN, after 215 µs rise latency plus the 3 ms echo timeout, with a
  60 ms trigger gap. Publication requires `ultrasonic_service`; no obstacle
  STOP or detour is expected yet. USB is disconnected; Wi-Fi is a local sink.
- Physical contact response, escape/reverse from contact, single-wheel
  contact effects, echo geometry, full phase decoding, and the HTML viewer
  are later work. Contact is latched until the next process/scenario, even
  for a separating command. This is a safety stop, not contact dynamics.

External call hooks record public results/state at non-advancing timestamps.
Complete phase inference, comparison with unhooked runs, and printf step
sequence checks remain build plan step 7 work.

## Scenario schema

[`core/scenario.py`](core/scenario.py) is the executable version 1 schema.
The scenario files supply every required field and show the exact object
shapes. [`core/protocol.py`](core/protocol.py) converts validated inputs to
the private native protocol. Unknown/duplicate keys, invalid types, ranges,
unsupported noise/timing/contact modes, unsorted button events, and missing
values fail before firmware starts. No defaults fill missing model values.

Values can be explicit assumptions:

```json
{"value": 22, "assumed": true, "note": "IR spacing guess; measure later"}
```

Every assumption, path, value, and note is kept in `header.assumed_values`.
The future viewer will show this list. Published 315 mm tile size and 18 mm
line width are specifications, not assumptions. The marked 22 mm sensor
spacing, 3 mm footprint, and sensing x = 119 mm remain guesses. The native
world repeats the gap check through its actual stroke/footprint/ADC model:
22 mm spacing passes and 14 mm fails. Measurements must replace guesses.

Coordinates are mm; JSON headings are degrees; `*_us` times are integer
microseconds. Physical L/C/R ADC order is mandatory. Zero-rotation straight
tiles run along +x through the row midpoint. The first row starts at
y = −157.5 mm, so the line is at y = 0. Curve tiles use the assumed quarter
circle at a tile corner with radius 157.5 mm. Cross/blank tiles, rotated
tiles, and arbitrary polyline/arc strokes are supported.

The footprint model is a 3 mm uniform one-dimensional aperture across the
nearest line band. Coverage is the aperture/line intersection length divided
by aperture width. It is not a measured optical response. ADC endpoints are
mixed by coverage, rounded, and clamped to 0–4095. No noise is added. At joins
or overlapping strokes, the strongest coverage wins; full optical union
area and foam seam effects are not modelled.

Scenario time starts before the 2 s boot wait. GP20/GP21 events are active
low at absolute times. The supplied press is at 2.1 s, release at 2.3 s.
Clock reads advance by the selected quantum (100 µs normally), sleeps by
exact duration, and conversions/trace writes do not advance time. Physics
uses at most 100 µs steps and splits at button/snapshot/end events. Contacts
use smaller conservative steps. CPU limits remain 5 s soft/6 s hard;
the wall watchdog remains 10 s.

## Trace and checks

The header records revisions, source SHA-256 hashes, compiler/commands,
scenario, assumptions, clock policy, and model status. Events include text
fragments, buttons, duties, servo commands, packets, hooks, and first contact.
World snapshots are saved at 0 and every 10 ms, plus a non-grid end time.
They contain rear-axle pose, speeds, fractional `counts`, integer
`encoder_counts` as returned to firmware, duties, raw ADC values, transformed
IR points, and the contact latch. Rectangle dimensions stay in the header.
Text lines displayed by the runner use the completing fragment timestamp.

There is no wall timestamp, process ID, or output path in a trace. Identical
source/build/scenario inputs must give byte-identical traces. Build/revision
changes intentionally change metadata. A source hash mismatch is an error.
A watchdog, native error, or incomplete stream produces an explicitly
incomplete trace. Output replacement is atomic; scenario completion does
not mean that firmware completed a detour.

Tests cover the milestone 1 clock/schema/watchdog contracts, real calibration,
straight/spin/reverse/coast equations, fitted unloaded coast, tile joins,
cross/curve IR, production gap checks, thin-box/circle no-tunnelling checks,
real following, contact and no penetration, repeatability, and source hashes.
The clock/device probe uses an internal physics-disabled configuration only
to keep clock-wrap tests independent; normal scenarios always enable physics.

For numeric model evidence alone:

```sh
sim/build/sim_model_probe
```
