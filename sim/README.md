# Host firmware simulator — milestone 1

This host build reads the real `src/` files. It does not change or link to
firmware CMake targets. See the [simulation record](../docs/SIMULATION.md)
for the design, assumptions, and remaining work.

## Build and run

Requirements: Linux, a host C compiler (GCC or Clang), CMake 3.20 or later,
and Python 3.9 or later with its standard library. No Pico SDK, network,
credentials, Python packages, or browser is required for this milestone.

From the repository root:

```sh
cmake -S sim -B sim/build -DSIM_FIRMWARE=detour_test
cmake --build sim/build
sim/build/robot_sim --scenario sim/scenarios/box_straight.json --output sim/output/box
ctest --test-dir sim/build --output-on-failure
```

`robot_sim` is a generated Python launcher. It validates JSON, sends a
numeric device/time configuration to the native `firmware_sim` process,
and writes `trace.json`. This uses Python's JSON parser instead of a
vendored C parser. The C process runs the unchanged firmware entry point.

The selected source list and per-source renames are in
[`targets/detour_test.cmake`](targets/detour_test.cmake).
Only `detour_test.c` gets `main=firmware_main` and the module call renames.
All retained firmware sources get `printf=sim_printf`. Simulator source
files have strict warnings with `-Werror`; firmware has warnings enabled
without changes to its sources. Unsupported targets or SDK symbols fail.

## Milestone boundary

- The real button handling, boot wait, and spin calibration run.
- Wheels never move. Encoder counts stay zero; IR reads the recorded floor
  ADC values, even if the scenario places the centre sensor on the line.
- Thus GP21 starts calibration, but calibration must fail on a real
  firmware wheel-stall check. A completed trace is not a completed detour.
- Servo commands are clamped and recorded. Ultrasound gives only NO_RETURN,
  after 215 µs rise latency plus the 3 ms echo timeout, with a 60 ms trigger
  gap. Result publication still requires `ultrasonic_service`.
- USB is disconnected; the local Wi-Fi sink is ready. No network runs.
- World, motor response, contact, IR overlap, echo geometry, phase decoding,
  and the HTML viewer are later milestones. Their scenario fields are
  validated and kept in the header, not used for physics yet.

External module call hooks are implemented and record public results/state
at non-advancing timestamps. Complete phase inference, input/state display,
comparison with unhooked runs, and printf step-sequence checks remain build
plan step 7 work once full driving scenarios can reach those calls.

## Scenario schema

[`core/scenario.py`](core/scenario.py) is the executable version 1 schema.
[`scenarios/box_straight.json`](scenarios/box_straight.json) supplies every
required field and shows the exact object shapes. Unknown keys, duplicate
keys, invalid types/units/ranges, unsupported noise/timing, unsorted button
events, and missing values fail before firmware starts. No defaults fill
missing model values. Future model parameters are validated too.

Values can be explicit assumptions:

```json
{"value": 22, "assumed": true, "note": "IR spacing guess; measure later"}
```

Every assumption, its path, value, and note is kept in `header.assumed_values`.
The future viewer will show this list. Published tile sizes and 18 mm line
width are specifications, not assumptions. The 22 mm IR spacing and 3 mm
footprint must pass the C–L gap consistency check with the 18 mm line;
14 mm spacing must fail. Coordinates are mm; headings are degrees in JSON;
`*_us` times are integer microseconds. Physical L/C/R ADC order is mandatory.

The straight tile at zero rotation runs along +x through its row midpoint.
The first row starts at y = −157.5 mm, so its line is at y = 0. The start
pose and box placement are explicit guesses. The 50 ms motor response-time
values are explicit provisional inputs, not measured coast constants, and
have no physical effect in milestone 1.

Scenario time starts at process start, before the firmware's 2 s boot wait.
GP20/GP21 events are active-low press/release edges at absolute times.
The supplied GP21 press is at 2.1 s and release at 2.3 s. No calibration
state is injected. The clock advances 100 µs on reads and by the full sleep
interval on sleep calls; conversions and trace writes do not advance it.
The CPU watchdog allows 5 CPU seconds (6 hard limit) for the native process;
a 10 s wall watchdog also catches a process that blocks without using CPU.

## Trace and checks

The header records source revision, all firmware `.c`/`.h` SHA-256 hashes
(except local Wi-Fi credentials), compiler and compile commands, scenario,
assumptions, clock policy, and placeholder status. Events include original
text fragments, buttons, duties, servo commands, packets, hooks, and the
exact scenario end. Text displayed by the runner is stamped when a newline
completes; original fragments keep their own times in the trace.

There is no wall-clock timestamp, process ID, or output path in a trace.
The same source tree, build, and scenario must give byte-identical traces.
Different source commits or compiler settings intentionally change metadata.
A source hash mismatch during a run is an error. A watchdog, native error,
or incomplete event stream produces an explicitly incomplete trace; it
cannot count as firmware success. Output replacement is atomic.

The tests cover schema errors, assumption capture, the 14/22 mm gap check,
clock-read/sleep/deadline rules and wrap, active-low buttons, fixed encoders,
ADC floor order, servo clamp, ultrasonic service timing/gap, packet bytes,
console escaping, real GP20/GP21 calibration failure, repeatability,
read-only source hashes, and a real CPU-watchdog kill.
