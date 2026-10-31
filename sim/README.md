# Host firmware simulator

This host build reads the real `src/` files. It does not change or link to
firmware CMake targets. See the [simulation record](../docs/SIMULATION.md)
for the design, model equations, assumptions, and remaining work.

## Build and run

Requirements: Linux, a host C compiler (GCC or Clang), CMake 3.20 or later,
and Python 3.9 or later with its standard library. No Pico SDK, network,
credentials, or Python packages are required. The generated viewer needs
only a modern browser; it does not need Node or Playwright.

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
`trace.json` plus `run.html`. This uses Python's JSON parser instead of a vendored C parser.
The C process runs the unchanged firmware entry point.

The selected source list and per-source renames are in
[`targets/detour_test.cmake`](targets/detour_test.cmake).
Only `detour_test.c` gets `main=firmware_main` and module call renames.
All retained firmware sources get `printf=sim_printf`. Simulator source
files have strict warnings with `-Werror`; firmware has warnings enabled
without changes to its sources. Unsupported targets or SDK symbols fail.

## Open or regenerate the viewer

Each run writes `run.html` next to `trace.json`, including failed/incomplete
runs. Open `sim/output/box/run.html` directly in a browser (`file://`), or
copy that single HTML file to another machine. It embeds CSS, JavaScript,
scenario and display data; it makes no network request and does not fetch
the JSON file. To regenerate from an existing trace without running firmware:

```sh
python3 sim/viewer/render.py sim/output/box/trace.json
# Optional destination:
python3 sim/viewer/render.py sim/output/box/trace.json --output sim/output/review.html
```

- Play/pause or space; speeds 0.25×, 1×, 4×, 16×.
- Drag the time scrubber; previous/next step goes to exact hooked transitions.
- Fit all sets the camera bounds; focus robot shows a closer view.
- Drag the floor to pan; wheel zoom keeps the point under the pointer fixed.
- Coordinates are mm, +y up. Blue is **true pose**; orange is the saved
  **firmware estimate**, visible only during inferred DETOUR. The estimate
  converts the module's S-frame x/y/heading through the recorded detour
  origin and side. It is held start-of-step data, not unrecorded live travel.
  The heading difference is shown without investigating or correcting it.
- Only drawing pose is interpolated. Commands, raw ADC/counts, phase, steps,
  echoes and console change at saved times. Gaps hide true pose rather than
  extrapolating. The console is text only; the ASSUMED section lists every
  assumption. Incomplete traces have a warning banner.

The display keeps all world samples, dt observations/return words, step
changes, motor/servo commands (including duplicates), console fragments,
triggers, results and contacts. Unused hook details/packets stay in the full
trace. Compact display data keeps the reference box HTML below 3 MB.
Regeneration from the saved trace is byte-identical to the runner output.

`sim_offline_viewer` tests data fidelity, both S-frame sides, regeneration,
escaping and incomplete data. Its optional browser check uses an installed
Node Playwright package. If it is not locally resolvable, set
`SIM_PLAYWRIGHT_MODULE=/absolute/path/to/playwright` for CTest. Without a
browser package that check reports a skip; no runtime dependency is added.
The separate `sim/tests/viewer_browser.cjs` can save persistent screenshots:

```sh
SIM_PLAYWRIGHT_MODULE=/absolute/path/to/playwright node sim/tests/viewer_browser.cjs \
  sim/output/box/run.html sim/output/box/trace.json sim/output/viewer-check
```

## Optional local environment editor

After the build above, start the standard-library Python server:

```sh
python3 sim/viewer/serve.py --build sim/build
# Open http://127.0.0.1:8765
# Optional port: add --port 9000
```

Use the printed `127.0.0.1` URL, not a network address or `localhost` alias.
Configure/build once as above. On each **Run**, the server checks native
C sources, compiler header dependencies and compile commands by content
hash, then calls `cmake --build` under the run/save lock. A normal source
edit uses an incremental build. Missing verification state, a different
binary, or changed content with old/preserved file times forces a clean
build so an old binary cannot pass the check. The first Run can therefore
be slower. A 120 s process-group timeout applies to the build.

The page shows **Rebuilt in X s** or **Build up to date**. Build failure
shows compiler output as plain text and does not start a simulation, even
if an older good binary exists. Output is also saved in
`sim/build/editor-build.log` (last 256 KiB shown on failure). Build state
stays in the ignored build directory. If the build is not configured, the
page still shows the configure command. Stop the server with Ctrl-C.

Only native build inputs trigger C compilation. Editing the server's own
Python does **not** rebuild or reload it: manually restart the server to
use Python changes. Source numbers are edited in `src/detour_test.c`, not
from the page. This is a local tool; do not expose it through a proxy.

1. Select a base scenario. Its model, guesses, motor profile and target are
   shown read-only. The server also refuses changes to those fields.
2. Set box width (across y), depth (along x), centre and angle, or add/remove
   a box or bottle. Width/depth are local dimensions before rotation.
   Resizing keeps the centre fixed. To keep a front face at x=740 with
   depth 480 and angle 0, set centre x=980.
3. In **Edit floor**, drag an obstacle to move it. Snap is in mm (0 disables
   snap). Empty-space dragging pans; wheel zooms. Dimensions are labelled.
   Enable **Select tiles**, click a grid cell, select straight/curve/cross/
   blank, and use Rotate 90°. Grid size/origin stay fixed to the base.
4. Set start x/y/heading and run length. Length is the scenario end time,
   including boot and calibration; firmware's own deadlines do not change.
5. Save source edits, then click **Run (build if needed)**. A full saved run returns, then playback
   starts. Edit floor returns to the draft without modifying that trace.
6. Click history entries to switch quickly between saved end views and
   their drafts. Each entry shows sizes, result/contact, the firmware's own
   `[run] detour ...` summary and a short `src/detour_test.c` content hash.
   A short/calibration-only run can have no summary; its build hash remains.
7. **Save as scenario** writes a new `sim/scenarios/NAME.json`, never an
   overwrite. Names use 1–64 characters from `[a-z0-9_-]`. Saved files use
   the same schema as hand-written scenarios.

Only environment fields can change. Clock, seed, events, track specs and
primitives, model and firmware fields remain locked to the chosen base.
Only one simulation (or save) can run at a time; another request gets a busy
message. Validation errors appear beside a field where possible. No invalid
request starts a simulation. The existing native watchdogs still apply.

Each run is retained under `sim/output/web/SESSION-COUNT-HASH/`, with the
edited `scenario.json`, complete `trace.json`, offline `run.html`, and text
logs. The history covers this server session; files remain after shutdown.
Open or copy `run.html` without the server exactly as before.

CTest includes standard-library API/security tests. Optional browser evidence:

```sh
SIM_PLAYWRIGHT_MODULE=/absolute/path/to/playwright node sim/tests/editor_browser.cjs sim/build
```

This starts an ephemeral loopback server, tests real reruns and saving,
removes its temporary saved-scenario fixture, and writes screenshots, timing
and equivalence evidence to `sim/output/web_check/`. Playwright is for tests
only, not a dependency of the editor or the simulator.

## Milestone boundary

- The real boot wait, buttons, spin calibration, line processing, steering,
  and run limits execute. Calibration is not injected or bypassed.
- [`core/world.c`](core/world.c) supplies duty/speed response, signed
  fractional encoders, differential drive, the robot rectangle, tile lines,
  sensor overlap, conservative contact detection, and sector echo geometry.
- [`scenarios/no_box_straight.json`](scenarios/no_box_straight.json) has 30
  straight tiles and no box. Calibration succeeds; following ends at the
  real 30 s run limit, with no searches in the reference run.
- [`scenarios/box_straight.json`](scenarios/box_straight.json) keeps the
  120 × 120 mm box on the line at x = 800 mm. Boxes now default to specular
  echoes (assumed 20° incidence cut, corner echo off). See the
  [material model and recorded fit limits](../docs/SIM_ECHO_FIT.md).
  With ideal material, the accepted reference firmware enters
  FOLLOW → STOP → SCAN → DETOUR → FAILED, ending at its 25 s detour time
  limit. It chooses LEFT; no contact occurs. This is not claimed as success.
- [`scenarios/unloaded_reference.json`](scenarios/unloaded_reference.json)
  selects the unloaded curve and the explicit synthetic coast fit. The
  loaded scenarios use marked 50 ms acceleration/coast constants per wheel.
- Servo pulses clamp to 1100–1900 µs and set angle at once, without slew.
  The real firmware retains its settle waits. The sensor face uses the
  x = 65 mm axis and the marked 0 mm face offset.
- [`devices/ultrasonic.c`](devices/ultrasonic.c) implements timed nearest
  box/circle surface echoes in a closed ±15° sector, not only the centre ray.
  Geometry is sampled at trigger time. Rise latency is 215 µs; echo time is
  2d/343 with unit conversion and integer pulse rounding. No echo times out
  3 ms after rise. The 16-bit result, 60 ms gap, busy rejection, initial NONE,
  fault streak, and debug values follow the real API. Only service publishes
  a pending result; injected STUCK_HIGH publishes immediately. Optional
  falling-edge callbacks run at scheduled completion, not publication time.
- Physical contact response, escape/reverse from contact, single-wheel
  contact effects are later work. Contact is latched
  until the next process/scenario, even for a separating command. This is a
  safety stop, not contact dynamics. The contact regression test injects
  NO_RETURN to reproduce the milestone 2 contact path explicitly.
- USB is disconnected; Wi-Fi is a local sink, not a transport model.

External call hooks record public results/state at non-advancing timestamps.
[`targets/detour_decode.py`](targets/detour_decode.py) derives run phases and
step timelines without feeding any state to firmware. Module pose/heading
are relative start-of-step S-frame values; live `world_pose` is separate.
Inferred phase boundaries retain pending caller assignments and the last
context-confirmed phase (`phase_last_confirmed`); idle/calibration
are NOT_RUNNING, not stale FOLLOW. Printed step order and module pose/heading
are cross-checked: the ideal reference box run has eight matching transitions.
A separate `-DSIM_OBSERVE_HOOKS=OFF` build gives identical non-hook native
events. Mismatches are saved under `firmware_observation.step_crosscheck`,
not hidden by changing firmware or geometry.

## Scenario schema

[`core/scenario.py`](core/scenario.py) is the executable version 1 schema.
The scenario files supply every required field and show the exact object
shapes. [`core/protocol.py`](core/protocol.py) converts validated inputs to
the private native protocol. Unknown/duplicate keys, invalid types, ranges,
unsupported noise/timing/contact/servo modes, unsorted events, and missing
values fail before firmware starts. No defaults fill missing model values.

Values can be explicit assumptions:

```json
{"value": 22, "assumed": true, "note": "IR spacing guess; measure later"}
```

Every assumption, path, value, and note is kept in `header.assumed_values`.
The viewer shows this list. Published 315 mm tile size and 18 mm
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
An event can instead be `{"at_us": 0, "ultrasonic_fault": "NO_RISE"}`.
Allowed persistent modes are NORMAL (remove injection), NO_RISE (2 ms rise
timeout), STUCK_HIGH, and NO_RETURN. A mode is sampled on each accepted
trigger; it does not change an already pending echo.
Clock reads advance by the selected quantum (100 µs normally), sleeps by
exact duration, and conversions/trace writes do not advance time. Physics
uses at most 100 µs steps and splits at button/fault/echo/snapshot/end events.
At ties, integrate old commands, apply buttons then fault events, complete
echoes, and save the snapshot. Firmware publication still requires service. Contacts
use smaller conservative steps. CPU limits remain 5 s soft/6 s hard;
the wall watchdog remains 10 s.

## Trace and checks

The header records revisions, source SHA-256 hashes, compiler/commands,
scenario, assumptions, clock policy, and model status. Events include text
fragments, buttons, duties, servo commands, packets, hooks, and first contact.
World snapshots are saved at 0 and every 10 ms, plus a non-grid end time.
They contain rear-axle pose, speeds, fractional `counts`, integer
`encoder_counts` as returned to firmware, duties, raw ADC values, transformed
IR points, servo angle, and the contact latch. Rectangle dimensions stay in
the header. Trigger events save the sampled origin, beam heading, nearest
surface point, distance, obstacle index, and fault mode. A geometric `hit`
can be out of range; the published result word decides validity. Completion and publication
events are separate; they must not be interpreted as the same timestamp.
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
Echo tests cover 59/60/511/512 mm, off-axis hits, misses, gap/busy rejection,
fault streak saturation/reset, immediate STUCK_HIGH, 32-bit wrap, delayed
publication, callbacks, and in-flight servo changes. The real 13-point scan
is checked against hand geometry; the full detour is compared with an
unhooked build and printed labels. See the simulation record for the exact
reference timeline and handoff-problem A–G comparison.
The clock/device probe uses an internal physics-disabled configuration only
to keep clock-wrap tests independent; normal scenarios always enable physics.

For numeric model evidence alone:

```sh
sim/build/sim_model_probe
```
