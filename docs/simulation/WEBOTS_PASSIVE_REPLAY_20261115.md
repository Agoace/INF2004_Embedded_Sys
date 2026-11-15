# Passive Webots codec replay — 2026-11-15

## Approved scope

Simulation only. No Pico, serial port, firmware control, navigation or motor
application. The operator approved one existing world,1ms basicTimeStep and
10s simulated time, headless. The subsequently approved
[driving step](WEBOTS_DRIVING_REPLAY_20261115.md) has a separate scope/result;
it does not change this passive evidence.

Chosen world: `courses/home_mat_1_assumed.json`, with
`calibration/robot_assumed.json`. Assumptions are retained in the generated
manifest; neither physical sensor accuracy nor motor calibration is established.

## Data path and time domains

`controllers/passive_replay/passive_replay.py` runs exactly10000 steps of1ms.
It sets both wheel torques to0 once, centres the servo once, and never applies
any scripted OUTPUT command. All scripted wheel commands are deliberately
nonzero (+/-1200); decoding/consumption is not actuation.

Each step samples actual Webots wheel encoders, servo position, accelerometer,
direction-only compass and Supervisor position. Pose is diagnostic only and
never enters SENSOR. Wheel angles become relative encoder counts using the
assumed2535 counts/revolution and common +Y wheel-axis convention. Both totals
start at0. A per-wheel jump over16 counts clears that encoder's VALID bit.

The REAL frozen wire codec encodes SENSOR and validates/decodes scripted OUTPUT
frames in strict order. This is a scripted peer, NOT a HELLO/ARM/session-clock
negotiation or firmware implementation. Only encoder channels can be VALID on
the wire. ADC, buttons, IMU, sonar and physical timing VALID/NEW bits stay CLEAR.
Raw IMU/compass data is retained, not falsely presented as properly phased
400/75Hz register samples or calibrated magnetic field. Optical and sonar
emulation remain outside this first step.

The first available sensor sample is at absolute simulation1000us, after the
first physics step. It is relative SENSOR source0. Subsequent relative sources
are1000us apart, while absolute simulation ends at10000000us. Both times are
retained; no time-zero sensor observation is invented.

Sampling has direct CLOCK_MONOTONIC_RAW start/end brackets. SENSOR encoding
completion and OUTPUT consumption have separate RAW event timestamps. Encoding,
counter updates and numeric/binary buffering follow the sample bracket and affect the
next iteration's wall time naturally. There are no per-step CSV/JSON/hex/file
writes or flushes. Records are bounded to10000 samples and1,240,000 wire bytes;
CSV/JSON export and replay occur after stepping. No extra writer thread/process,
scheduling change, timer tuning or GC setting is introduced.

## Diagnostic counts, never fatal

The unpaced fast-mode wall reference is first RAW capture plus source_us.
This is NOT simulation-time equality or a qualified device-clock estimate.
For each sample let start/end be RAW differences from that reference; the
reported diagnostic phase is max(abs(start),abs(end+1us)). Counts include:

- Early sample: start<0; never labelled a tolerated late capture.
- Phase over250us and over600us: includes early fast-mode deviations.
- Positive late: start>=0 and end+1us>250us.
- More than3 positive late in any rolling1000 source slots.
- More than2 consecutive positive late source slots.

Rolling and burst excesses are counted on EVERY affected slot, not disjoint
windows. They NEVER abort stepping or change source slots. Nonfinite samples,
skipped simulation steps, broken codec/data or exhausted storage DO fail the
smoke: tolerance counting is not permission to fabricate or drop observations.
Fast-mode early counts are expected; do not interpret their large absolute
phase as host stalls or compare them to real-device source-phase percentiles.

## Run and process ownership

The Qt launcher needs the operator-approved Xwayland display even with rendering
and interaction disabled. Use only DISPLAY=:0, existing desktop owner PID15613;
:1 and new display servers were NOT approved. The authority is selected by file
name; its contents are never read or logged by the tool.

```sh
DISPLAY=:0 XAUTHORITY="$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)" \
  python3 sim/webots/tools/passive_smoke.py \
  --output build/webots/passive-NEW --timeout-seconds 90
```

The tool generates a fresh local project below this repository's build/ and
executes exactly the approved form:

```text
timeout 90s webots --batch --mode=fast --no-rendering --minimize --stdout --stderr <world>
```

Private XDG config/tmp paths avoid global configuration changes. The Webots
wrapper is unchanged. `owned-process.json` records command, environment scope
and the created PGID. Cleanup signals ONLY that owned process group; never
kill by process name or touch the desktop owner. No server/install is started.

## Retained evidence and replay

- `passive-result/samples.csv`: numeric samples, absolute simulation time and
  RAW brackets; all raw device values retained.
- `passive-result/io.bin` and `io.csv`:20000 ordered SENSOR/OUTPUT_IGNORED frames
  with slot/source time, simulation time, sample RAW brackets, per-event RAW
  encoding/consumption timestamps, byte offsets and lengths.
- `passive-result/summary.json`: channels, clock scope, counts and wire SHA256.
- `passive-replay-check.json`: controller's post-step byte replay check.
- `passive-smoke-summary.json`: independent launcher replay, process/log result.

`pil/passive_replay.py::replay_export` rebuilds every packet, counter and I/O
order from the recorded inputs. Corrupted bytes/order or inconsistent summary
fails. It reproduces recorded-input results, NOT the original wall scheduling.

Webots can discard a final stdout banner during simulationQuit. Completion
requires fresh complete artifacts, matching controller/independent replay,
10000 samples/10s, clean exit and clean log; stdout banner presence is recorded
but not a completion requirement. No extra physics step is added to drain it.

## Observed headless result

Final evidence: `build/webots/passive-home1-io-20261115T084942Z/`.
Owned PGID218650; exit0;10000 samples,10.000s simulation; no warning/error log;
no motor application or hardware use. Replay SHA256:
`fb8e69c80202ac01999594fa4f5f3d7ece864d3712277c394765048a79604cd1`.

Diagnostic counts: early9999, positive late0, phase>250us9999, phase>600us9998,
rolling-quota/burst excess0. These mostly reflect faster-than-wall simulation,
not a real-time claim. Encoder totals stayed0 in this passive run; scripted
OUTPUTs still carried nonzero commands and were ignored.

The earlier no-DISPLAY attempt exited134 before controller startup; retained
at `build/webots/passive-home1-20261115T083307Z/`. The first DISPLAY=:0 run
completed/replayed data but exposed the lost-banner false-negative check;
retained at `build/webots/passive-home1-display0-20261115T083531Z/`.
Neither failed evidence directory was overwritten or relabelled.

Red-first host tests cover real codec/order/byte replay, corruption, nonzero
encoder observations, bounded storage, nonfinite/skipped inputs, early/late
nonfatal counting, project hashes, a10000-step fake Supervisor that rejects
nonzero torque and the lost-banner case. This proves only the passive data
path/accounting. Applying motor commands requires separate review/approval.
