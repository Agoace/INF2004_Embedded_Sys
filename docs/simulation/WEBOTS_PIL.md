# Webots PIL host implementation

## Status and limits

Stage 0 is host-only. The branch starts at `451e562`. No Pico is flashed and no
serial port is opened by these tests. A software result is not Probe timing,
physical sensor accuracy, stack, CPU, deadline, stop latency or safety acceptance.

The implementation contains a JSON course builder, immutable optical scene,
explicit robot profile, duty-driven plant, robot PROTO, PNG-textured world
generator, PC wire codec, conservative clock bounds and bounded host transport.
Three illustrative worlds pass a Webots R2025a load/initial-pose smoke. That
smoke uses passive zero wheel torque; it is not a driving or sensor-model test.
An opt-in host serial soak worker is now under test. It is not connected to
Webots and has no measured hardware result. The Pico sensor/edge runtime is not
implemented here. Do not treat the host state machine as firmware.

SW6 owns the frozen PIL v1 protocol and C codec. SW4 owns `pil/s3_*` geometry,
optics and assets. SW5 owns other `sim/webots/` source files. Main owns integration,
shared documentation indexes, images and hardware operations. No file under
`app_program/` changes in this work.

## Run host tests

From the repository root:

```sh
PYTHONPATH=sim/webots python3 -m unittest discover -s sim/webots/tests -v
```

The tests use the Python standard library and do not need Webots or a serial
package. Course integration needs SW4 canonical modules. Before Main combines
the branches, supply their source explicitly without copying it into SW5:

```sh
PYTHONPATH=sim/webots:/path/to/SW4/sim/webots \
  python3 -m unittest discover -s sim/webots/tests -v
```

Without those modules, 12 course integration tests report a skip. The complete
validation below supplied committed SW4 source and had no such skips. The
independent SW6 golden fixture is copied without changes into
`sim/webots/tests/fixtures/pil_wire_v1.json`. The cross-codec tool requires that
this copy is byte-identical to the authoritative fixture in the codec checkout.
It compiles the real C source on the host, not a replacement C decoder:

```sh
python3 sim/webots/tools/check_c_interop.py \
  --codec-root /path/to/checkout-with-SW6-codec \
  --expected-revision 0611052065e6785eaa69a704ae041e7e5579e612 \
  --output build/webots/c-interop
```

Stage 0 validation: 67 host tests passed; 3026 C/PC comparisons passed using
26 golden vectors plus 3000 CRC-valid semantic mutations. This check was rerun
against committed SW6 `0611052065e6785eaa69a704ae041e7e5579e612`; its C codec,
header and authoritative fixture matched that revision before the run. Invalid
C decode left its destination unchanged. These numbers describe this slice,
not hardware or Webots execution. The retained rerun log is
`build/wip/pil_plan/c-interop-0611052-final.log`. Canonical course/PNG sources
were read from SW4 `dfb05db5580acc9f3a240c3efbe86a6b6d5d32db`. Each generated
manifest also retains their exact content hashes. The recovery review reran all
67 tests without skips and all 3026 C/PC comparisons. Its logs are
`build/wip/pil_plan/recovery-host-tests.log` and
`build/wip/pil_plan/recovery-c-interop.log`.

## Entry points

| File below `sim/webots/` | Purpose |
| --- | --- |
| `pil/course_builder.py` | Strict JSON schema, grid/port checks, canonical SW4 patches, boxes, closed sine-profile hump mesh and explicit paper thickness. |
| `pil/world.py` | Deterministic complete projects under `build/`, textures, colliders and provenance manifests. |
| `courses/*_assumed.json` | Illustrative 4-tile, 6-tile and 12-tile connected courses; not surveyed home/school mats. |
| `protos/RoboPico.proto` | Axle-centred body, driven wheel joints, passive swivel/trailing caster, GP15 servo head and IMU/debug sensor mounts. |
| `controllers/stage0_preview/` | Passive zero-torque preview, no UART or navigation. |
| `tools/load_smoke.py` | Bounded load/initial-pose test with explicit PID-group cleanup and retained proof/log. |
| `pil/serial_io.py` | Explicit Linux Debug Probe by-id selection and bounded nonblocking 921600 8N1 I/O. Import opens nothing. |
| `pil/gate1.py`, `tools/gate1_soak.py` | Transport-only neutral-source soak, CSV events and measured FAIL/INCOMPLETE summary. No Webots or physics ACK. |
| `pil/gate1_diagnostics.py`, `tools/gate1_report.py` | Offline stopped-USB D1 decode, exact coverage checks and conservative all-frame age bounds. No port access. |
| `pil/gate1_rx_timing.py`, `pil/gate1_measurement.py`, `tools/gate1_correlate.py` | Strict stopped RX1 parser and offline RAW PC/full-pcap/Pico timing correlation. Optional USB layer stays unknown when absent. Never promotes a verdict. |
| `tools/gate1_usbmon.sh` | Optional operator-approved sudo capture of one resolved USB bus. Foreground PID, operator-readable stderr and privilege drop; no UART open. |
| `pil/gate1_attribution.py` | Optional bounded numeric timing tail. Sleep/wake, loop stages, GC callbacks and CSV serialization/flush stamps. Saved after the serial stop/close attempt. |
| `pil/gate1_trial.py` | Explicit D-TRIAL metadata/manifest only. Clock400us/SYNC20ms is opt-in; default clock/phase and native acceptance are not redefined. |
| `tools/check_d1_interop.py` | Host-compiled real C D1 producer and independent PC age-bound checks, using a synthetic fixture only. |
| `pil/gate1_replay.py`, `tools/gate1_dry_run.py` | No-device recorded-byte/timestamp replay through the real no-Webots soak worker. |
| `pil/scene.py` | Convex planar surfaces, oriented boxes, spatial broadphase and immutable first-hit snapshots. |
| `pil/robot_profile.py` | Required numeric parameters, sources and assumption flags; mount-coordinate conversion. |
| `calibration/robot_assumed.json` | Explicit provisional model, not a calibrated robot. |
| `pil/motor_plant.py` | Loaded DM1 interpolation, assumed torque/coast response, wheel-count quantization and servo slew. |
| `pil/wire.py` | COBS, CRC32C, explicit LE fields, payload/range validation and bounded incremental decoding. |
| `pil/clock.py` | Offset intervals, drift expansion, fixed apply timeline and lookahead checks. |
| `pil/transport.py` | Host control lifecycle, bounded queues, leases and physics-applied acknowledgements; no actual serial I/O. |
| `tools/check_c_interop.py` | Real C/PC interoperability and CRC-valid semantic mutation checks. |

## Coordinates and scene API

Use ENU: world X east, Y north, Z up. Body x is forward, y is left and z is up;
yaw is left-positive. JSON/model geometry uses millimetres. Webots uses metres
and radians. The rigid body frame origin is the driven axle midpoint, at wheel
centre height. With nominal 65 mm wheels, it is 32.5 mm above a flat supporting
surface. For a surveyed flat sensor clearance, subtract that radius exactly
once to obtain axle-relative mounting Z. The sensor does not follow the terrain.

Support Z is zero and nominal EVA top is Z=10 mm. Tile size is 315 x 315 x 10 mm;
line width is 18 mm and nominal curve radius is 157.5 mm. SW4 canonical geometry
is the only source for optical strokes and assets; do not recreate its masks.

`SceneSnapshot.from_patches(patches)` copies structural SW4 `PolygonPatch`
fields `surface_id`, `material_id`, `vertices_mm` and `layer`. Convex planar
vertices are world mm and CCW as seen from the outward normal. Tilted triangles
are supported. Unsupported, non-planar, non-convex and duplicate-ID surfaces
are rejected at construction. Scene snapshots contain no mutable source list.

`first_hit(origin_mm, direction, max_distance_mm)` always returns `SurfaceHit`.
It normalizes a nonzero finite direction and returns physical mm distance.
Successful fields are `point_mm`, `normal`, `distance_mm`, `material_id`,
`local_uv_mm`, `surface_id`, `valid=True`, `reason=""`. Invalid fields are `None`,
`valid=False` with `miss`, `invalid_ray` or `ambiguous_material` reason. Never
convert a miss into a white floor reading.

UV origin is patch vertex zero, U follows its first edge, and V is normal cross
U. It is not necessarily tile-centred UV. Nearest physical surface wins. Within
1e-7 mm distance, higher material layer wins; different materials at the same
layer are ambiguous. Same-material ties select the lowest surface ID. Layers
are optical metadata, not raised collision surfaces. Material IDs include
`MAT_WHITE`, `MAT_LINE`, `PAPER_WHITE`, `PAPER_INK`; opaque boxes may use `BOX`.
The scene returns that material without inventing an optical ADC calibration.
The physical support below the EVA uses `SUPPORT`, an intentionally uncalibrated
optical material; leaving the tiles does not silently return a white ADC.

## Assumptions to replace with operator inputs

All entries in `robot_assumed.json` retain a value, source and `assumed` flag.
The following table is the replacement checklist. A default value permits host
model tests; it does not make the value measured or physically accepted.

| Quantity | Current value or rule | Evidence limit / required input |
| --- | --- | --- |
| Track / encoder ratio | 111 mm / 2535 counts per revolution | Operator track and unloaded 50-turn ratio; not tyre slip acceptance. |
| Wheel / distance conversion | Nominal 65 mm / firmware 12414 counts per metre | Effective loaded radius and ground travel are unmeasured. |
| Total mass | **ASSUMED 1 kg** | Weigh the robot with its current battery and fixtures. |
| Wheel / caster / head mass | **ASSUMED 40 g each / 20 g / 20 g** | Measure component mass or approve the approximation. |
| Body envelope | 190 x 160 mm; front reach **ASSUMED 123 mm** | Full width is conservative, not a chassis CAD model; front offset is uncertain. |
| Body height / COM | **ASSUMED** 35 mm box, box centre Z25 mm above axle; COM x30/Z20 mm | Survey mass distribution and actual collision shape. |
| Caster | **ASSUMED** x80 mm, radius10 mm | Retained WB80 is a hypothesis, not a caster survey. |
| Contact / inertia / torque | **ASSUMED** friction0.8, wheel inertia2.1125e-5 kg m², torque cap0.12 Nm | Loaded transient and contact measurements are absent. |
| Motor response | **ASSUMED** drive/coast50 ms; static threshold12 percent | DM1 used a12 percent kick; static breakaway and time constants were not measured. Zero drive means passive coast, not active braking. |
| Motor running map | DM1 loaded VIN, separate wheels/directions, 0–70 percent | Recorded steady speeds; not the old unloaded host table. |
| High-duty model | **ASSUMED / UNVALIDATED** linear extension of60–70 percent slope up to80 percent | High-duty wheelie/unsettled rows are excluded. Larger duty fails model validation; this does not raise firmware drive permission. |
| Servo / sonar | Axis x65 mm measured; axis Z20 mm and face offset0 **ASSUMED**; slew277.78 deg/s **ASSUMED** | Settle fit is not a measured angular response or face survey. |
| GP26/GP27 line pose | x119 mm / clearance9 mm **ASSUMED**; separation14.5 mm physical | Do not relabel old fitted20.9 mm spacing or derived x123 mm as a survey. |
| GP28 pose | x80 +/-5 mm approximate; y77 mm left; flat clearance5.5 mm nominal | Standoff response outside calibration is unvalidated; no terrain-following mount. |
| IMU pose | x60 mm and axle Z97.5 mm **ASSUMED** | Assumes retained130 mm height was floor-relative; original datum is unconfirmed. |
| Magnetic field | **ASSUMED** world0/0.3/-0.4 gauss | Illustrative strength/inclination, not local calibration. |
| Encoder phase | **ASSUMED** nearest/ties-away half-count phase | Counts derive from joint rotation, not ground displacement; fractional phase is retained. |
| Home mats / props / station | **ASSUMED** 4-tile and 6-tile loops, 12-tile school-like network | Actual home mat photos/layouts are missing. All start/prop placements are illustrative. STATION is omitted until marker and ports are supplied. |
| Example school props | **ASSUMED** box100x180x100 mm, right-wheel sine hump80x30x10 mm, 16 longitudinal segments | Static geometry, not a physical hump/contact calibration. |
| Example barcode sheets | **ASSUMED** 400x50 mm paper, 40 mm bar height, neglected thickness0 | Full389.5 mm quiet length fits; paper may span tiles. Zero thickness is explicit, not a survey. |
| Other PROTO geometry | **ASSUMED** wheel width16 mm, caster width6 mm/trail8 mm, swivel radius3 mm, caster mass split75/25 percent, head30x35x20 mm | These defaults and damping0.0001/0.00001 and servo torque0.1 Nm are also listed in each generated manifest; replace from survey/model evidence. |

The plant returns torque, not requested wheel speed. It does not duplicate the
real firmware PI, startup kick or rise limiter. Encoder endpoint differences
above16 counts per wheel per1ms slot invalidate the model, not create an
unbounded simulated-edge loop. Servo slew uses <=1ms steps.

## Wire and timing boundary

The SW6 record `docs/simulation/PIL_PROTOCOL.md` is authoritative once integrated.
Version1 uses COBS + zero delimiter, CRC32C, schema50494c01, 40 B SENSOR and32 B
OUTPUT payloads. SENSOR is66 wire bytes and OUTPUT58 bytes: at9216008N1 they
consume71.6 and62.9 percent respectively, in separate FULL-DUPLEX directions.
Those rates do not prove USB/Probe latency. UART carries no ASCII console.

Kernel TIMER stays real. Fixed total source-to-Pico apply delay is4ms, source
expiry5ms, and endpoint availability must be <=3ms from source capture. The
endpoint supplies simulated Gray transitions in the PRECEDING1ms interval; late
edges are never compressed or replayed. Pure simulated pending edges use no PIO.
A1ms physics step does not imply strict hardware lockstep.

The PC codec is stateless. `HostTransport` additionally checks session/nonce,
handshake, neutral initial frames, progressing snapshots, bounded queues and
leases. ARM requests enter pending; only matching ARMED OUTPUT acknowledging
SENSOR0 or later accepts the run. GPIO key pulses eventually need100ms minimum.
No navigation command is synthesized by the transport. Skeleton capabilities
cannot arm. Reset/new session is required after a fault; no automatic reconnect
or rearm is implemented.

SYNC intersects conservative offset constraints from all accepted observations
with exact100ppm cones and outward query rounding. At least eight observations
are required before ARM. It never divides jitter by sqrt(N). The candidate250us half-width and phase limits can fail.
A refresh must arrive within1s AND before uncertainty headroom expires. No
minimum serialization subtraction is used without a separate proven bound.
The timeline epoch never slides when clock estimates change.

PC writes and actual device staging are different events. The reference
`validate_available` requires actual fake/device availability time; it cannot
infer it from a completed PC write. A device timestamp/diagnostic is needed for
Gate1 evidence. `physics_applied` is explicit and only then advances SENSOR ACK.
Repeated OUTPUT reports can have unchanged10ms actuator timestamps; leases use
new1kHz snapshot time and sequence instead. Duplicate or stale reports never
renew progress. Raw Pico PWM and a PC model-local safety zero must remain
separate observations.

Evidence queues fail closed when full. The caller must drain them to retained
logs. RX chunks are limited to4096 bytes. A new packet after an expired5ms gap
cannot revive the session, even if it has a fresh snapshot. Conservative OUTPUT
snapshot age and non-regressing SENSOR acknowledgements are checked separately. A blocked/paused controller must inhibit before advancing physics;
best-effort DISARM must not delay local inhibition. The serial worker must not
call Webots APIs. Corrupt packets are discarded until a delimiter; there is no
search inside a damaged packet for a hidden command.

## Generate and load a preview

The operator-installed Webots R2025a wrapper was used without modification.
The generator needs SW4 geometry/assets in the combined tree or on the explicit
package path described above. It refuses output outside this checkout's
`build/` and refuses to overwrite an existing project.

```sh
PYTHONPATH=sim/webots python3 -m pil.world \
  --course sim/webots/courses/school_like_assumed.json \
  --robot sim/webots/calibration/robot_assumed.json \
  --output build/webots/school-preview

PYTHONPATH=sim/webots python3 sim/webots/tools/load_smoke.py \
  --course sim/webots/courses/school_like_assumed.json \
  --robot sim/webots/calibration/robot_assumed.json \
  --output build/webots/school-load-smoke
```

The generated project has local PROTO/controller/PNG files and no remote asset
URLs. Source patches feed both the CPU scene and visuals. Flat, zero-thickness
paper/ink is baked into tile-top PNGs; raised paper has its actual collider and
one paper/ink texture at its physical top. Boxes and humps are never painted
onto the floor texture. PNG edge padding to powers of two, with cropped UV,
avoids Webots resampling without changing the canonical mask or raising ink.
Overlapping sheets and unsupported paper draping/floating are rejected.

Every layout keeps assumed/source metadata. Rotation, neighbour ports, widths
and positions are checked. Open ends require explicit `allow_open_ports=true`
and remain in manifest diagnostics. STATION requires a supplied convex marker
and explicit cardinal ports. JSON rejects unknown fields and has tile/texture/
mesh budgets. Default curve tessellation error is at most0.01 mm. A static sine
hump mesh has closed collision sides; uniform material does not invent painted
line draping. Geometry, robot parameters, source hashes and generated file
hashes are retained in `manifest.json`.

The bounded smoke installs a separate zero-torque test controller in its
**generated** project and records that overlay's hashes. It uses GNU `timeout`,
a hard outer timeout and only its own PID group for cleanup; it does not change
the installed wrapper or compatibility libraries. It records a JSON proof and
checks200 simulated1ms steps, finite sensor/joint values, plausible initial axle
height and stationary acceleration magnitude. This is not a firmware verdict.
All three example worlds passed again in the recovery review with no Webots
errors/warnings. Proofs and logs are in `build/webots/recovery-review/`. Initial
parser/texture warnings were repaired before the final runs. No GUI/operator
usability, motion-controller behavior, optics, driven hump, wall-clock1ms or
hardware result is implied. Logs/proofs remain under `build/webots/` and
`build/wip/pil_plan/`. No launched Webots process is intentionally left running.

## Gate 1 host code: not hardware acceptance

The transport-only runner and D1 reader have fake-pair and reference-fixture
tests. The combined host suite ran 135 tests successfully, with three existing
expected S3 model failures; the 3026 frozen C/PC comparisons also passed. Logs
are in `build/wip/pil_plan/gate1-d1-host-tests.log` and
`build/wip/pil_plan/gate1-host-c-interop.log`. These are host software results,
not Gate 1 hardware results. No serial device was listed or opened during
development. Hardware execution needs separate operator
permission. The default CLI refuses access unless `--hardware-authorized` is
present. That option is an explicit request, not evidence of operator approval.

The adapter selects exactly one `/dev/serial/by-id/*Debug_Probe*` symlink. If
more than one is present, an exact by-id selector is required. A guessed
`ttyACM` path is rejected. The link is exclusive, nonblocking 921600 8N1 with
no flow control. The runner does not reconnect or rearm after a fault.

At port open, reset kernel input, then read and log discarded bytes for at least
300ms (three BOOT periods). A Debug Probe can retain a stale64B USB buffer;
a valid BOOT followed by its truncated successor is not a fresh startup stream.
After the discard window, require two CRC/range-valid BOOTs with the same nonce
and sequence+1 before HELLO. A gap requires a later consecutive pair. These
startup steps do not change active timing bounds or Pico error counters.
The attempt3 regression fixture `sim/webots/tests/fixtures/pil_gate1_attempt3_stale_rx.txt`
contains the original received bytes, including stale BOOT0, partial BOOT1 and
a valid HELLO_ACK glued to it. The red test selects the stale nonce and fails;
the corrected worker discards those bytes and selects a later fresh BOOT pair.

Bytes can also start inside a BOOT packet. Before the first fully
valid BOOT only, malformed startup fragments are retained and counted as
`bootstrap_discarded_*`. The decoder resumes only at packet delimiters; it never
scans inside a damaged packet. A valid first BOOT is accepted without requiring
an extra leading delimiter. Startup junk does not establish readiness, create a
session or renew progress. After that BOOT, malformed packets fail immediately,
even if BOOT and corrupt bytes arrived in the same read. `decoder_total_*`
retains all errors; `decoder_*` excludes only pre-BOOT discards. Pico counters
are never erased. The original attempt2 RX chunk now accepts BOOTseq2 after one
startup discard. This is an offline regression check, not a hardware result.
The regression suite ran157 tests OK with the same three expected S3 failures;
logs are `build/wip/pil_plan/bootstrap-resync-red.log`,
`bootstrap-resync-green.log` and `attempt2-bootstrap-replay.log`.

### Gate 1 proven directional minima

Only the explicit cap4 transport profile credits these fixed wire-time minima.
Normal full-simulator sessions and generic `ClockBounds` keep zero defaults.
The proof is SW6's `docs/simulation/PIL_GATE1_CLOCK_PROOF.md` at commit
`25b968ae9bf95fea884f6b3098b9e78274c94d4b`, for runtime
`5c7361a4462d07ab6df6bb60f35a9141fe5c82ca` and unchanged image `0017cfa0...`.

| Direction | Complete characters credited | Safe rate ceiling | Integer minimum |
| --- | ---: | ---: | ---: |
| Queued PC request to Pico delimiter receive stamp |33|985000 bit/s|`floor(330000000/985000)-1 =334us` |
| Pico reply generation stamp to complete PC receive |41|921000 bit/s|`floor(410000000/921000)-1 =444us` |

The request/reply have34/42 wire bytes, respectively. Do not credit the final
stop bit: RX can complete before that bit ends. Pico t2 follows delimiter DR
read; t3 precedes encoding, queueing and all reply bytes. USB, processing and
scheduling add delay; none receives an assumed minimum. Named constants in
`pil/clock.py` derive the334/444us floors from the stated rate ceilings.

The profile requires125MHz peripheral clock and UART IBRD8/FBRD31:
`125000000/(16*(8+31/64)) =920810.313...` baud. A successful stop-bit sample
alone does **not** prove a Probe rate limit. The request's known `00` delimiter,
normal8N1 framing, stable divisors and matching CRC/echo/sequence must establish
byte correspondence. Its final LOW data sample precedes HIGH stop/idle.
PL01116x sampling with early-phase/divider margins gives the conservative
request ceiling985000; the reply ceiling921000 includes the100ppm conversion.
The proof record states the complete derivation and association conditions.

PC records identify both minima but keep their proof INCOMPLETE until matching
stopped D1 arrives. The D1 clock/divisor must match exactly, and COUNTS indices
BAD_FRAME11, OVERRUN13, FRAMING14, PARITY15 and BREAK16 must all be zero. Missing
or unknown evidence leaves the proof INCOMPLETE and prevents transport subset
qualification. A measured UART fault remains an overall FAIL; it is not
silently downgraded. Clean counters make this proof ELIGIBLE, not a Gate1 PASS.
Other raw queue, coverage, image and active-cohort checks remain necessary.

The estimator rejects minima whose sum exceeds the observed RTT. It retains
outward quantization,100ppm drift, eight-observation intersection, refresh
headroom and the **unchanged250us** ARM limit. No minimum-RTT guess, assumed USB
latency or statistical reduction is used.

The real attempt4 fixture `sim/webots/tests/fixtures/pil_gate1_attempt4_sync.json`
contains all eight exchanges and the attempted ARM time. Replay without minima
had585.5us half-width with the earlier recursive rounding and correctly refused
ARM; minima gave196.5us. Exact cones now give583us without minima and194us
with334/444us at that same time. This is a recomputation of an old
failed attempt, not a successful new hardware run. Red/green logs are
`build/wip/pil_plan/wire-minima-red.log`, `wire-minima-integration-red.log` and
`wire-minima-green.log`.

### Safe all-SYNC cones and RAW clock

PC hardware timestamps now use `clock_gettime_ns(CLOCK_MONOTONIC_RAW)`, not
NTP/chrony-adjusted `CLOCK_MONOTONIC`. There is no slewed-clock fallback. The
summary and run metadata identify the clock domain. Missing, injected or non-RAW
clock evidence cannot qualify the physical minima/transport subset. Earlier
attempts used MONOTONIC: their replays remain conditional, not physical proof.

Keep the100ppm outer rate bound. For each SYNC interval `[lo_i, hi_i]` at`t_i`,
query bounds at`t` are `max_i(lo_i-rho*(t-t_i))` and
`min_i(hi_i+rho*(t-t_i))`, where`rho=100/1000000`. Two exact integer-scaled
intercepts retain all constraints in constant space. Quantization remains a
separate outward1us; fractional drift is retained and endpoints round outward
only at query time. An exact integer-time refresh deadline preserves the250us
limit and1s refresh limit. Empty intersections still fail.

A measured historical rate is for reporting only. Without a proved rate-change
limit, the next instant can take either physical extreme; fitting a constant
skew cannot safely narrow future aging. Tests include600s with20ppm skew and a
later change to-80ppm, plus an abrupt unseen rate change after the last SYNC.

Attempt5's17-SYNC replay ends at241us instead of248us. At SENSOR658's PC write,
cones give242.5us instead of250us. This removes repeated rounding loss, not the
physical uncertainty. A seeded600s resampling of nine active exchanges still
found expiry at100/50/20/10ms periods; even10ms had263us maximum despite243.5us
p99. These are conditional simulations, not evidence of future latency tails.
**The production SYNC period remains unchanged:100ms maximum interval.** No
250us bound,100ppm bound, wire format, baud or physical verdict is changed.

The no-Webots worker sends a synthetic neutral 1 kHz source. Only timing,
released buttons and zero encoder endpoints are valid; optical, IMU and sonar
models are explicitly unavailable. OUTPUT is observed, not applied in physics.
SENSOR OUTPUT-ACK stays no-ACK. This is not a robot or navigation test.

Each fresh output directory contains `events.csv` and an atomic `summary.json`.
The CSV retains byte chunks, PC write-completion times, source captures with
conservative clock intervals, frame sequences/payloads and local inhibition.
A completed PC write is never called Pico endpoint availability. Queue growth,
missing/duplicate/reordered frames, invalid packets, stale clocks, late source
steps, nonzero Pico error counters or nonzero transport-profile PWM stop the
run and report FAIL. Local inhibition precedes exit work. A planned stop has a
bounded 5 ms nonblocking DISARM-write budget and a 100 ms report-drain budget.
The drain retains final active reports before the inhibited status. Write byte
counts never claim device delivery. Failed runs do not wait for this drain.
The port is then closed. Stop/drain time does not count as active soak time.

Reported latency distributions use integer-microsecond nearest-rank
p50/p95/p99 and maximum values. They cover SYNC round trip, OUTPUT source-age
upper bounds and report interarrival time. Source phase and offset uncertainty
are separate measurements. Drift is a conservative offset-change interval,
not proof of oscillator frequency. Default active-stream duration is 600 s;
the summary records whether that duration was reached. Logging overhead and
host scheduling remain part of the measured run, not hidden latency corrections.

The approved transport-only mode is an explicit opt-in:
`--transport-only-cap4`. It requests only the real-TIMER capability. The normal
host default still requires all frozen operational bits (`0x1f`). It cannot
arm a cap4-only image. The soak can retain an operator-supplied `--image-sha256`
and expected `--profile-flags`. Missing image/placement evidence cannot qualify
the transport subset. None of these options permits hardware access without
separate operator approval.

The candidate limits remain fixed: nominal pipeline 4 ms, source expiry 5 ms,
endpoint availability 3 ms, clock half-width 250 us and source phase 250 us.
Native first-C-entry jitter and logical apply jitter each have a candidate
250 us upper bound. A measured miss is FAIL and stops the run. No bound is
increased automatically. The 4 ms value is the fixed nominal schedule; actual
source-to-apply age is reported separately and must meet the 5 ms upper bound.

### Operator D-TRIAL: opt-in clock400us / SYNC20ms

See the [2026-11-14 operator decision](GATE1_D_TRIAL_20261114.md).
`--trial-clock-400-sync20` explicitly selects **TRIAL / NOT GATE 1 PASS** on the
C1 measurement baseline only, not C2/C3. Defaults remain250us/100ms. Source
phase250, native jitter250, exact upper/lower D1 age checks and3/4/5ms stay.
All trial callers must reject capture before the estimated midpoint target.
Keep the trial manifest with external binary captures. Trial completion cannot
be promoted to standard transport qualification or physical acceptance.

### Approved latency measurement, not a timing-bound change

Use `gate1_soak.py --measure-io` only for an approved measurement run. The
option adds RAW timestamps around write calls, select/read calls and successful
receive processing. Processing time includes CRC/decode **and** transport
validation; it is not a pure codec benchmark. Empty polls are not logged.
The option does not change UART bytes, tty flags, traffic, the adaptive100ms
maximum SYNC interval, the100ppm guard,250us clock limit or3/4/5ms bounds.
Logging and clock reads add host overhead. Compare with a separate
uninstrumented baseline; do not attribute all new delay to the Probe.
With `--measure-io`, C1 also retains a65536-row numeric tail in memory, with loop
IDs and RAW stamps for sleep entry/wake, read/receive, event drain, SYNC/tick,
capture, GC start/stop, CSV serialization and flush. It does not disable GC.
`attribution.csv` and `attribution.json` are saved after the serial stop/close
attempt. Metadata declares overwritten tail rows, nested callbacks not recorded
and the measured internal array-write bracket. That bracket excludes the first
clock call and counter maintenance; it is NOT the full observer cost or a hard
bound. Report whole-call microbenchmarks separately. The bounded tail is not
complete600s timing evidence.

A rejected capture retains its original computed start/end phase, exact excess,
clock intervals and limit250 in `capture-rejected.json` and a
`source_capture_rejected` event. These are written only after local inhibit,
the bounded DISARM attempt and serial close attempt. A failed close is declared,
not reported as success. An unqualified clock gives unknown phase, not an
invented number. Rejection does not queue a SENSOR or advance the source slot.
C1 changes measurement only: RX-first order, synchronous logging,50us sleep,
all gate values and stop behavior remain. Capture-first and queued logging need
separate changes and separate operator trials.

`pil/gate1_rx_timing.py` reads the stopped `PILG1RX1` block. It checks its
bounds, canonical order, CRC, reserved fields and matching D1 identity/reason/
first-stop trace. Native TIMER differences use modulo32-bit arithmetic.
Allocated but unfinished rows are valid and retain unknown timestamps. A
completed dispatch is **not** proof that a SENSOR was staged successfully.
The combined C underflow/clean fixtures and wrapped64-entry plus16-entry tail
fixture come from SW6 `3af60d18f05f82471a1364ad645b97a0e0ddd802`.
They are synthetic host tests, not hardware evidence. Decoded fixture hashes:
underflow `3b11631a8f561df600a8b1e784fedc243a7e5b3333207e9416971d482a110e04`;
clean `e27320790310ba1ff99b446a1d3f20c541595040e75093cfbd3404814909f1b3`;
ASCII `8701fc51749a4eaf686fa8da84d9ca0c7b7d3d233872026b40261ee6e9530a98`.
The binary fixtures are stored as `pil_gate1_rx1_*.hex.txt`; decode with
`bytes.fromhex()` before use. The released RX1 UP image is248320B,
SHA256 `71cf7cb2b3bd51fbd798a6d6c7a8e3d6543c0d9fd6ca129ac5d5eba1dc248b4a`.
A file hash is not proof that the operator installed that image.

`tools/gate1_correlate.py` combines PC evidence, optional full-snaplen classic
usbmon pcap and the complete stopped Pico CDC file. It uses only the Python
standard library. It supports USB pcap link types189/220, both byte orders,
and microsecond/nanosecond file timestamps. It selects the exact runtime bus,
device and CDC bulk endpoints. It pairs reused URB IDs, matches full payload
bytes and retains split-frame byte coverage. It can combine short full-payload
URBs to find a unique16-byte-or-longer association. Pending association storage
is limited to64 URBs/4096 bytes; ambiguity stays unknown. This is offline byte
association, not live UART resynchronization. A USB Request Block (URB) is a
host-controller transfer request; one URB can contain multiple USB packets.
Missing/short/failed transfers do not create complete-frame timing. A USB
completion is not a UART wire stamp.

The binary usbmon clock is REALTIME, not RAW. The reader checks that pcap
record timestamps match the usbmon header timestamps. RAW-bracketed REALTIME
samples give enclosing RAW **intervals**, not fitted/interpolated point times.
These intervals are explicitly conditional on no unobserved backward REALTIME
step. Missing enclosing samples, observed backward samples or timestamp-domain
mismatch leave mapping unknown. These mappings NEVER enter the Gate1 clock
estimator or change a verdict. SENSOR PC-write-to-Pico-RX bounds use retained
source-clock intervals with the unchanged100ppm guard, only with RAW and matching
D1 baud/UART proof. They never subtract clock midpoints. Native RX-to-worker,
decode, dispatch and alarm durations need no PC clock conversion.

The operator must resolve the Probe bus/device again immediately before each
capture and retain that result. Device numbers change after re-enumeration.
The operator verified CDC bulk OUT `0x02`, IN `0x83`; notification `0x81` and
CMSIS-DAP endpoints `0x04`/`0x85` are not UART data. Do not hard-code a previous
device number or use the notification interval as a bulk timing bound.

After authorization, use a second terminal for this **optional** capture.
`USB_BUS` must be the resolved Probe bus; `EVIDENCE` must be an existing absolute,
operator-owned fresh directory with mode0700. The wrapper runs in the foreground,
records its exact PID, then drops tcpdump to the operator UID. It creates and
changes ownership of `usbmon.stderr` before redirecting stderr, including when
called from a sudo shell. Do not redirect stderr outside the wrapper or reuse
an existing output. It requires the original non-root sudo identity. Stop it from the
run terminal only after the PC and stopped Pico captures finish. Check that the
PID still identifies this tcpdump command and this evidence path before sending
`kill -INT`. Do not stop other captures or use a process-name kill.

```sh
sudo bash sim/webots/tools/gate1_usbmon.sh "$USB_BUS" "$EVIDENCE"
```

If permission, module or interface access is unavailable, skip this optional
step. Do not change permissions on all usbmon devices. A bus capture can contain
other devices' data; keep it private and capture only the approved bus.
Wait for tcpdump to finish and retain its packet/drop statistics. Its drops,
short snapshots and unmatched transfers limit attribution. Do not read a pcap
while tcpdump is still writing it.

Add `--measure-io` to the approved soak command and keep the existing fresh Pico
replug watcher, separate binary CDC capture and exact final stack-line wait.
The RX1 image emits the block between D1 and that stack line. Example offline
report commands, after both captures close:

```sh
python3 sim/webots/tools/gate1_correlate.py --pc "$EVIDENCE/pc-soak" \\
  --stopped "$EVIDENCE/pico-usb.bin" --usbmon "$EVIDENCE/usb.pcap" \\
  --usb-bus "$USB_BUS" --usb-dev "$USB_DEV" --ep-out 0x02 --ep-in 0x83 \\
  --output "$EVIDENCE/latency.json"
# Without usbmon: omit its path and all USB selectors. URB layer stays UNKNOWN.
```

The attempt6 regression fixture `pil_gate1_attempt6_usb_rx.json` retains only
selected Probe payloads, PC RX bytes, clock brackets and the stopped stack line.
After65 discarded USB prefix bytes, the fresh chunks were1–15 bytes each. The
old single-URB16-byte association threshold never recovered them. Aggregation
now recovers all13 SYNC completions without changing the capture. The unchanged
stack line uses CRLF; the reader accepts LF or CRLF for that line only. RX1 rows
still require their exact LF envelope. These reader fixes do not change the
original underflow FAIL or turn close-time USB cancellations into UART errors.

The tool preserves the PC verdict and stopped D1 fault reason. Missing RX1 is
INCOMPLETE diagnostic evidence, not a new transport verdict. Report generation
exit0 means only that the JSON was written. It cannot issue physical Gate1 PASS.

### Stopped D1 evidence

The SW6 record `docs/simulation/PIL_GATE1_DIAGNOSTICS.md` owns the exact approved
D1 schema. D1 is a separate stopped USB stream, not a UART v1 extension. Its
132-byte maximum decoded body uses a dedicated reader. UART v1 stays at 128
bytes with unchanged types, fields and fixtures. No live USB evidence load is
added. A later operator capture must retain the Pico USB bytes as a regular
file; the offline reporter never opens a serial port.

```sh
python3 sim/webots/tools/gate1_report.py \
  --pc-run build/webots/gate1-run \
  --usb-dump build/webots/gate1-stopped-usb.bin \
  --output build/webots/gate1-correlated.json
```

The active cohort is the consecutive group of armed transport frames before
planned DISARM. The reader requires exact record order, identity, CRCs, final
chain CRC, all 12 metric records and complete cohort counts. Identical complete repeated dumps
are permitted; changed or partial snapshots cannot be combined. Optional trace
records remain partial diagnostics, never proof of all-frame coverage.

Every PC capture brackets the actual neutral fixture capture. Stored Pico-time
intervals include then-valid clock uncertainty, drift and outward timestamp
rounding. Clock SYNC rounding is separate from drift expansion. For source n,
let B be its nominal capture time and [lo,hi] its conservative captured interval.
The reader computes H=max(B-lo) and L=max(hi-B) over the staged prefix.

| All-frame bound | Calculation |
| --- | --- |
| Availability upper age | max(Ahi-B) + 1 us + H |
| Availability lower age | min(Alo-B) - L |
| Actual apply upper age | max(Phi-B) + 1 us + H |
| Actual apply lower age | min(Plo-B) - L |

A/P are the Pico's before/after publication/apply TIMER brackets. The extra
1 us expands the Pico upper read exactly once. Device extrema do not already
contain it. The reporter also checks endpoint slack, both jitter limits,
chronology, the contiguous staged/applied prefixes and every active OUTPUT
ACK. It requires operator image/placement metadata, valid clock/source records,
planned-stop inhibition and a full ten-minute cohort before it can qualify even
the transport subset. Bad or absent evidence remains INCOMPLETE; a measured
limit violation, reported Pico fault or proved OUTPUT loss is FAIL.

Device availability/apply/jitter p50/p95/p99 remain UNMEASURED because D1 retains
all-event extrema, not every device timestamp. No percentiles are inferred from
OUTPUT ACKs, PC writes or the small optional trace. A successful host/fake run
and even a qualified transport subset remain overall **INCOMPLETE**, never a
full physical Gate 1 PASS. The separate hardware fault campaigns and later
functional sensor gates are not closed by these tools.

The real C D1 reducer/encoder/chain producer was compiled and checked on the
host with `tools/check_d1_interop.py` against committed SW6
`0752fdf2a8c0fc3c92e84d4408c6c466e204a36d`. The tool verified its C source and
headers against that revision before compilation. Its synthetic fixture has 600004 staged
frames, 600001 applied frames and three cancelled tail frames across TIMER
wrap. With declared synthetic capture intervals B +/-5 us, independent PC bounds
are availability [995,1008] us, actual apply [4015,4030] us, endpoint slack at
least1997 us, apply jitter at most25 us and native first-C-entry jitter at
most11 us. Fixture SHA256 is
`0407811d725221b58321478c66584655a0eba701447382d126f8d16ea6634d40`.
The full source hashes and result remain under
`build/webots/gate1-d1-c-interop-0752fdf/` and
`build/wip/pil_plan/gate1-d1-c-interop-0752fdf.log`. These are **synthetic host checks**,
not Pico/Probe measurements or a ten-minute hardware result.

### Pre-flash no-device dry-run

Run the host tests first, then use a fresh output directory:

```sh
PYTHONPATH=sim/webots python3 -m unittest discover -s sim/webots/tests -v
python3 sim/webots/tools/gate1_dry_run.py \
  --fixture sim/webots/tests/fixtures/pil_gate1_cap4_synthetic_replay.json \
  --output build/webots/preflash-dry-run
```

This path imports no serial adapter and opens no device. It uses a committed,
explicitly SYNTHETIC host recording, not saved Pico/Probe measurements. It feeds
158 recorded RX chunks through the real worker and requires exact byte/time
matches for all58 PC writes. The fake clock accelerates a40ms active source;
it does not measure laptop pacing or prove a ten-minute physical run.

The first negotiated OUTPUT is inhibited (`0x04`), then pending after ARM
(`0x14`), then ARMED (`0x01`, first SENSOR ACK0), then inhibited after DISARM.
The fixture generator and tests retain this sequence without inventing physics.
To regenerate the reference intentionally, use
`PYTHONPATH=sim/webots python3 sim/webots/tests/generate_gate1_replay_fixture.py`.
Review any byte changes; do not regenerate just to hide a regression.

Expected dry-run result: CLI exit0, `host_fixture_match: true`,
`soak_verdict: INCOMPLETE`, `physical_gate1_pass: false` and
`serial_device_opened: false`. HOST DRY-RUN PASS means only agreement with the
software recording. Retain `events.csv`, `summary.json` and `dry-run.json`.
A mismatch fails the host check. No hardware authorization flag is used.

Validation:157 host tests OK, including seven replay checks, with the same
three expected S3 model failures. The recorded-soak CLI matched all chunks.
Logs are `build/wip/pil_plan/home-preflash-host-tests.log` and
`build/wip/pil_plan/home-dry-run-validation.log`. No serial discovery/open,
Webots launch, Pico image build or flash was performed for these checks.

## Remaining work and hardware gate

The Webots preview is not connected to `HostTransport` or `MotorPlant`; runtime
integration needs separate approval after host review and transport Gate1.
The current SW4 GP28 optics is a slow offline reference with recorded-model
failures preserved; it is not a1kHz backend. GP26/GP27 profiles are unmeasured
and deliberately unavailable. Do not substitute GP28 calibration or invented
ADC defaults. This slice does not claim a full calibrated sensor simulation.

The scenario runner and firmware-evidence evaluator remain later work.
Scenario verdicts must use only firmware MQTT/USB events and declared expected
sequences/counts/reasons/bounds. Simulator pose, optical truth or contact cannot
manufacture PASS. Keep simulator diagnostics separate. Default live actions stay
disabled; optical source2 is SHADOW. No SIM approval authorizes physical actions.

Gate1 waits for the operator and hardware. It must measure actual Probe baud,
full-duplex load, source/apply ages and uncertainty, native alarm jitter, queue
fill, clock drift, maximum—not only percentile—latency, sustained10-minute runs,
pause/unplug/fault/wrap and reset/no-auto-rearm. Stop and report a missed bound;
do not enlarge4/5ms limits silently. No hardware gate is closed by Stage0 tests.
