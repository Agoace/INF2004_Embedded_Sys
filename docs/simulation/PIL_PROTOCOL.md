# PIL protocol v1

Status: frozen Stage 0 contract; software tests only. No real transport,
UART, edge scheduler or IMU register model is implemented by Stage 0.
Gate 1 requires the Debug Probe and spare Pico. Do not flash Stage 0 images.

PIL means processor-in-the-loop: Webots models the course; a spare Pico runs
the real kernel. The implementation branches start at `451e562`. SW6 owns this
contract and the C codec; SW5 owns the PC bridge. Changes to the frozen contract
require both codecs, fixtures, this record and review before use.

## Link and scope

UART0, 921600 baud, 8N1, no flow control. GP0 Pico TX connects to Probe U RX;
GP1 Pico RX connects to U TX; connect common GND, not power. UART is binary only.
USB CDC console remains separate. `SIM=1` forbids UART console/crash mirroring.
Pure simulated GPIO pending edges only: no PIO or additional hardware.

`drv_time_hw.c`, kernel clocks and watchdog stay real. Six SIM skeletons replace
motor, line ADC, barcode buttons, IMU, ultrasonic and GPIO hardware adapters.
They are fail-closed, not functional sensor emulators. Real line alarm 0 remains.
No native controller/decoder/source file or kernel/template source is changed.

## Envelope and constants

`COBS(header || payload || CRC32C) || 00`. COBS removes zero bytes inside a
packet; zero terminates it. All integers are explicitly little-endian, signed
integers use two's complement. Never send a native C struct.

CRC32C: reflected polynomial `82f63b78`, initial/final XOR `ffffffff`;
ASCII `123456789` gives `e3069283`. CRC covers header and payload; store it LE.
Maximum decoded body 128 B, payload 104 B, encoded packet including delimiter
130 B. A decoder clears a partial packet after an oversize packet/delimiter;
it must not scan arbitrary byte offsets inside a failed packet for a command.

Schema ID `50494c01`; version 1; step 1000 us; pipeline 4000 us; source expiry
5000 us; queue eight slots; session maximum 1800000 ms (30 minutes).
`ffffffff` is the no-ACK sentinel. Per-type sequences start at zero and wrap.
Capability bits: 0 coherent ADC/encoder, 1 LSM303 register interface,
2 real TIMER, 3 locally timed sonar, 4 pure simulated GPIO; mask `0000001f`.
These are interface capabilities, not hardware acceptance. Stage 0 skeletons
must not announce operational capabilities or ARMED. Unknown bits are rejected.

Header (20 B):

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u8 | version, 1 |
| 1 | u8 | type, below |
| 2 | u16 | exact payload length |
| 4 | u32 | nonzero host session; BOOT uses zero |
| 8 | u32 | sequence in this type's stream |
| 12 | u32 | source time in this type's clock domain |
| 16 | u32 | peer ACK in the stream specified below |

A full decoded frame must pass version, type, length, reserved, range and CRC
checks before publication. Unknown types/flags/ranges reject the whole frame.
Malformed isolated frames increment a counter and do not refresh progress;
a missing required slot subsequently latches the timing fault. No partial apply.

## Types and control payloads

All offsets here and below are payload-relative. Every control integer is u32
unless stated otherwise. Every session repeated in a payload must equal the
header. Boot nonce and host session are nonzero observation labels, not security
credentials. A fresh boot nonce is required after reset; stale data cannot arm.

| Type | Name | Payload | Source clock | Header ACK |
| --- | --- | --- | --- | --- |
| 0 | BOOT | 16 B | Pico TIMER | no-ACK |
| 1 | SENSOR | 40 B | model elapsed us | last OUTPUT applied in physics |
| 2 | OUTPUT | 32 B | Pico snapshot TIMER | last SENSOR applied locally |
| 3 | HELLO | 24 B | PC monotonic low32 us | last BOOT received |
| 4 | HELLO_ACK | 28 B | Pico TIMER | accepted HELLO sequence |
| 5 | ARM | 8 B | PC monotonic low32 us | latest HELLO_ACK received |
| 6 | DISARM | 4 B | PC monotonic low32 us | last OUTPUT received or no-ACK |
| 7 | SYNC_REQ | 8 B | PC monotonic low32 us | last SYNC_REPLY or no-ACK |
| 8 | SYNC_REPLY | 16 B | Pico transmit snapshot TIMER | accepted SYNC_REQ |

- BOOT: nonce@0, capabilities@4, baud=921600@8, schema@12. Send at 10 Hz while
  unarmed; no BOOT after ARM_PENDING. No session is established by BOOT alone.
- HELLO: boot_nonce@0, host_session@4, required_caps@8, step=1000@12,
  pipeline=4000@16, expiry=5000@20. It is accepted only while unarmed, for the
  current boot nonce/schema and supported capabilities. Unsupported requests
  receive no ACK, not a false successful negotiation.
- HELLO_ACK: echoes the six accepted HELLO words and apply_epoch@24. Source time
  is ACK generation time. Epoch is at least 100 ms after generation, rounded UP
  to the native alarm-0 grid. Repeated unarmed HELLO probes may replace the
  provisional epoch. ARM must reference the latest ACK; it fixes that epoch.
- ARM: boot_nonce@0, session@4. It enters ARM_PENDING, not immediate ARMED.
  Matching nonce/session/ACK, zero encoder origin and neutral buttons are
  required. Frames 0 and 1 must be valid/staged before epoch, with each later
  endpoint meeting its lookahead deadline. Failure at the fixed epoch latches.
  PC infers acceptance ONLY from a matching OUTPUT with ARMED and without
  LINKFAULT/INHIBITED, ACKing SENSOR 0 or later. Sending ARM is not acceptance.
- DISARM: reason@0, 1 user, 2 pause, 3 exit, 4 clock, 5 input, 6 output lease,
  7 model fault, 8 reset. A syntactically valid DISARM is always accepted in
  every state, including a stale/zero session; stopping must not depend on a
  live session. It never arms, clears a latch or changes the epoch. Repetition
  is harmless. Reset/new session is required before another run.
- SYNC_REQ: PC monotonic transmit us, u64@0. Header time is its low32. Accepted
  for the established session while unarmed, pending or armed; it is diagnostic,
  not an actuator control and never renews a lease or changes the epoch.
- SYNC_REPLY: echoed PC transmit u64@0, Pico full-request receive TIMER@8,
  reserved zero@12; header source is Pico response generation/transmit snapshot.
  Capture before queuing response bytes; retain actual queue latency in bounds.

Duplicate/reordered controls do not change state or renew progress. DISARM is
the only harmless repetition exception. Use unsigned difference `d=a-b`:
`a` is newer iff `0<d<80000000`. No comparison spans half a counter period.

## SENSOR, type 1, 40 B

| Offset | Type | Field |
| --- | --- | --- |
| 0,2,4 | u16 each | L GP27, C GP26, barcode GP28 ADC |
| 6 | u8 | pressed GP20 bit0, GP21 bit1; other bits zero |
| 7 | u8 | echo mode/fault flags |
| 8,12 | i32 each | normalized L/R encoder totals |
| 16 | i16 x3 | accel register words X,Y,Z |
| 22 | i16 x3 | mag raw counts X,Y,Z |
| 28,30 | u16 each | echo rise delay and HIGH width, us |
| 32,34 | u16 each | echo servo generation and pulse, us |
| 36 | u16 | validity/NEW mask |
| 38 | u16 | missed model steps, saturating, not wrapping |

missed_steps counts model step boundaries omitted since the preceding emitted
SENSOR (zero for the initial frame), saturating at65535. It is not a wrapping
lifetime counter; any nonzero value makes the run invalid.

SENSOR sequence n is exactly elapsed model time/1000; first n=0, time=0. No
missing steps may be replaced by replayed observations. The 30-minute bound
keeps source time below half-range. An unchanged timestamp with a new sequence
is invalid. Pico TIMER may wrap; unwrap near the negotiated epoch, and use
bounded modulo differences. Test wrap with epoch near `ffffffff`.

ADC ranges 0..4095 even if invalid; black HIGH. Bridge sends ONE post-eight-read
average, not eight independent invented samples; Pico does not average again.
Validity means the declared model/material/geometry profile supports evaluation,
not merely bytes arrived or that physical calibration passed. Unapproved unknown
geometry clears validity. Approved assumed geometry remains explicitly labeled
in run metadata; no truth/diagnostic pose enters firmware verdict inputs.

Encoder totals are modulo32, forward-positive for BOTH wheels, 2535 quadrature
counts/revolution with fractional accumulation. Pico converts right logical
forward to raw negative. Start both at zero. At most 16 counts per wheel per
1 ms slot; greater bounded-difference magnitude is a timing/model fault, not an
unbounded edge loop. Interpolate transitions uniformly over the PRECEDING slot
and reach the endpoint before ADC. Do not assign native counts or replay edges.

Fault byte: bits0..1 echo mode NORMAL=0/NO_RISE=1/STUCK_HIGH=2/HIGH_NO_FALL=3;
bit2 NACK, bit3 timeout, bit4 wrong identity, bit5 bad configuration readback;
bits6..7 zero. Timeout overrides NACK, which overrides normal bus operations.
Identity/config faults affect those register reads, not fake attitude outputs.
Timeout/NACK suppress incoming IMU NEW samples; identity/config faults are
configuration-test injections and do not alter sensor ODR by themselves.

Validity bits: 0/1/2 ADC L/C/barcode, 3/4 encoder L/R, 5 accel NEW, 6 mag NEW,
7 accel VALID, 8 mag VALID, 9 sonar cache VALID, 10 buttons VALID,
11 timing VALID, 12 sonar NEW; bits13..15 zero. NEW requires corresponding VALID
and the specified phase; otherwise reject. Deliberately invalid channels invoke
native sensor failure paths, not a transport fault by themselves. Timing-invalid
or missed_steps!=0 latches the whole-slot fault. ADC native read fails if any
ADC channel invalid. Invalid encoder emits no edges, exposing native stall.
Invalid buttons mean released. Bus faults override VALID/NEW, never yield data.

Accel HR12 saturates [-2048,2047] then multiplies by 16, so low4 bits are zero.
Default +/-2g gain is 1 mg per HR12 count. Sensor axes +X right,+Y forward,+Z up;
stationary level Z=+1000mg. Mag signed raw range [-2048,2047] except -4096 invalid
sentinel; gains 1100 XY/980 Z counts/gauss. Sentinel requires mag VALID/NEW clear.
Pico presents magnetometer register order big-endian X,Z,Y. Field strength is a
declared physical/model parameter, not an unscaled compass unit vector.

### IMU phase and source identity

For sample k>=0, accel time is 2500*k us; mag time is EXACT rational
40000*k/3 us. Both first samples are at zero. Frame n contains NEW iff a sample
is due in ((n-1)*1000,n*1000], with n=0 specially including zero. There is at
most one sample/device/frame. Use k=floor(400*S/1000000) or
floor(75*S/1000000), with wide integer intermediates; do not accumulate rounded
periods. For records, mag timestamp is floor of the rational time; interpolation
uses the exact rational time before rounding.

PC linearly interpolates bracketing 1 ms source states, then quantizes nearest,
ties away from zero, then saturates. This is an explicit model approximation,
not exact substep physics. FIFO insertion time is original source phase shifted
by epoch, not packet arrival. Power-down/bypass resets discard before their
cutoff; no data is fabricated while disabled. Lost NEW samples are never
reconstructed as measured samples. Due, valid, non-bus-faulted IMU requires NEW;
missing population is an invalid run, not a complete 400/75Hz data stream.

### Sonar cache

Only sonar NEW replaces cache and its ORIGINAL source timestamp. Repeated cache
fields with NEW clear never refresh age. NEW requires VALID and pulse1100..1900.
Generation increments on effective pulse CHANGE only, initially zero with
initial initialized pulse; uint16 wrap is allowed. Check BOTH pulse and generation.
Changing generation invalidates the previous cache immediately. A matching tag
is not proof of physical angle: PC models mechanical slew and evaluates rays at
actual modeled pose. Source time is the frame with sonar NEW, including pipeline.

At actual trigger, require cache age upper bound<=5000us and matching tag;
otherwise no echo edges, so native failure logic acts. Latch descriptor at pulse
start: later cache updates cannot alter an in-flight echo. A servo change during
an in-flight echo cancels remaining synthetic edges; no old-angle valid result.
NORMAL requires rise delay1..1999us, HIGH1..60000us. Width>3000 yields native
NO_RETURN; about38000us models no-object HC-SR04+ HIGH followed by a late fall.
NO_RISE requires delay/width zero and stays low. STUCK_HIGH requires zero fields,
sets HIGH before trigger, so native driver emits no pulse. HIGH_NO_FALL requires
delay1..1999,width0; rises then remains HIGH until reset. Invalid cache overrides
all modes: low/no new edges, never a fabricated return. Strict native thresholds
remain 2000us rise,3000us high,60000us trigger gap and511mm field limit.

## OUTPUT, type 2, 32 B

| Offset | Type | Field |
| --- | --- | --- |
| 0,2 | i16 each | effective signed L/R PWM compares |
| 4 | u16 | effective servo pulse, us |
| 6 | u8 | applied buttons, bits0..1 |
| 7 | u8 | state flags |
| 8 | u16 | servo generation |
| 10 | u8 | input queue depth0..8 |
| 11 | u8 | reserved zero |
| 12 | u32 | last actual actuator-write Pico TIMER |
| 16,20 | u32 each | actual trigger pulse sequence/start TIMER |
| 24,26,28,30 | u16 each | dropped inputs,bad frames,missed schedules,UART overruns |

PWM range [-6250,6250], using same integer compare quantization as real driver
wrap6249; duty=compare/6250, NOT requested speed/ideal permille. Profile-specific
controller limits remain enforced by the real controller; the wire range does
not authorize a larger drive. Servo1100..1900; zero only uninitialized AND
inhibited. Counter fields wrap modulo65536; missed_steps in SENSOR saturates.

State bits: 0 ARMED,1 LINKFAULT,2 INHIBITED,3 INPUT_UNDERFLOW,4 ARM_PENDING;
other bits zero. ARMED cannot coexist with LINKFAULT/INHIBITED/PENDING.
UNDERFLOW requires LINKFAULT+INHIBITED. PENDING requires INHIBITED. Effective
PWM must be zero when not ARMED. Servo may retain its initialized pulse while
inhibited; that is not permission to emit a physical PWM on the spare Pico.

Dropped inputs counts complete, syntactically valid SENSOR frames refused by
session/sequence/staging/deadline policy (including duplicates). Bad frames
counts envelope/version/type/length/CRC/range failures; do not count the same
packet in both counters. Missed schedules counts demanded1ms apply slots that
could not execute coherently. UART overruns counts observed-and-cleared hardware
overrun indications, not an invented number of lost bytes. All four wrap.

OUTPUT seq and snapshot TIMER progress every new 1 kHz report. Actuator-write
TIMER may legitimately stay unchanged between real10ms controller updates and
must NOT renew the output lease. Trigger seq counts actual10us pulses, not
failed native calls that find STUCK_HIGH before issuing a pulse.

## Fixed timeline, clocks and lifecycle

For source/model S and fixed Pico epoch E, modulo32:

- apply(S)=E+S, on the native alarm0 grid.
- intended source capture=apply(S)-4000us, INCLUDING transport.
- fully decoded/CRC-checked/staged endpoint deadline=apply(S)-1000us.
- generic source expiry=apply(S)+1000us, i.e.5000us source age.

The endpoint is required for its PRECEDING1ms edge interval. Generic expiry is
not permission to miss the stricter <=3000us source-to-availability gate. Both
slot deadline and conservative original-source-age bound must pass. InitialS0
is neutral baseline, not invented past edges. At least frames0/1 before epoch;
no precomputing future measurements and pretending they were captured earlier.

Apply the entire ADC/encoder instant atomically. Do not delay ADC alone while
injecting current counts. Native read/ISR times stay real; log separately from
measurement/arrival/apply. On any missing/late/overflow/timing-invalid slot,
latch all inputs unavailable and output PWM zero immediately, not after5ms.
No queue eviction of required slots; a duplicate is ignored, overflow latches.
All latches require Pico reset and a new nonce/session. No auto-rearm.

The separate 5000us no-progress leases use progressing SENSOR source+seq / OUTPUT
snapshot+seq. CRC-valid repetitions do not renew them. PC inhibits MODEL motors
on missing output independently of Pico-reported PWM; log both distinctly.
If physics is paused/blocked, inhibit before the next physics step. Best-effort
DISARM must not delay local inhibition. Button key pulses are100ms minimum;
no auto-repeat, rearm or synthetic press on reconnect. Serial workers never call
Webots APIs. No unbounded catch-up physics, virtual Pico clock or epoch sliding.

### Clock qualification (candidate hard gates, NOT measurements)

SYNC has PC t1 transmit/t4 complete receive, Pico t2 complete-request receive/
t3 pre-response snapshot. Offset=Pico-PC lies in [t3-t4,t2-t1]. Intersect eight
observations after expanding older bounds for100ppm drift; do not use statistical
sqrt(N) reduction. Proven minimum per-direction serialization may tighten bounds:
use actual wire length and a conservative maximum baud1.02*921600; floor minimum
wire duration. Never subtract guessed USB/queue delays. PC clock is monotonic.

Require conservative offset half-width<=250us, INCLUDING aging, and source-step
boundary phase error<=250us. Combine uncertainty+pacing errors when bounding
actual source/available/apply/cache age; never call the sum250us. Refresh within
1s AND before headroom expires: halfwidth grows100us/s. A150us fresh bound can
last at most1s; a250us bound has no remaining headroom. Empty/too-wide intervals,
stale/missing sync and frequency-assumption violations fault. PC may slowly
correct pacing to its clock map; it never changes E or the4ms schedule.

ACK reception must retain at least20ms setup lead under conservative clock
bounds. If not, do not ARM. Clock/bootstrap timing is tested with synthetic
clocks in Stage0, not represented as a working physical link.

## Stage0 / Gate1 proof boundary

The stateless C API is [pil_wire.h](../../app_program/sim/pil_wire.h).
[Shared fixtures](../../tests/fixtures/pil_wire_v1.json) include decoded bytes,
COBS packets and accepted/rejected cases. The independent reference generator
is [generate_pil_fixtures.py](../../tests/generate_pil_fixtures.py) (`--check`).
Syntax/range validation is not session, lease, clock or sensor emulation.

Stage0: C codec + byte-exact fixtures + PC interoperability, malformed/boundary/
clock-failure host tests, six fail-closed skeletons and SIM Make selection.
Non-SIM ELF/UF2 must be byte-identical to451e562 with the same compiler, SDK,
source/build paths and public settings across all38 documented profiles.
No runtime UART, edge or register model and no hardware run is implied.

Gate1 later: actual Probe/Pico full-duplex1kHz qualification. Candidate hard
bounds above must pass, including maxima (not only p99), sustained10-minute run,
source/Pico clock drift, <=3ms availability, aligned ADC/encoder endpoints,
queue occupancy0..8, source/cache age<=5ms, real alarm wake jitter, preload,
wrap, deliberate pause/unplug and reset/no-auto-rearm. No timing-fault-free claim
without recording uncertainty and raw extrema. Stop/report a miss; never enlarge
4/5ms limits silently. UART rate alone is not a latency guarantee. SIM CPU or
host tests are not acceptance of physical sensors, bus load or real robot timing.
