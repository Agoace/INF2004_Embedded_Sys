# Gate 1 — host-only implementation plan

Main approved this plan on base9831991. Work is in progress.
No flash, serial access or hardware timing test is authorized.

## Current checkpoint

The reducer, D1 codec, neutral session state machine and bounded packet stream
are supplied. There is NO Pico UART runtime, new task or Gate1 image yet.
Do not flash Stage0 skeleton images. The sections below describe approved
remaining work, not a claim that a physical link runs.

The [D1 diagnostic contract](PIL_GATE1_DIAGNOSTICS.md) is normative.
The frozen [UART protocol](PIL_PROTOCOL.md) is unchanged.

## Build and scope

Add PIL_GATE1=1 as an explicit SIM=1-only image, suffix _sim_gate1 and separate
profile invalidation key. Default/unset adds no flags, names or source changes
to the 38 ordinary profiles. SIM=1 without PIL_GATE1 keeps Stage 0 skeletons.
Require explicit WIFI=none NET=none MQTT=0 CONSOLE=usb_cdc CRASH_REPORT=0.
Reject TEST/BENCH/DIAG/IMU_STREAM/BARCODE_CAPTURE/live/fake-command selections.
Start with UP; compile UP, split and forced-core0 gate-1 images for link checks.
No template source edit, SDK startup, SDK IRQ/time runtime or PIO.
Native kernel time, line alarm0 and motion watchdog remain real.

A SIM-only --wrap=usermain boot wrapper initializes time/nonzero boot entropy,
creates a static core-0 transport task, then calls the unchanged real usermain.
The worker must acquire completed native startup and a valid alarm0 grid before
BOOT/HELLO. Failure to initialize stays inhibited. It must not create a second
motion watchdog or bypass native watchdog feeding. New task stack is separately
painted/reported; no modification of the native app_task registry is needed.

IMPORTANT ownership correction: startup currently initializes the sample sera
UART0 driver even with USB-only console. Propose an app-owned config include
ONLY for gate-1 devinit.o: include original config_device.h, then override
DEVCNF_USE_SER to 0. Preserve ADC/IIC and every other option. No template file
changes; do not silently replace a live sample-driver interrupt handler.
This build-only override needs Main approval as part of this plan.

Gate-1 BOOT advertises ONLY real TIMER capability bit2 = 0x4, not 0x1f.
SW8 adds an explicit transport-only required_caps=4 option. The normal robot
HostTransport default remains 0x1f and cannot arm this image as a full simulator.
Neutral SENSOR has validity 0xC18 (timing/buttons/zero encoder endpoints);
ADC/IMU/sonar are intentionally invalid. No edges or register model are built.
No physics exists: SENSOR peer ACK remains NO_ACK; PC records OUTPUT observed,
never physics_applied. Pico must not require an invented physics ACK.
ARMED in this explicitly labelled profile means only the neutral transport
schedule is running, not sensor/robot qualification or physical PWM permission.
Native SIM motor callbacks can retain the actual software-effective neutral
compare/servo/write timestamp with real quantization; no physical PWM is emitted.
Reject/latch unexpected non-neutral motion in this diagnostic-only run. Trigger
sequence stays zero because the skeleton emits no actual trigger pulses.

## Proposed SW6 files

- build_make/pico_rp2040.mk: option validation, guarded defines, suffix, profile
  key and gate-1-only link wrap; radio/network prohibition.
- build_make/mtkernel_3/app_program/subdir.mk: gate-1 source selection and the
  two per-object flags below; no normal-source reordering.
- app_program/sim/pil_gate1_device_config.h: the startup serial override above.
- app_program/sim/pil_gate1_session.[ch]: pure state machine, eight stage slots,
  per-type sequence admission, fixed epoch, leases and fail-closed transitions.
- app_program/sim/pil_gate1_stream.[ch]: bounded packet RX/TX queues, delimiters,
  oversize discard-to-next-delimiter, no scanning for embedded commands.
- app_program/sim/pil_gate1_uart_hw.[ch]: SDK-headers-only UART0 GP0/GP1 registers,
  FIFO, baud divisor from real kernel peripheral clock, no clock retuning.
- app_program/sim/pil_gate1_task.c: kernel IRQ registration, core-0 static task,
  wrapper/startup, work bounds, real TIMER1 progress-lease alarm and cleanup.
- app_program/sim/pil_gate1_line_time.c: returns real drv_time_us and retains the
  native module's last read for the exact original first-C-entry snapshot.
- app_program/sim/pil_gate1_metrics.[ch]: all-event reductions and bounded trace.
- app_program/sim/pil_gate1_dump.[ch]: stopped-only USB diagnostic serialization.
- app_program/subsystem3/drv_line_sim.c: gate-1-only alarm/ack hooks.
- app_program/subsystem2/drv_motor_sim.c: gate-1-only neutral effective-output
  observation; physical behavior remains disabled.
- tests/test_pil_gate1_*.c, tests/pil_checks.cmake and tests/test_pil_make.py:
  production-source host tests and register/kernel doubles.
- docs/simulation/PIL_GATE1.md and PIL_GATE1_DIAGNOSTICS.md, map and index:
  approved scope, tests, exact USB schema and later operator procedure.
No existing native application C source is edited. pil_wire v1 is unchanged.

Second per-object flag: ONLY gate-1 drv_line.o gets
-Ddrv_time_us=pil_gate1_line_time_us. This keeps the native algorithm/source
unchanged while its original first entry read is still a real TIMER read.
The immediately following drv_line_hw_ack hook observes that retained value
before the module's next time read. Capture is native FIRST-C-ENTRY, not the
hardware IRQ assertion instant. Host tests must prove this call ordering and
that all native time reads still return real time. No instrumented object may
survive a gate-1 -> ordinary profile switch.

## Runtime and fail-closed behavior

Kernel registers UART0 IRQ20 on core0; hardware adapter registers no IRQ itself.
Bound RX/TX FIFO work per interrupt; no CRC, printf, polling wait or packet encode
on the IRQ stack. RX stores complete delimited packets plus full-request receive
TIMER and physical error flags in a bounded queue. Worker validates CRC/ranges
before any state mutation. Oversize/failed packets cannot refresh progress.
SYNC t2 is the IRQ complete-delimiter receive snapshot; t3 is taken before reply
encoding/queuing. Queue/serialization delay remains in the conservative bounds.
Hardware overrun counts observed-and-cleared indications, never guessed bytes.
Raw queue loss has its own diagnostic counter, not a fake hardware-overrun count.

HELLO supports only cap4 and frozen constants. ACK epoch >=100ms after generation,
rounded up relative to the native alarm0 grid, including TIMER wrap. ARM binds
latest ACK/nonce/session and enters pending. Require neutral frames0/1 at epoch
and every subsequent endpoint by its preceding-interval deadline. Eight slots,
no required-slot eviction, no compressed replay or epoch shift. Duplicates do
not renew progress; drop counts follow frozen-v1 definitions. Timing-invalid,
missing, late, overflow, excessive encoder delta or expired source latches all
inputs unavailable and effective PWM zero. DISARM is accepted with any session
when syntactically valid; it never clears a latch. No auto-rearm or nonce change.

Native alarm0 hook demands each logical slot and its next endpoint, atomically
consumes a staged neutral frame, and publishes an OUTPUT snapshot to a bounded
worker queue at1kHz. It does NOT inject ADC/encoder observations. Distinguish
native ADC failures (expected skeleton behavior) from PIL missed-schedule faults.
TIMER1 is reserved for the independent 5000us no-progress safety deadline;
do not silently add a1ms polling grace or expire pending ARM before its fixed
initial source window. Missing lookahead always faults earlier when applicable.
Register/clean up TIMER1 with kernel APIs on core0. No other alarm user was found
in current application sources; recheck ownership during implementation.

## RAM, stack and software evidence

No heap allocation for transport, queues, metrics or task stack. Budget <=16KiB
new static RAM INCLUDING a2048B aligned painted worker stack: eight RX packets,
eight TX packets, eight SENSOR slots, eight OUTPUT work snapshots,64x64B trace,
12 extrema and fixed scratch. Publish actual ARM map delta and sizeof manifest.
Task buffers own full words; core0 IRQ/task handoff uses short saved-IRQ sections
and release/acquire ordering. No packed cross-core RMW or cross-core DI fiction.

Target <=256B additional IRQ call chain (not a claim before ARM inspection).
Decode's existing288B static frame stays on the worker task, never an IRQ.
Use strict ARM stack-usage plus call-chain review, GCC and Clang ASan/UBSan host
seams for the REAL production session/stream code. Hardware stack watermark,
CPU cost, IRQ-off duration and native jitter are Not run until separate release.

Tests: all stream split points/coalescing/noise/oversize/delimiter recovery;
CRC-valid bad controls and stale sessions; DISARM in every state; repeated
HELLO/new epoch, stale ACK/ARM; neutral0/1 missing/late;8/9-slot admission;
duplicate/gap/overflow; exact deadline/expiry ties; no duplicate lease renewal;
TIMER/sequence wrap and30-minute limit; injected IRQ/task publication order;
UART FIFO/error indication/baud/IRQ ownership; initialization-failure cleanup;
clock t2/t3 capture order; metrics vs brute-force EVERY-frame host oracle;
aggregate H/L bounds and quantization; dump corruption/truncation/gaps/identity;
normal-required-cap1f rejection. Retain full C547 suite and SW8 interoperability.

Rebuild all38 ordinary profiles at the SAME public source/build path, compiler,
SDK and public example settings as451e562 and9831991. Compare all76 literal
ELF/UF2 hashes including debug metadata; zero exceptions. Keep SDK proofs and
zero warnings. Build the3 gate-1 placement profiles and retain symbol/driver
proofs of no Wi-Fi/MQTT/SDK runtime or physical PWM. Exercise transitions among
SIM0,Stage0 SIM1,Gate1 SIM1 and back to SIM0. Docs/template8/diff checks required.

## Later hardware pass/fail (NOT authorized now)

With operator-selected Debug Probe + spare Pico and separate Pico CDC evidence:
>=10-minute full-duplex1kHz neutral run, exact record coverage and no unwanted
radio/network identity; all frozen handshake/preload/session/reset semantics;
conservative clock halfwidth<=250us including aging, capture phase<=250us,
refresh<=1s AND headroom, >=20ms ACK setup lead, fixed nominal4000us schedule;
availability upper<=3000us AND preceding-slot deadline; source-age upper<=5000us;
no required gaps/eviction/overrun/missed schedule or unrecorded queue overflow.
Report actual source->apply min/max bounds separately from nominal4ms.
5000us expiry NEVER excuses missed3000us availability. No threshold is raised.

Main approved4ms as the fixed nominal schedule, not exact actual age.
Actual source-to-apply bounds must be reported separately. Native alarm/apply
jitter has a CANDIDATE250us threshold, to revisit only after P1 hardware data;
it must NEVER be raised silently. No claim of entire Gate1 PASS while required thresholds
or evidence are absent. Run deliberate pause/unplug/replay/CRC/overrun/reset and
no-auto-rearm campaigns separately from the clean soak. Report observed maxima,
uncertainty and raw counters, not just percentiles. Aligned ADC/encoder emulation,
IMU/sonar/cache behavior and robot timing remain outside this transport-only
image; they cannot acquire a functional acceptance label from this soak.

## Checkpoint 2 software evidence

Production session/stream tests were written RED first. Full GCC and Clang
ASan/UBSan each pass565/565. Tests cover10000 continuous slots, pending preload,
missing lookahead, exact late/lease boundaries, eight/nine slot capacity,
stale sessions, duplicate/no-progress renewal, SYNC timestamps, unconditional
DISARM, first-latch preservation, counter overflow, TIMER/queue-index wrap,
queue-full preservation and oversize discard through the next delimiter.
All storage remains caller-owned; a core0 caller must exclude IRQ access to
session mutations. UART hardware/task binding is the next checkpoint.

## Checkpoint 1 software evidence

Tests were written before the new headers/implementation; retained RED logs
show missing production interfaces. GREEN tests exercise real producer code.
The full strict GCC and Clang ASan/UBSan suites each pass563/563, including
16 new checks. The12 independent D1 fixtures test envelope acceptance only;
CRC-correct payload bytes are not a qualified run cohort.

The metric oracle checks360000 events (30000 per ID), first-extreme retention,
raw-reference consistency across TIMER wrap, empty/sentinel serialization,
half-range ambiguity, bad depth and count-overflow destination preservation.
D1 tests check exact independent golden bytes, every truncated prefix, wire
bit corruption, two known concatenated manifest-chain CRCs, bounded malformed
input and100000 deterministic noisy packets. No failure mutates its caller.

The reproducible tests/pil_gate1_fixture.c host generator uses production
reducers/encoder to produce a SYNTHETIC complete15-record cohort across wrap:
600001 logical applies,600004 stages,3 tail cancellations. It is NOT a UART
runtime, lifecycle execution, hardware trace or a physical timing measurement.
SW8 independently decoded it and reported availability bounds995..1008us,
apply/source bounds4015..4030us, slack>=1997us, apply jitter<=25us, native
first-C-entry jitter<=11us with synthetic H=L=5us. Its evaluator keeps overall
physical_gate1_pass=false/INCOMPLETE. These numbers describe injected test
values, not Pico performance or attainable Debug Probe latency.

Strict ARM compiles both new units in all38 retained SIM configurations:
76/76 pass with warnings as errors. Static compiler frames: reducer observe20B,
metric payload64B; D1 encode312B, decode288B, chain152B, header helper16B.
These are individual frames, not full call-chain or hardware watermarks.
Neither decoder belongs on an IRQ. New helpers own no writable static RAM;
future caller-owned state, queues, trace and task stack must still meet16KiB.

All38 ordinary profiles rebuilt with zero warnings and retained SDK proofs.
All76 literal ELF/UF2 SHA256 values equal the451e562 baseline after this
checkpoint, including debug metadata. The merged9831991 base retains those
same native inputs. [Full hash values](PIL_STAGE0_HASHES.json) are unchanged.
No metadata exclusion or compiler/source-path change was used.

Raw local logs are in build/pil-gate1/. The shared independent envelopes are
tests/fixtures/pil_gate1_d1.json. No template file or existing application C
source is changed at this checkpoint. All transport/hardware gates remain open.
