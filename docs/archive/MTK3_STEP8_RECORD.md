# STEP 8 historical software, hardware and reboot records

These three source records describe different revisions. A PASS applies only
to its stated revision; later source sections do not extend it. Statements of
pending work are historical, not a current staging checkout ledger.

## Source: MTK3_STEP8_VALIDATION.md

# STEP 8 software validation

## Scope and evidence

Production code was checked at `ed21052`. Owner merges through `b1ae9f6`
change no candidate file content. A repeated six-profile object/ELF comparison
at that merge tip also passes. Compiler: GNU Arm Embedded 14.3.1; local Pico
SDK revision `98a542c1`; host x86_64. Builds use `-j4`.

No push, flash, broker connection or physical test was performed. The main
checkout remains at `mtk3-staging` `8ec3523`. All new hardware rows are **Not
run**; STEP 7 passes are historical, not validation of these images.

Private evidence is in `/tmp/step8-results/` (0700 directory, private files).
ELF/UF2 files can contain private settings. Do not publish them or flash SDK
proof images. Public protocol fixtures contain synthetic data only.

## Clean builds and strict ARM checks

Each profile ran a complete clean build with **zero warnings**. Sizes are GNU
`size` bytes, not measured free memory.

| Profile | text | data | bss | Strict changed units |
| --- | ---: | ---: | ---: | ---: |
| Default SMP1 / CYW43 / NET none | 310004 | 220 | 126380 | 26 |
| SMP0 / no radio / NORMAL | 63132 | 152 | 119848 | 24 |
| SMP0 / no radio / TEST | 54100 | 152 | 119276 | 24 |
| SMP0 / no radio / BENCH | 41966 | 148 | 95460 | 24 |
| SMP0 / CYW43 / lwIP / MQTT / NORMAL | 373236 | 356 | 184320 | 27 |
| SMP0 / CYW43 / lwIP / MQTT / TEST | 360444 | 356 | 183360 | 27 |

**152/152** ARM compiler invocations passed
`-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`, with stack-usage
output, for changed production C units compiled by each profile. SDK headers
are system headers for this own-code check. Some listed units are inactive
under a profile's conditional compilation. No `__atomic_*` runtime helper
is linked in the production ELFs.

Retained SDK link-only probes pass **6/6**. They force-link the selected GPIO,
PWM, ADC, alarm, TIMER, ultrasonic, watchdog and reset references without SDK
startup/runtime. They are not hardware tests and must never be flashed.
The template checker reports **7 paths** against `c3b0954`; this checks file
membership, not the correctness of the code.

## Host checks

Combined strict suite: **111/111**, with zero compiler warnings. Owner suites:
base **27/27**, motion **34/34**, line **31/31**, scan **33/33**, telemetry
**69/69**. These overlap; do not add them into an independent coverage total.
Dashboard worker reports 121 dashboard cases and 46 decoder cases inside the
Python test executables, plus an offline headless-browser rendering check.

Coverage includes bounded odd/change/wrap board reads, queue saturation and
cleanup, retained/stale sample ages, token policy, dedicated ESTOP, command
policy combinations, C-packed/Python-decoded binary frames, unknown and
malformed versions, reconnect, ERR_MEM, PUBACK failure, DUMP abort, live-log
gaps and generation changes. DUMP throughput simulation for 4501 rows and
512 events gives 2.8 s at 20 ms RTT, 14.2 s at 100 ms, and 28.4 s at 200 ms.
The 20-second target does not hold at 200 ms RTT.

Added checks cover signed mean overflow edges, obstacle nearest-point ties,
all 32768 PROFILE words, LWT CONNECT settings and retained ONLINE retries,
duty/heading/rate packing, integer sine accuracy, forward/reverse path length,
detour outcomes/abort/lateral maximum, run counters, steady-command speed
error, summary retry/reconnect, and CPU-load arithmetic including wrap and
unknown values. The sine-table absolute error is below 0.0001; CPU synthetic
one-second arithmetic error is at most 1 permille. Neither is target sensor
accuracy or a measurement of target CPU load.

## Style-only code identity

Feature and style commits are separate. The original style batches
`95c56de`, `aac9c01`, `4614012` each matched their pre-style baseline across
six profiles (875 object/ELF records per comparison).

After the approved feature additions, style commit `ed21052` matches the
post-feature baseline `e183d35` across **905 object/ELF records, zero
differences**. Repeating after owner merges gives the same result. Comparisons
include object disassembly, relocations, section data, symbols and sizes,
plus linked ELF code/data/symbols. Debug/comment/attribute sections and
compiler-generated `wm4.*` identifier spelling are normalized. This is a
code/data comparison, not raw debug-bearing object-byte identity.

A deliberate mutation changed the state period from 200 to 201 ms. The
comparator reported differences and exited with status 1. The mutation was
restored before the style proof. Missing files and an empty comparison also
fail the checker. No intended production code/data difference exists in the
style-only batch.

Scope includes the STEP 7–8 task/board/transport files, `app_main.c`, the new
observers, and changed ranges of the seven recorded template files. It does
not include the earlier large `subsystem2_test.c` image or unchanged pure
navigation algorithms. Existing public names were not renamed. Deferred
naming observations: unprefixed file statics, `s_` statics in log_stream,
mixed public prefixes (`rm_`, `robot_`, `telemetry_`, `mc_`, `drv_`, `vc_`), and
mixed legacy typedef naming. This is a scoped BARR-C-style cleanup, not a
claim of complete formal coding-standard compliance for the repository.

## Runtime ownership and limits

The strong `cyw43_utk_app_poll` is application-owned; the weak template demo
is discarded. Only the existing Wi-Fi task calls lwIP/MQTT. Wi-Fi is created
directly at priority 20. Application priorities remain motion 10, line 11,
sonar 12, vehicle 13, telemetry 30 and health 31. The CPU meter is priority
32 (this configuration's lowest priority), not the initially proposed 139.
USB remains priority 3.

Early kernel TIMER initialization still follows clock startup and precedes
devices/tasks. The bounded WATCHDOG_TICK setup uses no SDK headers or
uninitialized globals. Application TIMER setup remains idempotent verification;
cold-power S0-U01 is still required. No controller settings, routes, SCAN,
detour or line logic changed. Summary observers cannot return drive commands.
The idle meter never feeds the watchdog.

Static NORMAL MQTT data+bss is **184676 bytes**. This is a link result, not a
free-heap or runtime margin measurement. Registered application stacks now
include the 512-byte CPU meter. Strict individual function frames include
vehicle task 776 B, MQTT hook 272 B, log-stream service 144 B, telemetry task
264 B and idle-meter loop 16 B. Individual frames are not complete call-chain
or interrupt-nesting budgets. Wi-Fi's 8192-byte stack is outside app stack
painting. Loaded stack margin >=256 B remains **Not run**.

Wi-Fi hook execution time is not measured on hardware. The hook makes bounded
board attempts and bounded publish attempts, but lwIP and preemption determine
actual wall time. Routine traffic, retained status/summary, live log and DUMP
share the unchanged 256-byte MQTT output ring. ERR_MEM is retried without
blocking driving. Frame-size/host simulations are not a timing certificate.

The CPU meter uses the same non-inlined, non-cloned loop for the 4 ms
interrupts-disabled boot calibration and its lowest-priority task. Symbol
inspection confirms one `idle_meter_loop`. It replaces WFI sleep, increases
power use, and measures core 0 only. With SMP=0, that includes Wi-Fi and
interrupt time. Target accuracy and idle-board expected range remain
**not established**; [CPU record](kernel/CPU_LOAD.md) gives the procedure.

## Remaining hardware acceptance

Use [the runbook](s1-telemetry/MQTT_RUNBOOK.md). All are **Not run**: S0-U01 cold power;
S1-U04 reconnect/will timing; S1-U05 stream age/loss/rates, distance/duty/
odometry, profile/detour/summary decode and loaded hook/stack timing; S1-U06
commands; S1-U07 idle-board CPU measurement; S2-U01 independent motion/Wi-Fi
windows; independent M1/M2 with MQTT. No MQTT result gates local driving
acceptance. Link loss is not ESTOP. **GP21 is the real ESTOP** in NORMAL;
TEST/BENCH do not consume vehicle command queues. Lift wheels for TEST.

## Source: MTK3_STEP8_HARDWARE_RESULTS.md

# STEP 8 hardware results — 2026-11-07

These results were supplied by the operator. They apply to the earlier
STEP 8 image, not to the later boot-label, absent-board, diagnostic or SCAN
retry changes. A build or host test does not extend a hardware PASS to a new
image. Private USB/MQTT captures remain under ignored `build/mtk_logs/` in
the main checkout. No private configuration or raw capture is committed.

## Shared platform and MQTT

| Check | Operator evidence | Result and limit |
| --- | --- | --- |
| S0-U01 cold power | TIMER check error 0 | PASS for tested image |
| S2-U01 Wi-Fi TEST | Wi-Fi priority 20; late 0. First window 9508..10248 us, lifetime 9508..10491 us. Second window 9518..10482 us | PASS. These are separate windows, not one fabricated range |
| S2-U05 motion-only watchdog | Operator repeated the destructive reset check | PASS; physical GP21 and watchdog remain independent of network |
| S1-U07 CPU | Idle 117 permille; traffic heartbeat 139 permille; calibrated peak 4026729 iterations/s | Target load values observed. No independent absolute CPU-accuracy measurement was supplied |
| S1-U04 connection/LWT | Watchdog reset; broker restart returned robot in about 2 s; power-cut OFFLINE in about 7 s | PASS for these cases. A retained will is not a physical ESTOP |
| S1-U05 live log/DUMP | M1 live CSV 216/216 rows byte-identical to GP20 USB rows; median age 20 ms; DUMP about 1 s | PASS for captured run, not a completeness claim for every QoS 0 run |
| S1-U06 dashboard commands | Uncalibrated START refused; CALIBRATE/START/DUMP/ESTOP worked; wrong token refused, `command_lost +2` | PASS for these cases; not all queue-full/duplicate combinations were target-tested |

The actual radio interval is about 20 ms, not the configured 2 ms request.
A 10 ms kernel tick plus `tk_dly_tsk` extra-tick semantics quantizes the
request to about two ticks (roughly 50 polls/s). This is a known port
property. It is not changed by the boot/session fix. The measured MQTT
median row age is consistent with this interval, but does not prove every
message's end-to-end latency.

## Driving observations

| Run | Evidence | Result and limit |
| --- | --- | --- |
| M1-MQTT | Gap 9916..10087 us; rows 216, events 11; response maximum 1470 us; faults 0; behavior unchanged | PASS for basic line/live-log run |
| Later MQTT ESTOP run | Gap 8952..11038 us; rows 46, events 6; response maximum 1086 us; faults 4 (ESTOP) | Command/stop observed. Lifetime gap exceeds the 11000 us gate by 38 us; cause is not identified by this log |
| M2 initial box scans | Echoes 224..250 mm at all angles | `DT_FAIL_NO_SIDE` is correct for an all-blocked profile |
| M2 16:04–16:05, three attempts | Box 17 x 12 x 7 cm; stood 12 cm wide/17 cm tall and also once 17 cm wide. Echoes 214..290 mm at -30..+30 degrees | `DT_FAIL_NO_SIDE`; neither side had a usable clear point |
| M2 16:13 | Same placement as a failed run, box 17 cm wide. Closest 212 mm at +10; only left step 6 (+30) free/no echo; right blocked | Basic detour PASS: LEFT, 22041 ms, path 995 mm, max lateral 318 mm, returned to line, line losses 0; stopped by dashboard ESTOP; summary `detours_succeeded=1` |
| M2 16:13 timing/log | Gap 9902..10099 us; response maximum 1201 us; rows 1418; overruns 0 | PASS for this observed run |

The M2 decision margin was one outer scan point. Small placement/echo changes
can switch it between no-side and LEFT. The scan/profile/detour logic of this
tested image was unchanged since `445084c`; the result is not evidence of a
software regression. A later operator-approved reverse/retry change needs
its own hardware checks; no historical PASS validates that new behavior.

The USB header's motion gap is lifetime, not per run. The larger gap printed
after the second run could have occurred during the earlier GP20 dump,
idle/calibration, or driving. The old log has no release/entry/mutex timestamp
trace. See [the timing investigation](MOTION_TIMING_INVESTIGATION.md); do not
attribute the delay to Wi-Fi, SCAN or ESTOP without a diagnostic capture.

## Remote dashboard

The operator verified Tailscale Serve on 2026-11-07: live data and all buttons
worked from the remote desktop. Plain HTTP through the tailnet was used;
short-hostname Host/Origin rejection was intentional. HTTPS certificate
provisioning failed on the hotspot and needs a normal-network retry.
See [remote dashboard instructions](s1-telemetry/MQTT_DASHBOARD.md#remote-view-through-tailscale).

## Boot/session image results — 2026-11-07

Operator-supplied results for the accepted core image `5bda9ee`, MQTT TEST
profile. Private logs: `build/mtk_logs/s8_reboot_test_20261107-171326.txt`
in the main checkout.

| Check | Operator evidence | Result and limit |
| --- | --- | --- |
| Boot label across S2-U05 reset | Boot IDs 2729381450 then 3017455292 (operator report) | PASS (operator report). The ignored `mqtt_20261107-171326.csv` is not this evidence: its DUMP of 142 rows/4 events matches the later DIAG session, with IDs 2819020132 (retained) and 1821749757 |
| Two-boot history | S1-U02 accepted 34 rows on each boot; both 34-row sets kept | PASS for this two-boot case |
| LWT/status across reset | OFFLINE with the old ID and ONLINE with the new ID in the same second | PASS |
| Reset reason | Watchdog | PASS |
| Absent TEST vehicle | `vehicle_busy` 0, unavailable mask 8 (vehicle bit) | PASS |
| S0-U01 | TIMER 1009753 and 1010053 us per 1010 ms, error 0 | PASS on both boots |
| S2-U01 | Window 9514..10484 us (boot 1), 9520..10255 us (boot 2); late 0, missed 0 | PASS. Separate windows |
| S2-U05 | Watchdog reset after 99 ms; no repeat reset | PASS |

## DIAG=1 motion timing capture — 2026-11-07

Private logs: `build/mtk_logs/s8_diag_172602.txt` and
`build/mtk_logs/s8_diag2_172730.txt` in the main checkout.

| Step | Evidence |
| --- | --- |
| Run, MQTT ESTOP, first GP20 | Diagnostic observations 0; lifetime gap 9896..10085 us; dump `tm_usb_dropped` 0, delta 0 |
| Second GP20, about 10 s later | 18 observations, all with phase `0x10000` (first USB dump active) |
| Worst-gap snapshot | gap 14467 us, release_gap 9995 us, dispatch 4488 us, mutex wait 15 us, prior `motion_status` hold 46 us, queued 0 |
| Latest snapshot | gap 12783 us, release_gap 9997 us, dispatch 2802 us, wait 11 us |
| Lifetime gap after the dump | 5533..14467 us |

**Measured cause:** post-release dispatch delay while the GP20 USB dump runs.
The cyclic release interval stayed about 9995 us, and mutex wait was at most
15 us. MQTT, ESTOP, SCAN and the motion mutex are not the cause. The USB
console task at template priority 3 outranked motion 10 while it drained the
dump. The approved fix is the USB console priority change in the [template ledger](../architecture/TEMPLATE_CHANGES.md).

## New-image checks still required

- Cold-power boot IDs across power cuts, watchdog resets and broker restarts.
- Two boots with generation 2/index 0 both preserved in CSV/history.
- TEST absent vehicle shown separately from genuine board contention.
- DIAG=1 release/entry/acquisition evidence around GP20 dump, commands and SCAN.
- Any later SCAN reverse/retry behavior, GP21 cancellation and timeout reason.

## Source: MTK3_STEP8_REBOOT_FOLLOWUP.md

# STEP 8 boot/session and timing follow-up

## Scope and delivery

Baseline: `mtk3-staging` at `82cf1db`. Candidate: `mtk3-staging-next`, worktree
`/tmp/mtk3-s8fix`. The main checkout is not changed. Changes are local only;
no push or flash was performed. The dashboard `--allow-origin` behavior is
preserved. Helper integration uses real `--no-ff` merges, not a rebase.

SCAN option C is separate on `mtk3-scan-reattempt`. It is not merged into
this core candidate and is not a proposed fix for the unmeasured timing
cause. Its driving changes need separate review and hardware acceptance.

## Changes

| Area | Correction | Important limit |
| --- | --- | --- |
| Firmware boot label | Nonzero ROSC-derived u32, initialized once before application startup; direct registers/TIMER spacing, bounded pair extraction; added to observations | Random label, not persistent/cryptographic or guaranteed unique; source failure disables observation startup only |
| Log wire/storage | v2 outer frame carries one ID; batch header/control 36 B, max batch 228 B; embedded events and KR1 remain 16 B | No per-row ID duplication; output ring remains 256 B |
| Client/dashboard | Boot/session-scoped deduplication, refill/gap/DUMP and live baselines; preserve both boots even at generation 2/index 0; label old summaries without switching live boot | Legacy inference is ambiguous; same-ID OFFLINE-to-ONLINE can conservatively duplicate rows in a visible local session |
| Health | Main-board unavailable mask plus aggregate count, separate from per-board true-contention counts | Empty secondary histories/reports are not absent boards; only `E_TMOUT` counts contention |
| Sonar observer API | Separate unready `E_OBJ`, writer conflict `E_TMOUT`, and no-next-entry `E_NOEXS` | Only the observation adapter and host tests use this API; acquisition/driving callers are unchanged |
| Motion investigation | Opt-in `DIAG=1` fixed snapshots of release, entry/acquisition, phase and prior API hold; GP20-only output | No new scheduling/control decision; actual cause still needs an operator capture |
| Documentation | Actual Wi-Fi cadence ~20 ms, operator hardware results, wire byte-cost table, timing limits and future per-run proposal | Earlier-image PASS does not validate new firmware |

Authoritative [wire layouts and before/after byte table](../s1-telemetry/MQTT_WIRE.md#minimum-bit-wire-cost)
justify each ID. State and sonar retain explicit IDs: queued old observations
can arrive after new-boot evidence, and uptime alone cannot bind them safely.
State adds 20 payload bytes/s at 5 Hz. Reserved barcode/IMU and commands do
not grow. A 32-bit label avoids the large birthday-collision risk of 16 bits;
see [entropy assumptions and limits](../s1-telemetry/MQTT_WIRE.md#boot-identity-and-sessions).

## Timing finding and proposed next action

The later ESTOP log reports lifetime gap 8952..11038 us, 38 us over the
unchanged gate. It does not locate the event. The first GP20 header precedes
its long dump; that dump, idle and calibration can contaminate the next
header's lifetime extrema. The entry timestamp precedes the mutex. Therefore
neither MQTT, SCAN, ESTOP, USB nor a mutex can honestly be named as measured
cause from the old min/max-only log.

[The investigation](MOTION_TIMING_INVESTIGATION.md) documents the diagnostic
build, expected distinctions and operator procedure. Queued wakes and a
release/entry snapshot race are visible; race bit 17 means dispatch delay
must not be interpreted. No template USB/IRQ path was modified. Any later
causal fix must follow the diagnostic evidence and operator review.

A per-run gap window beside lifetime metrics is **proposed, not added**.
Count only complete in-window gaps; exclude the crossing gap. Preserve
lifetime extrema/counters. Define the start/stop boundary with the operator
before changing the production API or acceptance reporting.

## Validation record

Final candidate: helper `ce59bf1` merged with `--no-ff` on top of core `b65c276`.

| Check | Result |
| --- | --- |
| Host CTest (C plus Python wrappers) | 116/116 PASS |
| Python decoder/log and dashboard suites | 62 and 137 PASS against the C fixture emitter |
| Firmware profiles, warning-free with `-j4` | 7/7: default, robot, TEST, BENCH, MQTT, MQTT TEST, MQTT DIAG |
| Strict ARM invocations on changed C units | 91/91 PASS (13 per profile) |
| SDK link proof | 7/7 PASS; checker now includes the DIAG profile field |
| Docs, template ledger, `git diff --check` | PASS; template ledger 7 paths |
| Secret hygiene | No private settings tracked or in the diff |

Independent diagnostic isolation proof at `50d2e52` versus `164f616`:

- Six existing DIAG=0 profiles; 923 object/ELF records, zero allocated
  code/initialized-data, relocation, symbol or BSS-size differences.
- Debug/DWARF and associated macro `.group` index metadata are excluded;
  this is not a claim that raw debug-bearing files have identical bytes.
- Deliberate watchdog 100-to-101 ms mutation rejected in `motion.o` and ELF
  disassembly; restored 100 immediately. No mutation artifact was deployed.
- DIAG=1 has a distinct `_diag` artifact name. Synthetic host tests separate
  dispatch delay, mutex wait and late cyclic dispatch, including wrap and
  snapshot-race handling; production motion task also runs in a DIAG=1 host
  variant. Synthetic timestamps are not hardware evidence.

Builds use `-j4`. Strict ARM checks add `-Wall -Wextra -Wconversion -Wshadow
-Wformat=2 -Werror`. SDK link checks use the real target headers. The template
ledger remains seven paths against `c3b0954`; this task adds no deviation.
Private evidence is below `/tmp/step8-results/`, mode 0700/0600. The candidate
worktree is mode 0700 and private configuration 0600, ignored and untracked;
private settings are not copied into tests, docs or Git.

## Hardware acceptance and outstanding work

[Operator results from 2026-11-07](MTK3_STEP8_RECORD.md) record cold
TIMER, Wi-Fi timing, CPU values, watchdog/LWT, live 216/216 rows, command
checks and the M2 detour. M2 clearance depended on one outer scan point;
that is a placement/echo margin, not evidence that unchanged software regressed.
The two S2 timing windows remain separate in the record.

New-image hardware is **Not run**. The operator must review and schedule:

1. Cold-power/watchdog/broker boot-label and two-boot history checks.
2. TEST absent vehicle versus genuine board contention.
3. DIAG=1 capture around GP20, MQTT ESTOP and SCAN; report cause/proposal
   before any scheduling or driving-path timing fix.
4. Separate SCAN option C review and its own reverse/cancellation/timeout tests.
5. Remote HTTPS certificate retry on normal Wi-Fi; hotspot HTTP/Tailscale
   live data and buttons already passed on the earlier dashboard.

The physical GP21 ESTOP and motion-only watchdog are unchanged. Wi-Fi owns
all lwIP/MQTT calls; network failure never gates driving or watchdog feeds.
