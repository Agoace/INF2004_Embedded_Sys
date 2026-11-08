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
**not established**; [CPU record](../kernel/CPU_LOAD.md) gives the procedure.

## Remaining hardware acceptance

Use [the runbook](../s1-telemetry/MQTT_RUNBOOK.md). All are **Not run**: S0-U01 cold power;
S1-U04 reconnect/will timing; S1-U05 stream age/loss/rates, distance/duty/
odometry, profile/detour/summary decode and loaded hook/stack timing; S1-U06
commands; S1-U07 idle-board CPU measurement; S2-U01 independent motion/Wi-Fi
windows; independent M1/M2 with MQTT. No MQTT result gates local driving
acceptance. Link loss is not ESTOP. **GP21 is the real ESTOP** in NORMAL;
TEST/BENCH do not consume vehicle command queues. Lift wheels for TEST.
