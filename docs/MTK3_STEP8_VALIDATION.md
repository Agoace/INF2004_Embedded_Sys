# STEP 8 software validation

## Scope

Production inputs were validated at `c6f3228`, with later owner/base merges
changing documentation only. Compiler: GNU Arm Embedded 14.3.1; local Pico
SDK revision `98a542c1`; host x86_64. Builds use `-j4`. No push, flash,
network connection, broker installation or physical test was performed.
The main checkout remains separate. New-image hardware results are
**Not run**; previous STEP 7 passes remain historical.

Private evidence is outside Git in `/tmp/step8-results/` (mode 0700).
ELF/UF2 files can contain private settings. Do not publish them or flash
SDK proof images. Public protocol fixtures use only synthetic data.

## Clean builds and strict ARM compilation

Each profile ran `make clean`, then a complete build. Every build had
**zero warnings**. Sizes are GNU `size` bytes, not runtime free memory.

| Profile | text | data | bss | Strict touched units |
| --- | ---: | ---: | ---: | ---: |
| Default SMP1 / CYW43 / NET none | 310004 | 220 | 126380 | 19 |
| SMP0 / no radio / NORMAL robot | 63132 | 152 | 119848 | 17 |
| SMP0 / no radio / TEST | 54100 | 152 | 119276 | 17 |
| SMP0 / no radio / BENCH | 41966 | 148 | 95460 | 17 |
| SMP0 / CYW43 / lwIP / MQTT / NORMAL | 365880 | 336 | 182888 | 20 |
| SMP0 / CYW43 / lwIP / MQTT / TEST | 356152 | 336 | 182188 | 20 |

**110/110** actual ARM compiler invocations passed
`-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror` for changed
production C units that the profile compiles. SDK headers use system-header
paths for these own-code checks; no warning is suppressed in owned sources.
Some app units are conditionally inactive in a profile. Wi-Fi/LwIP template
units are checked only where the profile builds them. No `__atomic_*`
runtime symbol is linked in any of the six production ELFs.

## Host and retained SDK link checks

- Combined strict host suite: **92/92**, zero build warnings.
- Final independently built owner suites: base **26/26**, motion **33/33**,
  line **30/30**, scan **32/32**, telemetry **54/54**. These overlap the
  combined suite; do not add them as independent coverage totals.
- Retained SDK link-only probes: **6/6**. Each force-links PWM/GPIO/ADC,
  alarm, ultrasonic, timer/tick, watchdog and reset references in a separate
  ELF. They have no unresolved symbols and preserve the no-SDK-startup/
  runtime boundary. This is not a hardware result. Never flash them.
- Template ledger: **7 recorded paths** against `c3b0954`. The checker
  verifies the file set, not the correctness of each edit.

Host coverage includes atomic-word odd/change/wrap reads; bounded queue
full/error cleanup; generation/highwater live logs; signed C-packed/Python
frames; malformed lengths/version/reserved bits; default/configured token
policy for all opcodes; no-token laptop ESTOP; reconnect/ERR_MEM/pending
QoS1/PUBACK/ABORT; and producer-stale/missing health across microsecond
wrap. The earlier health implementation fails both new regressions.

Vehicle tests include 4096 policy combinations, source-age saturation and
microsecond/millisecond wrap, preemption during board collection and the
real vehicle implementation with transport doubles. They show that failed
live-event enqueue and rejected DUMP do not cancel driving, uncalibrated
START does not start, and ESTOP consumes first and drains queued START.
The clock/priority tests use register/kernel doubles. No host test measures
physical motor motion, Wi-Fi timing or actual interrupt behavior.

## Final MQTT ELF ownership and boot proof

The NORMAL MQTT map/nm/disassembly establish:

- `cyw43_utk_app_poll` is strong `T` at `0x10016394`, owned by
  `app_program/subsystem1/robot_mqtt.o`. The template weak default at
  `mqtt_utk.o` is discarded. No linker wrap is used.
- Existing Wi-Fi task calls `sys_check_timeouts` at `0x10006b40`, the
  named application hook at `0x10006b4a`, then full-poll end at
  `0x10006b4e`. Only that task owns lwIP/MQTT calls.
- `cyw43_utk_start` loads literal priority **20** at `0x10006ed8` before
  `tk_cre_tsk` at `0x10006ee2`. It is direct creation, not later demotion.
  USB remains priority3; motion/line/sonar/vehicle are10/11/12/13.
- `Reset_Handler` calls `knl_startup_hw` at `0x10014bee` before data/BSS
  and kernel startup. `knl_startup_hw` calls `startup_clock` at
  `0x10014940`, then checks/writes WATCHDOG_TICK (`0x4005802c`) with
  `0x20c` (ENABLE plus 12 cycles), checks RUNNING and bounds the wait to
  100000 reads. Its failure branch loops closed at `0x1001498a`, before
  devices/tasks. The helper uses no SDK header or uninitialized global.

The application TIMER setup remains idempotent verification; the 2% rate
check remains a driving prerequisite. Cold-power S0-U01 is still required.

### ARM C declaration issue

On this actual ARM newlib toolchain, `int32_t` is `long`, whereas kernel
`INT`/`ER` are `int`. Both are 32 bits, but are not compatible C declaration
types. Host `int32_t == int` hid the issue. Shared code now defines the
Wi-Fi task/start exactly as the public fixed-width header declares,
removes the duplicate incompatible task declaration, and includes that
same SDK-free header at the device-init caller rather than redeclaring
start as `ER`. Real ARM compilation validates all three boundaries.

## Memory and measurement limits

NORMAL MQTT static `data+bss` is **183224 bytes**. The kernel uses 256 KiB
main SRAM, leaving **78920 bytes** before dynamic allocations. The 2048-byte
exception-stack reservation reduces the initial kernel allocation area to
about **76872 bytes** (`__bss_end=0x2002cbb8`, before allocator setup).

Known dynamic stack requests are initial1152, USB2176 and Wi-Fi8320 bytes
(including each 128-byte kernel context addition). Five MBF requests total
4780 bytes: RAM4096, sonar256, live events320, normal commands96, ESTOP12.
Subtracting these requests leaves about **60444 bytes before allocator
metadata and any later allocation**. This is a link/configuration budget,
not a measured free-heap claim. lwIP's heap/pools are already in bss; app
stacks are static. MQTT reserves 21504 app-stack bytes, including the
inactive bench region; the NORMAL active regions total17408.

Flash `text+data` is366216 bytes, about17.5% of nominal2MiB. Largest strict
own-function frames include vehicle768, hook304, vehicle-start288 and
health/telemetry264 bytes. These are individual frames, not maximum call
chains or interrupt nesting. The 8192-byte Wi-Fi stack is template-owned
and outside the app fill-pattern helper. Loaded physical margins of at
least256 untouched bytes per app task, and adequate template-stack margin,
remain **Not run**.

Motion and line publication are always present, independent of MQTT.
Motion cost is included before watchdog feed and in reported work; line
sample cost includes publication and its published cost can lag one sample.
Full Wi-Fi poll wall/work and start-gap counters include preemption; they
are not CPU percentages. Probe-max excludes its final stores and is not a
calibrated subtraction from measured work. Changing task priority does not
bound interrupt-off time or IRQ interference. Physical loaded checks must
measure that cost, not infer it from the clean build.

## Remaining acceptance

Use [the operator runbook](MQTT_RUNBOOK.md). All of these are **Not run**:
S0-U01 cold power on NORMAL/TEST/BENCH/MQTT, S1-U04 reconnect, S1-U05
rates/loss/CSV and polling/stack data under load, S1-U06 NORMAL-robot
commands, S2-U01 independent motion/Wi-Fi windows, and independent M1/M2
with MQTT. No network result gates local driving acceptance. Link loss is
not ESTOP; GP21 is the real ESTOP in NORMAL. TEST/BENCH do not consume
vehicle command queues; lift wheels for TEST.
