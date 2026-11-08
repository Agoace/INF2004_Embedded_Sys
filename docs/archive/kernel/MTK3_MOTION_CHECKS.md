# μT-Kernel motion bring-up checks

## Scope and local revisions

This is a historical step-1 record. The later
[hardware watchdog record](KERNEL_WATCHDOG.md) supersedes its statements
that the watchdog counter/source is absent and records current checks.

This record covers step 1 only: shared GPIO/time platform, the S2 motor
task, SDK hardware adapter and RAM floor bench. No line/vehicle/scan task
is connected. No hardware was flashed by this change. Operator-supplied
2026-11-06 post-flash results now include S2-U01/U02/U03 and functional
B4-lifted passes. Cold-power S0-U01, S2-U04 and floor B4 remain **Not run**.

- Base `mtk3`: `1806ed0`, from `c24abf6`; cold-power tick initialization,
  rate check, S0-U01 and host ratio tests in `165a574`; correct unsigned
  `RELTIM` delay interface in `1806ed0`.
- `mtk3-motion`: `f0b4194`, from `3d7b35e`; base merges `e9fb754` and
  `f0b4194`; zero-PWM clock gate, failure-path tests and operator hardware
  records in `7ea8fcf`; navigation refresh `efa3950`.
- Candidate code merges: `mtk3-staging-next` through `bb6b979`;
  initial base/motion merges `4927e95`/`6bfd56f`, delay-type correction
  merges `3b47be3`/`bb6b979`.
  Both base and motion use real `--no-ff` merges; no history was rewritten.
- `mtk3-staging` remains at `d9cfea6` in the main checkout. The operator
  will review the candidate before Main_Worker advances it. No push.

This change uses `/tmp/mtk3t`, `/tmp/mtk3b` and `/tmp/mtk3mo`, outside
School-Repos. The base and motion worktrees are removed after merging.
Cold-power fix build/check logs are in `/tmp/mtk3-cold-results/`.
Initial results were in `/tmp/mtk3-motor-results/`; per-tick fix results
were in `/tmp/mtk3-notify-results/`. Host and firmware build directories
and link-probe outputs are removed after checks. The main
checkout is unchanged. Build output and the dummy bare-metal configuration
are not committed.

## Build and host checks

Toolchain: Arm GNU 14.3.Rel1, GCC 14.3.1; Pico SDK 2.3.0 at
`/home/yangxuanlee/pico/pico-sdk`.

| Check | Result |
| --- | --- |
| `make SMP=0 WIFI=none -j8` | Pass; 0 warnings |
| `make SMP=0 WIFI=none TEST=1 -j8` | Pass; 0 warnings |
| `make -j8` (default SMP=1, USB, radio) | Pass; 0 warnings |
| Strict ARM compile of six touched platform/motion/startup units | 6 per profile, 18 total; pass, 0 warnings |
| Retained SDK hardware link probe, all three profiles | Pass; no unresolved symbols |
| Committed `scripts/check-sdk-hw-link.py`, all three profiles | Pass |
| Independent strict host CMake/CTest suite | 23/23 pass |
| Simulator, `SIM_FIRMWARE=detour_test` | 6/6 pass |
| Bare-metal Pico W CMake configure and full build | Pass; placeholder config removed after build |
| Documentation checker | Pass |
| Template kernel/library/device/include/config/tool sources | Unchanged from `template/main` |
| Pure `motor_ctl.[ch]` | Unchanged from original `mtk3-motion` module commit `40190f8` |
| Credential history/tree check | No committed `src/wifi_config.h` or `config/wifi_secrets.h` |

Strict flags: `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`.
SDK headers are external system headers. The host scheduler/hardware
doubles test mutex ownership, core affinity, encoder signs, last-tick
count snapshots, stale stop, ESTOP ordering, GPIO dispatch/acknowledgment
and queued/late wake counters. The notify test also checks one client wake
per completed tick, after mutex release, and no client wake before
registration or after disabling it. New time doubles cover ratio 1.0,
42.7 and its inverse, 0.97/1.03 failures at a 2% limit, exact 0.98/1.02
boundaries, counter wrap and kernel errors. Motion doubles check that
both initialization and rate-check failures leave motor PWM zero and
create no mutex/cyclic/task or bench. These are not hardware timing tests.

The cold-power fix reran all three clean firmware builds and SDK link
proofs, the strict 23-test host suite, the 6-test simulator suite, the full
bare-metal Pico W build and the documentation checker. Fresh build paths:
`/tmp/mtk3-cold-host`, `/tmp/mtk3-cold-sim`, `/tmp/mtk3-cold-bm`.
Bare-metal configuration used only a temporary copy of
`src/wifi_config.h.example`; it was removed after the build.
Base-only strict host tests passed 12/12; motion-only tests passed 15/15.
The extra strict ARM check found a signed `TMO` cast where the real
`tk_dly_tsk()` argument is unsigned `RELTIM`; the code and host double were
corrected, then all three make profiles, SDK proofs, strict ARM units,
host tests and simulator tests were rerun successfully.

## Image sizes and SDK change

Sizes below are bytes from `arm-none-eabi-size`, normal images only.
The link-only probe is not in these images.

| Profile | Text | Data | BSS | Text change from `d9cfea6` |
| --- | ---: | ---: | ---: | ---: |
| UP, radio off | 40266 | 28 | 75916 | +504 |
| UP TEST, radio off | 38086 | 24 | 18300 | +648 |
| Default SMP, radio on | 286138 | 92 | 82356 | +532 |

Data and BSS are unchanged from `d9cfea6`. The app SDK hook keeps section
garbage collection enabled for all profiles, including radio-off. The normal
bench reserves 2400 static rows (57600 bytes); TEST builds omit that log
and bench task. No kernel capacity/configuration value was changed.

## SDK link result and boundaries

The exhaustive include/source list and repeat commands are in
[Motor controller — SDK hardware subset](../../s2-motion/MOTOR_CONTROLLER.md#sdk-hardware-subset)
and [`build_make/app_sdk_hw.mk`](../../../build_make/app_sdk_hw.mk).

- PWM and resets: inline SDK functions, no .c library source required.
- GPIO: SDK `gpio.c`; reuse the existing radio object when enabled.
  Unused SDK IRQ helpers are discarded, not linked to an SDK IRQ runtime.
- ADC: SDK `adc.c` compiles; the forced link probe retains `adc_init()`
  and calls inline GPIO/select/read operations. It is never run. There is
  no step-2 line driver in this change.
- Timer: hardware header and inline `timer_time_us_32()` compile/link;
  no timer/alarm/time initialization .c files are added. The application
  64-bit read stays a stable high/low register read.
- Tick source: SDK `hardware_ticks/ticks.c`, runtime-free on RP2040.
  `tick_start(TICK_WATCHDOG, 12)` is retained in the forced probe and normal
  images. The adapter separately polls the real RUNNING bit with a bounded
  CPU wait; SDK 2.3.0 `tick_is_running()` checks ENABLE only on RP2040.
  No watchdog .c source or watchdog reset counter is enabled.
- No SDK startup, stdlib, IRQ, sync, multicore, clocks or time runtime
  sources are added. `clock_get_hz()` is not called by the motor adapter
  or probe. The template's radio compatibility shim with that name
  already returns constant 125 MHz; it is not SDK clock state.
- Fixed 125 MHz gives div 1/wrap 6249 motor PWM and div 125/wrap 19999
  servo PWM. The SDK reset/unreset helper is usable without runtime and
  releases PWM with `USE_PTMR=0`. CLK_ADC is already configured to 48 MHz
  by template startup; this is a source check, not a hardware result.
- SDK 2.3.0 GPIO/timer headers have warnings under the strict conversion
  flags. `-isystem` separates vendor headers from application diagnostics.
  Motor setup uses constant pin calls so GCC can check RP2040 slice bounds
  in the SDK's shared RP2040/RP2350 helper. No warning is disabled in our code.

## Interrupt priority and open checks

GPIO IRQ13: unshifted priority **1**, core 0 only, `tk_def_int(TA_HLNG)`.
USB IRQ5 keeps reset priority **0** because the existing USB bridge does
not write its priority. GPIO is below USB, above SysTick/PendSV priority 3.

Operator post-flash S2-U01/U02/U03 and B4-lifted functional checks passed;
see the exact results and lifted-versus-floor limits in the S2 log.
S0-U01 on the fixed image, S2-U04 and floor B4 remain **Not run**. Read
[the S2 test log](../../../testing/subsystem2/TEST_LOG.md) before any hardware
work: **lift both wheels before power with TEST=1**, which sends a real
brief speed command. The normal bench needs GP21 to start after 1 s;
GP21 again latches ESTOP, GP20 prints the saved run. Never reset before
dumping. No printing occurs during a run; dumps pause after eight rows.

The bench records one row after each completed 10 ms motion tick, as in B3.
Motion wakes the registered bench task after releasing its mutex. The bench
uses `tk_slp_tsk(20)` and drains extra wakes with `tk_can_wup()`; extra wakes
and tick gaps count lost rows without double counting. `skipped_rows` must
be 0 in a normal run. Do not sample with `tk_dly_tsk(10)`: this kernel adds
one 10 ms `TIMER_PERIOD` to a task delay, causing about 20 ms between rows.
Button polling remains 20 ms and step timing uses `drv_time_ms()`.
Task functions must not return: a failed bench registration prints one
error line, then calls `tk_ext_tsk()`. Motion has an endless task loop;
subsystem test functions run inside the initial task, not as task entries.

## Cold-power defect and remaining hardware gate

A cold VIN boot exposed a TIMER source rate about 42.7 times too slow:
11165 kernel motion ticks but TIMER task gaps only 233..235 us. A 500 ms
rest lasted about 21 s before an unexpected forward step; the row buffer
filled. The local source log is `build/mtk_logs/usb_console.txt`.

The RP2040 WATCHDOG_TICK reset value enables a divide-by-512 tick, not the
required divide-by-12 tick from 12 MHz `clk_ref`. A post-flash reboot can
retain boot ROM setup and hide the fault. `usermain()` now initializes the
tick before subsystems; motion sets zero motor PWM, then initializes and
checks TIMER against actual elapsed kernel time, rejecting error above 2%.
S0-U01 checks about 1 s at a tighter 1% limit. It is **Not run** on hardware.
Test every kernel image after a cold power-on, not only after a flash.

Template Wi-Fi `time_us_64()` in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c` reads this same TIMER
and has the same defect. Early application initialization covers later
calls; no template kernel/library files were changed. Floor B4 stays
blocked until the fixed image passes cold-power S0-U01.

If task-gap jitter exceeds 1 ms, stop floor testing and review Option H;
it is not implemented. Do not treat compile/link success as approval of
hardware timing, electrical behaviour or the full RTOS vehicle plan.

## STEP 8 early kernel TIMER source

This supersedes the old boot-order dependency, not its historical results.
[Template changes](../../architecture/TEMPLATE_CHANGES.md) now permit reviewed shared edits.
`Reset_Handler` calls `knl_startup_hw` before data/BSS/kernel initialization;
that function selects clk_ref/XOSC then performs bounded register-only
WATCHDOG_TICK setup (12 cycles, ENABLE, RUNNING), before any device/task.
`drv_time_init` remains idempotent verification and the 2% rate check is
unchanged. Motion alone still feeds the watchdog. New-image cold-power
S0-U01 on robot/TEST/BENCH and MQTT is **Not run**. No warm reboot or build
substitutes for that check. Current build/strict/link results are in
[Kernel robot](KERNEL_ROBOT.md); the old unchanged-template table applies
only to its original STEP 1 revision.
