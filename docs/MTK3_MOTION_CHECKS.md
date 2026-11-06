# μT-Kernel motion bring-up checks

## Scope and local revisions

This record covers step 1 only: shared GPIO/time platform, the S2 motor
task, SDK hardware adapter and RAM floor bench. No line/vehicle/scan task
is connected. No hardware was flashed. All on-target tests remain **Not run**.

- Base `mtk3`: `c24abf6`; platform commit `3149f18`, SDK hook `ec45183`,
  external-header policy `07f2669`, repeatable link proof `c24abf6`.
- `mtk3-motion`: `3d7b35e`; base merged before implementation (`4535e90`),
  task/bench `8ef6709`, SDK adapter `959c2b0`, constant-pin setup `f268bdd`,
  per-tick bench notification `e5c96fe`, safe task exit `3d7b35e`.
- Candidate code merges: `mtk3-staging-next` through `c5917e2`.
  Both base and motion use real `--no-ff` merges; no history was rewritten.
- `mtk3-staging` remains at `2117122` in the main checkout. The operator
  will review the candidate before Main_Worker advances it. No push.

All work uses `/tmp/mtk3m` and a temporary motion-branch worktree,
`/tmp/mtk3mo`, outside School-Repos. The motion worktree was removed after
merging. Initial results are in `/tmp/mtk3-motor-results/`; the per-tick
fix rerun logs are in `/tmp/mtk3-notify-results/`. Host and firmware build
directories and link-probe outputs are removed after checks. The main
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
| Strict ARM compile of new platform/motion units (initial bring-up) | 9 per profile, 27 total; pass |
| Retained SDK hardware link probe, all three profiles | Pass; no unresolved symbols |
| Committed `scripts/check-sdk-hw-link.py`, all three profiles | Pass |
| Independent strict host CMake/CTest suite | 22/22 pass |
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
registration or after disabling it. These are not timing or hardware tests.

The per-tick fix reran all three clean firmware builds and SDK link proofs,
the strict 22-test host suite, the 6-test simulator suite, the full
bare-metal Pico W build and the documentation checker. Simulator output
used `/tmp/mtk3-notify-sim` because `/tmp/simb` had another checkout's cache.
Bare-metal configuration used only a temporary copy of
`src/wifi_config.h.example`; it was removed after the build.

## Image sizes and SDK change

Sizes below are bytes from `arm-none-eabi-size`, normal images only.
The link-only probe is not in these images.

| Profile | Text | Data | BSS | Text change from register adapter at `1aa506b` |
| --- | ---: | ---: | ---: | ---: |
| UP, radio off | 39762 | 28 | 75916 | -31102 |
| UP TEST, radio off | 37438 | 24 | 18300 | -31566 |
| Default SMP, radio on | 285606 | 92 | 82356 | +236 |

Data is unchanged from the register adapter; BSS adds 4 bytes for the
notification client task ID. The UP text decrease
is **not** an adapter-only saving: the app SDK hook enables section garbage
collection for radio-off profiles, which the template previously enabled
only with radio on. The default SMP profile already used it. The normal
bench reserves 2400 static rows (57600 bytes); TEST builds omit that log
and bench task. No kernel capacity/configuration value was changed.

## SDK link result and boundaries

The exhaustive include/source list and repeat commands are in
[Motor controller — SDK hardware subset](MOTOR_CONTROLLER.md#sdk-hardware-subset)
and [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk).

- PWM and resets: inline SDK functions, no .c library source required.
- GPIO: SDK `gpio.c`; reuse the existing radio object when enabled.
  Unused SDK IRQ helpers are discarded, not linked to an SDK IRQ runtime.
- ADC: SDK `adc.c` compiles; the forced link probe retains `adc_init()`
  and calls inline GPIO/select/read operations. It is never run. There is
  no step-2 line driver in this change.
- Timer: hardware header and inline `timer_time_us_32()` compile/link;
  no timer/alarm/time initialization .c files are added. The application
  64-bit read stays a stable high/low register read.
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

S2-U01/U02/U03/U04 and B4 remain **Not run**. Read
[the S2 test log](../testing/subsystem2/TEST_LOG.md) before any hardware
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

There is no measured jitter, physical stop, encoder sign or B4 result yet.
If task-gap jitter exceeds 1 ms, stop floor testing and review Option H;
it is not implemented. Do not treat compile/link success as approval of
hardware timing, electrical behaviour or the full RTOS vehicle plan.
