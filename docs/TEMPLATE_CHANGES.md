# Local course-template changes

## Scope and reference

The operator permits small, justified template changes for STEP 8. Template
source is no longer frozen. Commit shared changes on `mtk3`, one change per
commit, and merge them into module and integration branches with `--no-ff`.
Do not change the original `main` branch. The reference is the imported
`template/main` revision `c3b0954`. The following table is the complete list
of changed files under `kernel/`, `lib/`, `device/`, `include/`, `config/`
and `etc/` against that reference. No SDK startup/runtime is added.

## Deviations

| File | Change and reason | Re-apply after a lecturer update |
| --- | --- | --- |
| [kernel/sysdepend/pico_rp2040/hw_setting.c](../kernel/sysdepend/pico_rp2040/hw_setting.c) | After `startup_clock`, establish the 1 us TIMER source before any device or task. On failure, stop before device startup. | Keep the call after clk_ref selects the 12 MHz XOSC and before device/task startup. Do not reset TIMER. |
| [kernel/sysdepend/pico_rp2040/devinit.c](../kernel/sysdepend/pico_rp2040/devinit.c) | Include the public fixed-width Wi-Fi start declaration, not a separate kernel `ER` declaration. This completes caller/definition ABI consistency. | Keep the public declaration at every caller; the header has no SDK dependency. |
| [kernel/sysdepend/pico_rp2040/timer_tick.h](../kernel/sysdepend/pico_rp2040/timer_tick.h) | New register-only helper: WATCHDOG_TICK `0x4005802c`, cycles 12, ENABLE bit 9, RUNNING bit 10. Reuse correct configuration; at most 100000 RUNNING checks. No SDK headers. | Check addresses/bits against the RP2040 register specification and retain the bounded wait and host tests. |
| [lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c](../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c) | Create Wi-Fi at priority 20 directly. Add consistent lifetime/window poll timing and start-result readers. Use the public fixed-width declaration. | Keep core-0 ownership, USB 3, motion/sensors/vehicle 10–13, Wi-Fi 20 and telemetry 30. Keep short local interrupt masks around timing copies/updates. |
| [lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c](../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c) | Measure full poll wall time and start-to-start gaps. Call named app hook after `sys_check_timeouts` on link-up polls under `TM_MQTT`. Do not log the configured SSID. | Keep all lwIP calls on this task and keep private settings out of logs. Preserve the existing 2 ms requested delay; actual delay uses the kernel tick. |
| [lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h](../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h) | Declare app hook and bounded timing/window/start-result API. | Keep source and declaration types identical. ARM newlib `int32_t` is `long`, while kernel `ER`/`INT` are `int`; equal width does not make C declarations compatible. |
| [lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c](../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c) | Weak app hook calls the existing demo. A strong application definition replaces it without a linker wrap. | Keep the weak default in a different translation unit from the caller. Prove the final caller binds to the strong app function. |

The original priority 4 placed the radio below USB 3 but above the template
qualification task. The robot places the network below driving tasks: a
network problem must not delay a motor update. No priority wrapper, SDK
runtime, extra core or network-linked motor/watchdog path is used.

## Timing contract

`cyw43_utk_timing_read` copies lifetime counters. Window begin/end maintain
independent counters and exclude polls crossing a boundary. Lifetime values
never reset. A second begin or end without begin returns `E_OBJ`; null
output returns `E_PAR`. Readers and updates use short core-0 interrupt masks,
not a mutex or retry loop. The largest timing copy is 28 bytes. No driver,
network call or wait runs inside these masks.

The 2 ms requested Wi-Fi delay is not a 2 ms poll contract. A 10 ms kernel
tick plus `tk_dly_tsk` extra-tick semantics yields about 20 ms between polls
(roughly 50 polls/s), confirmed by the operator. The follow-up documents this
property without changing it. Boot identity and opt-in motion diagnostics
add no template-source deviation; this ledger remains seven paths.

Metrics are **wall time, not CPU load**: polls, complete start-to-start gaps,
maximum gap, gaps strictly above 10000 us, maximum and accumulated work
interval, and maximum probe bookkeeping interval. All include interruption
and preemption. Work excludes the requested sleep; gap includes it. Probe
cost excludes its final max stores. Totals/counters use uint32 modulo
arithmetic; cumulative work wraps after about 71.6 minutes of measured work.
On-target cost is not yet measured; `probe_max_us` supplies that evidence.
The same independent window API supports TEST S2-U01 and a driving session.
A no-poll window is not evidence of network performance.

## Cold-power guarantee

The reset WATCHDOG_TICK divider does not provide the required 1 MHz TIMER
clock. Startup now establishes it immediately after the reference clock,
before device initialization, initial task creation or Wi-Fi start. This
removes the former dependency on initial-task priority and application boot
order. `usermain` still calls the idempotent `drv_time_init`; motion retains
the unchanged 2% TIMER-to-kernel check and motion-only watchdog feed policy.
A correct tick is checked, not restarted. Cold-power hardware checks on the
new robot, TEST, BENCH and MQTT images remain **Not run**.

## Update and check procedure

1. Inspect lecturer changes before merging a new `template/main` revision.
2. Re-apply only the listed deviations. Review clock, task ownership and ABI.
3. Update this ledger and its pinned reference if the template moves.
4. Run `python3 scripts/check-template-changes.py`. It checks the file list,
   not the correctness of each change.
5. Run host tests, strict real-header ARM builds, firmware profiles and SDK
   link proof. Inspect hook binding and startup order in map/disassembly.
6. Repeat cold-power S0-U01 and radio/MQTT/motion hardware checks. Do not
   replace a Not-run result with a software build result.

The next proposed template change, a picotool USB reset interface, is queued
for a later task. It is not implemented here.
