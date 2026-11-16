# Kernel bench step 0

## Scope

Run the real radio-free robot tasks on a spare Pico, in UP and split mode.
`BENCH_KERNEL=1` selects bench-only hardware providers. Do not connect this
image to the robot. Coding workers do not flash hardware.

Alarm1 supplies 3800 encoder edges per second, alternating wheels, for
600 seconds. Alarm0 remains the native line-service timer. The virtual GPIO
provider feeds the unchanged GPIO dispatcher and native motion callback.
No provider assigns native encoder counts. A missed source opportunity or
pending-edge collision stops the generator; it never replays missed edges.

## Build and run

From the repository root, with the documented compiler and SDK configured:

```sh
make -C build_make SMP=0 WIFI=none NET=none MQTT=0 BENCH_KERNEL=1 -j8
make -C build_make SMP=1 WIFI=none NET=none MQTT=0 BENCH_KERNEL=1 -j8
```

Images end in `_bench_kernel_step0.uf2`. The operator loads each image on
the spare Pico and records USB output from boot through the final dump.
The generator starts after application startup. After it stops, the core-0
button provider publishes stopped statistics before a virtual GP20 press.
The press starts the normal dump about 500 ms after the stop. Do not press
GP21; this step does not start a vehicle session.

The report follows `[timeline]`. First come the robot `[load]` line, a
`[bench_load]` line and the P1 lines (`[kcall]`, maxima, overrun). The robot
prints these only with MQTT, so the bench prints them here:

| Line | Window and source |
| --- | --- |
| `[load]` | Core-0 idle-meter load over the whole window: one sample at window open, one at stop. Split prints core 1 as 65535 (no core-1 meter). |
| `[bench_load]` | Calibrated meter rate; `WRAP_RISK` if over 7146368 per second (the 32-bit count can wrap in 601 s). |
| P1 | Run epoch 1 opens at window start and closes (epoch 2) at stop. |
| `[bench_top]` | One line per P1 counter: count of values >= 100 us and the 20 largest values in the window (`--wrap=rt_call/rt_max`, `bench_top.c`). |

The bench starts the robot idle meter (`bench_idle_meter.c`, priority 32,
768 B stack, not in `[stack]`).

Then `[bench]` gives the measurement window, scheduled
and ACK-delivered totals, generator time and the cost verdict. A second
`[bench]` line gives each wheel's scheduled, delivered, native counted and
`diff` values, followed by `COUNT_MATCH` or `COUNT_MISMATCH`.
`COUNT_MATCH` needs overflow 0, scheduled == delivered and `diff 0` on both
wheels.

Native counts come from the public bounded
`motion_snapshot_try_read()` API in `app_program/subsystem2/motion.h`.
The report reads the last motion-tick board only after stopped statistics
are available. The generator has stopped, pending IRQs have drained, and
normal motion ticks continue before the dump. A failed board read prints
`counts unavailable COUNT_MISMATCH`; it is not a pass. `diff` means native
count minus ACK-delivered count; extreme invalid differences saturate at
signed 32-bit limits for the console formatter.

## RAM hot-path diagnostic (UP only)

`make SMP=0 FORCE_CORE0=0 WIFI=none BENCH_KERNEL=1 BENCH_KERNEL_RAM=1`
builds `..._bench_kernel_step0_ramdiag`. Same run procedure and report.

| Item | Rule |
| --- | --- |
| What moves | Sections in `app_program/bench_kernel/ram_hot_path.list`: dispatcher, interrupt entry, timer, `disint`, the `[kcall]` kernel calls and their wait/ready helpers, line and encoder ISRs, bench providers, `drv_time_us`. |
| How | `objcopy --rename-section .text.X=.data.ramfunc.X` on compiled objects; the unchanged template linker copies `.data.*` to RAM. No source or template edit. |
| Proof | `ram_proof.py` runs after the link and fails the build if a listed symbol is not in RAM. Output: `<image>.ramproof` (count, bytes, flash callees, indirect calls). |
| Compare | Against the same image with `BENCH_KERNEL_RAM=0`: `[kcall]` mean and max, `[load]`, ISR and line maxima. |



Pass each image only when all of these conditions hold:

- No reset during the complete 10-minute run; retain boot/reset output.
- Window is 600000000 us, with 2280000 scheduled and delivered edges.
- Each wheel has 1140000 scheduled, delivered and native counted edges,
  with `diff 0` and `COUNT_MATCH`.
- Coalesced and overflow counters are zero; the cost verdict is `OK`.
- Generator cost is below 3 percent of core 0 (`cost_bp < 300`).

`COUNT_MATCH` alone is not a pass (reset, window and cost still apply).
`GEN_STOPPED`, `BAD_WINDOW`, `COST_HIGH`, missing output or
`COUNT_MISMATCH` rejects the run. The real motion watchdog remains enabled.

## Limits and software checks

This is an encoder/interrupt load test, not a robot or sensor acceptance
run. Motor and servo writes are retained in RAM; no motor PWM is emitted.
Line samples are constant 1800. IMU operations return NACK, and sonar has
no echo. Thus IMU and sonar use error paths, not normal sensor workloads.
The generator measures its handler body, not kernel IRQ entry/exit cost
(approximately 1–2 us per call, not included in `gen_us_sum`).
Actual interrupt timing, reset behavior and cost require operator evidence.
No hardware run has been performed by this worker.

The recovered package passed GCC and Clang ASan/UBSan host suites
(531 tests each), and UP/split firmware builds. The runtime host test checks
2280000 events and publication before even a late first button poll. The
native encoder integration test uses the real dispatcher, motion callback
and controller. Report tests check native matching and mismatching counts.
These host checks do not establish hardware acceptance.

Ordinary sources are unchanged. The bench directory is excluded from
ordinary links. SW7's 76-file ELF/UF2 comparison against `4d583c6`, including
debug data, is the ordinary-image gate; rerun it after bench changes.
