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

The report follows `[timeline]`. It gives the measurement window, scheduled
and ACK-delivered totals, generator time and the cost verdict. A second
`[bench]` line gives each wheel's scheduled, delivered, native counted and
`diff` values, followed by `COUNT_MATCH` or `COUNT_MISMATCH`.

Native counts come from the public bounded
`motion_snapshot_try_read()` API in `app_program/subsystem2/motion.h`.
The report reads the last motion-tick board only after stopped statistics
are available. The generator has stopped, pending IRQs have drained, and
normal motion ticks continue before the dump. A failed board read prints
`counts unavailable COUNT_MISMATCH`; it is not a pass. `diff` means native
count minus ACK-delivered count; extreme invalid differences saturate at
signed 32-bit limits for the console formatter.

## Pass rule

Pass each image only when all of these conditions hold:

- No reset during the complete 10-minute run; retain boot/reset output.
- Window is 600000000 us, with 2280000 scheduled and delivered edges.
- Each wheel has 1140000 scheduled, delivered and native counted edges,
  with `diff 0` and `COUNT_MATCH`.
- Coalesced and overflow counters are zero; the cost verdict is `OK`.
- Generator cost is below 3 percent of core 0 (`cost_bp < 300`).

`COUNT_MATCH` alone is not a pass: a failed generator can stop with matching
partial counts. `GEN_STOPPED`, `BAD_WINDOW`, `COST_HIGH`, missing output or
`COUNT_MISMATCH` rejects the run. The real motion watchdog remains enabled.

## Limits and software checks

This is an encoder/interrupt load test, not a robot or sensor acceptance
run. Motor and servo writes are retained in RAM; no motor PWM is emitted.
Line samples are constant 1800. IMU operations return NACK, and sonar has
no echo. Thus IMU and sonar use error paths, not normal sensor workloads.
The generator measures its handler body, not kernel IRQ entry/exit cost.
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
