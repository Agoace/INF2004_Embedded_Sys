# CPU load observation

## Method

MQTT builds create `idle_meter` at priority 32, the lowest priority in this
kernel configuration. It runs on core 0 and uses a static 512-byte stack.
It does not sleep, lock a mutex, call the network, or feed the watchdog.
Other application tasks and interrupts can preempt it.

At the start of `usermain`, after TIMER initialization and before application
startup, calibration disables interrupts for a 4000 us TIMER window. The
calibration and task call the same `noinline,noclone` counting function. The
loop checks TIMER and increments a volatile counter. Its measured count rate
is the zero-load reference. It is not the highest rate observed during normal
operation, which would include permanent interrupt work.

The priority-31 health task samples count and time once per second:

```text
load_permille = 1000 - round(1000 * count_rate / calibration_rate)
```

Results clamp to 0–1000. Missing calibration, the first sample, and zero elapsed
time return 65535 (unknown). Unsigned differences handle count and TIMER wrap.
Calibration or task-creation failure is diagnostic only; it does not gate
startup or change motion-only watchdog feeding.

## Meaning and limits

- Interrupt execution and all tasks on core 0 reduce the count and count as
  busy time. With SMP=0, Wi-Fi also runs on core 0.
- An SMP build measures core 0 only, not aggregate two-core load.
- The task replaces kernel WFI sleep. Power consumption increases. Its low
  priority prevents it from preempting driving, but target timing must still
  be checked after this change; no zero-overhead claim is made.
- Cache state, memory access, calibration boundaries and sampling boundaries
  can affect the estimate. There is no claimed target accuracy yet.
- Per-task execution time is not available cheaply from this kernel. No
  per-task accounting or dispatcher modification is included.

## Evidence

`tests/test_cpu_load.c` checks all integer loads from 0 to 1000, unknown states,
clamping, counter/TIMER wrap and non-integral reference rates. Host arithmetic
error is at most 1 permille for those synthetic one-second samples. This is
not a measurement of target accuracy.

The MQTT and MQTT TEST builds compile without warnings. Symbol inspection
shows one shared `idle_meter_loop`; calibration is not a separate optimized
copy. Full profile validation is still required.

S1-U07 prints load and reference rate after a 2.2-second yield in the TEST
image. Use an idle board with wheels lifted. Record the Wi-Fi link state,
three successive readings, deadline counters and stack margin. Compare the
reported value with an independent target timing measurement before stating
an accuracy bound.

**Operator target observation, 2026-11-07:** idle 117 permille; heartbeat
under traffic 139 permille; calibrated peak 4026729 iterations/s. These are
observed values, not an independent absolute-accuracy proof or an approved
acceptance range. See [hardware evidence](MTK3_STEP8_HARDWARE_RESULTS.md).
No device was flashed for the later boot/session diagnostic follow-up.

See [wire contract](MQTT_WIRE.md#heartbeat),
[test rows](../testing/subsystem1/TEST_LOG.md), and
[application stacks](APPLICATION_TASKS.md).
