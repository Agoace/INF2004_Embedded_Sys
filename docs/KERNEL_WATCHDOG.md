# Kernel hardware watchdog

## Contract and startup

The motion owner starts the RP2040 hardware watchdog at 100 ms only after
its task and cyclic handler start successfully. Only a completed motion
tick feeds it, after motor writes and mutex release. No cyclic interrupt,
USB dump, vehicle task or TEST elapsed sampler feeds it. Failure or a hang
before a completed tick therefore cannot keep it alive. If motion init or
startup fails, no watchdog is started and no driving is permitted.

`drv_watchdog_start(ms)` requires an earlier successful `drv_time_init()`
in this boot. It accepts 1..8388 ms, starts only once, and checks the actual
WATCHDOG_TICK ENABLE/RUNNING/divider before enabling. It does not change
the tick source. A failed time initialization clears the readiness flag.
This protects only after watchdog startup, not hangs before startup.

SDK 2.3.0 `hardware_watchdog/watchdog.c` provides runtime-free RP2040
`watchdog_enable`, `watchdog_update`, `watchdog_caused_reboot` and
`watchdog_enable_caused_reboot`. The SDK loads `ms * 2000`, not `ms * 1000`,
for RP2040-E1 (the counter decrements twice per 1 us tick). It selects PSM
reset for all domains except ROSC/XOSC and pauses on DBG0, DBG1 and JTAG.
A 100 ms request loads 200000. This is the same SDK setup as the bare-metal
heartbeat. No SDK bootrom, flash, clock, IRQ, sync or time runtime source
is added; new bootrom/flash/boot-lock include paths are declarations only.
The forced SDK link probe retains every used watchdog entry point.

The hardware counts without CPU/interrupt/kernel work. Timeout resets the
chip and returns motor pins to their reset input state; startup initializes
motor PWM to zero again. Physical motor stop and reset latency must still
be checked with wheels lifted. A halted debugger deliberately pauses the
watchdog, so the about-100-ms bound does not apply while debug-paused.
RAM run logs do not survive a reset. This does not replace ESTOP, stale
command handling, safe wiring or the cold-power clock test.

## Last reset reason

The driver snapshots the reason before enable overwrites SDK scratch4:

| Report | Register evidence |
| --- | --- |
| `power/RUN` | Watchdog reason is zero; power, RUN-pin or other hard reset cannot be distinguished here |
| `watchdog` | TIMER reason only and SDK enable magic `0x6ab73121` in scratch4 |
| `other` | A watchdog reset without that magic, or a forced/unknown reason |

The SDK/boot ROM target-reboot magic is `0xb007c0d3`; normal software reboot
may clear scratch4 to zero. Picotool/bootrom reboots through those paths
therefore report `other`, not an app timeout. A tool that physically asserts
RUN is indistinguishable from `power/RUN`. No claim is made that `other`
uniquely identifies picotool. Dumps use the cached boot snapshot, not the
scratch4 value from the newly started watchdog.

## S2-U05 two-boot TEST procedure

This is a destructive reset test, **Not run**. It runs last, after S1..S5,
not inside the early S2 test sequence. Both wheels must be lifted.

1. Cold-power a TEST image. Save earlier test output. S2-U05 stops the motors,
   prints a warning, and allows 1 s for USB delivery while normal feeding
   continues. It then asks motion to suppress feeds after its next completed
   tick and final feed. Motion writes a TEST marker to scratch0 and starts
   the elapsed measurement. The initial TEST task records elapsed ms in
   scratch1 until reset; it never feeds. Expect reboot about 100 ms after
   that final feed. If no reset occurs by 500 ms, it prints FAIL.
2. Do not reset or power-cycle again. Reconnect the USB terminal if required
   and wait for the next boot's later TEST output. S2-U05 reports
   `previous boot: watchdog reset after N ms`, explicitly labeled as the
   last observed pre-reset sample, not an exact timestamp. An observed
   90..110 ms and an own TIMER timeout give PASS. A scope or independent
   timer is needed for a strict physical reset/stop-time measurement.

Scratch0/1 are reserved for this application test; SDK/bootrom scratch4..7
are not used for test metadata. The marker is consumed into RAM before
watchdog enable. On the second boot S2-U05 reports the result and does not
repeat the reset, preventing an endless reset loop. A power/RUN/other reset
invalidates the marker; only an own timeout can produce a previous-test
result. TEST feed suppression and elapsed-write APIs are absent from normal
images. Do not use a debugger halt during this test.

Host tests cover startup prerequisites, no feed before start, cached reset
reason, reason/magic classification, one-time marker consumption, elapsed
wrap, task/cyclic/watchdog ordering and one feed per completed tick. They
do not prove hardware timeout, PWM reset state or physical stopping.
