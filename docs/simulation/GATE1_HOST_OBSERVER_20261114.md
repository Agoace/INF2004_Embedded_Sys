# Attempt15 host observation — 2026-11-14

**TRIAL / NOT GATE 1 PASS.** Operator-approved measurement only on the no-C3
C1+C2+C2b+C2c line. Timing limits and source scheduling do not change.

`--measure-io` adds a preallocated32768-span numeric tail for select, read and
sleep. Each span records RAW wall start/end, calling-thread CPU nanoseconds
(`time.thread_time_ns`) and RUSAGE_THREAD context-switch/page-fault deltas.
The CPU/rusage samples enclose the wall bracket and include observer work.
They are diagnostic observations, not exact off-CPU subtraction or estimator
inputs. Counter failure is explicit -1/UNKNOWN, not zero. No procfs is read in
an iteration. Existing read-return timestamps are taken BEFORE post-observer
work; t4 is not silently moved to the end of counter collection.

Files `host-spans.csv/json` are a bounded tail, not full-run proof. They export
after serial close, with overwrite/error counts and trial identification.
Whole-run host snapshots `host-before.json` and `host-after.json` are collected
outside the loop. They record thread/process identity, policy/nice/affinity,
thread status/schedstat, IRQ/softirq counters, cgroup CPU limits/throttling and
available CPU frequency/idle-state files. Missing/denied files are explicit
UNKNOWN. Discovery/read sizes are bounded. Current frequency and idle deltas
cannot identify a particular stall's cause.

No setter is added: no timer slack, sleep request, switch interval, priority,
GC, governor or affinity change. No kernel tracing and no extra writer thread.

Run the no-device observer benchmark:

```sh
python3 sim/webots/tools/gate1_host_observer_bench.py
```

The20000-sample local whole begin/end benchmark measured p50=3.967us,
p95=4.038us,p99=4.318us,max24.556us; a single RAW-clock baseline p50=0.190us.
This is NOT a hard bound. Three spans per nonempty iteration can add roughly
12us of median sampling work before other instrumentation. Compare against the
prior baseline; observer load can change phase/packet alignment and tails.

Interpretation: large wall time with little thread CPU supports an off-CPU
interval, but does not identify another process or IRQ. Involuntary-switch
increments support preemption, not its duration. Unchanged switch counters do
not rule out interrupts/firmware stalls. schedstat fields are runtime ns,
runqueue wait ns and timeslices, not voluntary/involuntary-switch counts;
disabled statistics must not be interpreted as no contention.
