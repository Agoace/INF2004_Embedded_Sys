# Host observation — attempts15 and16B, 2026-11-14

**TRIAL / NOT GATE 1 PASS.** Operator-approved measurement only on the no-C3
C1+C2+C2b+C2c line. Timing limits and source scheduling do not change.

`--measure-io` adds a preallocated32768-span numeric tail for select, read and
sleep, plus the attempt16B receive span. Each span records RAW wall start/end, calling-thread CPU nanoseconds
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

Attempt16 partB adds one receive bracket per main-loop iteration, including
empty reads. RAW sampling starts before the existing read_exit marker and ends
after receive_exit. This includes I/O clock-pair/log serialization, raw_rx,
decode/CRC/transport validation and receive-event serialization. It closes on
errors in a finally block. The original read-return t4 remains before observer
sampling. Planned-stop drains and post-failure RX tails do not start this new
bracket. It uses the SAME preallocated ring and existing CSV columns; receive
is a new kind, not a new estimator input. More spans shorten retained history.

Snapshots retain per-CPU cpuidle usage/time/name/latency/disable and add read-only
`/sys/devices/system/cpu/cpu*/power/pm_qos_resume_latency_us`. Missing files are
explicit UNKNOWN. There is NO access to `/dev/cpu_dma_latency`. The per-CPU
sysfs value is NOT proof of the operator's separate global PM-QoS held-file
request/value. Compare per-CPU C3 usage/time deltas only over the observed
interval; no entries during one trial are not a future timing bound.

No setter is added: no timer slack, sleep request, switch interval, priority,
GC, governor or affinity change. No kernel tracing and no extra writer thread.
Operator-owned PM-QoS partA is outside this tool and needs separate provenance.

Run the no-device observer benchmark:

```sh
python3 sim/webots/tools/gate1_host_observer_bench.py
```

The attempt16B20000-sample local whole receive begin/end benchmark measured
p50=3.998us,p95=4.077us,p99=5.510us,max22.282us; a single RAW-clock baseline
p50=0.181us. This is NOT a hard bound. The added receive span costs roughly4us
median per iteration. Four spans per nonempty iteration can add roughly16us
of median sampling work before other instrumentation. Compare against the
prior baseline; observer load can change phase/packet alignment and tails.

Interpretation: large wall time with little thread CPU supports an off-CPU
interval, but does not identify another process or IRQ. Involuntary-switch
increments support preemption, not its duration. Unchanged switch counters do
not rule out interrupts/firmware stalls. schedstat fields are runtime ns,
runqueue wait ns and timeslices, not voluntary/involuntary-switch counts;
disabled statistics must not be interpreted as no contention.
