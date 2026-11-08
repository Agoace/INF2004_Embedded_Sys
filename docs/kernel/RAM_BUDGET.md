# RAM budget

## What it is

This record shows how the robot program uses the RP2040 RAM. It also gives
the steps to use if the RAM becomes too small. Use it before you add a
module (IMU, barcode) and for the Change Requirement 1 (CR1) RAM comparison.

The RP2040 has 264 KB of RAM: 256 KB (262144 B) main RAM and two 4 KB
scratch banks. The program data and the kernel system memory are in the
main RAM, so the percentages below use 262144 B.

## How it was measured

- Image: normal MQTT robot image
  (`make SMP=0 WIFI=cyw43 NET=lwip MQTT=1`), built 2026-11-08 from
  `689e43e`. The firmware is the same at tag `cr1-pre-imu` (`e37c873`).
- Static RAM = `data` + `bss` from `arm-none-eabi-size`.
- Groups come from `arm-none-eabi-nm -S --size-sort` (one line for each
  RAM symbol). The full symbol list is in the private file
  `build/mtk_logs/cr1_pre_imu_20261108.txt` (not in Git).

## Results: static RAM at `cr1-pre-imu`

| Group | Main symbols | Bytes | Part of main RAM |
| --- | --- | ---: | ---: |
| RAM log for DUMP | `rows` (4501 rows x 16 B = 90 s at 20 ms), `events` | 78160 | 29.8 % |
| lwIP network memory | `memp_memory_PBUF_POOL_base` 36771, `ram_heap` 16403, other pools | 56881 | 21.7 % |
| Static application task stacks | vehicle 4096, telemetry 4096, bench 4096, motion 3072, line 2048, sonar 2048, health 2048, idle meter 512, kernel 512 | 22528 | 8.6 % |
| Kernel control blocks | `knl_*` tables | 11124 | 4.2 % |
| USB console | endpoints, `tx_buffer` 4096, queue | 6989 | 2.7 % |
| CYW43 radio state | `cyw43_state` | 2460 | 0.9 % |
| Other | small symbols | 6501 | 2.5 % |
| **Total static (`data` 368 + `bss` 184416)** | | **184784** | **70.5 %** |
| Not static | | 77360 (about 75.5 KB) | 29.5 % |

The "not static" RAM is not all free. The kernel takes these from its
system memory when the program starts:

| Dynamic item | Bytes (from source) |
| --- | ---: |
| Wi-Fi task stack (lwIP build) | 8192 + 128 |
| USB console task stack (SMP=0) | 2048 + 128 |
| Telemetry message buffer | 4096 |
| Sonar request buffer | 256 |
| Network event/command queues | less than 1024 |
| Initial task stack and kernel use | not measured |
| **Known dynamic total** | **about 15.5 KB** |

The group rows add up to 184643 B; the difference to 184784 B is alignment.

Estimate of really free RAM: about 75.5 - 15.5 = **about 60 KB**.
This is an estimate. Measure the free kernel system memory before you use
it for a decision (open item below).

## Expected need of the remaining modules

| Module | RAM use | Estimate | Measured |
| --- | --- | ---: | --- |
| IMU (subsystem 4) | Static stack, bias/support/filter state, snapshot and libm state | about 3 KB | **4028 B**, including 2048 B stack; step2b below |
| Barcode, edge list | Task or line-task work, about 30 bar widths | about 3 KB | Not yet |
| Barcode, raw capture | 141 mm at about 100 mm/s, 1 ms samples, 2 B each | about 6 KB | Not yet |

With measured IMU and estimated barcode options, the combined need is
about 13 KB of the original estimated 60 KB. A RAM shortage is not expected.
Record barcode measurements when implemented; this is not the CR1 baseline.

## IMU step 2 measured delta

Both MQTT images were built locally from clean objects with ARM GNU
14.3.rel1, `SMP=0 WIFI=cyw43 NET=lwip MQTT=1` and identical public fixtures.
The before image was rebuilt from **`b569adc`**, not inferred from the older
`689e43e` table. No device, network or private configuration was used.

| MQTT measurement | Before `b569adc` | Observe-only step 2 | Delta |
| --- | ---: | ---: | ---: |
| text | 374944 | 390892 | +15948 |
| data | 368 | 904 | +536 |
| bss | 184416 | 187612 | +3196 |
| **Static main RAM: data + bss** | **184784** | **188516** | **+3732** |
| Part of 262144 B | 70.5% | 71.9% | +1.4 percentage points |
| Main RAM not statically reserved | 77360 | 73628 | -3732 |

The delta **already includes** the 2048 B `imu_stack`; do not add the stack
again. The remaining 1684 B covers core/calibration state, raw/probe state,
the snapshot and its bounded board, count/timing state, one task-registry
entry, the vehicle calibration epoch, alignment and libm-related state.
The math-library errno path also retains newlib state (`_impure_data` 512 B,
`_impure_ptr` 4 B, `__sf` 312 B): 828 B of this delta. No compact CR1 fields,
reduced queues, smaller stacks or packed log rows were used to save RAM.

Registered application regions now total 24064 B in MQTT (including the
inactive 4096 B bench reserve). The historical group table above also
includes a 512 B kernel stack, so its comparable stack group is now
24576 B. Existing dynamic buffers and template-owned stacks are unchanged;
the 73628 B remainder is not all free system memory.

Host `sizeof` output is core 236 B, core output 124 B, probe 44 B and
snapshot 244 B; strict task/board tests also pass. ARM sizes above are
authoritative because host pointers/alignment can differ.
The nine-unit strict ARM build emits stack-use records: task entry 16 B,
`sample()` 368 B, `imu_core_step()` 72 B in the MQTT profile. These local
frames are not a whole-call-tree bound: libm, kernel calls and interrupt
context still need margin. The 2048 B registered stack has fill-pattern
high-water reporting in TEST/health. Its physical high-water acceptance is
**Not run**; do not reduce it from host or compiler-local frame sizes.

### CPU estimate and measurement limit

The bus workload remains two reads at 100 Hz. Earlier step-1 hardware read
wall time was about 530..566 us per 10 ms period: **5.3..5.7%** of one
core-period before the new core/publication work. This is an old-image
observation, includes preemption, and is not measured step-2 exclusive CPU.

The old **6..10% of core0** provisional healthy-device planning
estimate (about 0.6..1.0 ms per10 ms) was too low: dfade14 hardware later
reported about13..14% wall-work, as recorded below. Only the bus component had prior
hardware evidence; the allowance for software float/math/boards is not a
target measurement or a guaranteed upper bound. Fault/setup paths can cost
more, and task14 remains below all driving tasks10..13.

The reproducible host-only benchmark is
[tests/test_imu_core_benchmark.c](../../tests/test_imu_core_benchmark.c):
one million moving updates, `-Os`, 73.6..74.8 ns/update across three local
runs. Run the CMake target `test_imu_core_benchmark` from the host build.
This proves host throughput only, **not Cortex-M0+ cost** or bus behavior.

Target snapshots/one-second TEST diagnostics expose `work_last/max/total_us`
and `compute_last/max/total_us`. Work covers the full sample, including
publication, and appears one cycle later. Compute covers pure-core work and
bounded calibration replay. These are wall times and include preemption;
they are not exclusive task CPU times. For a stable interval, compare work
counter differences to elapsed time and the existing idle-meter change.
Record the interval, faults, max gap, missed wakes, USB drops and every
stack margin in [S4-U04](../../testing/subsystem4/TEST_LOG.md#observe-only-step-2-and-step-2b).
Actual step-2 CPU, stack depth and motion timing under this load are Not run.

## IMU step 2b measured delta

Same public fixture/toolchain/profile as above; no private log/config read.

| MQTT static RAM | Bytes | Delta |
| --- | ---: | ---: |
| Rebuilt original b569adc baseline | 184784 | reference |
| dfade14 step2 | 188516 | +3732 |
| Step2b: data904 + bss187908 | 188812 | **+4028 vs original; +296 vs step2** |

The 2048 B static IMU stack is already included, not an extra dynamic
allocation. Static remainder is73332 B before dynamic kernel use. No CR1
packing or queue/stack reduction was used. Added state is the two-axis yaw
compensation, support heights/quality window and larger bounded snapshot.
The GP21 drive state lives only on the TEST observer's existing stack.

Host sizes: core388, output192, probe44, snapshot312 B. Strict ARM compiler
local frames: sample528, core-step112, entry16 B (not whole-call-tree bounds).
Physical IMU high-water752/2048 B was measured only on dfade14; it does NOT
validate these larger frames or the powered TEST observer's stack.

Operator dfade14 hardware: work1.3..1.4 ms/10 ms (about13..14% wall-work),
max2.48 ms, core about0.63 ms, read about0.54 ms. There were three unexplained
resets. This supersedes the older6..10% estimate, not exclusive CPU evidence.
Step2b retains work/core/read last/max/total counters, includes checked
phase replay, and adds only one roll sine (pitch sine is reused). New target
work/stack/timing MUST be recorded after an approved run; **Not run** here.
Do not project host throughput onto the Cortex-M0+. See
[S4 evidence and pending checks](../../testing/subsystem4/TEST_LOG.md#step-2b-pending-hardware).

## Steps if the RAM becomes too small

Use the steps in this order. Stop when there is sufficient RAM. Measure
RAM before and after each step.

| Step | Change | Gain | Risk or cost |
| ---: | --- | ---: | --- |
| 1 | Do not reserve `bench_stack` in the robot image (only the bench task uses it) | about 4 KB | Small; check that no robot task uses it |
| 2 | Set task stacks from the measured high-water marks (keep 256 B margin) | about 5-8 KB | Small; needs all-load stack data |
| 3 | Shorter RAM log: 90 s to 60 s or 30 s | 24 KB or 48 KB | The start of long runs is lost from DUMP |
| 4 | Smaller log row: 16 B to about 10 B with minimum-bit fields | about 27 KB | Work and a new decoder; this is also CR1 work |
| 5 | Log every 40 ms, not every 20 ms | 36 KB | Less detail in analyses |
| 6 | Smaller lwIP pools (`PBUF_POOL_SIZE`, `MEM_SIZE`) | 10-20 KB | More Wi-Fi loss; template configuration change |
| 7 | Remove the RAM log and DUMP | 78 KB | Runs without Wi-Fi have no data; 20 ms detail lost |

## CR1 order

CR1 asks for RAM and message sizes before and after the minimum-bit change.
The team decided (2026-11-08):

1. Build the IMU and the barcode reader in the normal style first.
2. Set tag `cr1-baseline` when both modules work. Measure RAM and message
   sizes there. This is the CR1 "before" value.
3. Do the CR1 work on all modules. Step 4 above (smaller log row) is the
   largest RAM gain for CR1.
4. Measure the "after" values against `cr1-baseline`.

Tag `cr1-pre-imu` is an intermediate point only. It is not the CR1
baseline.

## Limits and open items

- Measure the free kernel system memory on the robot (dynamic stacks and
  buffers included). The 60 KB value is an estimate.
- Static sizes do not show stack depth. See
  [Kernel tasks and stacks](KERNEL.md) for the high-water method.
- Telemetry rate changes (CR1 110 ms period) do not change static RAM, but
  they can fill the telemetry and lwIP buffers faster.
