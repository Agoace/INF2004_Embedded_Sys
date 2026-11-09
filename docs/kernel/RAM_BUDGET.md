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
| IMU (subsystem 4) | Static stack, bias/support/roll state, snapshot and libm state | about 3 KB | **5888 B** in MQTT, including 2048 B stack; diagnostic C below |
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
[S4 evidence and pending checks](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679).

## IMU step 2c measured delta

Same public fixture/toolchain as step2b; no firmware flashed.

| MQTT static RAM | Bytes | Delta |
| --- | ---: | ---: |
| Original b569adc baseline | 184784 | reference |
| Step2b75ff329 | 188812 | +4028 |
| Step2c: data904 + bss188308 | 189212 | **+4428 original; +400 step2b** |

Static remainder72932 B before dynamic kernel use. The2048 B IMU static
stack is included; no queue/stack reduction, packing or CR1 work is used.
New storage is the bounded roll mean/median state, output copy and INT1
counters. STREAM grows424 B overall: common400 B plus net24 B for its
observer Welford/timestamp state and link alignment.

Host core/output/probe/snapshot sizes644/228/68/372 B. Strict ARM local
frames: sample784, core120, roll136, entry16 B. These are NOT total stack
bounds; sorting uses a bounded20-float local array (capacity31 configurable).
New physical high-water/CPU acceptance is **Not run**.

Step2b hardware supplied by the operator: work average1562/max2530 us,
read564/705 us, core818/1053 us, max gap18.4 ms, missed0,21400 reads and
zero faults/resets with a new cable. Average work is15.62% at100 Hz, not
exclusive core CPU use. It does not validate this changed math/snapshot.
Step2c adds a roll sine, expected-pitch asin and bounded median, keeping
work/core/read counters. Hardware timing, both stacks and flat/driven noise
must be measured, not inferred from host throughput or step2b wall time.
See [current checks](../../testing/subsystem4/TEST_LOG.md#step-2c-hardware-on-8c5d679).

## IMU step 2d measured delta

Same public example settings and toolchain as step 2c; no firmware flashed.

| MQTT static RAM | Bytes | Delta |
| --- | ---: | ---: |
| Original b569adc baseline | 184784 | reference |
| Step 2c 8c5d679 | 189212 | +4428 |
| Step 2d: data 904 + bss 188580 | 189484 | **+4700 original; +272 step 2c** |

The common growth is the probe state: FIFO counters, the 48 B raw batch
and the float mean. The probe is held three times (probe, snapshot value
and board copy); the snapshot is now 460 B. MQTT text grows 1164 B.
The 7888 B raw capture exists ONLY in the IMU_STREAM image: its bss grows
8176 B (capture plus the common 288 B). Other images keep no capture
storage or code. The 2048 B IMU static stack is unchanged.

Strict ARM local frames (`-fstack-usage`, IMU_STREAM): IMU `sample` 800 B
(step 2c 784 B), `imu_probe_read` 128 B including the 48 B burst buffer,
`imu_hw_read` 40 B; TEST observer `subsystem4_test` 776 B, snapshot read
488 B. These are frames, not total stack bounds. The estimate was a bus
read of about 1.1 ms per 10 ms at 4 samples, versus about 0.56 ms before;
see [IMU step 2d](../s4-imu/IMU.md#read-path-and-the-2-ms-deadline).
Floor hardware on `0207b54` measured read_us average 1315/max 1610 us,
work_us average 2699/max 3224 us (about 27 % of core 0 wall time; step 2b
1562 us), core_us 1164/1430 us, 3.913 samples per poll and zero faults.
The increase is mostly busy-wait I2C: a follow-up item. Physical stack
high-water was not supplied. See
[step 2d hardware](../../testing/subsystem4/TEST_LOG.md#step-2d-hardware-on-0207b54).

## IMU step 3 measured delta

Same public example settings and toolchain as step 2d; no firmware flashed.

| MQTT static RAM | Bytes | Delta |
| --- | ---: | ---: |
| Original b569adc baseline | 184784 | reference |
| Step 2d 0207b54 | 189484 | +4700 |
| Step 3: data 904 + bss 188996 | 189900 | **+5116 original; +416 step 2d** |

Main RAM not statically reserved: 72244 B. MQTT text grows 3620 B. The
2048 B IMU stack is unchanged.

| Step 3 item (MQTT ELF symbols) | Bytes |
| --- | ---: |
| Detector state in `core` (10-sample window, sum, counters) plus its 44 B output | 148 |
| Hump output inside the snapshot `value` and its board copy | 88 |
| IMU hump board (52) and its writer copy `hump_value` (48) | 100 |
| Vehicle last good copy `hump_copy` and its flag | 49 |
| Run result `run_hump` (6) and its DUMP board `run_hump_board` (12) | 18 |
| Summary board growth (summary 48 to 52 B) | 4 |
| `run_epoch`, `hump_emitted`, alignment | 9 |

The snapshot holds the hump output because `imu_core_output_t` holds it;
88 B can be removed later if the snapshot copies only selected fields.
The GP20 DUMP line reads the 12 B run board; `telemetry_info_t` does not
grow, so the telemetry message queue keeps its size. DUMP rows stay 16 B.

Strict ARM local frames (`-fstack-usage`, before -> after): IMU `sample`
792 -> 944 B (STREAM 800 -> 952 B), `imu_snapshot_try_read` 488 -> 528 B,
`imu_hump_try_read` 80 B, `imu_core_step` 144 B, `imu_hump_step` 48 B;
MQTT `vehicle_task` 792 -> 848 B, `telemetry_task` 264 -> 280 B; TEST
`subsystem4_test` 776 -> 816 B. These are frames, not total stack bounds.
H1 on `1480b37`, image `mtk3pico_smp0_usb_cdc_test_imu_stream.uf2`,
measured IMU high-water 1448/2048 B, free 600 B (dfade14: 752 B used).
Sonar used 320/2048 B; telemetry used 464/4096 B. These operator-supplied
STREAM results exceed the 256 B free IMU margin. Same-boot robot image
`d51d27b` later measured IMU 1456/2048 B (592 B free); see diagnostic C
below for the new frames and required repeat.
See [IMU step 3](../s4-imu/IMU.md#step-3-hump-report).

## IMU diagnostic C measured delta

Public example settings, same ARM toolchain, base `d51d27b`. No new image
was flashed. MQTT static RAM is 190672 B (data 904 + bss 189768), +772 B.
Main RAM not statically reserved is 71472 B; MQTT text grows 4692 B.

| New storage (MQTT ELF symbols) | Bytes |
| --- | ---: |
| Pure diagnostic state `hump_diag`: 10 observations, current/winning records, counters | 380 |
| Service writer `diag_value` and observation `diag_board` | 72 + 76 |
| Vehicle `diag_copy`, `run_diag`, `run_diag_board` | 72 + 72 + 76 |
| Vehicle `have_diag_copy`, `line_run_epoch` | 1 + 4 |
| Reused roll heights, travel delta and validity retained in core | 16 |
| Alignment | 3 |

The 10-observation diagnostic window is OUTSIDE core's rollback copy and
outside the normal snapshot. No telemetry message, DUMP row or MQTT wire
structure grows. The 72 B report board has one writer and bounded readers.

| Profile | text | data | bss | bss change vs d51d27b |
| --- | ---: | ---: | ---: | ---: |
| Default | 339244 | 760 | 131748 | +772 |
| Robot | 92372 | 688 | 125212 | +772 |
| BENCH | 41982 | 148 | 95460 | 0 |
| MQTT | 403848 | 904 | 189768 | +772 |
| TEST | 84692 | 688 | 124332 | +544 |
| MQTT-TEST | 392044 | 904 | 188492 | +552 |
| STREAM | 82892 | 688 | 132164 | +624 |

Strict ARM local frames: `sample` 944 -> 1000 B (STREAM 952 -> 1016 B),
core step unchanged 144 B, diagnostic step 56 B plus latch 16 B; diagnostic
board read 96 B. MQTT vehicle 848 -> 872 B; telemetry 280 -> 352 B;
STREAM observer unchanged 816 B; diagnostic USB printer 168 B. The print
runs after STOP or on the larger telemetry/observer stack, not on IMU.

Operator supplied robot `d51d27b` high-water 1456/2048 B, free 592 B;
STREAM on 1480b37 used 1448 B. The extra robot sample frame is 56 B; the
new diagnostic branch (56 + 16 B) is smaller than the existing 144 B core
step frame. The initial frame-only estimate was about 536 B free.

Diagnostic C hardware on `29d57de`, boot `1600432639`, measured IMU
free 536 B initially (high-water 1512/2048 B), then free 480 B after
hump runs (high-water 1568/2048 B). The remaining policy margin is
480 - 256 = 224 B. Hump runs exercise event/latch paths beyond the flat
case. The observed 56 B increase matches the diagnostic-step frame,
but does not prove the exact deepest call chain or interrupt contribution.
High-water is cumulative since task start; frame sizes are not whole-stack
bounds. No stack/queue reduction is used. Further trace/filter changes
are deferred; future changes require a new named stack measurement.
See [hardware record](../../testing/subsystem4/TEST_LOG.md#diagnostic-c-hardware-on-29d57de).

Heartbeat stack_min_free 272 B in the d51d27b session was idle_meter
(size 512, high-water 240), not IMU. See all named stacks and the shared
barcode-sensor scrape condition in
[H0/H2-short](../../testing/subsystem4/TEST_LOG.md#step-3-h0-and-h2-short-on-d51d27b).

## H3 source-label build delta

Source-labelled MQTT images retain data 904 + bss 189768 = 190672 B for
either source (no RAM growth). Text is 403988 B corrected / 403984 B
filtered, versus 403848 B on diagnostic C: +140 / +136 B. Strict ARM
IMU sample remains 1000 B; vehicle is 880 B (+8), telemetry 360 B (+8),
initial task 24 B. Named stack readings for these images are pending H3.
See [source builds](BUILD.md#h3-source-comparison-images).

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
