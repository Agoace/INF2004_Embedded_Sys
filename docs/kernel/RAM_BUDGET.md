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
| IMU (subsystem 4) | Task stack about 2 KB, calibration, pitch and hump state | about 3 KB | Not yet |
| Barcode, edge list | Task or line-task work, about 30 bar widths | about 3 KB | Not yet |
| Barcode, raw capture | 141 mm at about 100 mm/s, 1 ms samples, 2 B each | about 6 KB | Not yet |

Worst case is about 12 KB of the estimated 60 KB. A RAM shortage is not
expected. Record the measured value in this table when a module is merged.

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
