# IMU safety port and UP validation

## Scope and provenance

Current (2026-11-24): the safety port is integrated in the demo robot image.
Real-IMU push-over runs stopped the robot with LIFT and OVERTURN; see
[capture results](IMU_SAFETY_CAPTURE.md#results-staging-581ab85-image-3_imucap-37eba5a4).
The hump report is separate and does not command motion. The port and bench
results below record measurements on their named revisions.

The original manual port used `mtk3-imu-int`, based on staging `d3890d0`. Do not merge or
change the teammate branch. Source: smojjofy, `8ac6e38..038ff23` (initial
implementation `8ac6e38` included). The source branch's run logs record that
branch's runs. UP robot results are linked above; the original port checks
are listed below.

## Port steps

- A: pure detector and synthetic tests, from `038ff23`. Retain thresholds
  and physics. Reject non-finite/degenerate vectors and future observations.
- B: IMU service board and explicit applied-request token. Preserve staging
  task/core placement and existing hump/diagnostic behavior.
- C: small pure Vehicle safety gate, with bounded glue at existing wake,
  start and stop boundaries. A stop latches once; only an explicit writer
  acknowledgement can release it. No console output in the running path.
- D: additive summary stop reasons and UI labels; preserve staging barcode,
  obstacle, decoder and token-file behavior.
- E: matched UP resources, stack frames, task budgets and spare-Pico plan.

## Detector contract

The IMU task owns the detector. Inputs are mapped, bias-corrected acceleration
(X right, Y forward, Z up), never the encoder-compensated hump estimate.
There is no kernel, I/O, allocation, output or actuator call in the detector.
Loops have fixed bounds. Calibration pose acceptance rejects infinity too.

Triggers count consecutive usable observations, not continuous wall time.
A missing observation neither increments nor resets a trigger count. A
large timestamp gap does not turn five observations into a five-period
continuous tilt claim. Reader freshness stops an active vehicle separately.
An unusable observation resets the detection counters. Host tests cover the
missing-observation case explicitly. The clear window, unlike the trigger
counts, requires continuous fresh, upright, near-1 g, stationary evidence.

The port adds finite-vector validation even when a caller incorrectly sets
`accel_valid`. Invalid/zero vectors cannot report VALID. A reader rejects a
negative signed age as unavailable; ordinary uptime wrap remains supported.
The thresholds and gravity/lift separation stay those of the source. Slow
level lifting and some combined tilt/lift motions remain blind spots; this
is not an independent guarantee that a vehicle is on the floor.

## Integration contracts

The sole IMU writer publishes a coherent safety report. The report contains
an applied-request token. A request has a nonzero token and is consumed
once. Only a successful clear updates the applied token. An empty FIFO does
not refresh the observation or apply a request. A refused clear does not
change the receipt. A failed or interrupted board read leaves its caller's
output unchanged. The service keeps staging's task placement seam.

The pure Vehicle gate has no kernel or actuator dependency. Freshness is
50 ms. A local press starts a bounded 1000 ms recovery intent only after a
1000 ms safe-clear window. Requests retry at 100 ms. A matching nonzero
receipt and a fresh, valid, upright, near-one-g pose release the latch.
An old NONE report, another token, or an unsafe matching receipt cannot
release it. Unsafe input or expiry cancels the intent. Recovery does not
start a run: a later press is required. Remote commands cannot clear it.

A first latch queues ESTOP and one fault event. Later observations retain
the stop without repeating either action. The shell remains in IMU FAULT
through ordinary cleanup. Only a successful recovery resets it. Start,
calibration, delay and run paths use the gate. Idle ticks still process
commands when the IMU is missing; for example, DUMP remains available.
The task parks before the safety check, so a stale board cannot create a
busy loop. Console output occurs only after the active run has stopped.

Summary v3 adds reasons 14–16 and shell fault 6. The decoder retains the
v1/v2 limits. Dashboard changes add labels only. No packet layout, barcode,
scan geometry, configuration-token reader or live snapshot contract changes.

## Status

- A: `d1495c6`, pure detector and defensive tests.
- B: `66ed213`, sole-writer board and successful-clear receipt.
- C: `2f2b182`, pure Vehicle gate, bounded glue and persistent shell FAULT.
- D: `7db62d3`, additive labels and tests; also retain idle command handling while
  safety is unavailable and correct an unsigned-cast compiler warning.
- Fresh host checks: GCC 600/600 and Clang ASan/UBSan 600/600 CTest entries
  pass. Tests include refused clear/retry, empty FIFO, unchanged failed-read
  output, pre-request NONE, wrong receipt, unsafe receipt, future age,
  expiry, first-edge stop, absent-IMU cold start, persistent shell FAULT,
  summary reason and legacy version limits. These are host results, not
  UP timing or real-robot safety acceptance.
- Merge of staging `e1e4ac1` into `mtk3-imu-int`: `1269c7a`.
- E: UP resources, kernel-service list, spare Pico W BENCH1 and the
  TEST-only synthetic bench ([IMU_SAFETY_BENCH.md](IMU_SAFETY_BENCH.md)).
  GCC 644/644 and Clang ASan/UBSan 644/644 pass. The 38 default profiles
  (76 ELF/UF2 files) are byte-identical to `1269c7a`.
- BENCH2 (TEST synthetic image of `f4ffd0e`) ran on the spare Pico W.
  Results and corrections are in
  [IMU_SAFETY_BENCH.md](IMU_SAFETY_BENCH.md#bench2-record-test-image-f4ffd0e--home-secrets).
  The Vehicle does not run in TEST images, so BENCH2 covers the IMU
  service, detector, boards and clear path only.
- F: ROBOT synthetic profile (`IMU_SAFETY_BENCH=robot_synthetic`) for
  BENCH3. It is the full robot demo app with the synthetic stream and
  per-section timers. Starts trigger alternating OVERTURN and LIFT, so the
  real Vehicle gate, ESTOP, FAULT and events run. GCC 645/645 and Clang
  ASan/UBSan 645/645 pass. The 76 default files stay byte-identical to
  `91f9b48`.
- BENCH3 (ROBOT synthetic image of `3809dec`) ran on the spare Pico W.
  The gate latched OVERTURN and LIFT with the Vehicle idle and each GP21
  recovery cleared it. IMU overruns and lost wakes were 0. Results,
  limits and open findings are in
  [IMU_SAFETY_BENCH.md](IMU_SAFETY_BENCH.md#bench3-record-robot-synthetic-image-3809dec--home-secrets).
- G: IMU safety events move to the additive event kind 10 IMU_SAFETY
  ([MQTT_WIRE.md](../s1-telemetry/MQTT_WIRE.md#kind-10-imu_safety)).
  Kind 3 keeps motion fault bits only. One recovery event per press (the
  duplicate PENDING from the C glue is removed) and a new CLEARED event.
  Test `test_vehicle_network_imurecover` was red first. The IMU stack
  stays 2048 B. The first real-IMU gate required `imu_task` free
  ≥ 256 B, with 512 B to be added below the 384 B margin. The later capture
  runs showed 504–528 B free; see [results](IMU_SAFETY_CAPTURE.md#results-staging-581ab85-image-3_imucap-37eba5a4).
- H: one kind 3 ESTOP event per IMU stop. The stop edge reset the kind 3
  dedupe (left from the kind 3 IMU word), so the end-of-attempt motion
  fault report sent word 4 again 3 ms later (K10_1 school run, `dd26a38`).
  The reset is removed; the host motion stub now keeps ESTOP latched, and
  the IMU network cases count one word 4 (red first).
- I: opt-in ROBOT latch capture `IMU_SAFETY_CAPTURE=1`
  ([IMU_SAFETY_CAPTURE.md](IMU_SAFETY_CAPTURE.md)) to find which rule
  fires for a push-over (K10_1 reported LIFT), and a GP21 ESTOP capture
  for a gentle lift that does not latch. Detector unchanged.
  [Results](IMU_SAFETY_CAPTURE.md#results-staging-581ab85-image-3_imucap-37eba5a4):
  push-overs gave LIFT once and OVERTURN once; the label does not matter
  (operator decision), and the settled-pose event is cancelled. Demo
  checklist: reseat the IMU connector.

## UP resources (E)

Builds use the public example credentials, `arm-none-eabi-gcc` 14.3.1,
`-O2`. Base is staging `e1e4ac1`. Candidate is `1269c7a`. Data is 0 in
all builds. Static RAM is `__bss_end` minus 0x20000000; the total is
270336 B.

| Profile | Text base → cand | BSS base → cand | Static cand | Free cand | RAM_PROOF |
| --- | --- | --- | --- | --- | --- |
| Demo: `SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1` | 442936 → 447520 (+4584) | 201324 → 201476 (+152) | 222052 | 48284 (base 48436) | 77/77, 19544 B, same as base |
| Capture: demo + `BARCODE_FAIL_CAPTURE=1` | 446560 → 451136 (+4576) | 218812 → 218964 (+152) | 239884 | 30452 (base 30604) | 77/77, 19772 B |
| Bench (E): demo + `TEST=1 IMU_SAFETY_BENCH=1` | 395812 | 200612 | 210724 | 59612 | 62/62, 8592 B |
| Bench (F, with section timers): same flags | 396296 | 201724 | 211836 | 58500 | 62/62, 8592 B |
| ROBOT synthetic (F): demo + `IMU_SAFETY_BENCH=robot_synthetic` | 447852 (demo 447520) | 205764 (demo 201476) | 226340 | 43996 (demo 48284) | 77/77, 19544 B, same as demo |

The ROBOT synthetic BSS adds mainly the reporter stack (2048 B) and three
568 B statistics copies to the demo.

The home-secrets demo image that Main flashed for BENCH1 had text 447504
and BSS 201476.

ARM `-O2 -fstack-usage` frames, base → candidate. These are static frame
sizes, not high-water values: IMU `sample` 1000 → 1016 B, detector step
64 B, `vehicle_task` 1016 → 1040 B, Vehicle `wait_tick` 152 → 176 B,
gate step 40 B, safety reader 72 B.

## Spare Pico W BENCH1 (E)

Main ran the demo image of `1269c7a` on a spare Pico W. It had home
secrets, no IMU, no sensors and no motors. This is evidence for the idle
and absent-IMU error paths only. It is not timing evidence for the
detector.

- Boot, Wi-Fi and MQTT state/heartbeat were correct.
- GP21: start refused, `KR1 reset other; IMU unavailable`, event kind 3
  word 32771, no calibration. GP20 DUMP completed, `tm_usb_dropped 0`.
- overruns 0, lost wakes 0, faults 0, Vehicle `response_max_us` 370.
- Aggregate CPU with Wi-Fi/MQTT: 213–229 permille in 47 MQTT heartbeats
  (`mqtt_BENCH1.csv`); USB `[load]` 229.
- Stack minimum free (B): motion 2608, line 1552, sonar 1752, IMU 664
  (absent-IMU path), Vehicle 2520, telemetry 3084, Health 1376,
  idle_meter 520, cyw43 7136.

## Kernel services checked vs d3890d0 (E)

`git diff d3890d0 1269c7a -- kernel include config lib` is empty, so the
kernel is the same in both the base and the candidate.

| Service | Used by | Port change | Check vs d3890d0 |
| --- | --- | --- | --- |
| IMU 10 ms cyclic: `tk_cre_cyc`, `tk_sta_cyc`, handler `tk_wup_tsk` | IMU service | none | Same. UP has no cyclic affinity attribute. |
| `tk_slp_tsk`, `tk_can_wup`, priority 14 | IMU task | none | Same. `app_prc_core1()` gives `TP_PRC1` (core 0) in UP. |
| `tk_slp_tsk(20 ms)`, priority 13 | Vehicle wait | Safety check runs after the wait | Same wait. The task parks before the check. |
| Board seqlock: 32-bit `__atomic_load_n`/`__atomic_store_n`, SEQ_CST | Safety report board | One new board | Same mechanism. No spinlock, no interrupt mask, no libatomic call in the ELF. |
| Clear request word: load, then store | Vehicle gate | New | One requester. No read-modify-write helper on ARMv6-M. |
| `vehicle_estop` → `motion_estop`: `tk_loc_mtx(mtx_motion, TMO_FEVR)`, `mc_estop`, `drv_motor_hw_stop` | Stop edge | First latch edge only | Same path as every other ESTOP. No new mutex. It can wait for the motion mutex holder, so it is not strictly nonblocking. No WCRT claim. |
| `drv_time_us`, `drv_time_ms` | IMU, Vehicle | Reads only | Same. |
| `tm_printf` | Vehicle idle | Once per stop, after the run stopped | Not in the IMU task or the running path. |
| `DI`/`EI` | — | none added | Motion's existing encoder DI/EI is the same. |
| Task creation or placement | — | none added | Same task table and priorities. |

Bench-only services, in the synthetic images only: `tk_cre_tsk` with
`TA_USERBUF` (TEST priority 21, ROBOT priority 29); `tk_sta_tsk`;
`tk_dly_tsk`; `tk_slp_tsk(TMO_FEVR)` (TEST). The section timers call
`drv_time_us` six more times per IMU tick.

## Numbers that are not valid here

The source branch ran on the old SMP kernel (`a03f9e5`). Its stack spare
(476–532 B), compute times, CPU values and boot gate are "old kernel,
a03f9e5" values. Do not use them as UP evidence. No teammate capture or
host duration is UP evidence.
