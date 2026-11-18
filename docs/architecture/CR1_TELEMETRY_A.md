# CR1 telemetry A, diagnostic faults and T110

## Scope and approval

The operator approved T110 option A, A-EVENT, A-INDEX, A-SCAN, A-HEALTH,
A-SONAR, A-INFO and diagnostic fault bits 0..9 on 2026-11-18. Category B
and MQTT wire changes are not approved. Native public value types, units,
identities, queue capacity and admission safety must stay unchanged.

| Step | State |
| --- | --- |
| T110 | Software checks pass; robot measurement pending |
| F1 shared interface | Next step; no publisher is added by the interface |
| A-EVENT, A-INDEX | Pending implementation |
| A-SCAN, A-HEALTH, A-SONAR, A-INFO | Pending implementation |
| F combiner and output | Pending implementation and owner publishers |

The starting revision is `33d2e4c`, which includes the approved sonar,
Vehicle and barcode A changes. The canonical common baseline remains
`7350da0`; SW7's existing map and size reports are authoritative. Do not
build a second common baseline.

## T110

Change only `PM_RUN_STATE_MS` from 200 to 110. Keep
`state_at = now + period`. This is a **110 ms deadline; expected gap
~120 ms with 20 ms poll; robot measurement pending**. It is not an exact
fixed-rate schedule. No Wi-Fi task or kernel scheduling change is made.

| Setting | Before | After |
| --- | --- | --- |
| RUN state deadline | 200 ms | 110 ms |
| IDLE state | 1000 ms | Unchanged |
| RUN / IDLE heartbeat | 1000 / 5000 ms | Unchanged |
| Live log / KR1 rows / Health | 100 / 20 / 1000 ms | Unchanged |
| USB dump yield | 20 ms per eight records | Unchanged |
| State payload | Binary, version 5, 76 B | Unchanged |
| Model gap with 20 ms polls | 200 ms | 120 ms |
| Model state payload rate | 380 B/s | 633.33 B/s |

`test_telemetry_mode` checks eligibility at 109/110 ms, timer wrap and
polls at 0,20,40... ms. The model sends at 0,120,240... ms. The real
`robot_mqtt` schedule test also checks 109/110 ms. RUN and grace tests
expect 18 publications in a 2000 ms window with fine polling. IDLE and
heartbeat expectations do not change.

Dashboard stale limits remain RUN state 1000 ms, IDLE state 2500 ms,
RUN heartbeat 2500 ms and IDLE heartbeat 12000 ms. Producer age 65535
still means unavailable. No decoder threshold is changed.

## Binary MQTT audit

| Topic | Version | Payload bytes |
| --- | --- | ---: |
| state / sonar / obstacle | 5 / 2 / 3 | 76 / 36 / 32 |
| event / status / heartbeat | 2 / 2 / 4 | 20 / 6 / 108 |
| detour / summary / barcode | 2 / 3 / 2 | 36 / 52 / 28 |
| log | 2 | 36 control; batch 36 + 16 per record, maximum 228 |
| cmd, dashboard or CLI to robot | 1 | 22 |
| imu | Reserved | No active publisher; reserved binary layout |

All active robot MQTT payloads are already binary, including status's
retained offline will, log batches and tokened commands. No JSON-to-binary
saving is claimed. The dashboard's browser-to-laptop HTTP/SSE traffic is
JSON; its MQTT command is produced by `pack_command()` as bytes. USB and
CSV evidence are text, not MQTT payloads.

The library demo in `mqtt_utk.c` contains text `hb %u`. In SW7's canonical
ELF its weak hook is discarded and the strong `robot_mqtt` hook is bound.
No `mqtt_utk_start` or `heartbeat_timeout` symbol remains in that ELF.
This dormant demo is not an active robot MQTT publisher.

## RAM and build evidence

| Revision / step | Static RAM B | Change B | Evidence |
| --- | ---: | ---: | --- |
| Common baseline `7350da0` | 241228 | Reference | SW7 canonical map |
| Starting integration `33d2e4c` | 240808 | -420 from common baseline | Existing integrated A results |
| T110 | 240808 | 0 from starting integration | Local after map |

T110 leaves S1's module RAM at 80402 B. RAM-log rows stay 72016 B,
events 6144 B and scan rows 576 B at this step. Queue allocations stay
4096 B telemetry, 384 B events, 96 B commands and 12 B ESTOP. Later A
steps must report their own delta; do not count the earlier -420 B again.
New F storage is a cost, not a packing saving.

The T110 demo profile uses Arm GCC 14.3.1, Cortex-M0+ Thumb:

```sh
make -C build_make --output-sync=target -j4 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 BARCODE_MEASURED_CLEAR_MM=100 'BARCODE_MAP=F=LEFT' JUNCTION_TEST_CMD=NONE
```

T110 software results: 684/684 host tests, zero compiler/assembler warnings,
`RAM_PROOF PASS 79/79`, 21212 B. The red period test failed at the old
200 ms value before the change. Three old MQTT RUN-count assertions also
failed after the change; they now check the new deadline and counts.
No expected IDLE or heartbeat rate was relaxed.

Ignored evidence is under `build/wip/cr1/`: `t110-red.log`,
`t110-host.log`, `t110-build.log` and `t110/ram_report.txt` plus the
copied ELF/map/proof. Host Make-profile tests can invalidate build objects;
copy each after ELF/map before another full host run. This is not a source
or firmware change.

## Robot gate: 110 ms runs only

No new 200 ms robot run is permitted. Use existing files under
`~/Projects/robot-logs/20261118_post_freeze/`.

| Existing evidence | Observation / permitted use |
| --- | --- |
| `mqtt_FIX2.csv`, primary network reference | 796 RUN-tagged frames; 788 adjacent RUN pairs; 493 missing sequence values between endpoints (38.49%) |
| Same RUN pairs | Receipt gap median / p95 / p99 / maximum: 215.669 / 1435.722 / 2735.613 / 3586.181 ms |
| FIX2 first recorded session | Over 492325 ms reported uptime: publish_ok +5052, publish_failed +8101; mixed RUN/IDLE and topics |
| `robot_FIX2.log`, final epoch 13 | KR gap 9979..10023 us; lost telemetry, lost wakes and overruns zero; response maximum 2432 us; Telemetry stack 1096 used / 3000 free |
| `robot_LAST.log`, epoch 1 | Secondary stopped reference: gap 9983..10015 us; same three counters zero; response maximum 2480 us; Telemetry stack 1104 used / 2992 free |
| `mqtt_LAST.csv` | 1069 state frames but no active/RUN-tagged frames; not a RUN network reference |

The FIX2 file SHA256 is
`906a49270d1e1d8ac976628d7662a252c52fd5400ffdb79f2b370f83dbf3a005`.
Sequence gaps include enqueue, broker and receiver loss, not only radio
loss. Publish counters include mixed-topic local and asynchronous results;
their ratio is not a QoS0 delivery probability. LAST contains the old lost
sonar result behaviour. Do not attribute course differences only to T110.

After Main builds the combined image, the operator records 110 ms runs
on the same course/hotspot and records firmware, boot/run IDs and battery.
Compare RUN and active windows, state sequence gaps, publish counter deltas,
KR task gaps, lost wakes, overruns and first-overrun. Check Telemetry and
Wi-Fi stack high-water, aggregate CPU, admission lost/busy/full state and
queue losses. Do not substitute board-read busy counters for admission
busy. Per-task CPU and minimum live PBUF/heap headroom need approved
bounded diagnostics; existing logs do not provide those values.

The canonical PBUF allocation is 36771 B for 24 buffers; the lwIP heap
allocation is 16403 B for a 16 KiB arena. Runtime lwIP statistics are off.
These allocations and static unreserved RAM are not measurements of free
network headroom. No 110 ms hardware CPU, stack, headroom or timing pass is
claimed. If the robot gate fails, report it; do not reduce another stream
or increase a pool without approval. No flash or robot run was done here.
