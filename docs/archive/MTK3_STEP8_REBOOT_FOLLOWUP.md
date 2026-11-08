# STEP 8 boot/session and timing follow-up

## Scope and delivery

Baseline: `mtk3-staging` at `82cf1db`. Candidate: `mtk3-staging-next`, worktree
`/tmp/mtk3-s8fix`. The main checkout is not changed. Changes are local only;
no push or flash was performed. The dashboard `--allow-origin` behavior is
preserved. Helper integration uses real `--no-ff` merges, not a rebase.

SCAN option C is separate on `mtk3-scan-reattempt`. It is not merged into
this core candidate and is not a proposed fix for the unmeasured timing
cause. Its driving changes need separate review and hardware acceptance.

## Changes

| Area | Correction | Important limit |
| --- | --- | --- |
| Firmware boot label | Nonzero ROSC-derived u32, initialized once before application startup; direct registers/TIMER spacing, bounded pair extraction; added to observations | Random label, not persistent/cryptographic or guaranteed unique; source failure disables observation startup only |
| Log wire/storage | v2 outer frame carries one ID; batch header/control 36 B, max batch 228 B; embedded events and KR1 remain 16 B | No per-row ID duplication; output ring remains 256 B |
| Client/dashboard | Boot/session-scoped deduplication, refill/gap/DUMP and live baselines; preserve both boots even at generation 2/index 0; label old summaries without switching live boot | Legacy inference is ambiguous; same-ID OFFLINE-to-ONLINE can conservatively duplicate rows in a visible local session |
| Health | Main-board unavailable mask plus aggregate count, separate from per-board true-contention counts | Empty secondary histories/reports are not absent boards; only `E_TMOUT` counts contention |
| Sonar observer API | Separate unready `E_OBJ`, writer conflict `E_TMOUT`, and no-next-entry `E_NOEXS` | Only the observation adapter and host tests use this API; acquisition/driving callers are unchanged |
| Motion investigation | Opt-in `DIAG=1` fixed snapshots of release, entry/acquisition, phase and prior API hold; GP20-only output | No new scheduling/control decision; actual cause still needs an operator capture |
| Documentation | Actual Wi-Fi cadence ~20 ms, operator hardware results, wire byte-cost table, timing limits and future per-run proposal | Earlier-image PASS does not validate new firmware |

Authoritative [wire layouts and before/after byte table](../s1-telemetry/MQTT_WIRE.md#minimum-bit-wire-cost)
justify each ID. State and sonar retain explicit IDs: queued old observations
can arrive after new-boot evidence, and uptime alone cannot bind them safely.
State adds 20 payload bytes/s at 5 Hz. Reserved barcode/IMU and commands do
not grow. A 32-bit label avoids the large birthday-collision risk of 16 bits;
see [entropy assumptions and limits](../s1-telemetry/MQTT_WIRE.md#boot-identity-and-sessions).

## Timing finding and proposed next action

The later ESTOP log reports lifetime gap 8952..11038 us, 38 us over the
unchanged gate. It does not locate the event. The first GP20 header precedes
its long dump; that dump, idle and calibration can contaminate the next
header's lifetime extrema. The entry timestamp precedes the mutex. Therefore
neither MQTT, SCAN, ESTOP, USB nor a mutex can honestly be named as measured
cause from the old min/max-only log.

[The investigation](MOTION_TIMING_INVESTIGATION.md) documents the diagnostic
build, expected distinctions and operator procedure. Queued wakes and a
release/entry snapshot race are visible; race bit 17 means dispatch delay
must not be interpreted. No template USB/IRQ path was modified. Any later
causal fix must follow the diagnostic evidence and operator review.

A per-run gap window beside lifetime metrics is **proposed, not added**.
Count only complete in-window gaps; exclude the crossing gap. Preserve
lifetime extrema/counters. Define the start/stop boundary with the operator
before changing the production API or acceptance reporting.

## Validation record

Final candidate: helper `ce59bf1` merged with `--no-ff` on top of core `b65c276`.

| Check | Result |
| --- | --- |
| Host CTest (C plus Python wrappers) | 116/116 PASS |
| Python decoder/log and dashboard suites | 62 and 137 PASS against the C fixture emitter |
| Firmware profiles, warning-free with `-j4` | 7/7: default, robot, TEST, BENCH, MQTT, MQTT TEST, MQTT DIAG |
| Strict ARM invocations on changed C units | 91/91 PASS (13 per profile) |
| SDK link proof | 7/7 PASS; checker now includes the DIAG profile field |
| Docs, template ledger, `git diff --check` | PASS; template ledger 7 paths |
| Secret hygiene | No private settings tracked or in the diff |

Independent diagnostic isolation proof at `50d2e52` versus `164f616`:

- Six existing DIAG=0 profiles; 923 object/ELF records, zero allocated
  code/initialized-data, relocation, symbol or BSS-size differences.
- Debug/DWARF and associated macro `.group` index metadata are excluded;
  this is not a claim that raw debug-bearing files have identical bytes.
- Deliberate watchdog 100-to-101 ms mutation rejected in `motion.o` and ELF
  disassembly; restored 100 immediately. No mutation artifact was deployed.
- DIAG=1 has a distinct `_diag` artifact name. Synthetic host tests separate
  dispatch delay, mutex wait and late cyclic dispatch, including wrap and
  snapshot-race handling; production motion task also runs in a DIAG=1 host
  variant. Synthetic timestamps are not hardware evidence.

Builds use `-j4`. Strict ARM checks add `-Wall -Wextra -Wconversion -Wshadow
-Wformat=2 -Werror`. SDK link checks use the real target headers. The template
ledger remains seven paths against `c3b0954`; this task adds no deviation.
Private evidence is below `/tmp/step8-results/`, mode 0700/0600. The candidate
worktree is mode 0700 and private configuration 0600, ignored and untracked;
private settings are not copied into tests, docs or Git.

## Hardware acceptance and outstanding work

[Operator results from 2026-11-07](MTK3_STEP8_HARDWARE_RESULTS.md) record cold
TIMER, Wi-Fi timing, CPU values, watchdog/LWT, live 216/216 rows, command
checks and the M2 detour. M2 clearance depended on one outer scan point;
that is a placement/echo margin, not evidence that unchanged software regressed.
The two S2 timing windows remain separate in the record.

New-image hardware is **Not run**. The operator must review and schedule:

1. Cold-power/watchdog/broker boot-label and two-boot history checks.
2. TEST absent vehicle versus genuine board contention.
3. DIAG=1 capture around GP20, MQTT ESTOP and SCAN; report cause/proposal
   before any scheduling or driving-path timing fix.
4. Separate SCAN option C review and its own reverse/cancellation/timeout tests.
5. Remote HTTPS certificate retry on normal Wi-Fi; hotspot HTTP/Tailscale
   live data and buttons already passed on the earlier dashboard.

The physical GP21 ESTOP and motion-only watchdog are unchanged. Wi-Fi owns
all lwIP/MQTT calls; network failure never gates driving or watchdog feeds.
