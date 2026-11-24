# CR1 Vehicle F: Vehicle owner fault word

## Scope and decision

Operator approved this diagnostic mirror on 2026-11-18. Code commit:
`c35f448`, parent `85b3b0b`. The interface is
[`fault_mask.h`](../../app_program/platform/fault_mask.h).
Health alone combines owner words. This word is not an actuator input.
It adds no latch, clear rule, task, message or MQTT field. Category A and
B storage savings are both zero: this change adds diagnostic storage.

Current (2026-11-24): this change is merged. The comparisons and tests below
are the 2026-11-18 substep evidence, including the then-current 90 s limit.
The demo now permits 300 s. Combined CR1 image 12 checked lift stop,
fault-mask output, timing and stacks; see [project status](PROJECT_STATUS.md#areas).
This does not establish an isolated before/after timing comparison.

## Representation and ownership

| Item | Rule |
| --- | --- |
| Storage | File-local `vehicle_fault_board` in `vehicle_task.c`: 8 B, sequence plus one aligned word |
| Single writer | Vehicle task only; no ISR, reader or other task writes this board |
| Bit 4, `FAULT_IMU_SAFETY` | Set exactly when `imu_gate.latched != IMU_SAFETY_NONE` |
| Bit 6, `FAULT_SHELL_OTHER` | Set exactly when shell state is `VSM_FAULT` and cause is neither `VSM_FAULT_SAFETY` nor `VSM_FAULT_IMU` |
| Other bits | Always zero |
| Publication | Build a local value with `vehicle_fault_value`, then call `fault_mask_publish` for the whole word |
| Reader | `vehicle_fault_mask_try_read`: one bounded coherent read; false leaves output unchanged |
| Before task initialization | Unavailable, not a successful all-clear |
| TEST/BENCH | No board allocation; reader returns false and leaves output unchanged |

`vehicle_sm.c` sets `VSM_FAULT_SAFETY` only when motion fault input is
nonzero. Vehicle passes `s.faults & MC_FAULT_LATCHED`: left/right stall
and ESTOP. The transient stale-command flag is not passed. Thus SAFETY
is the motion cause, not an additional independent shell fault.
`VSM_FAULT_IMU` is the IMU stop cause. Bit 6 excludes these two causes
only. FAILED, NO_ROUTE, NO_MODULE and HANDOFF_LOOP remain shell faults.
Their existing subtype and detail stay separate and unchanged. A shell
IMU cause alone does not set bit 4: only the actual gate latch does.

Publication occurs at task initialization, each Vehicle observation,
after each IMU gate evaluation, after shell steps and stops, and after
shell resets. This includes stop and reset paths without MQTT. The word
mirrors current state; it does not retain an independent history. A gate
clear removes bit 4 on the same evaluation. A shell reset removes bit 6.
An IMU latch and another shell fault can set both bits at once.

The board uses the existing atomic word accesses, barriers and sequence
check. Readers copy first and use only the local value. No shared packed
read-modify-write or local-interrupt-only lock is added. A later core-1
reader uses the same interface.

## Software evidence

- Strict GCC host suite before the integration merge: **692/692**,
  **0 warnings** (685 existing plus seven new cases).
- New tests cover all byte-valued latch and cause inputs in each shell
  state, both independent bits, initial absence, odd sequence, torn read,
  unchanged output on failure, and unavailable TEST/BENCH providers.
- Real Vehicle task tests observe IMU latch, stale IMU stop, recovery,
  other shell fault, excluded motion/IMU shell causes, final shell clear,
  repeated non-consuming reads and unchanged fault detail.
- The ownership guard checks one private board, one whole-word publish
  call site and no access from another source file.
- Red proof: temporarily replace `vehicle_fault_value` with a zero stub.
  The value, real shell-other and real IMU-lift cases all fail (ctest
  exit 8). Restore the implementation; all 692 tests pass. The stub is
  not committed.
- Review found that the inherited work did not publish immediately after
  `vsm_step`. `vehicle_stop` does not publish the observation board.
  Explicit shell-step, stop and reset publications close this gap.
- Existing junction, live barcode, SEARCH grace, expiry, detour handover
  and network tests pass without changes to their control rules.

Local evidence is under ignored `build/wip/f-takeover/`: `red-test.log`,
`red-build.log`, `green-test.log`, `green-build.log`, `before-build.log`,
`after-build.log`, `before-ram.txt` and `after-ram.txt`.
No helper push or flash was performed. Robot timing and stack acceptance
are not claimed.

## Build cost

Both images use clean `git archive` trees, public example secrets,
Arm GNU 14.3.Rel1, GCC 14.3.1, and the same demo profile:

```sh
make -C build_make --output-sync=target -j12 \
  SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1 \
  BARCODE_FAIL_CAPTURE=1 BARCODE_LIVE_ENABLE=1 \
  BARCODE_MEASURED_CLEAR_MM=100 BARCODE_MAP="F=LEFT" JUNCTION_TEST_CMD=NONE
```

| Quantity | Before `85b3b0b` | After `c35f448` | Change |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 240808 B | 240816 B | +8 B |
| Vehicle linked inputs | 2504 B | 2512 B | +8 B |
| Owner board | 0 B | 8 B | +8 B |
| ELF text, `size` | 458196 B | 458356 B | +160 B |
| ELF bss, `size` | 218416 B | 218424 B | +8 B |
| RAM functions | 79 / 21212 B | 79 / 21212 B | 0 |
| Warnings, case-insensitive scan | 0 | 0 | 0 |

`scripts/ram_report.py` gives the static totals. Both images pass
**RAM_PROOF 79/79**. The three +8 B rows describe the same allocation,
not separate costs. Other linked RAM groups and alignment are unchanged.
Messages, MQTT bytes, stack allocations and task periods do not change.
Unreserved link-time RAM is not measured runtime free RAM.

## Integration check

Merge `73fadc7` brings `mtk3-cr1` at `66810d1` into this branch with
`--no-ff`. The sonar owner board is now also present. Keep this comparison
separate from the isolated Vehicle cost above. Both comparison images
again use clean archives, public example secrets and the same demo line.

| Quantity | CR1 `66810d1` | Merged `73fadc7` | Vehicle change |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 240816 B | 240824 B | +8 B |
| Vehicle linked inputs | 2504 B | 2512 B | +8 B |
| Sonar linked inputs | 953 B | 953 B | 0 |
| ELF text, `size` | 458364 B | 458524 B | +160 B |
| ELF bss, `size` | 218424 B | 218432 B | +8 B |
| RAM functions | 79 / 21212 B | 79 / 21212 B | 0 |
| Warnings, case-insensitive scan | 0 | 0 | 0 |

Both images pass RAM_PROOF 79/79. The merged strict GCC and Clang
ASan/UBSan suites each pass **693/693**, with 0 build warnings. Leak
checks and halt-on-UB are enabled for the sanitizer run. Documentation,
template (eight deviations) and diff checks pass. The ARM message size
manifest remains unchanged. These checks do not establish robot timing
or stack acceptance.

Local integration logs have the `merged-` prefix under
`build/wip/f-takeover/`. The merged public-example UF2 SHA-256 is
`d7f9c9dcab5735817dff21f1ed9e29b4126eaac6605ec43357285dcf15ba9a19`.

## STOP capture integration

Health API change `402e8fa` is merged with `--no-ff` as `ef87362`.
Vehicle call commit `45ed77c` adds one call to
`robot_health_faults_stop(epoch)` at the start of `hump_finish`, after an
explicit owner-word publication. Both the Vehicle and timing epochs are
still odd. All completed runs use this one exit. TEST/BENCH do not run
it; non-MQTT builds omit the Health call.

The later `vsm_stop` remains in its original place after the stop events
and summary. Moving it would change their shell-state field. A normal
RUNNING-to-IDLE transition changes neither owner fault bit. Shell faults
and IMU latches already have their final values at capture. No wire,
actuator, clear or latch rule changes.

Vehicle copies current owner values through the Health API. It does not
combine them. Health remains the only combined-mask writer; its separate
F2a tests check the real capture board and frozen run history. See
[the telemetry record](CR1_TELEMETRY_A.md).

Caller integration evidence:

- Before the call exists, five real Vehicle task cases fail: remote
  ESTOP, GP21 operator stop, shell fault, IMU lift, and the real 90 s
  time limit. Each has a missing STOP capture (ctest exit 8).
- The Health API test double reads the real Vehicle owner board. It
  requires bit 6 for shell FAILED and bit 4 for IMU lift/stale stops.
  Recovery can later clear the current word, not the recorded copy.
- The same double requires equal odd Vehicle/timing epochs at capture,
  rejects a duplicate epoch, and records exactly one capture per run.
  All real task cases, including two-run barcode cases, check the final
  capture count against the number of completed runs. No-start cases
  require zero captures.
- Strict GCC and Clang ASan/UBSan suites each pass **697/697**, with
  **0 build warnings**. Leak checks and halt-on-UB remain enabled.
- Documentation, eight-template-deviation and diff checks pass. The ARM
  message manifest is unchanged across the call commit.

The clean-archive comparison below isolates the Vehicle call from F2a's
own storage/packing changes. Both images use public example secrets and
the demo command above. Do not compare these totals directly with the
older integration table as a Vehicle packing saving.

| Quantity | API merged `ef87362` | Call added `45ed77c` | Change |
| --- | ---: | ---: | ---: |
| Static RAM, map parser | 239280 B | 239280 B | 0 |
| Vehicle linked inputs | 2512 B | 2512 B | 0 |
| Telemetry linked inputs | 78862 B | 78862 B | 0 |
| ELF text, `size` | 467500 B | 467628 B | +128 B |
| ELF bss, `size` | 216888 B | 216888 B | 0 |
| RAM functions | 79 / 21212 B | 79 / 21212 B | 0 |
| Warnings, case-insensitive scan | 0 | 0 | 0 |

Both images pass RAM_PROOF 79/79. Local logs are in ignored
`build/wip/f-stop/`: `red-test.log`, `green-test.log`, `clang-test.log`,
`before-build.log`, `after-build.log`, `before-ram.txt` and `after-ram.txt`.
The call-enabled public-example UF2 SHA-256 is
`31a36ddbb9ca4962f811db0c89a336c3aff8c9463b5d5a8b16fb0e7f6adfae89`.
No push, flash or robot timing/stack acceptance is claimed.
