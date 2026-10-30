# Line abstraction fork from YX line tracing

This record documents how the integrated firmware's Line component relates to
the YX line-tracing work, what was forked, what was deliberately changed, and
what was left alone. Read it before changing anything under `src/line/`, and
before syncing anything from the YX worktree.

## Provenance

- Source: `yx-line_tracing` worktree, commit `1ea095c` (`docs: correct the run 10
  LINE_LOST count`).
- The source worktree is **read only** for this work. Nothing in it is edited by
  the integrated firmware, including its `design-review/` diagrams.
- Forked into the integrated firmware as `src/line/`, not merged. The two trees
  evolve separately until the Planner schedules a sync.

## Where this lives

The component is its own worktree and branch, `yx-line-abstraction`, created from
`motor-line-manager`'s checkpoint `8cb131d`. It holds `src/line/`, the host suite
in `tests/line/test_line.c`, and this record. Nothing else in the tree is new, so
a deliberate merge from this branch adds the component and nothing else.

`motor-line-manager` holds **no copy**. Its `CMakeLists.txt` points at this
directory through the `LINE_ABSTRACTION_DIR` cache variable, which defaults to
the sibling worktree `../yx-line-abstraction`, so that tree builds only while this
checkout exists. Override it when the component lives elsewhere:

```bash
cmake -S . -B build -DLINE_ABSTRACTION_DIR=/path/to/yx-line-abstraction
```

Standalone verification needs neither a Pico nor the Manager:

```bash
cmake -S tests/line -B tests/line/build
cmake --build tests/line/build
ctest --test-dir tests/line/build --output-on-failure
```

`line_adc_pico.c` is the only platform file. It is compiled by the firmware
target that embeds the component, not by this project.

## Merge strategy

This branch exists so the component can be merged **deliberately** later, and it
is kept mergeable by staying out of the firmware build.

What a merge adds:

- `src/line/` - the component, 16 files.
- `tests/line/` - the host suite and its own CMake project.
- `docs/LINE_ABSTRACTION_FORK.md` - this record.

What it does **not** touch:

- `CMakeLists.txt`, the firmware project, is byte-identical to the branch point
  `8cb131d`. An earlier revision of this branch replaced it with a host-only
  project, which would have conflicted with the Manager's edits to the same file
  and risked dropping firmware targets. The host build lives in
  `tests/line/CMakeLists.txt` instead.

Expected conflicts, and how to resolve them:

| File | Conflict | Resolution |
| --- | --- | --- |
| `docs/README.md` | both sides add a row to the records list | keep both rows |
| `docs/AGENT_MAP.md` | both sides add rows to the tables | keep both sets of rows |

No source file is touched by both sides, so nothing else needs resolving.

After the merge, in `motor-line-manager`:

1. Point `line_core` at the local copy: `src/line/*.c` instead of
   `${LINE_ABSTRACTION_DIR}/src/line/*.c`, and `src/line` as its include
   directory.
2. Move `line_adc_pico.c` back into the `robocar_line_manager` source list.
3. Drop `LINE_ABSTRACTION_DIR`, or keep it only if a separate component checkout
   is still wanted.
4. Collapse the cross-tree links in `docs/README.md`, `docs/AGENT_MAP.md` and
   `docs/LINE_MANAGER.md` to local paths.

Until that merge happens, a fresh `motor-line-manager` checkout needs either this
worktree beside it or `-DLINE_ABSTRACTION_DIR=<path>`; configure fails with an
explicit message rather than a confusing build error later.

## What was forked

| File | Fork of | Notes |
| --- | --- | --- |
| `src/line/line_position.{h,c}` | `line_position.{h,c}` | Position, dwell, junction, side and calibration learner |
| `src/line/line_steer.{h,c}` | `line_steer.{h,c}` | P+D and calm policy, output changed |
| `src/line/line_spin.{h,c}` | `line_spin.c` (sequencing only) | Rewritten non-blocking |
| `src/line/line_adc_pico.c` | `line_sensors.c` (hardware only) | Seam + averaging |
| `src/line/line_types.{h,c}` | new | The Line-to-Manager contract |
| `src/line/line_health.{h,c}` | new | Runtime rail/stuck fault |
| `src/line/line_sampler.{h,c}` | new | 1 kHz cadence and overrun count |
| `src/line/line_producer.{h,c}` | new | Owns the sensor, the learner and the calibration |

Byte-identical files that were **not** copied: `line_detector.{h,c}`,
`ir_line_data.{h,c}`, `ir_module_pins.h`. The integrated tree keeps its own
copies and the standalone IR diagnostic targets keep using them.

Not taken at all: `vehicle_controller.c`, `drive.c`, `steer_test.c`,
`barcode_reader.c` (its YX variant differs from this tree's), the obstacle
stack, the Wi-Fi and flash logger, and `line_spin_calibrate()`'s blocking body.

## Deliberate changes

1. **No Vehicle Controller types.** Upstream's `line_position.h` includes
   `vehicle_controller.h` and returns `vc_event_t` carrying `vc_line_side_t`.
   The fork includes only `line_types.h`, uses a `line_side_t` with the same
   ordering, and returns a local `lp_event_t` of NONE / LOST / FOUND. No VC
   type reaches the Manager.

2. **`lp_invalidate()` exists.** Upstream's `lp_calib_begin()` leaves
   `LP_FLAG_CALIBRATED` set and `lp_calib_end()` returns early on low contrast
   without touching the flags, so a failed sweep silently kept the previous
   endpoints live. The fork clears everything at the start of every sweep, so a
   failed or cancelled sweep can never be followed.

3. **Elapsed-time learners.** Upstream advances the floor/line adaptation by
   1/256 and the side baseline by 1/16 *per update*, which assumes a fixed
   1 kHz. The fork scales both by the elapsed milliseconds, so a slower loop
   keeps the same effective response. At the nominal 1 ms period the arithmetic
   is identical to upstream.

4. **Steering is reparameterised.** Upstream emits two 7-bit wheel duty
   percentages and owns `min_pct`, `max_pct` and `right_trim_pct`. The fork
   emits Motion's signed steer plus the calm choice:
   `steer = round(127 * (gain_pct * position + kd_pct * change) / 10000)`,
   clamped to ±127. `gain_pct` and `kd_pct` keep their upstream meaning, and the
   duty-space fields are gone because per-wheel compensation belongs to Motion's
   PI.

5. **The sweep is non-blocking.** Upstream's `line_spin_calibrate()` calls
   `drive_set_wheel()` and `sleep_us()` and owns the quarter-turn geometry. The
   fork only sequences three signed rotations and waits for Motion to report
   completion, so Motion keeps PWM and the encoders and nothing blocks the loop.
   Amplitude is one eighth of a wheel turn per segment (24 degrees on the 120 mm
   wheelbase) instead of a quarter, and the whole sequence is bounded at 6 s.

6. **Runtime sensor health was kept.** The YX calibration only checks contrast
   during a sweep. The fork also carries `line_health`, with the same rails
   (≤10, ≥4085) and hold (100 ms) the previous fixed-threshold detector used, so
   moving to the adaptive source does not silently lose the sensor fault.

## Behavioural divergence that remains

The fork is **not** output-equivalent to the YX runtime, and RT-0036 confirmed that
claim must not be made. Two inputs were restored after review; the rest are
deliberate.

Restored after the RT-0036 review:

- **Side hint.** `line_producer_update_travel()` accumulates the left-minus-right
  wheel counts and applies upstream's 20-count threshold to that total, calling
  `lp_set_hint()`. The accumulator resets when the line is centred, on a FOUND
  edge, and at each new calibration run, so a spin can never hand the following
  run an inherited heading. It is fed from Motion's non-consuming
  `motion_peek_odometer()`, so the IMU's consuming `motion_consume_travel()` feed
  keeps its own marks.
- **Junction straight-through bound.** The producer records the signed mean wheel
  travel at the junction hold's rising edge and packs
  `line_obs_junction_travel_done()` once the signed difference reaches 450
  counts, so reversing during a junction cannot spend the budget early. The
  Manager then stops forcing straight and resumes position steering.

Deliberate, and still divergent:

| Area | YX | Here | Why |
| --- | --- | --- | --- |
| Steering output | left/right duty percentages, plus `min_pct`, `max_pct`, `right_trim_pct` | signed steer plus a calm flag; Motion's per-wheel PI owns PWM | the Manager may not write PWM |
| Sweep amplitude | a quarter wheel turn per segment | one eighth (24 degrees), 6 s budget | narrower and bounded; MEASURE on the bench |
| Sweep mechanism | blocking, with direct wheel access | request/complete rotation commands | nothing may block the 1 ms loop |
| Loss disposition | the VC's own line-lost handling | SEARCH sweep, then `FAULT_RECOVERY_TIMEOUT` and a power cycle | RT-0032 P1-c |
| Buttons | VC input handling | press edge, 20 ms qualifier, roles chosen by state | RT-0032 P1-a |
| Calibration commit | a failed sweep could leave old endpoints live | every sweep start invalidates first | RT-0031 B4 |
| Junction straight-through gate | `lp_junction_hold()` only | `junction` or `junction_hold` | lets a junction register before the hold engages; the 450-count budget rule itself is upstream's |
| Hint sampling period | recomputed with the 10 ms steering update | recomputed on every 1 ms sample | finer resolution; the accumulated total is the same quantity |
| Search turn in the heading | explicitly excluded, by advancing `last_steer` during the search | the spin's differential accumulates into the heading and is cleared on the FOUND edge | the hint is only consumed while following, and a FOUND always clears it |

So a bench run whose path shape resembles YX is encouraging, but it is not proof of
equivalence. Record the measured behaviour in the bench notes rather than
asserting parity.

## Physical sensor order

`ir_module_pins.h` names GP26 as left and GP28 as right. The operator note with
the YX branch (recorded 2026-10-27) says the physical sides on this robot are the
other way round: **GP28 = left, GP27 = centre, GP26 = right**.

The correction is made in `src/line/line_adc_pico.c` and nowhere else.
`ir_module_pins.h` is byte-identical across the worktrees and is also included by
`robocar_ir_sensors.c`, `robocar_line_follow.c` and the standalone diagnostics, so
editing it would silently change those targets too.

## Not yet measured

- `LINE_SPIN_SWEEP_DEG` (24) is derived, not measured. Confirm on the bench that
  one segment crosses the line for every sensor without over-rotating.
- `MANAGER_STEER_GAIN_PCT` (100) and `MANAGER_STEER_KD_PCT` (50) are carried over
  in meaning from YX but were tuned against duty space. Re-tune against Motion's
  speed control and record the values here.
- Whether a one-eighth sweep is enough on a curve, where the line may leave by
  more than one sensor pitch during the 300 ms gap dwell.

## Sync rule

If the YX worktree changes `line_position.c` or `line_steer.c`, do not merge.
Re-apply the change to the fork by hand and note it in this record, because the
three deltas above (no VC types, invalidate, elapsed time) make an automatic
merge unsafe. Name the owner of that sync in the roundtable before it happens.
