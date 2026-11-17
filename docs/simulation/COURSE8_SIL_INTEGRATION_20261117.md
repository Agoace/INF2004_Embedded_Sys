# Figure-8 SIL integration review (2026-11-17)

## State

SW7 helper `b999756b120a8e4c6426cd88cfaf413ad0ce8936` is integrated into
`mtk3-hil` with a non-fast-forward merge from `448fcef`. It adds a separate
course8 controller and host tools. No Webots execution is authorized by
this merge. Gate1 remains **TRIAL INCOMPLETE / NOT GATE1 PASS**. Physical
GP28 remains **FAIL**. Host tests are not physical acceptance.

The staging-based course branch is **not merged**. Course input comes from
the pinned read-only Git archive in `sil_course8.py`. Firmware input also
comes from a pinned read-only archive; the working firmware tree is not
changed.

## Entry points

| File | Purpose |
| --- | --- |
| [`sil_course8.py`](../../sim/webots/pil/sil_course8.py) | Pinned course and firmware overlays, native library build and host-only kinematic check. |
| [`sil_course_path.py`](../../sim/webots/pil/sil_course_path.py) | Connected course reference; straight crossing traversal and a local progress window. |
| [`sil_ray_np.py`](../../sim/webots/pil/sil_ray_np.py) | NumPy batch equivalent of the scalar scene sampler for the LINE pair, not GP28. |
| [`course8_adapter.c`](../../sim/webots/controllers/sil_line/course8_adapter.c) | Native vehicle LINE/SEARCH path with junction wrapper; no barcode offers or sonar. |
| [`sil_course8.py` controller](../../sim/webots/controllers/sil_course8/sil_course8.py) | Separate Webots controller, bounded tilt handling, recorded-input replay and stop-on-error. |
| [`sil_course8_report.py`](../../sim/webots/pil/sil_course8_report.py) | Course progress, crossings, SEARCH, ramp/loose-arc observations and separate body/wheel yaw metrics. |
| [`sil_course8_run.py`](../../sim/webots/tools/sil_course8_run.py) | Preparation and opt-in single-run launcher; no launch is part of this review. |

Dependencies: NumPy and a native C compiler. No package installation was
performed for this integration.

## Review and model limits

- Both configurations use the archived firmware revision declared in the
  builder. The gain override applies only to the adapter translation unit.
- The existing ADC-delay model is imported unchanged. Zero delay adds no
  delay-specific manifest fields; the nonzero case records optical input
  separately from the delayed firmware input.
- Host checks compare all optical sample fields with the scalar sampler
  on flat, tilted, ramp and off-course fixtures. This is finite regression
  evidence, not a proof over every possible geometry or floating-point
  platform.
- Motor response uses the existing DM1 measured rows with interpolation;
  out-of-table duty is rejected. No new measured motor fit is claimed.
- Raised-wheel optical sampling uses explicit, uncalibrated height/tilt
  extrapolation. The ramp profile and scenario placement remain assumptions
  from the pinned course source, not new physical measurements.
- The native replay checks recorded ADC/count inputs against navigation
  outputs and motor-plant targets. It is not another physics run or an RTOS
  scheduling test. The separate junction diagnostic columns are not checked
  by that replay comparison.
- There is no barcode source, GP28 qualification, sonar, firmware watchdog
  or Pico task execution in this controller.

## Validation

```sh
PYTHONPATH=sim/webots python3 -m unittest discover -s sim/webots/tests -v
```

The worker-tip review ran 390 tests with three existing expected model
failures and one optional retained-record replay skipped because its
worktree did not contain those ignored files. All new course8 tests ran,
including the fake Supervisor and both ADC-delay paths. The integration
check can read the original retained recordings from the SW8 evidence
worktree without copying or changing them. The HIL integration suite did
this by redirecting only the retained-record test module's input root to
`/home/yangxuanlee/Projects/core-review-sw5`; all 390 tests completed with
three existing expected model failures and no skips. A file-based runner
with both the repository root and `sim/webots` on `PYTHONPATH` is required
for that wrapper: stdin execution cannot be re-imported by spawned workers.

Gate1 source, fixtures and tests, firmware, existing controllers, launcher
wrappers and DM1 implementation must remain byte-identical to `448fcef`.
The frozen host dry-run matched 158 RX chunks and 58 TX writes, with
`host_fixture_match: true`, overall INCOMPLETE, `physical_gate1_pass: false`
and no opened serial device. Fresh output is under
`build/webots/course8-merge-host-dryrun-20261117/`. No Webots was launched.

The current generator camera orientation is a separate follow-up change.
Generate a fresh project after a generator change; do not patch historical
worlds or their sealed manifests in place.
