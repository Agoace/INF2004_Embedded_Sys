# STEP 8 follow-up handoff

## Delivery

All changes are local. Do not push or flash without operator review.
The main `mtk3-staging` checkout remains at `8ec3523`. The candidate is
`mtk3-staging-next`; use `git rev-parse mtk3-staging-next` for its final tip.

| Work | Commit |
| --- | --- |
| Live-log wire contract | `162faa0` |
| Live QoS 0 stream and batched QoS 1 DUMP | `f9eab99` |
| Stream documentation | `f724465` |
| Original style batches | `95c56de`, `aac9c01`, `4614012` |
| Distance v2, obstacle report, retained status/LWT | `025e63a` |
| Duty/odometry v3, detour and run summaries | `542f400` |
| Calibrated CPU meter and complete wire documentation | `a1c84e1` |
| Final style-only cleanup | `ed21052` |
| Final dashboard implementation | `f2b863e` |

[Validation](MTK3_STEP8_VALIDATION.md) records the six clean builds, strict ARM
152/152, host 111/111, SDK probes 6/6 and style code-identity proof. Hardware
rows, actual hook wall time and CPU-load measurement remain **Not run**.
[Wire contract](../s1-telemetry/MQTT_WIRE.md) owns all byte layouts and reserved formats.

Owner implementations were synchronized as new commits, not by rebasing or
merging the full staging tree into each module. Their real `--no-ff` merges
back into the candidate change no candidate file content.

| Branch retained for review | Tip |
| --- | --- |
| `mtk3` | `04909c5` |
| `mtk3-motion` | `4dbc4fa` |
| `mtk3-line` | `60bbcfc` |
| `mtk3-scan` | `482ddda` |
| `mtk3-telemetry` | `7ca3177` |
| `mtk3-dashboard` | `f2b863e` |
| `mtk3-staging` | `8ec3523` (unchanged; operator must promote candidate) |

No push or staging-branch promotion was performed. Candidate-only integration
includes vehicle observer boards, command adapters and TEST reporting. The
base includes only the early CPU calibration hook, not robot integration.

## Local helper cleanup

Before removal, every helper tip below passed
`git merge-base --is-ancestor <tip> mtk3-staging-next`. Each associated
worktree had empty `git status --porcelain`. No helper was unmerged or dirty.
The dashboard worker confirmed its four worktrees were no longer in use.

`mtk3-dashboard` was retained and advanced from `2b542e21e5e5` to the final
merged dashboard tip `f2b863e36d9f`; its old worktree was removed. The following
11 helper names were deleted locally, with their worktrees removed where
present. No remote branch was changed.

| Deleted local helper | Recoverable tip |
| --- | --- |
| `mtk3-dashboard2` | `a1b29b0c7c54` |
| `mtk3-dashboard3` | `f2b863e36d9f` |
| `mtk3-health-age-tests` | `1ec0f7fd79f1` |
| `mtk3-log-client` | `b7b80f9b1690` |
| `mtk3-mqtt-tests` | `940ca0208e70` |
| `mtk3-mqtt-token-policy` | `b687e4bf430f` |
| `mtk3-network-tests` | `61710dbc64e8` |
| `mtk3-scan-history` | `44b283bbe208` |
| `mtk3-telemetry-proto` | `7368393339d0` |
| `mtk3-template-tests` | `76a41321dd73` |
| `mtk3-vehicle-network-tests` | `5b8d15a1c1bd` |

Recovery, if needed: `git branch <old-name> <tip>`. The commits remain
reachable through the candidate merge history. Other existing module branches
(`mtk3-vehicle`, `mtk3-search`, `mtk3-detour`) were not part of this helper
cleanup and were left unchanged.

## Review limits

- No public symbol was renamed for style. Remaining naming issues and scope
  exclusions are in the validation record.
- Obstacle width is unavailable, not invented. Barcode, IMU, JUNCTION and
  coarse/fine scan stage remain reserved; no such firmware behavior is added.
- Heading and lateral offset are encoder estimates. They do not correct slip
  or measure the physical line directly.
- CPU target accuracy and an idle-board expected range must be established by
  S1-U07. Synthetic arithmetic bounds are not target accuracy.
- Deferred Change Requirement 1 work (representation table, before/after
  memory comparison, unified fault mask and 110 ms period) is not included.
- No network failure gates driving or feeds its watchdog. MQTT ESTOP is best
  effort; GP21 remains the real ESTOP.
