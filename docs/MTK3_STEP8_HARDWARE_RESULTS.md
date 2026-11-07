# STEP 8 hardware results — 2026-11-07

These results were supplied by the operator. They apply to the earlier
STEP 8 image, not to the later boot-label, absent-board, diagnostic or SCAN
retry changes. A build or host test does not extend a hardware PASS to a new
image. Private USB/MQTT captures remain under ignored `build/mtk_logs/` in
the main checkout. No private configuration or raw capture is committed.

## Shared platform and MQTT

| Check | Operator evidence | Result and limit |
| --- | --- | --- |
| S0-U01 cold power | TIMER check error 0 | PASS for tested image |
| S2-U01 Wi-Fi TEST | Wi-Fi priority 20; late 0. First window 9508..10248 us, lifetime 9508..10491 us. Second window 9518..10482 us | PASS. These are separate windows, not one fabricated range |
| S2-U05 motion-only watchdog | Operator repeated the destructive reset check | PASS; physical GP21 and watchdog remain independent of network |
| S1-U07 CPU | Idle 117 permille; traffic heartbeat 139 permille; calibrated peak 4026729 iterations/s | Target load values observed. No independent absolute CPU-accuracy measurement was supplied |
| S1-U04 connection/LWT | Watchdog reset; broker restart returned robot in about 2 s; power-cut OFFLINE in about 7 s | PASS for these cases. A retained will is not a physical ESTOP |
| S1-U05 live log/DUMP | M1 live CSV 216/216 rows byte-identical to GP20 USB rows; median age 20 ms; DUMP about 1 s | PASS for captured run, not a completeness claim for every QoS 0 run |
| S1-U06 dashboard commands | Uncalibrated START refused; CALIBRATE/START/DUMP/ESTOP worked; wrong token refused, `command_lost +2` | PASS for these cases; not all queue-full/duplicate combinations were target-tested |

The actual radio interval is about 20 ms, not the configured 2 ms request.
A 10 ms kernel tick plus `tk_dly_tsk` extra-tick semantics quantizes the
request to about two ticks (roughly 50 polls/s). This is a known port
property. It is not changed by the boot/session fix. The measured MQTT
median row age is consistent with this interval, but does not prove every
message's end-to-end latency.

## Driving observations

| Run | Evidence | Result and limit |
| --- | --- | --- |
| M1-MQTT | Gap 9916..10087 us; rows 216, events 11; response maximum 1470 us; faults 0; behavior unchanged | PASS for basic line/live-log run |
| Later MQTT ESTOP run | Gap 8952..11038 us; rows 46, events 6; response maximum 1086 us; faults 4 (ESTOP) | Command/stop observed. Lifetime gap exceeds the 11000 us gate by 38 us; cause is not identified by this log |
| M2 initial box scans | Echoes 224..250 mm at all angles | `DT_FAIL_NO_SIDE` is correct for an all-blocked profile |
| M2 16:04–16:05, three attempts | Box 17 x 12 x 7 cm; stood 12 cm wide/17 cm tall and also once 17 cm wide. Echoes 214..290 mm at -30..+30 degrees | `DT_FAIL_NO_SIDE`; neither side had a usable clear point |
| M2 16:13 | Same placement as a failed run, box 17 cm wide. Closest 212 mm at +10; only left step 6 (+30) free/no echo; right blocked | Basic detour PASS: LEFT, 22041 ms, path 995 mm, max lateral 318 mm, returned to line, line losses 0; stopped by dashboard ESTOP; summary `detours_succeeded=1` |
| M2 16:13 timing/log | Gap 9902..10099 us; response maximum 1201 us; rows 1418; overruns 0 | PASS for this observed run |

The M2 decision margin was one outer scan point. Small placement/echo changes
can switch it between no-side and LEFT. The scan/profile/detour logic of this
tested image was unchanged since `445084c`; the result is not evidence of a
software regression. A later operator-approved reverse/retry change needs
its own hardware checks; no historical PASS validates that new behavior.

The USB header's motion gap is lifetime, not per run. The larger gap printed
after the second run could have occurred during the earlier GP20 dump,
idle/calibration, or driving. The old log has no release/entry/mutex timestamp
trace. See [the timing investigation](MOTION_TIMING_INVESTIGATION.md); do not
attribute the delay to Wi-Fi, SCAN or ESTOP without a diagnostic capture.

## Remote dashboard

The operator verified Tailscale Serve on 2026-11-07: live data and all buttons
worked from the remote desktop. Plain HTTP through the tailnet was used;
short-hostname Host/Origin rejection was intentional. HTTPS certificate
provisioning failed on the hotspot and needs a normal-network retry.
See [remote dashboard instructions](MQTT_DASHBOARD.md#remote-view-through-tailscale).

## New-image checks still required

- Cold-power boot IDs across power cuts, watchdog resets and broker restarts.
- Two boots with generation 2/index 0 both preserved in CSV/history.
- TEST absent vehicle shown separately from genuine board contention.
- DIAG=1 release/entry/acquisition evidence around GP20 dump, commands and SCAN.
- Any later SCAN reverse/retry behavior, GP21 cancellation and timeout reason.
