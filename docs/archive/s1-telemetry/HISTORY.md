# Telemetry history

The telemetry records below were removed from the tree on 2026-11-16.
The current design and operation are in
[TELEMETRY.md](../../s1-telemetry/TELEMETRY.md); bytes and versions stay in
[MQTT_WIRE.md](../../s1-telemetry/MQTT_WIRE.md). Each removed file is still
in Git, unchanged.

Read one file:

```sh
git show 569672a:docs/archive/s1-telemetry/KERNEL_MQTT.md
```

Commit `569672a` is on `origin/mtk3-staging`. Use the path from the table.
Relative links inside an old file can point to files that have moved.

## Files

| Path at `569672a` | What it holds | Now in |
| --- | --- | --- |
| `docs/s1-telemetry/OPERATION.md` | Broker, private settings, recorder, dashboard, commands, page, Tailscale, troubleshooting | [TELEMETRY.md](../../s1-telemetry/TELEMETRY.md#operation-broker-recorder-and-dashboard) (full text) |
| `docs/archive/s1-telemetry/KERNEL_MQTT.md` | First MQTT design: state v3 fields, throughput assumptions, refill simulation 2.8/14.2/28.4 s, Wi-Fi poll cadence about 20 ms, 1 us tick at startup | History only |
| `docs/archive/s1-telemetry/KERNEL_TELEMETRY.md` | First telemetry owner: 4096 B message buffer, priority-30 task, by-value messages | History only |
| `docs/archive/s1-telemetry/MQTT_DASHBOARD.md` | Old dashboard panel tables, scan fan, run path with 111 mm track | History only |
| `docs/archive/s1-telemetry/MQTT_RUNBOOK.md` | Old broker and recorder runbook | History only |
| `docs/archive/MTK3_STEP8_RECORD.md` | STEP 8 software validation, 2026-11-07 hardware results (M1/ESTOP/M2, box scan, USB gap 14467 us), boot-session follow-up | History only |
| `docs/archive/MTK3_FOLLOWUP_HANDOFF.md` | STEP 8 owner tips, cleanup and recovery | History only |
| `docs/archive/WIFI_TELEMETRY_DASHBOARD.md` | Old UDP bench dashboard (`scripts/telemetry_dashboard.py`) | History only |
| `docs/archive/TELEMETRY_SCHEMA.md` | Old instrument JSON contract; not the MQTT wire format | History only |

[STEER_WIFI_LOG.md](../STEER_WIFI_LOG.md) stays in the tree: the old
steering firmware in `src/` and `scripts/steer_log_receiver.py` cite it as
their packet format.
