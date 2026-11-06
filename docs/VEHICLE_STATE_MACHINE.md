# Vehicle state machine (Vehicle Controller shell)

The shell from [ADR: Vehicle Controller](ADR_VEHICLE_CONTROLLER.md). It
runs one module at a time and maps module outcomes to the next module.
Branch: `yx-vehicle-controller` (interface). Modules use `yx-mod-*`; merge only into `yx-staging`.

- Code (pure, host-tested): [`src/vehicle_sm.c`](../src/vehicle_sm.c), [`src/vehicle_sm.h`](../src/vehicle_sm.h)
- Host unit test: [`tests/test_vehicle_sm.c`](../tests/test_vehicle_sm.c)

[Vehicle Controller step 1](VEHICLE_CONTROLLER.md)
(`vehicle_controller.[ch]`) stays: its event word (`vc_event_t`,
`vc_line_side_t`) is used by the locked line detector
([`line_position.h`](../src/line_position.h)). The new shell has its own
name so that the line detector does not change.

## Steps

| Step | Scope | Status |
| --- | --- | --- |
| a | Shell: route table, start and step of the running module, handoff in the same tick, safety stop, operator start/stop/reset | Done 2026-11-04: host-tested |
| b | Line tracing module (wheel speeds in mm/s) | Done 2026-11-04: host-tested (`Vehicle Controller line run`); needs a track run (line tracing is locked) |
| c | Detour and scan modules | Done 2026-11-04: host-tested (`Vehicle Controller detour run`); needs a track run |
| d | Line search module | Done 2026-11-04: host-tested, same record |
| e | Glue on the Pico: detectors on the heartbeat, `vsm_step()` on the 10 ms flag, drive command to `mcp_*` | First version 2026-11-04: `vc_line_test` (LINE and SEARCH only; line sample on a 1 ms main-loop flag) |

## States

```text
          vsm_start(first)                outcome FAILED, no route, no module,
   IDLE ─────────────────▶ RUNNING ─────▶ handoff loop, or a latched motor fault
    ▲  ◀──── vsm_stop() ────  │ ▲                         │
    │                         └─┘ outcome → route →       ▼
    └───────── vsm_reset() ─────────────────────────── FAULT
                              next module (same tick)
```

| State | Drive command | Leaves on |
| --- | --- | --- |
| IDLE | STOP | `vsm_start()` |
| RUNNING | SPEED with the running module's wheel speeds and servo angle | `vsm_stop()` → IDLE; a fault → FAULT |
| FAULT | STOP | `vsm_reset()` → IDLE. `vsm_stop()` and `vsm_start()` do nothing |

## One step (every 10 ms)

1. RUNNING and a latched motor fault (`mc_faults & MC_FAULT_LATCHED`):
   FAULT (SAFETY, detail = fault bits). The module is not stepped.
2. Not RUNNING: return STOP.
3. Step the running module. Before the call the shell sets the wheels
   to 0 and the servo to its last angle (a module that does not set the
   servo keeps it).
4. RUNNING outcome: return SPEED with the module's values.
5. FAILED: FAULT (FAILED, detail = payload, the module's reason).
6. Look up (running module, outcome type) in the route table. No route:
   FAULT (NO_ROUTE, detail = outcome word). Route to a module with no
   functions: FAULT (NO_MODULE, detail = module number).
7. Start the next module with the outcome payload and go to step 3 in
   the same tick. Up to 3 handoffs in one step; the 4th is FAULT
   (HANDOFF_LOOP).

Each handoff (also the start and the change to FAULT) is recorded in
`vsm.last` (time, from, to, outcome word) and counted in `vsm.handoffs`,
for the log.

## Outcome word (16 bits)

| Bits | Field |
| --- | --- |
| 0–3 | Outcome type (4 bits) |
| 4–15 | Payload (12 bits, 0–4095; larger values are clamped). The shell passes it to the next module unchanged |

| Type | Value | Payload |
| --- | --- | --- |
| RUNNING | 0 | — |
| OBSTACLE | 1 | Distance (mm) |
| SIDE | 2 | Defined by the scan and detour modules (side, distance) |
| DONE_LINE | 3 | — |
| DONE_NO_LINE | 4 | Side to search |
| LINE_LOST | 5 | Last side (`vc_line_side_t`) |
| FOUND | 6 | — |
| FAILED | 7 | Module reason code |

## Route table (`VSM_DEFAULT_ROUTES`)

| Running | Outcome | Next |
| --- | --- | --- |
| Line tracing | OBSTACLE | Scan |
| Scan | SIDE | Detour |
| Detour | DONE_LINE | Line tracing |
| Detour | DONE_NO_LINE | Line search |
| Line tracing | LINE_LOST | Line search |
| Line search | FOUND | Line tracing |
| Line search | OBSTACLE | Scan |

Any other outcome, and FAILED, stop the run (FAULT).

## Module interface

| Function | Use |
| --- | --- |
| `start(ctx, payload, now_ms)` | Called at the handoff, with the payload of the outcome that chose this module (or of `vsm_start()`) |
| `step(ctx, in, now_ms, &drive)` | Called every 10 ms while the module runs. Reads the detectors in `in` (a `struct vsm_inputs` defined by the application), fills `drive`, returns RUNNING or an outcome |

`vsm_step()` returns a `vsm_drive_t` (left and right mm/s, servo degrees,
action SPEED or STOP). The application glue passes it to
`mcp_set_speed()` and `mcp_set_servo()`, or `mcp_stop()`. The shell does
not call the motor controller itself, so it stays host-testable.

## Host unit test

```sh
cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
   tests/test_vehicle_sm.c src/vehicle_sm.c -o build/test_vehicle_sm
./build/test_vehicle_sm
```

The test uses fake modules. It covers the outcome word (layout, clamp,
type mask), registration and start (module 0, 5, without start or step),
the drive command (module speeds, servo kept), the handoff in the same
tick with the payload, every row of the default table, FAILED, no route,
no module, the safety stop (bits 0x01 and 0x02; none while IDLE), stop
and reset (also reset while running does nothing), and the handoff limit
(3 pass, the 4th is FAULT).

Mutation check (2026-11-04): 36 of 36 caught. The check also found one
redundant test (`id != VSM_MOD_NONE` in `has_module()`; module 0 can
never be registered), which was removed.

ARM size (`-Os`, Cortex-M0+): 807 bytes of code.
