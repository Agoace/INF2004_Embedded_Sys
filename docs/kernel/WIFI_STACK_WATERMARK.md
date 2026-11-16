# M9: Wi-Fi stack watermark

## Scope and status

Base: `d91c4c3`, after M4-M8. The operator approved two narrow edits in
already-recorded Wi-Fi template deviations. No push, flash or device test
is included. This change supplies the missing measurement path. It does
**not** establish the hardware 256 B margin gate.

## Ownership and creation

`app_program/platform/app_task.c` owns one static, 8-byte-aligned `cyw43`
stack and slot whenever `TM_WIFI_CYW43` is enabled, including non-MQTT
radio and lwIP profiles. Wi-Fi-off profiles reserve neither.

`cyw43_utk_start()` still constructs its kernel task request. The new
`app_task_start_config()` adapter checks the registered placement, paints
all bytes with `0xa5`, then creates and starts the task with `TA_USERBUF`.
It preserves the request's entry, context, priority 20, `TA_RNG1`, name
and affinity. It does not reuse the ordinary `TA_RNG3` task policy.
Creation remains on core 0; execution is core 1 in split mode and core 0
in UP or forced-core0 mode. UP registry report labels now also say core 0.

Capacity is unchanged. Non-USERBUF creation previously added
`DEFAULT_SYS_STKSZ` (128 B on this port) and rounded to 8 B. USERBUF uses
exactly the supplied size. The registered allocation therefore is:

| Profile | Old request | Old actual allocation / new static region |
| --- | ---: | ---: |
| Radio only | 6144 B | 6272 B |
| Wi-Fi with lwIP, with or without MQTT | 8192 B | 8320 B |

The adapter rejects a request that does not match the registered size.
The input request is not changed. Successful start registers one task ID.
Duplicate start fails before painting or kernel creation. Failed create
leaves no row. Failed start deletes the dormant task and leaves no row.
If deletion fails, its negative ID quarantines the slot until reboot:
retry returns `E_OBJ` without touching the still-owned buffer. The caller
retains the original start error. No allocation, wait or lock is added to
the create/start sequence.

These APIs are for the startup creator only. There is no runtime restart
or stack repaint contract. Wi-Fi metadata is ready before app observers
start; the M8 startup handoff remains unchanged.

## Measurement and the margin rule

The ordinary low-rate registry scan includes `cyw43`. Its persistent
paint records deep writes even if that call returned before the next
poll or report. It includes kernel initial context and radio startup,
not only steady-state polling. The old entry-SP call, poll-frame sampler,
SP words, getter and separate `wifi free <=` estimate are removed.

Health's existing 1 Hz scan includes this row in `stack_min_free`.
TEST/GP20 use the same normal report:

```text
[stack] cyw43 core 1 size 8320 high_water <used> free <free> OK
```

`free < 256` prints `LOW MARGIN`; 256 meets the reported minimum. A fully
written region reports zero, not an unsigned wrap. No stop, watchdog,
priority, poll delay, wire format or network-owner policy changes.
There is no scan or additional diagnostic call in the Wi-Fi poll path.
The maximum table still fits `APP_TASK_MAX_SLOTS = 10`; no bound increase
or caller truncation is needed.

The shared scan retains its monotonic high-water lock/atomic contract.
On contention it can report an older mark; take a later quiet scan before
acceptance. Paint is a historical written-byte estimate, not exact peak
SP depth or an overflow detector. A store equal to the paint or unwritten
reserved frame space can escape measurement. Initial, USB and interrupt
stack measurements remain separate. CR1's additional sizing rules remain
unchanged; this work does not shrink a stack.

## RAM: static reserve up, dynamic allocation down

Matched public-example SMP1 MQTT production builds, same toolchain and
profile, compared with `d91c4c3`:

| Measurement | Before | M9 | Delta |
| --- | ---: | ---: | ---: |
| Text | 419952 B | 419756 B | -196 B |
| Data | 1008 B | 1032 B | +24 B |
| BSS, including RAM vector | 191220 B | 199532 B | +8312 B |
| **Static RAM** | **192228 B** | **200564 B** | **+8336 B** |
| Dynamic Wi-Fi stack payload request | 8320 B | 0 B | -8320 B |

The static delta is 8320 B for the stack plus 24 B for its slot, minus
8 B for the removed SP words. The stack now appears in the map rather
than being allocated from the kernel pool. For radio-only profiles the
moved payload is 6272 B, not 8320 B.

This is **not** an 8320 B RAM saving. The kernel pool starts after BSS, so
its initial capacity also falls as static RAM grows. The payload accounting
difference is +16 B; allocator headers, fragmentation and remaining runtime
allocations are separate. No runtime heap-free result is claimed. The
linked `__bss_end` moves from `0x2002eee4` to `0x20030f74`. Never add the
removed heap request to the static remainder as if it were a second saving.

See also the [CR1 evidence note](../architecture/CR1_EVIDENCE.md#m9-wi-fi-stack-accounting).
The migration helper lacks the staging B2b additions; this is not a merged
vehicle RAM or timing baseline.

## Software evidence

Logs and reports are retained under `build/review/`:

- `m9-red-{config,build,tests}.log`: old production startup fails both bare
  radio and lwIP stack tests before implementation.
- `m9-gcc-{config,build,tests}.log`: GCC host suite, 336/336.
- `m9-clang-{config,build,tests}.log`: Clang ASan/UBSan/float-cast-overflow,
  leak detection, 336/336.
- `m9-matrix.log`, `m9-final-*.log`: 24 firmware profiles, zero warnings.
  Each profile also passes the retained SDK link probe; copies live under
  `m9-sdk-probes/`. These separate probe ELFs must never be flashed.
- `m9-strict-arm.log`: 81 app units in each profile plus both approved
  Wi-Fi units in 15 radio profiles: 1974 strict ARM checks. Flags include
  `-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror`. The kernel-facing
  Wi-Fi TEST wrapper also gets a local-name correction for `-Wshadow`.
- `m9-baseline-size.txt`, `m9-final-size.txt`, retained maps/ELFs and
  `m9-byte-identity.log`: resource and unchanged-boundary evidence.

`tests/test_template_wifi.c` links the real radio initializer and registry.
It checks aligned paint before creation, policy/context/capacity,
create/start/cleanup failure, safe retry, no duplicate repaint, returned
512 B deep-write retention, monotonic marks, 257/256/255/0 B boundaries,
bounded rows and all slots in UP/split/control configurations.
`tests/wifi_stack_health_checks.c` runs the real Health refresh once and
checks that Wi-Fi supplies the unique minimum at the full registry layout.
`tests/test_wifi_stack_contention.c` includes the real registry in one host
translation unit to force its private application lock busy, without a
production hook or linker wrap. It checks monotonic fallback and a later
quiet scan in UP/split/control. Wi-Fi-off checks remain in
`tests/test_app_task.c`.

Only `cyw43_utk_kernel.c` and `cyw43_utk.c` change under template paths.
Their existing [ledger rows](../architecture/TEMPLATE_CHANGES.md) are
updated: no ninth path. `sys_arch.h`, `lwipopts.h`, `cyw43_utk.h`, kernel
and SDK sources stay byte-identical. No linker wrap, kernel TCB access,
SDK startup, extra task or per-poll stack probe is introduced.

## Hardware gate: not run

1. Use the approved merged image. Capture cold boot, the printed core audit
   and the first ordinary `cyw43` row. Verify size 6272/8320 as appropriate.
2. Exercise association, DHCP, reconnect, MQTT commands, full/aborted DUMP
   and concurrent IMU/LINE work. Retain later quiet stack reports, including
   the deepest observed `cyw43` mark and Health minimum.
3. Require at least 256 B free for the Wi-Fi row after these paths. Treat
   `LOW MARGIN` or missing evidence as a failed or incomplete gate. Check
   the other named stacks and deadlines separately.
4. Repeat split/control timing and cold-power operation as required by the
   migration plan. Software builds do not prove hardware margin, placement,
   deadline, heap-free or CPU acceptance.
