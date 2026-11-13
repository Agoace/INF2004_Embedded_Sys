# PIL Stage 0 — platform software evidence

Reference: `451e562`. Scope: host-only codec and source-selection skeletons.
No runtime UART, edge scheduler, register emulator or hardware soak is supplied.
Do not flash the `_sim` images. Gate 1 remains blocked on the real Probe/Pico.

## Frozen interface

[PIL protocol v1](PIL_PROTOCOL.md) defines the 40 B SENSOR / 32 B OUTPUT payloads,
control acknowledgement, fixed timeline, IMU source phases, cache age, validity,
fault precedence and reset-only lifecycle. The C codec is stateless; passing it
is not proof of session freshness, a running emulator or physical timing.

The 26 shared fixtures have 13 accepted and 13 rejected packets, including
CRC-correct semantic errors. C tests check exact encode/decode, every truncated
prefix, every single wire-bit mutation, failed destination preservation, CRC
reference, sequence wrap, output capacity and 100000 deterministic noisy packets.
An independent Python reference reproduces the committed fixture bytes.
SW5 separately reported 3026 C/PC comparisons passing: 26 fixtures plus 3000
CRC-valid semantic mutations. That report is interoperability evidence, not a
hardware result or an extra test run hidden in the counts below.

## Six fail-closed adapters

The Make switch replaces only motor, line ADC, barcode buttons, IMU,
ultrasonic and GPIO hardware units. The skeletons emit no physical PWM/trigger,
no encoder edges, no button press and no invented ADC/IMU sample. ADC returns
read failure; IMU returns NACK (invalid arguments retain PARAM). Caller buffers
remain untouched. Barcode USB readiness still uses the real cached console
state, not a fabricated ready flag.

Real `drv_time_hw.c`, watchdog and native application logic stay linked. Line
alarm0 initialization, scheduling, force-pending and acknowledge use the real
TIMER registers. Host tests cover deadlines before/at/after now and timer wrap.
No source of an existing application C function was edited. No usermain wrapper
or SDK runtime is added. SIM=1 requires USB console and CRASH_REPORT=0.

## Results

| Check | Result |
| --- | --- |
| Full strict GCC host suite | 547/547 |
| Full Clang ASan/UBSan host suite | 547/547 |
| Independent original blocker/shim suite | 23/23 |
| Ordinary public-example firmware profiles | 38/38, zero warnings |
| Retained SDK link proofs, ordinary profiles | 38/38 |
| SIM skeleton versions of those profiles | 38/38, zero warnings |
| Strict ARM checks, seven new units x38 SIM profiles | 266/266 |
| Literal ordinary ELF and UF2 identity | 76/76 hashes equal |
| Same-profile SIM=0 ->1 ->0 rebuild | Correct provider/invalidation; ordinary hash restored |
| Template ledger | Eight deviations, unchanged |

The 38 profiles are 24 UP/split/control base profiles, two exclusive UP TEST
barcode captures, and A-D fake command profiles for each placement. Live barcode
actions stay disabled. SIM builds are link-only, not retained SDK proofs: the
existing SDK profile checker does not claim qualification of the new SIM key.

One new Make test initially used Python lexical sorting for expected GNU Make
wildcard order. GNU glob order differs in the current locale. The test-only fix
runs the ORIGINAL wildcard expression in an isolated native-only source tree,
then compares its exact order with the switch. It does not sort/reorder firmware
sources or weaken image equality. Existing tests are unchanged.

[Full baseline/Stage0 hash pairs](PIL_STAGE0_HASHES.json) record literal SHA256,
including ELF debug metadata, not just load segments. Both builds use the same
public source/build path, compiler, SDK and public example settings. No ignored
private operator settings are inputs. After the SIM matrix, the ordinary matrix
was rebuilt and compared again. No metadata exception was used.

## Resource limits

Non-SIM images are byte-identical: zero code, static RAM or behavior delta.
MQTT skeleton sizes (normal, non-TEST, non-BENCH) are:

| Placement | Ordinary static RAM | SIM skeleton static RAM |
| --- | --- | --- |
| UP | 199516 B | 199492 B |
| Split | 204468 B | 204444 B |
| Forced-core0 | 204468 B | 204444 B |

The 24 B decrease is omitted hardware-adapter state, not a functional simulator
RAM saving. There is no link state, transport queue, new task, heap allocation
or extra stack reservation. Unused codec functions are garbage-collected from
these skeleton images. Runtime simulation storage and CPU costs are NOT proved.

Strict ARM static compiler frames are 184 B for encode and 288 B for decode;
callers/helper frames are additional. Codec calls are not attached to an ISR.
A future interrupt implementation must explicitly review its full stack chain;
Stage 0 does not claim the earlier 256 B extra-IRQ-stack planning target passed.
No on-device CPU, latency, interrupt-off or stack watermark was measured.

Local raw builds/proofs are under `build/pil-proof/`; the operator review table
is `build/wip/pil_plan/STAGE0_HASH_TABLE.md` with JSON beside it. These local
paths are not required for checkout. Host tests and shared fixtures are tracked;
Gate 1 remains a separate, unperformed hardware qualification.
