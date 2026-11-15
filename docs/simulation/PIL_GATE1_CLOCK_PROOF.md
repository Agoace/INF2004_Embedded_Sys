# Gate 1 — directional serialization minima

## Scope and result

This is a source/timing proof, not a new Pico image or a hardware PASS. Operator
attempt4 obtained eight valid SYNC exchanges, but zero-minimum bounds correctly
refused ARM at about585.5us halfwidth. Keep the250us limit. Hardware work is on
hold until the operator resumes it. No coding worker accessed target hardware.

Source checkpoint: `5c7361a4462d07ab6df6bb60f35a9141fe5c82ca`.
UF2 SHA256: `0017cfa0c05ca69447bf1cd8d41d1801bbca909ed3c66e914e6901958034ba08`.
The [build handoff](PIL_GATE1_BUILD.md) and [clock contract](PIL_PROTOCOL.md)
remain applicable. No wire format, clock drift, quantization or expiry changes.

For the qualified link conditions below, use these conservative integer floors:

| Direction | Credited complete characters | Safe rate ceiling | Floor calculation | Final minimum |
| --- | ---: | ---: | --- | ---: |
| PC request to Pico receive stamp |33|985000 bit/s|floor(330000000/985000)-1|334us|
| Pico reply stamp to PC complete receive |41|921000 bit/s|floor(410000000/921000)-1|444us|

Names: `SYNC_REQUEST_WIRE_MIN_US=334`, `SYNC_REPLY_WIRE_MIN_US=444`.
The subtracted1us is an extra conservative floor margin, not a USB delay credit.
Do not silently apply these floors to another baud, frame size, device profile
or timestamp placement. Missing eligibility evidence means INCOMPLETE. A real
UART fault remains FAIL; this proof must not erase or downgrade it. Live use
before the stopped D1 arrives is provisional: final qualification requires that
D1 evidence, and a failed condition invalidates the attempt.

## Programmed clock and framing

The [UART adapter](../../app_program/sim/pil_gate1_uart_hw.c) rejects a clock
other than125000000Hz and programs IBRD=8, FBRD=31, LCR_H=`0x70`, CR=`0x301`.
Thus8 data bits, no parity, one stop bit: ten wire bits per complete character.
There is no CTS/RTS flow control. FIFO settings do not compress wire time.

```text
D = IBRD + FBRD/64 = 8 + 31/64 = 543/64
B_pico = 125000000 / (16 D) = 920810.3130755065 bit/s
T_pico = 1/B_pico = 1.086us
error versus921600 = -0.08568651524452629 percent
```

This is the programmed mean rate, not a physical frequency measurement.
`cpu_clock.c` selects the125MHz PLL/sys peripheral clock from the12MHz XOSC.
`drv_time_hw.c` uses the same reference divided to the1MHz watchdog/TIMER tick.
Their fixed ratio gives the rate above in Pico TIMER microseconds. Conversion
to PC microseconds retains the existing100ppm relative-rate condition; a faster
Pico makes a shorter wire duration, so the reply ceiling includes that margin:
`920810.3130755065 * 1.0001 = 920902.3941068139 < 921000`.

## Exact timestamp causality

| Stamp | Source and exact placement |
| --- | --- |
| PC t1 | `Transport.sync_request(pc_us)` puts the same queued PC timestamp into the request payload/header. The caller obtains it before encoding, queueing and any serial write of this request. It is not the write-completion log timestamp. |
| Pico t2 | `uart_irq` reads UARTDR, then calls `pil_gate1_rx_byte(..., drv_time_us(), ...)`. When that byte is the zero delimiter, the IRQ stores this stamp in the completed packet. It is not the first-byte stamp or a later task stamp. FIFO threshold, RX timeout and IRQ latency can only delay it. |
| Pico t3 | `pil_gate1_receive` calls `frame(..., PIL_SYNC_REPLY, ...)`, which reads real TIMER for the header source. Payload copy, encoding, queueing, direct FIFO priming and every reply wire byte occur afterward. |
| PC t4 | Complete reply/delimiter receive snapshot, after the Probe received the corresponding wire bytes. USB buffering, host scheduling and parsing cannot make them arrive earlier. |

The [RX stream](../../app_program/sim/pil_gate1_stream.c) retains t2 through
worker decode; the [session](../../app_program/sim/pil_gate1_session.c) echoes
it in SYNC_REPLY. The [runtime](../../app_program/sim/pil_gate1_task.c) does CRC
and state handling in the task, not in UART IRQ context. Prior packets in either
FIFO, encoding/processing, USB and scheduler delays are nonnegative. None is
subtracted as an assumed minimum.

Raw sizes are fixed: request header20 + payload8 + CRC4 =32 decoded bytes;
COBS plus delimiter makes34 wire bytes. Reply header20 + payload16 + CRC4 =40;
COBS plus delimiter makes42. We credit only the first33/41 full characters.
The final delimiter may enter RX FIFO at the stop-bit sample, before the full
last stop bit has elapsed. Therefore34*10 and42*10 bits are NOT used as floors.

## Probe-rate condition and receiver-derived ceiling

Pico source alone does not program or identify the Probe transmitter's divisor.
Also, receiving without an error does NOT establish the UART's usual guaranteed
all-data/all-phase tolerance interval. In particular, a stop sample at9.5 bit
periods can land in HIGH idle after an isolated character; that alone supplies
no upper transmitter-rate bound. The guaranteed-tolerance interval must not be
used as the converse of successful reception.

The end-to-end bound instead uses the known zero delimiter in each accepted
SYNC request. Preconditions are normal8N1 transmission, stable divisors during
the exchange, and correspondence between the PC's logged request and the
CRC-valid, exact-length Pico frame with its matching reply echo/sequence.
The final UART octet is known to be`00`, not an arbitrary unknown character.
It must decode as zero without UART errors.

Under PL01116x sampling, data bit7 is nominally sampled at8.5 receiver bit
periods after its start; the stop sample is nominally at9.5. Allow a full1/16
bit period EARLY, even though start detection normally adds delay. The earliest
last-data sample is then8.4375 receiver periods. Because transmitted data bit7
is LOW and its following stop/idle is HIGH, a correctly associated zero octet
requires this last-data sample before the transmitter's stop starts at9 of its
bit periods. In the ideal phase model this gives:

```text
8.4375 T_pico < 9 T_probe
B_probe < (9/8.4375) B_pico = (16/15) B_pico
```

Do not substitute the tighter stop-guarantee bound for this implication. For
fractional-divider/edge clock granularity, add another three125MHz peripheral
clocks of EARLY allowance: one for a maximum integer oversample tick rather
than its mean, one for fractional phase, and one for edge-synchronization phase.
This deliberately broadens the digital sampling-model bound:

```text
135 D - 3 = 1142.390625 peripheral clocks
T_earliest = (135 D - 3)/125000000 = 9.139125us
B_probe < 9 / T_earliest = 984776.989044356 bit/s
include100ppm PC/Pico unit conversion: <984875.4667432604 bit/s
round the safe ceiling UP to985000 bit/s
```

PL011 sampling reference: Arm PrimeCell UART(PL011) Technical Reference Manual,
receive operation: start is checked on the eighth Baud16 cycle; subsequent data
samples occur every16 Baud16 cycles. This proof also states the normal-framing
and byte-correspondence conditions explicitly. Error-free counters alone are
not a general-purpose baud meter for arbitrary or aliased bitstreams.

## Required end-to-end evidence

- Log both raw packets and the matching SYNC identity/echo, exact34/42 lengths,
  image hash, framing, divisor/clock context, minima and proof revision.
- Pico D1 must retain the full matching nonce/session and26-count schema.
  Zero-based COUNTS indices are BAD_FRAME11, OVERRUN13, FRAMING14, PARITY15,
  BREAK16. All must be zero; raw overflow/coverage fault checks also remain.
  COUNTS9 is TX_FULL, not BAD_FRAME. Never clear counters to qualify minima.
- Each accepted exchange must correspond to the normal known delimiter framing
  described above. If framing/byte association or rate stability cannot be
  established, do not credit these minima. Keep zero defaults outside this
  explicitly qualified transport profile.
- Keep outward TIMER/monotonic floor-to-us quantization separate and apply it
  once in clock bounds. Retain100ppm observation aging, eight-observation
  intersection, refresh/headroom limits and the250us ARM threshold.
- Reject impossible observations/minima; do not force an empty intersection to
  become valid. Re-evaluate the actual eight recorded exchanges, not an assumed
 585.5-(334+444)/2 result. Those numbers alone do not establish a physical PASS.

This record changes documentation only. The Pico image is unchanged. The PC
owner implements and tests conditional minima; only the operator resumes target
runs. Source evidence and any pending work are saved before HOLD.
