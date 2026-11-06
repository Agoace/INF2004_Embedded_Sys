# Kernel telemetry owner

S1 creates `mbf_telemetry` (4096-byte kernel-managed message buffer) and a
priority-30 core-0 task. Producers send fixed-size, by-value messages with
`telemetry_send()` and `TMO_POL`. They never wait for storage or USB. A
full message buffer increments the lifetime loss counter and returns the
error. No producer pointer is retained. Core-0 DI/EI protects counters
shared with producers; the consumer alone owns the row/event arrays.

Message kinds are BEGIN, ROW, EVENT, INFO and DUMP. BEGIN starts a new
session and resets array indices, not lifetime losses. The queued BEGIN
captures the producer loss baseline so losses after it cannot be hidden
by delayed consumption. The arrays keep 4501 16-byte KR1 rows and 512
12-byte VE events, unchanged from the previous robot. Storage exhaustion
also counts loss and sets storage_full. The vehicle stops its active
session if a producer send fails or storage_full is observed. Control does
not wait for recovery. This preserves the earlier stop-on-log-capacity
policy without blocking a producer.

GP20 is read by vehicle, which sends DUMP plus copied diagnostic metadata.
The low-priority consumer prints the header, stacks, KR rows, VE events and
`tm_usb_dropped` footer. It yields after each eight rows/events and at the
end; these are USB pacing waits, not periodic work. DUMP reserves busy
before enqueue, and BEGIN/new messages are rejected while the dump is
pending or active. The vehicle does not start a new session until busy
clears. RAM logs do not survive reset. Normal record formatting is absent
from the vehicle task; only error/start notices remain there.

This is event-driven, rather than the plan's illustrative 110 ms telemetry
period: drain immediately when scheduled, otherwise block on the MBF.
It reduces backlog and does not introduce drift or a periodic timer. No
MQTT or network runtime is added. This queue is the later publication
plug-in point; future network integration must still execute CYW43/lwIP
operations in the existing Wi-Fi owner, not this task or control tasks.

The transport does not change KR1 bit fields or VE text. Decoders continue
to ignore descriptive headers. The 4 spare KR1 flag bits contain only the
existing saturation flag and three unused bits; there is no spare byte
for barcode ADC, so the row is unchanged. GP20 metadata can show raw_bc.

Host doubles check value copying, TMO_POL, full-buffer losses, row capacity,
storage reset, dump exclusion, pacing, row/event output and stack report.
S1-U02/U03 on hardware remain **Not run** for this task-split image.
