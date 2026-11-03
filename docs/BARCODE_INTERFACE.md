# Barcode reader interface

Status: Implemented. The decoder stores run durations privately and reads Code
39 in either direction.

`src/barcode_interface.h` is the boundary between a polling caller, such as a
line task, and the barcode decoder.

The caller owns GPIO GP7 and the sampling schedule. It calls `barcode_init()`
once, then calls `barcode_sample()` with the digital sensor level and a
monotonic microsecond timestamp. It reads completed results with
`barcode_take_result()`.

The caller must not inspect a run buffer, classify narrow or wide bars, or
depend on the Code 39 table. Those details belong to the implementation.

`src/barcode_interface.c` implements the interface. It keeps the CODE39 table,
run buffer, quiet-zone detection, glitch handling, and decode details private.
The result mailbox holds one unread result; the caller must read it before a
later completed result is produced.

The polling period must be short enough to observe the narrowest barcode run
at the maximum permitted duty. Use several samples per narrow run. The duty
limit is a system policy; it is not part of this interface.

The standalone target polls GP7 every 250 microseconds and rejects runs
shorter than 750 microseconds. This is a safe starting point for a duty-capped
test. Reduce the polling period before accepting shorter runs.
