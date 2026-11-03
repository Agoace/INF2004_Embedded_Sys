# Barcode reader interface

Status: Interface only. The Code 39 implementation belongs in a later branch.

`src/barcode_interface.h` is the boundary between a polling caller, such as a
line task, and the barcode decoder.

The caller owns GPIO GP7 and the sampling schedule. It calls `barcode_init()`
once, then calls `barcode_sample()` with the digital sensor level and a
monotonic microsecond timestamp. It reads completed results with
`barcode_take_result()`.

The caller must not inspect a run buffer, classify narrow or wide bars, or
depend on the Code 39 table. Those details belong to the implementation.

`src/barcode_interface_stub.c` provides no results. It lets a consumer branch
build before the decoder implementation is merged. Replace that file with the
implementation without changing this public interface.

The polling period must be short enough to observe the narrowest barcode run
at the maximum permitted duty. Use several samples per narrow run. The duty
limit is a system policy; it is not part of this interface.
