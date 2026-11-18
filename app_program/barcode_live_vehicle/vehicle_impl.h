/* Forced include ONLY on vehicle_task.o in a BARCODE_LIVE_ENABLE=1 build:
 * route the pending-command check through the V1/V3 live policy. */
#ifndef BARCODE_LIVE_VEHICLE_VEHICLE_IMPL_H
#define BARCODE_LIVE_VEHICLE_VEHICLE_IMPL_H
#if !BARCODE_LIVE_ENABLE
#error "barcode_live_vehicle seam: BARCODE_LIVE_ENABLE=1 only"
#endif
#define jl_step blt_line_step
#define ja_init blv_init
#define ja_pending_route blv_pending_route
#define ja_reason_name blv_reason_name
#endif
