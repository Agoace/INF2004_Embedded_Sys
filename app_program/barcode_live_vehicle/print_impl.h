/* Forced include ONLY on junction_print.o in a BARCODE_LIVE_ENABLE=1
 * build: [JA] lines also name reasons 13 (expired) and 14 (search). */
#ifndef BARCODE_LIVE_VEHICLE_PRINT_IMPL_H
#define BARCODE_LIVE_VEHICLE_PRINT_IMPL_H
#if !BARCODE_LIVE_ENABLE
#error "barcode_live_vehicle seam: BARCODE_LIVE_ENABLE=1 only"
#endif
#define ja_reason_name blv_reason_name
#endif
