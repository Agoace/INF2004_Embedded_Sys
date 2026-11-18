#ifndef BARCODE_LIVE_VEHICLE_CONTROL_H
#define BARCODE_LIVE_VEHICLE_CONTROL_H
#include "../subsystem3/barcode_live_producer.h"
#if !BARCODE_LIVE_ENABLE
#error "control cache is live-only"
#endif
void blc_producer_uncached(barcode_live_producer_t *p,
                           const barcode_live_input_t *in);
extern uint32_t blc_control_read_misses;
#endif
