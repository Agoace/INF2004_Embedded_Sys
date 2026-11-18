/* Forced include ONLY on barcode_live_producer.o in live builds. */
#ifndef BARCODE_LIVE_VEHICLE_PRODUCER_IMPL_H
#define BARCODE_LIVE_VEHICLE_PRODUCER_IMPL_H
#include "control.h"
#define barcode_live_producer_observe blc_producer_uncached
#endif
