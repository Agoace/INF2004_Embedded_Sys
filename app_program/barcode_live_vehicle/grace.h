/* Live-only SEARCH eligibility grace; never changes the real mode. */
#ifndef BARCODE_LIVE_GRACE_H
#define BARCODE_LIVE_GRACE_H
#include "../subsystem3/barcode_live_source.h"
#define BLG_SEARCH_MS 200u
void blg_publish(const barcode_live_control_t *c, uint32_t start, bool active);
bool blv_decode_grace_active(const barcode_live_control_t *c, uint32_t now);
#endif
