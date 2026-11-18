/* Live-only pending-command policy (V1 expiry, V3 SEARCH). Compiled ONLY
 * when BARCODE_LIVE_ENABLE=1; default images do not contain it. */
#ifndef BARCODE_LIVE_VEHICLE_POLICY_H
#define BARCODE_LIVE_VEHICLE_POLICY_H

#include <stdint.h>
#include "../vehicle/junction_action.h"

/* V1: wheel travel from the decode (read_counts) that ends a pending live
 * command. About 2x the ~200 mm recommended placement (rule >= 160 mm). */
#define BLV_PENDING_MAX_MM 400u
/* New ja result reasons; phase stays JA_UNUSED (never acted). */
#define BLV_REASON_EXPIRED 13u
#define BLV_REASON_SEARCH 14u

/* Same contract as ja_pending_route. A live (source 2) pending command is
 * dropped on SEARCH entry or after BLV_PENDING_MAX_MM of forward travel. */
void blv_pending_route(ja_t *state, uint8_t module,
                       const int32_t counts[2], uint32_t now_ms);
const char *blv_reason_name(uint8_t reason);

#endif
