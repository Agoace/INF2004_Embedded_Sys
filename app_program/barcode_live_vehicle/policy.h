/* Live-only pending-command policy (V1 expiry, V3 SEARCH). Compiled ONLY
 * when BARCODE_LIVE_ENABLE=1; default images do not contain it. */
#ifndef BARCODE_LIVE_VEHICLE_POLICY_H
#define BARCODE_LIVE_VEHICLE_POLICY_H

#include <stdint.h>
#include "../vehicle/junction_action.h"

/* V1: wheel travel from decode (read_counts). Operator-approved 1000 mm
 * bound; placement rule >=160 mm is unchanged. Not a frame_end bound. */
#define BLV_PENDING_MAX_MM 1000u
/* A short natural SEARCH can put the line between the sensors. */
#define BLV_SEARCH_CANCEL_MS 200u
/* New ja result reasons; phase stays JA_UNUSED (never acted). */
#define BLV_REASON_EXPIRED 13u
#define BLV_REASON_SEARCH 14u

/* Pure policy state; one Vehicle owner, no cross-task access. */
typedef struct {
    uint32_t sequence;
    uint32_t epoch;
    uint32_t search_ms;
    bool searching;
} blv_t;

void blv_policy_route(blv_t *policy, ja_t *state, uint8_t module,
                       const int32_t counts[2], uint32_t now_ms);

/* Vehicle-owner wrappers. blv_init also resets the SEARCH timer. A live
 * pending command expires by distance or after 200 ms continuous SEARCH. */
void blv_init(ja_t *state, int16_t search_speed_mm_s, uint32_t epoch);
void blv_pending_route(ja_t *state, uint8_t module,
                       const int32_t counts[2], uint32_t now_ms);
const char *blv_reason_name(uint8_t reason);

#endif
