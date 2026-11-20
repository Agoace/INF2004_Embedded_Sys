/* Pure live pending policy; wraps ja_pending_route. No hardware calls. */
#include "policy.h"
#include "../subsystem3/barcode_live_source.h"
#include "../vehicle/junction_source.h"

_Static_assert(BLV_PENDING_MAX_MM > BARCODE_MEASURED_CLEAR_MM + 160u,
               "expiry must exceed the guard plus the 160 mm placement");
_Static_assert(BLV_REASON_EXPIRED > JA_TOTAL_LIMIT + 1u,
               "13 follows 11 and legacy 12");

static int32_t delta(int32_t value, int32_t origin)
{
    return (int32_t)((uint32_t)value - (uint32_t)origin);
}

/* Mean signed travel of both wheels; backward travel is never distance. */
static bool expired(const ja_t *s, const int32_t counts[2])
{
    const int64_t limit = ((int64_t)BLV_PENDING_MAX_MM * JA_COUNTS_PER_M +
                           999) / 1000;
    const int64_t mean =
        ((int64_t)delta(counts[0], s->command.read_counts[0]) +
         delta(counts[1], s->command.read_counts[1])) / 2;
    return mean >= limit;
}

void blv_policy_route(blv_t *p, ja_t *s, uint8_t module,
                       const int32_t counts[2], uint32_t now)
{
    if (s->phase == JA_PENDING && s->command.source == BC_LIVE_SOURCE) {
        if (p->sequence != s->command.sequence ||
            p->epoch != s->command.epoch) {
            *p = (blv_t){.sequence = s->command.sequence,
                          .epoch = s->command.epoch};
        }
        uint8_t reason = JA_OK;
        if (module == VSM_MOD_SEARCH) {
            if (!p->searching) {
                p->search_ms = now;
                p->searching = true;
            }
            if (now - p->search_ms >= BLV_SEARCH_CANCEL_MS) {
                reason = BLV_REASON_SEARCH;
            }
        } else {
            p->searching = false;
        }
        /* Distance expiry wins, including during a short SEARCH. */
        if ((module == VSM_MOD_LINE || module == VSM_MOD_SEARCH) &&
            expired(s, counts)) {
            reason = BLV_REASON_EXPIRED;
        }
        if (reason != JA_OK) {
            ja_finish(s, counts, now); /* PENDING -> UNUSED, consumed. */
            s->result.reason = reason;
            p->searching = false;
            return;
        }
        /* Operator 2026-11-20: a live command survives an obstacle SCAN
         * and DETOUR (it was CANCELLED). Junction evidence restarts; the
         * detour travel counts toward the distance bound. */
        if (module == VSM_MOD_SCAN || module == VSM_MOD_DETOUR) {
            ja_suspend(s);
            return;
        }
    } else {
        *p = (blv_t){0};
    }
    ja_pending_route(s, module, counts, now);
}

const char *blv_reason_name(uint8_t reason)
{
    if (reason == BLV_REASON_EXPIRED) {
        return "expired";
    }
    if (reason == BLV_REASON_SEARCH) {
        return "search";
    }
    return ja_reason_name(reason);
}
