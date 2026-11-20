/* V1/V3 live pending policy: expiry by wheel travel, cancel on SEARCH. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/barcode_live_vehicle/policy.h"
#include "../app_program/subsystem3/barcode_live_source.h"

/* mm_counts(10000) = 124140; 9999 mm rounds up to 124128 counts. */
#define LIMIT_COUNTS 124140

static ja_t pending(uint8_t source, int32_t base)
{
    ja_t s;
    blv_init(&s, 131, 1u);
    const ja_command_t c = {.epoch = 1u, .sequence = 5u,
        .read_counts = {base, base}, .command = JA_LEFT, .valid = 1u,
        .source = source};
    assert(ja_offer(&s, &c));
    assert(s.phase == JA_PENDING);
    return s;
}

static void route(ja_t *s, uint8_t module, int32_t l, int32_t r)
{
    const int32_t counts[2] = {l, r};
    blv_pending_route(s, module, counts, 100u);
}

int main(void)
{
    /* Expiry at exactly 10000 mm mean forward travel, not one count before. */
    ja_t s = pending(BC_LIVE_SOURCE, 1000);
    route(&s, VSM_MOD_LINE, 1000 + 124128, 1000 + 124128);
    assert(s.phase == JA_PENDING); /* 9999 mm. */
    route(&s, VSM_MOD_LINE, 1000 + LIMIT_COUNTS - 1, 1000 + LIMIT_COUNTS - 1);
    assert(s.phase == JA_PENDING && s.command.valid == 1u);
    route(&s, VSM_MOD_LINE, 1000 + LIMIT_COUNTS, 1000 + LIMIT_COUNTS);
    assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_EXPIRED);
    assert(s.command.valid == 0u && s.consumed == 5u);
    assert(s.result.claimed == 0u && (s.result.marks & JA_MARK_FINAL));
    assert(s.result.final_counts[0] == 1000 + LIMIT_COUNTS);

    /* Mean of both wheels; wrap-safe; reverse travel never expires. */
    s = pending(BC_LIVE_SOURCE, INT32_MAX - 10);
    route(&s, VSM_MOD_LINE, INT32_MIN + 2 * LIMIT_COUNTS - 21,
          INT32_MAX - 10);
    assert(s.phase == JA_PENDING); /* mean = LIMIT - 5: not yet. */
    route(&s, VSM_MOD_LINE, (int32_t)((uint32_t)INT32_MAX - 10u +
          2u * LIMIT_COUNTS), INT32_MAX - 10);
    assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_EXPIRED);
    s = pending(BC_LIVE_SOURCE, 0);
    route(&s, VSM_MOD_LINE, -3 * LIMIT_COUNTS, -3 * LIMIT_COUNTS);
    assert(s.phase == JA_PENDING);

    /* V3: only a continuous SEARCH of at least 200 ms cancels. */
    s = pending(BC_LIVE_SOURCE, 0);
    const int32_t short_counts[2] = {10, 10};
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 100u);
    assert(s.phase == JA_PENDING);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 299u);
    assert(s.phase == JA_PENDING); /* 199 ms. */
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 300u);
    assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_SEARCH);
    assert(s.command.valid == 0u && s.consumed == 5u);
    assert(!ja_offer(&s, &(ja_command_t){.epoch = 1u, .sequence = 5u,
        .command = JA_LEFT, .valid = 1u, .source = BC_LIVE_SOURCE}));

    /* Return to LINE keeps pending and resets the continuous timer. */
    s = pending(BC_LIVE_SOURCE, 0);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 100u);
    blv_pending_route(&s, VSM_MOD_LINE, short_counts, 299u);
    assert(s.phase == JA_PENDING && s.command.valid == 1u);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 1000u);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 1199u);
    assert(s.phase == JA_PENDING);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 1200u);
    assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_SEARCH);

    /* Expiry still applies during and after a brief SEARCH. */
    for (unsigned int after = 0u; after < 2u; ++after) {
        s = pending(BC_LIVE_SOURCE, 0);
        blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 100u);
        const int32_t far_counts[2] = {LIMIT_COUNTS, LIMIT_COUNTS};
        blv_pending_route(&s, after ? VSM_MOD_LINE : VSM_MOD_SEARCH,
                          far_counts, 150u);
        assert(s.phase == JA_UNUSED &&
               s.result.reason == BLV_REASON_EXPIRED);
    }

    /* Clock wrap and a new run reset the timer. */
    s = pending(BC_LIVE_SOURCE, 0);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, UINT32_MAX - 99u);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 99u);
    assert(s.phase == JA_PENDING);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 100u);
    assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_SEARCH);
    s = pending(BC_LIVE_SOURCE, 0);
    blv_pending_route(&s, VSM_MOD_SEARCH, short_counts, 500u);
    assert(s.phase == JA_PENDING);

    /* SCAN/DETOUR keep a live command (operator 2026-11-20); the detour
     * travel counts, so LINE after it expires at the same bound. */
    for (uint8_t m = VSM_MOD_SCAN; m <= VSM_MOD_DETOUR; ++m) {
        s = pending(BC_LIVE_SOURCE, 0);
        s.clear = true;
        route(&s, m, 10, 10);
        assert(s.phase == JA_PENDING && s.command.valid == 1u);
        assert(s.result.reason == JA_OK && !s.clear);
        route(&s, m, 2 * LIMIT_COUNTS, 2 * LIMIT_COUNTS);
        assert(s.phase == JA_PENDING); /* no expiry inside the detour */
        route(&s, VSM_MOD_LINE, LIMIT_COUNTS - 1, LIMIT_COUNTS - 1);
        assert(s.phase == JA_PENDING);
        route(&s, VSM_MOD_LINE, LIMIT_COUNTS, LIMIT_COUNTS);
        assert(s.phase == JA_UNUSED && s.result.reason == BLV_REASON_EXPIRED);
    }
    /* Fake source 1 keeps the old CANCELLED in SCAN/DETOUR. */
    for (uint8_t m = VSM_MOD_SCAN; m <= VSM_MOD_DETOUR; ++m) {
        s = pending(1u, 0);
        route(&s, m, 10, 10);
        assert(s.phase == JA_UNUSED && s.result.reason == JA_CANCELLED);
    }
    s = pending(BC_LIVE_SOURCE, 0);
    route(&s, VSM_MOD_LINE, 10, 10);
    assert(s.phase == JA_PENDING && s.result.reason == JA_OK);

    /* Fake source 1 is unchanged: survives SEARCH, never expires. */
    s = pending(1u, 0);
    route(&s, VSM_MOD_SEARCH, 10, 10);
    route(&s, VSM_MOD_LINE, 10 * LIMIT_COUNTS, 10 * LIMIT_COUNTS);
    assert(s.phase == JA_PENDING && s.command.valid == 1u);

    /* A consumed action is unchanged, even beyond expiry or in SEARCH. */
    s = pending(BC_LIVE_SOURCE, 0);
    s.phase = JA_BAR_CLEAR;
    s.result.phase = JA_BAR_CLEAR;
    s.result.claimed = 1u;
    s.command.valid = 0u;
    route(&s, VSM_MOD_SEARCH, 10 * LIMIT_COUNTS, 10 * LIMIT_COUNTS);
    assert(s.phase == JA_BAR_CLEAR && s.result.reason == JA_OK);
    assert(s.result.claimed == 1u);

    /* Only PENDING is touched: idle state stays idle. */
    blv_init(&s, 131, 1u);
    route(&s, VSM_MOD_SEARCH, 10 * LIMIT_COUNTS, 10 * LIMIT_COUNTS);
    assert(s.phase == JA_IDLE && s.result.reason == JA_OK);

    assert(strcmp(blv_reason_name(BLV_REASON_EXPIRED), "expired") == 0);
    assert(strcmp(blv_reason_name(BLV_REASON_SEARCH), "search") == 0);
    assert(strcmp(blv_reason_name(JA_CANCELLED), "cancelled") == 0);
    assert(strcmp(blv_reason_name(200u), "unknown") == 0);
    puts("blv policy PASS");
    return 0;
}
