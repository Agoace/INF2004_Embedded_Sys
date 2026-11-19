#include <assert.h>
#include <string.h>
#include "../app_program/vehicle/junction_live.h"

bool blv_decode_grace_active(const barcode_live_control_t *c, uint32_t now);
int barcode_live_control_publish(const barcode_live_control_t *c)
{
    (void)c;
    return 0;
}

int main(void)
{
    jlive_t s;
    ja_t a;
    ja_init(&a, 131, 1);
    jlive_start(&s, 1, 1);
    jlive_route(&s, VSM_MOD_LINE, 2);
    lp_t line = {.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN,
                  .norm = {600, 300}};
    ja_sample_t sample = {.valid = true, .line_stamp = 3};
    int32_t counts[2] = {0};
    jlive_tick(&s, &line, &sample, counts, 3, &a, false);
    const barcode_live_control_t bound = s.control;
    assert(bound.mode == BC_LIVE_PERMITTED);
    jlive_route(&s, VSM_MOD_SEARCH, 100);
    assert(s.control.mode == BC_LIVE_DISABLED);
    assert(s.control.eligibility_revision == bound.eligibility_revision);
    assert(blv_decode_grace_active(&s.control, 299));
    jlive_route(&s, VSM_MOD_LINE, 299);
    jlive_tick(&s, &line, &sample, counts, 299, &a, false);
    assert(s.control.mode == BC_LIVE_PERMITTED);
    assert(s.control.eligibility_revision == bound.eligibility_revision);
    assert(s.control.revision_ms == bound.revision_ms);
    assert(!blv_decode_grace_active(&s.control, 299));
    jlive_route(&s, VSM_MOD_SEARCH, 300);
    assert(!blv_decode_grace_active(&s.control, 500));
    jlive_tick(&s, &line, &sample, counts, 500, &a, false);
    assert(s.control.eligibility_revision != bound.eligibility_revision);

    /* Hard routes, busy, invalid input and actual black never get grace. */
    for (unsigned int k = 0; k < 6; ++k) {
        ja_init(&a, 131, 1);
        jlive_start(&s, 1, 1);
        jlive_route(&s, VSM_MOD_LINE, 2);
        line.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
        line.norm[0] = 600; line.norm[1] = 300;
        sample.valid = true;
        jlive_tick(&s, &line, &sample, counts, 3, &a, false);
        const uint32_t revision = s.control.eligibility_revision;
        jlive_route(&s, VSM_MOD_SEARCH, 100);
        barcode_live_control_t wrong = s.control;
        ++wrong.eligibility_revision;
        assert(!blv_decode_grace_active(&wrong, 110));
        if (k < 3) {
            const uint8_t route[3] = {VSM_MOD_SCAN, VSM_MOD_DETOUR,
                                      VSM_MOD_NONE};
            jlive_route(&s, route[k], 110);
        } else {
            if (k == 4) sample.valid = false;
            if (k == 5) {
                line.flags |= LP_FLAG_JUNCTION;
                line.norm[0] = line.norm[1] = 1000;
            }
            jlive_tick(&s, &line, &sample, counts, 110, &a, k == 3);
        }
        assert(s.control.eligibility_revision != revision);
        assert(!blv_decode_grace_active(&s.control, 110));
    }

    /* F2: a decoded record that waits for the clear guard survives one
     * clean LINE->SEARCH->LINE blip shorter than BLG_SEARCH_MS. */
    enum { CLEAN, JUNCTION, BLACK, STALE, BUSY, SCAN };
    static const struct { unsigned int ms; int kind; bool kept; } f2[] = {
        {20, CLEAN, true}, {40, CLEAN, true}, {180, CLEAN, true},
        {220, CLEAN, false}, {40, JUNCTION, false}, {40, BLACK, false},
        {40, STALE, false}, {40, BUSY, false}, {40, SCAN, false},
    };
    for (unsigned int k = 0; k < sizeof f2 / sizeof f2[0]; ++k) {
        ja_init(&a, 131, 1);
        jlive_start(&s, 1, 1);
        jlive_route(&s, VSM_MOD_LINE, 2);
        line.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
        line.norm[0] = 600; line.norm[1] = 300;
        sample.valid = true;
        int32_t c[2] = {0, 0};
        uint32_t t = 10;
        for (; t <= 100; t += 10) {
            c[0] = c[1] = (int32_t)t;
            sample.line_stamp = t;
            jlive_tick(&s, &line, &sample, c, t, &a, false);
        }
        /* Decoded at 95 ms, frame end at the current counts: the 100 mm
         * guard holds it (BARCODE_MEASURED_CLEAR_MM=100). */
        const barcode_live_record_t r = {.epoch = 1, .sequence = 1,
            .read_ms = 95, .read_counts = {c[0], c[1]}, .command = JA_LEFT,
            .valid = 1, .source = BC_LIVE_SOURCE,
            .frame_end_counts = {c[0], c[1]},
            .eligibility_revision = s.control.eligibility_revision,
            .letter = 'F', .direction = 1, .flags = BC_LIVE_REQUIRED,
            .version = BC_LIVE_VERSION};
        ja_command_t out;
        assert(!jlive_read(&s, &r, t, &out));
        assert(s.waiting);
        const uint32_t from = t;
        jlive_route(&s, f2[k].kind == SCAN ? VSM_MOD_SCAN : VSM_MOD_SEARCH,
                    from);
        for (t = from; t < from + f2[k].ms; t += 10) {
            c[0] += 3; c[1] -= 3; /* Pivot: the mean travel stays 0. */
            sample.line_stamp = t;
            const bool dark = f2[k].kind == JUNCTION || f2[k].kind == BLACK;
            line.flags = LP_FLAG_CALIBRATED |
                (dark ? LP_FLAG_LINE_SEEN | LP_FLAG_JUNCTION : 0u);
            line.norm[0] = line.norm[1] = dark ? 1000 : 100;
            sample.valid = f2[k].kind != STALE;
            jlive_tick(&s, &line, &sample, c, t, &a, f2[k].kind == BUSY);
            assert(!jlive_read(&s, NULL, t, &out));
        }
        jlive_route(&s, VSM_MOD_LINE, t);
        line.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
        line.norm[0] = 600; line.norm[1] = 300;
        sample.valid = true;
        /* Drive past the guard: 130 mm. */
        bool offered = false;
        const uint32_t back = t;
        for (; t < back + 1200; t += 10) {
            c[0] += 14; c[1] += 14;
            sample.line_stamp = t;
            jlive_tick(&s, &line, &sample, c, t, &a, false);
            if (jlive_read(&s, NULL, t, &out)) {
                offered = true;
                assert(out.sequence == 1 && out.command == JA_LEFT);
                break;
            }
            if (!s.waiting) break;
        }
        assert(offered == f2[k].kept);
        if (!f2[k].kept) {
            assert(s.report.reason == JLIVE_ROUTE ||
                   s.report.reason == JLIVE_CROSSED);
            assert(s.control.ack_sequence == 1u);
        }
    }
}
