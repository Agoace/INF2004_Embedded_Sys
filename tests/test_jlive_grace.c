#include <assert.h>
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
}
