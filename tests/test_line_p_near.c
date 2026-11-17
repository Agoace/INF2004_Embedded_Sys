/* Compile the real robot configuration and steering for each candidate. */
#include <assert.h>
#include <stdlib.h>
#include "../app_program/vehicle/line_p_near.h"

static int difference(int position)
{
    steer_mm_t steer;
    steer_mm_init(&steer, &LINE_CFG.steer);
    const steer_mm_word_t word = steer_mm_update(&steer, (int8_t)position);
    return steer_mm_left(word) - steer_mm_right(word);
}

int main(void)
{
    assert(LINE_CFG.steer.gain_pct == LINE_P_NEAR);
    assert(LINE_CFG.steer.kd_pct == 85u);
    assert(LINE_CFG.steer.far_gain_pct == 340u);
    assert(LINE_CFG.steer.far_from == 50u);
    assert(LINE_CFG.steer.base_mm_s == 146);
    assert(LINE_CFG.steer.min_mm_s == 62);
    assert(LINE_CFG.steer.max_mm_s == 378);
    assert(SEARCH_CFG.speed_mm_s == 131);
    assert(LINE_CFG.straight_counts == 450);
    assert(LINE_CFG.hint_counts == 20);
    assert(difference(0) == 0);
    /* One normal outer-region step changes each wheel by at most
     * ceil(84 * (P + 340) / 10000) mm/s. No boundary-sized jump. */
    const int step = (84 * (LINE_P_NEAR + 340) + 9999) / 10000;
    for (int sign = -1; sign <= 1; sign += 2) {
        const int boundary = abs(difference(sign * 51) -
                                 difference(sign * 50));
        assert(boundary <= 2 * step);
        assert(difference(sign * 50) == -difference(-sign * 50));
    }
    /* Original formula with candidate P, D reset on first update. */
    for (int position = -100; position <= 100; ++position) {
        steer_mm_config_t cfg = LINE_CFG.steer;
        cfg.gain_pct = LINE_P_NEAR;
        steer_mm_t reference;
        steer_mm_init(&reference, &cfg);
        const steer_mm_word_t word =
            steer_mm_update(&reference, (int8_t)position);
        assert(difference(position) ==
               steer_mm_left(word) - steer_mm_right(word));
    }
    return 0;
}
