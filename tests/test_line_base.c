/* Native config/steering check, not a simulation of vehicle position. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../app_program/vehicle/line_p_near.h"

int main(void)
{
    assert(LINE_CFG.steer.base_mm_s == LINE_BASE);
    assert(LINE_CFG.steer.min_mm_s == 62);
    assert(LINE_CFG.steer.max_mm_s == 378);
    assert(LINE_CFG.steer.gain_pct == 85u);
    assert(LINE_CFG.steer.kd_pct == 85u);
    assert(LINE_CFG.steer.far_from == 50u);
    assert(LINE_CFG.steer.far_gain_pct == 340u);
    assert(SEARCH_CFG.speed_mm_s == 131);
    assert(LINE_CFG.straight_counts == 450);
    assert(LINE_CFG.hint_counts == 20);
    steer_mm_t steer;
    steer_mm_init(&steer, &LINE_CFG.steer);
    steer_mm_word_t word = steer_mm_update(&steer, 0);
    assert(steer_mm_left(word) == LINE_BASE);
    assert(steer_mm_right(word) == LINE_BASE);
    unsigned int far = 0u;
    for (int position = -100; position <= 100; ++position) {
        steer_mm_reset(&steer);
        word = steer_mm_update(&steer, (int8_t)position);
        const int left = steer_mm_left(word);
        const int right = steer_mm_right(word);
        assert(left >= 0 && left <= 378);
        assert(right >= 0 && right <= 378);
        assert(left == 0 || left >= 62);
        assert(right == 0 || right >= 62);
        if (abs(position) > 52) { ++far; }
        if (position == 53) {
            printf("base %d position 53 requested_diff_mm_s %d\n",
                   LINE_BASE, left - right);
        }
    }
    /* Uniform input sweep only; cannot predict time spent far from centre. */
    assert(far == 96u);
    printf("uniform sweep far %u/201; not a robot far-share prediction\n",
           far);
    return 0;
}
