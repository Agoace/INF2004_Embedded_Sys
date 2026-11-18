/* LINE_KD=<n>: the value reaches steer_mm; every other LINE setting and
 * the D change clip (STEER_MAX_CHANGE) stay as the default image. */
#include <stdio.h>
#include <stdlib.h>

#include "../app_program/vehicle/line_kd.h"

static int failures;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            printf("FAIL line %d: ", __LINE__);                                \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while (0)

/* Left minus right after the step from 0 to 'to' (no far term: |to| <= 50). */
static int step_diff(int kd, int to)
{
    lt_config_t cfg = LINE_CFG;
    cfg.steer.kd_pct = (uint8_t)kd;
    steer_mm_t s;
    steer_mm_init(&s, &cfg.steer);
    (void)steer_mm_update(&s, 0);
    const steer_mm_word_t w = steer_mm_update(&s, (int8_t)to);
    return steer_mm_left(w) - steer_mm_right(w);
}

int main(void)
{
    CHECK(LINE_CFG.steer.kd_pct == LINE_KD, "kd %u", LINE_CFG.steer.kd_pct);
    CHECK(LINE_CFG.steer.base_mm_s == 120 && LINE_CFG.steer.gain_pct == 85u &&
              LINE_CFG.steer.min_mm_s == 62 && LINE_CFG.steer.max_mm_s == 378 &&
              LINE_CFG.steer.far_from == 50u &&
              LINE_CFG.steer.far_gain_pct == 340u,
          "other steer settings changed");
    CHECK(LINE_CFG.straight_counts == 450 && LINE_CFG.hint_counts == 20,
          "trace settings changed");
    CHECK(STEER_MAX_CHANGE == 25, "D change clip %d", STEER_MAX_CHANGE);

    /* The configured value is the one steer_mm uses: the step 0 -> 32
     * (t=20680..20720) gives exactly the result of kd = LINE_KD. */
    steer_mm_t s;
    steer_mm_init(&s, &LINE_CFG.steer);
    (void)steer_mm_update(&s, 0);
    const steer_mm_word_t w = steer_mm_update(&s, 32);
    const int got = steer_mm_left(w) - steer_mm_right(w);
    CHECK(got == step_diff(LINE_KD, 32), "steer diff %d, want %d", got,
          step_diff(LINE_KD, 32));
    if (LINE_KD != 85) {
        CHECK(got < step_diff(85, 32), "kd %d not below the default (%d/%d)",
              LINE_KD, got, step_diff(85, 32));
    }
    /* Clip unchanged: a step of 32 and one of 25 give the same D part,
     * within 2 mm/s (each wheel is rounded; base 120 shows 24 vs 26 at
     * kd 85, where an unclipped step of 32 gives about 33). */
    {
        const int d32 = step_diff(LINE_KD, 32) - step_diff(0, 32);
        const int d25 = step_diff(LINE_KD, 25) - step_diff(0, 25);
        CHECK(abs(d32 - d25) <= 2, "D part not clipped at 25 (%d/%d)", d32,
              d25);
    }
    /* Wheel limits hold for every position. */
    for (int p = -100; p <= 100; ++p) {
        steer_mm_reset(&s);
        const steer_mm_word_t x = steer_mm_update(&s, (int8_t)p);
        const int l = steer_mm_left(x), r = steer_mm_right(x);
        CHECK(l >= 0 && l <= 378 && r >= 0 && r <= 378 && (l == 0 || l >= 62) &&
                  (r == 0 || r >= 62),
              "limits at %d: %d %d", p, l, r);
    }
    printf("LINE_KD %d: step 0->32 diff %d mm/s (kd 85: %d)\n", LINE_KD, got,
           step_diff(85, 32));
    return failures != 0;
}
