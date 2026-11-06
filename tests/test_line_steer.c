/*
 * Host unit test for app_program/subsystem3/line_steer.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_line_steer.c app_program/subsystem3/line_steer.c -o build/test_line_steer
 *   ./build/test_line_steer
 */
#include <stdio.h>

#include "../app_program/subsystem3/line_steer.h"

static int failures;

static void expect(steer_t *s, int position, unsigned int left, unsigned int right,
                   const char *what) {
    const steer_word_t w = steer_update(s, (int8_t)position);
    if (steer_left(w) != left || steer_right(w) != right) {
        printf("FAIL %s: position %+d gave %u/%u, want %u/%u\n", what, position,
               steer_left(w), steer_right(w), left, right);
        ++failures;
    }
}

static steer_t make(uint8_t base, uint8_t gain, uint8_t kd, uint8_t min, uint8_t max) {
    const steer_config_t cfg = {base, gain, kd, min, max, 0, 0u, 0u};
    steer_t s;
    steer_init(&s, &cfg);
    return s;
}

int main(void) {
    steer_t s = make(30u, 100u, 0u, 0u, 100u);
    expect(&s, 0, 30u, 30u, "centred: straight");
    expect(&s, 50, 45u, 15u, "line right: left wheel faster");
    expect(&s, -50, 15u, 45u, "line left: right wheel faster");
    expect(&s, 100, 60u, 0u, "gain 100 at +100: inner wheel stops");
    expect(&s, -100, 0u, 60u, "gain 100 at -100");
    expect(&s, 33, 40u, 20u, "rounded to nearest");

    /* The same position gives the same ratio at another speed. */
    s = make(20u, 100u, 0u, 0u, 100u);
    expect(&s, 50, 30u, 10u, "ratio scales with base");

    s = make(30u, 50u, 0u, 0u, 100u);
    expect(&s, 100, 45u, 15u, "gain 50");

    /* Over max: the turn (difference) is kept. */
    s = make(40u, 100u, 0u, 0u, 50u);
    expect(&s, 50, 50u, 10u, "left capped, right lowered");
    expect(&s, -50, 10u, 50u, "right capped, left lowered");
    expect(&s, 0, 40u, 40u, "under max: unchanged");

    /* max above 100 is limited to 100. */
    s = make(80u, 100u, 0u, 0u, 127u);
    expect(&s, 50, 100u, 20u, "max limited to 100");

    /* Speed-based: min 10 is speed zero, base 30 is speed 20. */
    s = make(30u, 100u, 0u, 10u, 100u);
    expect(&s, 0, 30u, 30u, "speed-based: straight");
    expect(&s, 60, 42u, 18u, "speed-based: speed 32/8 -> duty 42/18");
    expect(&s, 90, 48u, 12u, "speed-based: inner speed 2 -> duty 12, not 0");
    expect(&s, 100, 50u, 0u, "speed-based: inner speed 0 -> stopped");

    /* steer_test settings: no cliff before the inner wheel stops. */
    s = make(20u, 150u, 0u, 12u, 30u);
    expect(&s, 25, 23u, 17u, "gain 150: +25");
    expect(&s, 50, 26u, 14u, "gain 150: +50 (was 30/0 with the cliff)");
    expect(&s, 60, 27u, 13u, "gain 150: +60");
    expect(&s, 66, 28u, 0u, "gain 150: +66 inner stops");
    expect(&s, 100, 30u, 0u, "gain 150: +100 capped");
    {
        unsigned int previous_inner = 20u;
        for (int pos = 1; pos <= 100; ++pos) {
            const steer_word_t w = steer_update(&s, (int8_t)pos);
            const unsigned int inner = steer_right(w);
            const int smooth = previous_inner >= inner && previous_inner - inner <= 1u;
            const int stops = inner == 0u && (previous_inner == 13u || previous_inner == 0u);
            if (!smooth && !stops) {
                printf("FAIL cliff at position %d: inner %u -> %u\n", pos, previous_inner,
                       inner);
                ++failures;
            }
            previous_inner = inner;
        }
    }

    /* D: no D part on the first update, then it reacts to the change. */
    s = make(30u, 100u, 50u, 0u, 100u);
    expect(&s, 20, 36u, 24u, "first update: P only");
    expect(&s, 40, 45u, 15u, "moving right: P 12 + D 3");
    expect(&s, 40, 42u, 18u, "not moving: P only");
    expect(&s, 20, 33u, 27u, "moving back: D opposes");
    steer_reset(&s);
    expect(&s, 40, 42u, 18u, "after reset: P only");

    /* D: the change is limited to STEER_MAX_CHANGE (25) per update. */
    s = make(30u, 100u, 50u, 0u, 100u);
    expect(&s, 0, 30u, 30u, "D limit: start");
    expect(&s, 100, 64u, 0u, "D limit: jump +100 counts as +25 (P 30 + D 4)");
    expect(&s, 0, 26u, 34u, "D limit: jump -100 counts as -25");

    /* steer_test settings with D: braking while the line comes back. */
    {
        const steer_config_t cfg = {25u, 200u, 200u, 12u, 50u, 1, 0u, 0u};
        steer_init(&s, &cfg);
        expect(&s, -30, 17u, 34u, "kd 200: line left, no D yet");
        expect(&s, -12, 27u, 24u, "kd 200: line returns fast: D turns the other way");
        expect(&s, -12, 22u, 29u, "kd 200: line still: P only");
    }

    /* Straight: base and trim, no D afterwards. */
    {
        const steer_config_t cfg = {20u, 100u, 50u, 12u, 30u, 1, 0u, 0u};
        steer_init(&s, &cfg);
        expect(&s, 40, 23u, 18u, "straight test: start at +40");
        const steer_word_t w = steer_straight(&s);
        if (steer_left(w) != 20u || steer_right(w) != 21u) {
            printf("FAIL straight: %u/%u, want 20/21 (no D from +40)\n", steer_left(w),
                   steer_right(w));
            ++failures;
        }
        expect(&s, 40, 23u, 18u, "after straight: P only, no D from 0 to +40");
    }

    /* set_config keeps the previous position: D goes on. */
    {
        const steer_config_t fast = {33u, 100u, 100u, 12u, 100u, 0, 0u, 0u};
        const steer_config_t calm = {25u, 100u, 100u, 12u, 100u, 0, 0u, 0u};
        steer_init(&s, &fast);
        expect(&s, 10, 35u, 31u, "set_config: fast, no D yet");
        steer_set_config(&s, &calm);
        expect(&s, 20, 29u, 21u, "set_config: calm, P 20 + D 10 from the kept +10 (28/22 without D)");
    }

    /* Calm mode. */
    {
        steer_calm_t c;
        steer_calm_init(&c);
        int ok = 1;
        ok &= steer_calm_update(&c, 0, false, 0u) == true;     /* starts calm */
        ok &= steer_calm_update(&c, 10, false, 299u) == true;  /* near 299 ms */
        ok &= steer_calm_update(&c, -20, false, 300u) == false; /* near 300 ms: fast */
        if (!ok) { printf("FAIL calm: start, then fast after 300 ms near\n"); ++failures; }

        ok = 1;
        ok &= steer_calm_update(&c, 49, false, 400u) == false;  /* 49: still fast */
        ok &= steer_calm_update(&c, -50, false, 410u) == true;  /* 50: calm */
        ok &= steer_calm_update(&c, 0, false, 420u) == true;
        ok &= steer_calm_update(&c, 21, false, 600u) == true;   /* 21 breaks "near" */
        ok &= steer_calm_update(&c, 0, false, 700u) == true;    /* near again from 700 */
        ok &= steer_calm_update(&c, 0, false, 999u) == true;
        ok &= steer_calm_update(&c, 0, false, 1000u) == false;
        if (!ok) { printf("FAIL calm: enter at 50, a break restarts the 300 ms\n"); ++failures; }

        ok = 1;
        ok &= steer_calm_update(&c, 0, true, 2000u) == true;    /* trigger */
        ok &= steer_calm_update(&c, 0, true, 2200u) == true;    /* trigger held */
        ok &= steer_calm_update(&c, 0, false, 2210u) == true;   /* near from 2210 */
        ok &= steer_calm_update(&c, 0, false, 2509u) == true;
        ok &= steer_calm_update(&c, 0, false, 2510u) == false;
        if (!ok) { printf("FAIL calm: trigger, 300 ms counted after it ends\n"); ++failures; }

        ok = 1;
        steer_calm_init(&c);
        ok &= steer_calm_update(&c, 0, false, 0xFFFFFF00u) == true;
        ok &= steer_calm_update(&c, 0, false, 0x0000002Cu) == false; /* 300 ms over the wrap */
        if (!ok) { printf("FAIL calm: time wrap\n"); ++failures; }
    }

    /* Trim: +1 on the right wheel. */
    {
        const steer_config_t trimmed = {20u, 100u, 0u, 12u, 30u, 1, 0u, 0u};
        steer_init(&s, &trimmed);
        expect(&s, 0, 20u, 21u, "trim +1: right wheel +1");
        expect(&s, 25, 22u, 19u, "trim +1 while turning right");
        expect(&s, 100, 28u, 0u, "trim +1: not on a stopped wheel");
        expect(&s, -50, 16u, 25u, "trim +1 while turning left");
        const steer_config_t trimmed_150 = {20u, 150u, 0u, 12u, 30u, 1, 0u, 0u};
        steer_init(&s, &trimmed_150);
        expect(&s, -60, 13u, 28u, "trim +1, gain 150: -60");
        expect(&s, -100, 0u, 30u, "trim +1: capped at max (33 -> 30)");
        const steer_config_t trimmed_no_min = {20u, 100u, 0u, 0u, 100u, 1, 0u, 0u};
        steer_init(&s, &trimmed_no_min);
        expect(&s, 100, 40u, 0u, "trim +1, min 0: stopped wheel stays stopped");
        const steer_config_t left_trim = {20u, 100u, 0u, 12u, 30u, -2, 0u, 0u};
        steer_init(&s, &left_trim);
        expect(&s, 0, 22u, 20u, "trim -2: left wheel +2");
    }

    /* Far gain: the detour_test settings without D (base 25, min 12,
     * max 50, gain 100, far gain 400 above 50). Speed 13. */
    {
        const steer_config_t far = {25u, 100u, 0u, 12u, 50u, 0, 50u, 400u};
        steer_t f;
        steer_init(&f, &far);
        expect(&f, 0, 25u, 25u, "far: centred");
        expect(&f, 30, 29u, 21u, "far: +30, gain 100 only");
        expect(&f, 50, 32u, 18u, "far: +50, no far part yet");
        expect(&f, 51, 32u, 18u, "far: +51, far part 400");
        expect(&f, 55, 35u, 15u, "far: +55, turn 7500");
        expect(&f, 60, 38u, 0u, "far: +60, full turn");
        expect(&f, 70, 45u, 0u, "far: +70, beyond full");
        expect(&f, -55, 15u, 35u, "far: -55, mirrored");
        expect(&f, -60, 0u, 38u, "far: -60, full turn");
        expect(&f, -50, 18u, 32u, "far: -50, no far part");

        const steer_config_t off = {25u, 100u, 0u, 12u, 50u, 0, 50u, 0u};
        steer_init(&f, &off);
        expect(&f, 60, 33u, 17u, "far gain 0: off");

        const steer_config_t from0 = {30u, 100u, 0u, 0u, 100u, 0, 0u, 100u};
        steer_init(&f, &from0);
        expect(&f, -25, 15u, 45u, "far from 0: same as gain 200");
        expect(&f, 25, 45u, 15u, "far from 0: same as gain 200, right");

        /* One unit past far_from already counts. */
        const steer_config_t edge = {50u, 0u, 0u, 0u, 100u, 0, 10u, 1000u};
        steer_init(&f, &edge);
        expect(&f, 10, 50u, 50u, "far: at far_from, nothing");
        expect(&f, 11, 55u, 45u, "far: one past far_from");
        expect(&f, -11, 45u, 55u, "far: one past -far_from");

        /* far_gain above 255 is kept (16-bit field). */
        const steer_config_t big = {30u, 0u, 0u, 0u, 100u, 0, 0u, 300u};
        steer_init(&f, &big);
        expect(&f, 10, 39u, 21u, "far gain 300 at +10");
    }

    /* Word layout. */
    s = make(30u, 100u, 0u, 0u, 100u);
    {
        const steer_word_t w = steer_update(&s, 50);
        if (w != (45u | (15u << 7))) {
            printf("FAIL word 0x%04X\n", w);
            ++failures;
        }
    }

    /* ---- Steering in mm/s ---- */

    /* Same rule: with the duty numbers as mm/s (trim 0), the output is the
     * same as steer_update() for a sequence with D and far gain. */
    {
        const steer_config_t pc = {25u, 100u, 100u, 12u, 50u, 0, 50u, 400u};
        const steer_mm_config_t mc = {25, 100u, 100u, 12, 50, 50u, 400u};
        steer_t d;
        steer_mm_t m;
        steer_init(&d, &pc);
        steer_mm_init(&m, &mc);
        const int seq[] = {0, 10, 30, 55, 80, 100, 70, 20, -5, -40, -60, -100, -100, -30, 0, 46, 61, 0};
        int bad = 0;
        for (unsigned int i = 0; i < sizeof seq / sizeof seq[0]; ++i) {
            const steer_word_t wd = steer_update(&d, (int8_t)seq[i]);
            const steer_mm_word_t wm = steer_mm_update(&m, (int8_t)seq[i]);
            if ((unsigned int)steer_mm_left(wm) != steer_left(wd) ||
                (unsigned int)steer_mm_right(wm) != steer_right(wd)) {
                printf("FAIL mm same rule at %d: mm %d/%d, duty %u/%u\n", seq[i], steer_mm_left(wm),
                       steer_mm_right(wm), steer_left(wd), steer_right(wd));
                ++bad;
            }
        }
        failures += bad;
    }

    /* Locked line tracing in mm/s (docs/LINE_STEER_MM.md): base 171, min 62,
     * max 378, gain 100, D 100, far gain 400 above 50. */
    {
        const steer_mm_config_t lk = {171, 100u, 100u, 62, 378, 50u, 400u};
        steer_mm_t m;
        steer_mm_init(&m, &lk);
        steer_mm_word_t w = steer_mm_update(&m, 0);
        if (steer_mm_left(w) != 171 || steer_mm_right(w) != 171) {
            printf("FAIL mm centred: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
        /* +20, change +20: turn 100*20 + 100*20 = 4000 -> 109*0.4 = 43.6 -> 44. */
        w = steer_mm_update(&m, 20);
        if (steer_mm_left(w) != 215 || steer_mm_right(w) != 127) {
            printf("FAIL mm +20: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
        /* +100 (change limited to +25): turn 10000 + 2500 + 400*50 = 32500
         * -> 354; left 525 -> 378, right 0 -> 0 (no reverse). */
        w = steer_mm_update(&m, 100);
        if (steer_mm_left(w) != 378 || steer_mm_right(w) != 0) {
            printf("FAIL mm +100: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
        /* Inner wheel at the floor: speed part above 0 gives at least 62. */
        steer_mm_reset(&m);
        w = steer_mm_update(&m, 99); /* turn 9900 + 400*49 = 29500 -> 322: right 109-322 < 0 */
        if (steer_mm_right(w) != 0) {
            printf("FAIL mm +99 right %d\n", steer_mm_right(w));
            ++failures;
        }
        steer_mm_reset(&m);
        w = steer_mm_update(&m, 40); /* turn 4000 -> 44: right 62 + 65 = 127 */
        if (steer_mm_right(w) != 127 || steer_mm_left(w) != 215) {
            printf("FAIL mm +40 no D: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
        /* Straight: base both, D forgotten. */
        (void)steer_mm_update(&m, 60);
        w = steer_mm_straight(&m);
        if (steer_mm_left(w) != 171 || steer_mm_right(w) != 171 || m.has_previous) {
            printf("FAIL mm straight: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
        w = steer_mm_update(&m, 40); /* no D after straight: same as +40 above */
        if (steer_mm_left(w) != 215) {
            printf("FAIL mm after straight: %d\n", steer_mm_left(w));
            ++failures;
        }
    }

    /* Limits: max above 511 is set to 511; word layout 9 + 9 bits. */
    {
        const steer_mm_config_t big = {400, 100u, 0u, 0, 2000, 100u, 0u};
        steer_mm_t m;
        steer_mm_init(&m, &big);
        if (m.cfg.max_mm_s != STEER_MM_MAX) {
            printf("FAIL mm max limit %d\n", m.cfg.max_mm_s);
            ++failures;
        }
        const steer_mm_config_t over = {400, 100u, 0u, 0, 512, 100u, 0u};
        steer_mm_init(&m, &over);
        if (m.cfg.max_mm_s != STEER_MM_MAX) {
            printf("FAIL mm max 512 -> 511: %d\n", m.cfg.max_mm_s);
            ++failures;
        }
        const steer_mm_config_t at = {400, 100u, 0u, 0, 511, 100u, 0u};
        steer_mm_init(&m, &at);
        if (m.cfg.max_mm_s != 511) {
            printf("FAIL mm max 511 kept %d\n", m.cfg.max_mm_s);
            ++failures;
        }
        /* +50: 400 + 200 = 600 -> 511, right 200 - 89 = 111. */
        const steer_mm_word_t w = steer_mm_update(&m, 50);
        if (w != (511u | (111u << 9))) {
            printf("FAIL mm word 0x%05lX\n", (unsigned long)w);
            ++failures;
        }
        if (steer_mm_left(0xFFFFFFFFu) != 511 || steer_mm_right(0xFFFFFFFFu) != 511 ||
            steer_mm_right(0x1FFu) != 0) {
            printf("FAIL mm word fields\n");
            ++failures;
        }
    }

    /* Max clamp pushes the other wheel below min: it stops (shared rule).
     * base 300, min 62, max 378, gain 100, at -70: left 62 + 238 - 167 = 133,
     * right 62 + 238 + 167 = 467 -> 378, left 133 - 89 = 44 < 62 -> 0. */
    {
        const steer_mm_config_t hi = {300, 100u, 0u, 62, 378, 100u, 0u};
        steer_mm_t m;
        steer_mm_init(&m, &hi);
        const steer_mm_word_t w = steer_mm_update(&m, -70);
        if (steer_mm_left(w) != 0 || steer_mm_right(w) != 378) {
            printf("FAIL clamp below min: %d/%d\n", steer_mm_left(w), steer_mm_right(w));
            ++failures;
        }
    }

    if (failures == 0) {
        printf("test_line_steer: all tests passed\n");
        return 0;
    }
    printf("test_line_steer: %d failure(s)\n", failures);
    return 1;
}
