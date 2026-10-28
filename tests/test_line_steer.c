/*
 * Host unit test for src/line_steer.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_line_steer.c src/line_steer.c -o build/test_line_steer
 *   ./build/test_line_steer
 */
#include <stdio.h>

#include "line_steer.h"

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
    const steer_config_t cfg = {base, gain, kd, min, max, 0};
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

    /* Below min: wheel stopped, not left at a duty that cannot turn it. */
    s = make(30u, 100u, 0u, 10u, 100u);
    expect(&s, 90, 57u, 0u, "3 % is below min 10: stop");
    expect(&s, 60, 48u, 12u, "12 % is above min: kept");

    /* D: no D part on the first update, then it reacts to the change. */
    s = make(30u, 100u, 50u, 0u, 100u);
    expect(&s, 20, 36u, 24u, "first update: P only");
    expect(&s, 40, 45u, 15u, "moving right: P 12 + D 3");
    expect(&s, 40, 42u, 18u, "not moving: P only");
    expect(&s, 20, 33u, 27u, "moving back: D opposes");
    steer_reset(&s);
    expect(&s, 40, 42u, 18u, "after reset: P only");

    /* Trim: +1 on the right wheel. */
    {
        const steer_config_t trimmed = {20u, 100u, 0u, 12u, 30u, 1};
        steer_init(&s, &trimmed);
        expect(&s, 0, 20u, 21u, "trim +1: right wheel +1");
        expect(&s, 25, 25u, 16u, "trim +1 while turning right");
        expect(&s, 100, 30u, 0u, "trim +1: inner wheel still stops");
        expect(&s, -50, 0u, 30u, "trim +1: capped at max, turn kept (31 -> 30, 10 -> 9 -> 0)");
        const steer_config_t left_trim = {20u, 100u, 0u, 12u, 30u, -2};
        steer_init(&s, &left_trim);
        expect(&s, 0, 22u, 20u, "trim -2: left wheel +2");
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

    if (failures == 0) {
        printf("test_line_steer: all tests passed\n");
        return 0;
    }
    printf("test_line_steer: %d failure(s)\n", failures);
    return 1;
}
