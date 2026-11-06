/* Host test for src/heartbeat.c.
 * Build: cc -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror -Isrc \
 *        tests/test_heartbeat.c src/heartbeat.c -o build/test_heartbeat
 */
#include <stdio.h>

#include "heartbeat.h"

static int failures;
#define CHECK(cond, ...)                                                                \
    do {                                                                                \
        if (!(cond)) {                                                                  \
            ++failures;                                                                 \
            printf("FAIL line %d: ", __LINE__);                                         \
            printf(__VA_ARGS__);                                                        \
            printf("\n");                                                               \
        }                                                                               \
    } while (0)

typedef struct {
    int calls;
    uint32_t last_us;
    uint32_t first_tick;
    hb_t *hb;
} probe_t;

static void hook(uint32_t now_us, void *ctx) {
    probe_t *p = ctx;
    if (p->calls == 0) p->first_tick = p->hb->ticks;
    ++p->calls;
    p->last_us = now_us;
}

static hb_t hb;
static uint32_t now;

static void ticks(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i) {
        hb_tick(&hb, now);
        now += HB_TICK_US;
    }
}

int main(void) {
    /* 1. Registration limits. */
    hb_init(&hb);
    probe_t p = {0, 0u, 0u, &hb};
    CHECK(hb_add_hook(&hb, 0u, 0u, hook, &p) == -1, "hook period 0 refused");
    CHECK(hb_add_hook(&hb, 10u, 10u, hook, &p) == -1, "hook phase = period refused");
    CHECK(hb_add_hook(&hb, 10u, 9u, 0, &p) == -1, "hook without a function refused");
    CHECK(hb_add_flag(&hb, 0u, 0u) == -1, "flag period 0 refused");
    CHECK(hb_add_flag(&hb, 10u, 10u) == -1, "flag phase = period refused");
    for (unsigned int i = 0; i < HB_MAX_HOOKS; ++i) {
        CHECK(hb_add_hook(&hb, 10u, 9u, hook, &p) == (int)i, "hook %u accepted", i);
    }
    CHECK(hb_add_hook(&hb, 10u, 0u, hook, &p) == -1, "hook table full");
    for (unsigned int i = 0; i < HB_MAX_FLAGS; ++i) {
        CHECK(hb_add_flag(&hb, 10u, 9u) == (int)i, "flag %u accepted", i);
    }
    CHECK(hb_add_flag(&hb, 10u, 0u) == -1, "flag table full");
    CHECK(hb_take(&hb, -1) == 0u && hb_take(&hb, (int)HB_MAX_FLAGS) == 0u, "take: bad flag is 0");

    /* 2. Hook period and phase: every 10 ticks, at tick 3, 13, 23, ... */
    hb_init(&hb);
    now = 5000u;
    probe_t a = {0, 0u, 0u, &hb};
    probe_t b = {0, 0u, 0u, &hb};
    (void)hb_add_hook(&hb, 10u, 3u, hook, &a);
    (void)hb_add_hook(&hb, 1u, 0u, hook, &b);
    ticks(3u);
    CHECK(a.calls == 0, "phase 3: not before tick 3");
    ticks(1u);
    CHECK(a.calls == 1 && a.first_tick == 3u && a.last_us == 8000u, "phase 3: at tick 3 (%u, %u us)",
          a.first_tick, a.last_us);
    ticks(96u);
    CHECK(a.calls == 10 && b.calls == 100, "100 ticks: 10 and 100 calls (%d, %d)", a.calls, b.calls);
    CHECK(hb.ticks == 100u, "tick count %u", hb.ticks);

    /* 3. Flags: posted every period; take returns the posts and clears. */
    hb_init(&hb);
    now = 0u;
    const int f10 = hb_add_flag(&hb, 10u, 0u);
    const int f50 = hb_add_flag(&hb, 50u, 49u);
    CHECK(hb_take(&hb, f10) == 0u, "flag: nothing before the first tick");
    ticks(1u);
    CHECK(hb_take(&hb, f10) == 1u && hb_take(&hb, f10) == 0u, "flag: one post, then clear");
    CHECK(hb_take(&hb, f50) == 0u, "flag 50: not posted at tick 0 (phase 49)");
    ticks(9u);
    CHECK(hb_take(&hb, f10) == 0u, "flag: not due at tick 9");
    ticks(1u);
    CHECK(hb_take(&hb, f10) == 1u && hb.missed_frames == 0u, "flag: due at tick 10, no miss");
    ticks(20u); /* ticks 11..30: posts at 20, 30 */
    CHECK(hb_take(&hb, f10) == 2u && hb.missed_frames == 1u, "flag: 2 posts, 1 missed (%u)", hb.missed_frames);
    hb.missed_frames = 0u;
    ticks(30u); /* ticks 31..60: posts at 40, 50, 60 */
    CHECK(hb_take(&hb, f10) == 3u && hb.missed_frames == 2u, "flag: slow main loop, 3 posts, 2 missed (%u)",
          hb.missed_frames);
    CHECK(hb_take(&hb, f50) == 1u, "flag 50 at phase 49");

    /* 4. Counter wrap: posted wraps past 65535. */
    hb.posted[f10] = 65535u;
    hb.taken[f10] = 65534u;
    ticks(10u); /* tick 70: post -> 0 */
    CHECK(hb.posted[f10] == 0u && hb_take(&hb, f10) == 2u, "wrap: 2 pending across the wrap");

    /* A flag number past the table gives 0 and changes nothing. */
    hb.gap_overruns = 5u;
    {
        const uint32_t missed = hb.missed_frames;
        CHECK(hb_take(&hb, (int)HB_MAX_FLAGS) == 0u && hb.missed_frames == missed, "take past the table");
    }

    /* 5. Gap overrun: more than 1.5 ms between ticks. */
    hb.missed_frames = 7u;
    hb.isr_max_us = 9u;
    hb.isr_overruns = 3u;
    hb_init(&hb);
    CHECK(hb.missed_frames == 0u && hb.gap_overruns == 0u && hb.isr_max_us == 0u && hb.isr_overruns == 0u &&
              hb.ticks == 0u,
          "init clears the counters");
    now = 0u;
    ticks(2u);
    now += 500u; /* 1.5 ms after the last tick: not an overrun */
    ticks(1u);
    CHECK(hb.gap_overruns == 0u, "gap 1500 us: no overrun");
    now += 501u; /* 1.501 ms */
    ticks(1u);
    CHECK(hb.gap_overruns == 1u, "gap 1501 us: overrun");
    /* The first tick never counts. */
    hb_init(&hb);
    hb_tick(&hb, 999999u);
    CHECK(hb.gap_overruns == 0u, "first tick: no gap");

    /* 6. Interrupt time. */
    hb_init(&hb);
    hb_isr_done(&hb, 40u);
    hb_isr_done(&hb, 25u);
    CHECK(hb.isr_max_us == 40u && hb.isr_overruns == 0u, "isr max 40");
    hb_isr_done(&hb, 999u);
    CHECK(hb.isr_overruns == 0u, "isr 999 us: no overrun");
    hb_isr_done(&hb, 1000u);
    CHECK(hb.isr_overruns == 1u && hb.isr_max_us == 1000u, "isr 1000 us: overrun");

    /* 7. Order inside a tick: hooks run before the flags are posted, so a
     * main loop that sees the flag sees this tick's hook work done. */
    hb_init(&hb);
    now = 0u;
    probe_t c = {0, 0u, 0u, &hb};
    const int f = hb_add_flag(&hb, 10u, 0u);
    (void)hb_add_hook(&hb, 10u, 0u, hook, &c);
    hb_tick(&hb, now);
    CHECK(c.calls == 1 && hb_take(&hb, f) == 1u, "hook and flag in the same tick");

    if (failures) {
        printf("test_heartbeat: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_heartbeat: all tests passed\n");
    return 0;
}
