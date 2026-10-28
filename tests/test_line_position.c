/*
 * Host unit test for src/line_position.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_line_position.c src/line_position.c \
 *      -o build/test_line_position
 *   ./build/test_line_position
 */
#include <stdio.h>
#include <stdlib.h>

#include "line_position.h"

static int failures;

#define CHECK(cond, ...)                  \
    do {                                  \
        if (!(cond)) {                    \
            printf("FAIL line %d: ", __LINE__); \
            printf(__VA_ARGS__);          \
            printf("\n");                 \
            ++failures;                   \
        }                                 \
    } while (0)

/* Sensor model: each sensor has its own floor and line ADC values.
 * coverage 0-100 = how much of the sensor sees the line. */
typedef struct {
    uint16_t floor[3];
    uint16_t line[3];
} model_t;

static uint16_t reading(const model_t *m, unsigned int s, int coverage) {
    return (uint16_t)(m->floor[s] +
                      ((int)m->line[s] - (int)m->floor[s]) * coverage / 100);
}

static void read3(const model_t *m, int cl, int cc, int cr, uint16_t out[3]) {
    out[0] = reading(m, 0u, cl);
    out[1] = reading(m, 1u, cc);
    out[2] = reading(m, 2u, cr);
}

/* Sweep the line across the sensors: mostly floor, a short time on the
 * line for each sensor, as when the robot is moved across the line. */
static void sweep(lp_t *lp, const model_t *m) {
    uint16_t raw[3];
    lp_calib_begin(lp);
    for (int rep = 0; rep < 3; ++rep) {
        for (int i = 0; i < 300; ++i) { read3(m, 0, 0, 0, raw); lp_calib_add(lp, raw); }
        for (int i = 0; i < 30; ++i) { read3(m, 100, 0, 0, raw); lp_calib_add(lp, raw); }
        for (int i = 0; i < 30; ++i) { read3(m, 0, 100, 0, raw); lp_calib_add(lp, raw); }
        for (int i = 0; i < 30; ++i) { read3(m, 0, 0, 100, raw); lp_calib_add(lp, raw); }
    }
}

static vc_event_t step(lp_t *lp, const model_t *m, int cl, int cc, int cr,
                       uint32_t t) {
    uint16_t raw[3];
    read3(m, cl, cc, cr, raw);
    return lp_update(lp, raw, t);
}

#define TYPE(e) vc_event_type(e)

/* Per-mille coverage, for the small gap rises. */
static uint16_t reading_pm(const model_t *m, unsigned int s, int pm) {
    return (uint16_t)(m->floor[s] + ((int)m->line[s] - (int)m->floor[s]) * pm / 1000);
}

static vc_event_t step_pm(lp_t *lp, const model_t *m, int l, int c, int r, uint32_t t) {
    const uint16_t raw[3] = {reading_pm(m, 0u, l), reading_pm(m, 1u, c),
                             reading_pm(m, 2u, r)};
    return lp_update(lp, raw, t);
}

/* Line reads HIGH (typical IR module: black reflects less). Sensors differ. */
static const model_t HIGH_LINE = {{250, 400, 320}, {3000, 2600, 3300}};
/* Line reads LOW (the assumption in the fixed-threshold detector). */
static const model_t LOW_LINE = {{3100, 2900, 3200}, {300, 500, 450}};

static void test_polarity_and_position(const model_t *m, bool line_high,
                                       const char *name) {
    lp_t lp;
    unsigned int bad = 9u;
    lp_init(&lp);
    sweep(&lp, m);
    CHECK(lp_calib_end(&lp, &bad) == LP_CALIB_OK && bad == 0u, "%s: calib", name);
    CHECK(lp_line_reads_high(&lp) == line_high, "%s: polarity", name);
    for (unsigned int s = 0; s < 3u; ++s) {
        CHECK(lp_floor(&lp, s) == m->floor[s] && lp_line(&lp, s) == m->line[s],
              "%s: sensor %u ends %u/%u", name, s, lp_floor(&lp, s), lp_line(&lp, s));
    }

    struct { int l, c, r, pos; } cases[] = {
        {0, 100, 0, 0},     /* centred */
        {100, 0, 0, -100},  /* under the left sensor */
        {0, 0, 100, 100},   /* under the right sensor */
        {100, 100, 0, -50}, /* between left and centre */
        {0, 100, 100, 50},
        {50, 100, 0, -25},  /* side from L: 50 * ((1000 - C) + L) / 1000 */
        {0, 100, 25, 12},
    };
    uint32_t t = 0u;
    for (unsigned int i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        (void)step(&lp, m, cases[i].l, cases[i].c, cases[i].r, t++);
        CHECK(abs(lp.position - cases[i].pos) <= 1,
              "%s: case %u position %d, want %d", name, i, lp.position, cases[i].pos);
    }
    (void)step(&lp, m, 100, 100, 100, t++);
    CHECK(lp_junction(&lp), "%s: junction", name);
    (void)step(&lp, m, 0, 100, 0, t++);
    CHECK(!lp_junction(&lp), "%s: no junction", name);
}

int main(void) {
    test_polarity_and_position(&HIGH_LINE, true, "line reads high");
    test_polarity_and_position(&LOW_LINE, false, "line reads low");

    lp_t lp;
    unsigned int bad;
    uint16_t raw[3];

    /* Not calibrated: no events, even with no line. */
    lp_init(&lp);
    read3(&HIGH_LINE, 0, 0, 0, raw);
    CHECK(TYPE(lp_update(&lp, raw, 0u)) == VC_EVENT_NONE, "uncalibrated: event");
    CHECK(TYPE(lp_update(&lp, raw, 500u)) == VC_EVENT_NONE, "uncalibrated: event");
    CHECK(lp_calib_end(&lp, &bad) == LP_CALIB_NO_DATA, "no data");

    /* A sensor that never saw the line fails the sanity check. */
    lp_init(&lp);
    lp_calib_begin(&lp);
    for (int i = 0; i < 100; ++i) {
        read3(&HIGH_LINE, i % 10 == 0 ? 100 : 0, 0, i % 10 == 5 ? 100 : 0, raw);
        lp_calib_add(&lp, raw);
    }
    CHECK(lp_calib_end(&lp, &bad) == LP_CALIB_LOW_CONTRAST && bad == 0x2u,
          "centre never saw the line: bad mask %u", bad);
    CHECK(!lp_calibrated(&lp), "failed calibration must not enable updates");

    /* LINE_LOST after 50 ms, with the side; LINE_FOUND on return. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 100, 0, 0u)) == VC_EVENT_NONE, "on line");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 100, 40, 0, 10u)) == VC_EVENT_NONE, "drift left");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 20u)) == VC_EVENT_NONE, "gone: timing");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 69u)) == VC_EVENT_NONE, "49 ms: no event");
    {
        const vc_event_t e = step(&lp, &HIGH_LINE, 0, 0, 0, 70u);
        CHECK(TYPE(e) == VC_EVENT_LINE_LOST && vc_event_payload(e) == VC_LINE_SIDE_LEFT,
              "50 ms: LINE_LOST left (type %d side %u)", TYPE(e), vc_event_payload(e));
    }
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 200u)) == VC_EVENT_NONE, "lost once only");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 40, 0, 210u)) == VC_EVENT_NONE, "40%%: not yet found");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 60, 0, 220u)) == VC_EVENT_LINE_FOUND, "found");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 60, 0, 230u)) == VC_EVENT_NONE, "found once only");

    /* A short gap (< 50 ms) gives no event. */
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 300u)) == VC_EVENT_NONE, "gap start");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 340u)) == VC_EVENT_NONE, "gap 40 ms");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 100, 0, 345u)) == VC_EVENT_NONE, "gap ends");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 350u)) == VC_EVENT_NONE, "new gap");
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 399u)) == VC_EVENT_NONE, "timer restarted");

    /* Right side, and centred (unknown side). */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    (void)step(&lp, &HIGH_LINE, 0, 30, 100, 0u);
    (void)step(&lp, &HIGH_LINE, 0, 0, 0, 1u);
    CHECK(vc_event_payload(step(&lp, &HIGH_LINE, 0, 0, 0, 60u)) == VC_LINE_SIDE_RIGHT,
          "lost right");
    (void)step(&lp, &HIGH_LINE, 0, 100, 0, 100u);
    (void)step(&lp, &HIGH_LINE, 0, 0, 0, 101u);
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, 160u)) == VC_EVENT_NONE,
          "dark from the centre: no LINE_LOST at 50 ms (may be a gap)");
    {
        const vc_event_t e = step(&lp, &HIGH_LINE, 0, 0, 0, 101u + LP_GAP_LOST_MS);
        CHECK(TYPE(e) == VC_EVENT_LINE_LOST && vc_event_payload(e) == VC_LINE_SIDE_UNKNOWN,
              "lost while centred: unknown side at 300 ms");
    }

    /* Robot starts off the line: LINE_LOST (position 0, so after 300 ms). */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    (void)step(&lp, &HIGH_LINE, 0, 0, 0, 0u);
    CHECK(TYPE(step(&lp, &HIGH_LINE, 0, 0, 0, LP_GAP_LOST_MS)) == VC_EVENT_LINE_LOST,
          "start off the line");

    /* Slow light change: the floor gets 600 counts brighter over 10 s.
     * The normalised floor must stay low, and the line must stay found. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        model_t drift = HIGH_LINE;
        unsigned int worst = 0u;
        for (uint32_t t = 0u; t < 10000u; ++t) {
            for (unsigned int s = 0; s < 3u; ++s) {
                drift.floor[s] = (uint16_t)(HIGH_LINE.floor[s] + t * 600u / 10000u);
            }
            (void)lp_update(&lp, (uint16_t[3]){reading(&drift, 0u, 0),
                                               reading(&drift, 1u, 100),
                                               reading(&drift, 2u, 0)}, t);
            if (lp.norm[0] > worst) worst = lp.norm[0];
        }
        CHECK(worst < LP_PERMILLE(LP_ADAPT_FLOOR_PCT), "floor drift: left sensor reached %u", worst);
        CHECK(abs((int)lp_floor(&lp, 0u) - (HIGH_LINE.floor[0] + 600)) < 30,
              "floor learned: %u", lp_floor(&lp, 0u));
        CHECK(lp_line_seen(&lp) && lp.position == 0, "still tracking");
    }

    /* Learning never squeezes the range below the sanity limit. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    for (uint32_t t = 0u; t < 20000u; ++t) {
        /* Left sensor always reads 85 % of its current range: each time the
         * line end learns it, the range gets smaller. The guard must stop
         * this at the sanity limit. */
        const uint16_t f = lp_floor(&lp, 0u);
        const uint16_t squeeze = (uint16_t)(f + (lp_line(&lp, 0u) - f) * 85 / 100);
        (void)lp_update(&lp, (uint16_t[3]){squeeze,
                                           reading(&HIGH_LINE, 1u, 100),
                                           reading(&HIGH_LINE, 2u, 0)}, t);
    }
    CHECK(lp_line(&lp, 0u) - lp_floor(&lp, 0u) >= (int)LP_MIN_CONTRAST_ADC - 1,
          "range kept: %u", lp_line(&lp, 0u) - lp_floor(&lp, 0u));

    /* Readings are per mille: 0.5 % coverage is visible. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    (void)step_pm(&lp, &HIGH_LINE, 5, 1000, 0, 0u);
    CHECK(lp.norm[0] >= 4u && lp.norm[0] <= 5u, "0.5 %% reads %u per mille", lp.norm[0]);

    /* Gap, left: C fades, L rises 1 % above its baseline, R does not. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        CHECK(lp_centred(&lp), "gap: centred");
        (void)step_pm(&lp, &HIGH_LINE, 10, 600, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.fade_source == LP_SIDE_SOURCE_GAP,
              "gap: side %u source %u", lp.fade_side, lp.fade_source);
        CHECK(lp.position == -20, "gap: C 60 %% gives %d, want -20", lp.position);
        const uint32_t dark = t;
        CHECK(TYPE(step_pm(&lp, &HIGH_LINE, 10, 0, 0, t++)) == VC_EVENT_NONE, "gap: dark");
        CHECK(lp.position == -50, "gap: in the gap %d, want -50", lp.position);
        CHECK(TYPE(step_pm(&lp, &HIGH_LINE, 10, 0, 0, dark + 50u)) == VC_EVENT_NONE,
              "gap: no LINE_LOST at 50 ms");
        CHECK(TYPE(step_pm(&lp, &HIGH_LINE, 10, 0, 0, dark + LP_GAP_LOST_MS - 1u)) ==
                  VC_EVENT_NONE, "gap: no LINE_LOST before 300 ms");
        const vc_event_t e = step_pm(&lp, &HIGH_LINE, 10, 0, 0, dark + LP_GAP_LOST_MS);
        CHECK(TYPE(e) == VC_EVENT_LINE_LOST && vc_event_payload(e) == VC_LINE_SIDE_LEFT,
              "gap: LINE_LOST LEFT at 300 ms");
    }

    /* Gap, right, mirrored. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 0, 0, 10, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_RIGHT && lp.position == 50,
              "gap right: side %u position %d", lp.fade_side, lp.position);
    }

    /* Both outers rise the same (the floor changed): no side. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 10, 600, 10, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_UNKNOWN, "equal rise: side %u", lp.fade_side);
        CHECK(lp.position == 0, "equal rise: position %d", lp.position);
        /* Unknown side in the dark: LINE_LOST after 300 ms, unknown side. */
        (void)step_pm(&lp, &HIGH_LINE, 10, 0, 10, 100u);
        const vc_event_t e = step_pm(&lp, &HIGH_LINE, 10, 0, 10, 100u + LP_GAP_LOST_MS);
        CHECK(TYPE(e) == VC_EVENT_LINE_LOST && vc_event_payload(e) == VC_LINE_SIDE_UNKNOWN,
              "equal rise: LINE_LOST unknown at 300 ms");
    }

    /* The side can arrive late, in the dark: then the position follows. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 0, 0, 0, 100u); /* dark, no rise yet */
        CHECK(lp.position == 0 && lp.fade_side == VC_LINE_SIDE_UNKNOWN, "late: no side yet");
        CHECK(TYPE(step_pm(&lp, &HIGH_LINE, 0, 0, 0, 160u)) == VC_EVENT_NONE,
              "late: no LINE_LOST at 60 ms");
        (void)step_pm(&lp, &HIGH_LINE, 10, 0, 0, 200u); /* L rises later */
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.position == -50,
              "late: side %u position %d", lp.fade_side, lp.position);
    }

    /* The baseline needs LP_BASELINE_MIN centred updates first. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < LP_BASELINE_MIN - 1u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 10, 600, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_UNKNOWN, "baseline not ready: side %u",
              lp.fade_side);
    }

    /* Hint: used when no sensor shows the side, and kept while fading. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        lp_set_hint(&lp, VC_LINE_SIDE_LEFT);
        (void)step_pm(&lp, &HIGH_LINE, 0, 600, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.fade_source == LP_SIDE_SOURCE_HINT &&
                  lp.position == -20,
              "hint: side %u source %u position %d", lp.fade_side, lp.fade_source,
              lp.position);
        lp_set_hint(&lp, VC_LINE_SIDE_RIGHT); /* robot now turns back */
        (void)step_pm(&lp, &HIGH_LINE, 0, 500, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.position == -25,
              "hint: side kept while fading (%u, %d)", lp.fade_side, lp.position);
        (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_UNKNOWN && lp.position == 0,
              "hint: cleared when centred");
        /* Newer evidence wins over the hint. */
        (void)step_pm(&lp, &HIGH_LINE, 0, 600, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_RIGHT, "hint right: %u", lp.fade_side);
        (void)step_pm(&lp, &HIGH_LINE, 10, 500, 0, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.fade_source == LP_SIDE_SOURCE_GAP,
              "gap rise wins over hint: side %u source %u", lp.fade_side, lp.fade_source);
        (void)step_pm(&lp, &HIGH_LINE, 0, 300, 200, t++);
        CHECK(lp.fade_side == VC_LINE_SIDE_RIGHT && lp.fade_source == LP_SIDE_SOURCE_SENSOR &&
                  lp.position == 45,
              "sensor wins: side %u source %u position %d", lp.fade_side, lp.fade_source,
              lp.position);
    }

    /* Junction: a side arm near L after the junction must not give LEFT. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 600, 1000, 600, 100u); /* junction */
        CHECK(lp_junction(&lp), "junction seen");
        (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 200, 150u); /* line goes right */
        CHECK(lp.fade_side == VC_LINE_SIDE_RIGHT, "junction: right %u", lp.fade_side);
        (void)step_pm(&lp, &HIGH_LINE, 30, 0, 0, 250u); /* arm near L, all dark */
        CHECK(lp.fade_side == VC_LINE_SIDE_RIGHT && lp.position == 50,
              "junction hold: side %u position %d, want RIGHT +50", lp.fade_side,
              lp.position);
        /* After the hold the gap rise works again. */
        for (t = 400u; t < 420u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        (void)step_pm(&lp, &HIGH_LINE, 10, 600, 0, t);
        CHECK(lp.fade_side == VC_LINE_SIDE_LEFT && lp.fade_source == LP_SIDE_SOURCE_GAP,
              "after the hold: gap rise LEFT (side %u source %u)", lp.fade_side,
              lp.fade_source);
    }

    /* The baseline does not learn during the hold. */
    lp_init(&lp);
    sweep(&lp, &HIGH_LINE);
    (void)lp_calib_end(&lp, &bad);
    {
        uint32_t t = 0u;
        for (; t < 20u; ++t) (void)step_pm(&lp, &HIGH_LINE, 0, 1000, 0, t);
        const int32_t before = lp.baseline_q8[LP_LEFT];
        (void)step_pm(&lp, &HIGH_LINE, 600, 1000, 600, 100u);
        for (t = 101u; t < 150u; ++t) (void)step_pm(&lp, &HIGH_LINE, 40, 1000, 0, t);
        CHECK(lp.baseline_q8[LP_LEFT] == before, "hold: baseline kept");
    }

    /* Line word layout. */
    lp.position = -50;
    lp.flags = (uint16_t)(LP_FLAG_LINE_SEEN | (VC_LINE_SIDE_LEFT << LP_FLAG_SIDE_SHIFT));
    CHECK(lp_word(&lp) == (0xCEu | 0x0100u | (1u << 10)), "word 0x%04X", lp_word(&lp));

    if (failures == 0) {
        printf("test_line_position: all tests passed\n");
        return 0;
    }
    printf("test_line_position: %d failure(s)\n", failures);
    return 1;
}
