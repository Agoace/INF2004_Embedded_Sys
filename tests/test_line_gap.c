/* Host test for LINE_GAP=1 (pair-mode gap rule, line_gap.h). Fixtures are
 * real KR1 rows (school run B146, 2026-11-17, 20 ms apart; raw L, raw C,
 * logged position). lp_update() is the gap build; lp_update_base() is the
 * unchanged module, for comparison. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../app_program/subsystem3/line_position.h"

vc_event_t lp_update_base(lp_t *lp, const uint16_t raw[LP_SENSORS],
                          uint32_t now_ms);

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

typedef struct {
    uint16_t l, c;
} row_t;

/* B146 learned ends at run end: L 208/3008, C 163/3843. */
static void start(lp_t *lp, uint16_t l, uint16_t c)
{
    lp_init(lp);
    lp_set_pair(lp, true, 0);
    lp_calib_begin(lp);
    for (int i = 0; i < 20; ++i) {
        const uint16_t f[LP_SENSORS] = {208u, 163u, 0u};
        lp_calib_add(lp, f);
    }
    const uint16_t hi[LP_SENSORS] = {3008u, 3843u, 0u};
    lp_calib_add(lp, hi);
    unsigned int bad;
    CHECK(lp_calib_end(lp, &bad) == LP_CALIB_OK, "calibration");
    const uint16_t r[LP_SENSORS] = {l, c, 0u};
    (void)lp_update(lp, r, 0u);
}

typedef vc_event_t (*update_fn)(lp_t *, const uint16_t *, uint32_t);

/* Feed rows at 1 kHz, blended linearly between rows (20 samples each).
 * pos[] gets the position at each row; returns the first LOST time or 0. */
static uint32_t feed(lp_t *lp, update_fn fn, const row_t *rows, int n,
                     uint32_t *now, int pos[])
{
    uint32_t lost = 0u;
    for (int k = 1; k < n; ++k) {
        for (int s = 1; s <= 20; ++s) {
            const int l = rows[k - 1].l + (rows[k].l - rows[k - 1].l) * s / 20;
            const int c = rows[k - 1].c + (rows[k].c - rows[k - 1].c) * s / 20;
            const uint16_t r[LP_SENSORS] = {(uint16_t)l, (uint16_t)c, 0u};
            ++*now;
            const vc_event_t e = fn(lp, r, *now);
            if (vc_event_type(e) == VC_EVENT_LINE_LOST && lost == 0u) {
                lost = *now;
            }
        }
        pos[k] = lp->position;
    }
    return lost;
}

static vc_event_t base_fn(lp_t *lp, const uint16_t *r, uint32_t t)
{
    return lp_update_base(lp, r, t);
}

static vc_event_t gap_fn(lp_t *lp, const uint16_t *r, uint32_t t)
{
    return lp_update(lp, r, t);
}

/* Max outward step: for a line moving toward +, the lowest position after
 * the fade starts, below the position where it started. */
static void crossing(const char *name, const row_t *rows, int n, int dir)
{
    lp_t g, b;
    int pg[16] = {0}, pb[16] = {0};
    uint32_t tg = 0u, tb = 0u;
    start(&g, rows[0].l, rows[0].c);
    start(&b, rows[0].l, rows[0].c);
    (void)feed(&g, gap_fn, rows, n, &tg, pg);
    (void)feed(&b, base_fn, rows, n, &tb, pb);
    int worst_g = 0, worst_b = 0;
    for (int k = 2; k < n; ++k) {
        /* Gap phase: the sensor the line moves to reads >= 1.5 %. */
        const int other = dir > 0 ? (rows[k].c - 163) * 1000 / 3680
                                  : (rows[k].l - 208) * 1000 / 2800;
        if (other < 15) {
            continue;
        }
        /* dir +1: positions must not fall (L->C, line moves right). */
        const int step_g = dir * (pg[k] - pg[k - 1]);
        const int step_b = dir * (pb[k] - pb[k - 1]);
        if (step_g < worst_g) {
            worst_g = step_g;
        }
        if (step_b < worst_b) {
            worst_b = step_b;
        }
    }
    printf("%s: base", name);
    for (int k = 1; k < n; ++k) {
        printf(" %d", pb[k]);
    }
    printf(" | gap");
    for (int k = 1; k < n; ++k) {
        printf(" %d", pg[k]);
    }
    printf("\n");
    /* Sign: the gap build never steps outward by more than noise. */
    CHECK(worst_g >= -3, "%s: outward step %d", name, worst_g);
    CHECK(worst_b < -3, "%s: fixture must show the base defect (%d)", name,
          worst_b);
    CHECK(dir * pg[n - 1] > 0, "%s: ends on the new side (%d)", name,
          pg[n - 1]);
}

int main(void)
{
    /* (1) L->C, t=3180..3360 and t=3640..3800 (line moves right). */
    static const row_t a[] = {
        {2405, 189}, {2391, 189}, {2308, 190}, {2148, 201}, {1933, 208},
        {1505, 237}, {744, 617},  {237, 1516}, {200, 1687}, {193, 1664}};
    crossing("L->C t3180", a, 10, +1);
    static const row_t a2[] = {{2317, 180}, {2110, 189}, {1773, 210},
                               {1242, 275}, {410, 1531}, {214, 2023},
                               {193, 2011}};
    crossing("L->C t3660", a2, 7, +1);
    /* (2) C->L, t=54760..54920 and t=66540..66700 (line moves left). */
    static const row_t c[] = {{223, 1716}, {230, 2121},  {246, 1670},
                              {424, 670},  {1648, 1212}, {2501, 269},
                              {2728, 222}, {2756, 232}};
    crossing("C->L t54780", c, 8, -1);
    static const row_t c2[] = {{267, 1869}, {248, 1886}, {261, 1585},
                               {381, 994},  {1475, 223}, {2381, 168},
                               {2110, 153}, {2181, 156}};
    crossing("C->L t66560", c2, 8, -1);

    /* (3) Curve exit, t=13660..13740 then dark: L stays at its floor.
     * Position, flags and LOST time identical to the base module. */
    {
        static const row_t e[] = {{213, 1552}, {207, 1379}, {193, 326},
                                  {194, 215},  {190, 199},  {190, 199},
                                  {190, 199},  {190, 199},  {190, 199},
                                  {190, 199},  {190, 199}};
        lp_t g, b;
        int pg[16], pb[16];
        uint32_t tg = 0u, tb = 0u;
        start(&g, e[0].l, e[0].c);
        start(&b, e[0].l, e[0].c);
        const uint32_t lg = feed(&g, gap_fn, e, 11, &tg, pg);
        const uint32_t lb = feed(&b, base_fn, e, 11, &tb, pb);
        CHECK(memcmp(pg + 1, pb + 1, 10 * sizeof(int)) == 0, "exit: positions");
        CHECK(g.flags == b.flags && lg == lb && lg != 0u,
              "exit: flags %04x/%04x LOST %u/%u", g.flags, b.flags, lg, lb);
        /* Dark from about 22 ms: LOST at the 50 ms limit, not 300 ms. */
        CHECK(lb > 50u && lb < 100u, "exit: LOST at 50 ms limit (%u)", lb);
    }

    /* (4) Both sensors weakly on (gap) for 400 ms: LOST at 300 ms, not
     * an endless hold; the base gives LOST at 50 ms (outward memory). */
    {
        /* L fades with C at its floor (dark, outward memory), then C
         * rises to about 11 % while L stays at about 18 %. */
        static const row_t h[] = {
            {2400, 190}, {1300, 200}, {800, 200}, {700, 560}};
        lp_t g, b;
        int pg[8], pb[8];
        uint32_t tg = 0u, tb = 0u;
        start(&g, h[0].l, h[0].c);
        start(&b, h[0].l, h[0].c);
        (void)feed(&g, gap_fn, h, 4, &tg, pg);
        (void)feed(&b, base_fn, h, 4, &tb, pb);
        uint32_t lg = 0u, lb = 0u;
        const uint16_t r[LP_SENSORS] = {700u, 560u, 0u};
        for (uint32_t t = 0u; t < 400u; ++t) {
            ++tg;
            ++tb;
            if (lg == 0u &&
                vc_event_type(lp_update(&g, r, tg)) == VC_EVENT_LINE_LOST) {
                lg = t;
            }
            if (lb == 0u && vc_event_type(lp_update_base(&b, r, tb)) ==
                                VC_EVENT_LINE_LOST) {
                lb = t;
            }
        }
        CHECK(lg != 0u && lg >= 250u && lg <= 300u, "gap: LOST at %u ms", lg);
        CHECK(lb != 0u && lb < 60u, "base: LOST at %u ms", lb);
        CHECK(g.position > -10 && g.position < 10, "gap: held %d", g.position);
    }

    /* (5) No step when one sensor crosses LP_LOST_PCT (gap entry/exit):
     * C fixed at 25 % (no end learning between 20 and 80 %), L from 40 %
     * down to 20 % and back. */
    {
        lp_t g;
        start(&g, 2000u, 163u);
        int prev = 1000, worst = 0;
        uint32_t t = 1u;
        for (int pass = 0; pass < 2; ++pass) {
            for (int k = 0; k <= 200; ++k) {
                const int pct10 = pass == 0 ? 400 - k : 200 + k; /* 0.1 % */
                const uint16_t r[LP_SENSORS] = {
                    (uint16_t)(208 + 2800 * pct10 / 1000),
                    (uint16_t)(163 + 920), 0u};
                (void)lp_update(&g, r, ++t);
                if (prev != 1000) {
                    const int d = abs(g.position - prev);
                    if (d > worst) {
                        worst = d;
                    }
                }
                prev = g.position;
            }
        }
        CHECK(worst <= 1, "continuity: step %d at the 30 %% boundary", worst);
    }

    /* (6) Random pair inputs with the other sensor at its floor while one
     * sensor alone sees the line: identical to the base module. */
    {
        lp_t g, b;
        start(&g, 1500u, 163u);
        start(&b, 1500u, 163u);
        uint32_t seed = 7u;
        int diff = 0;
        for (uint32_t t = 1u; t < 20000u; ++t) {
            seed = seed * 1103515245u + 12345u;
            const int side = (int)((t / 500u) % 2u);
            const uint16_t on = (uint16_t)(200u + (seed >> 8) % 3000u);
            const uint16_t r[LP_SENSORS] = {side ? 208u : on, side ? on : 163u,
                                            0u};
            const vc_event_t eg = lp_update(&g, r, t);
            const vc_event_t eb = lp_update_base(&b, r, t);
            diff += eg != eb || g.position != b.position || g.flags != b.flags;
        }
        CHECK(diff == 0, "floor other: %d differing samples", diff);
    }

    if (failures != 0) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("test_line_gap: ok\n");
    return 0;
}
