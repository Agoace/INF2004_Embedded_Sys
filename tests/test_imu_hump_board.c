/* CR1 hump board: the packed 16 B board gives the same summary v3 values
 * and the same live HUMP event as the float report, for every input.
 * ref_summarize() and ref_live() are verbatim copies of the code before
 * the board (imu_hump.c imu_hump_summarize, vehicle_task.c hump_poll). */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/platform/app_board.h"
#include "../app_program/subsystem4/imu_hump.h"
#include "../app_program/subsystem4/imu_settings.h"

_Static_assert(sizeof(imu_hump_board_t) == 16u, "hump board is 16 B");
_Static_assert(APP_BOARD_WORDS(imu_hump_board_t) == 4u, "4 board words");

static imu_hump_summary_t ref_summarize(const imu_hump_report_t *r, bool have,
                                        uint32_t run_epoch, uint32_t now_ms,
                                        uint32_t max_age_ms)
{
    imu_hump_summary_t out = {
        .peak_01mm = (uint16_t)IMU_HUMP_PEAK_NONE,
        .count = 0u,
        .flags = IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_FAULT,
        .gated = 0u};

    if (r == NULL || !have || (run_epoch & 1u) == 0u ||
        r->hump.run_epoch != run_epoch ||
        (uint32_t)(now_ms - r->timestamp_ms) > max_age_ms) {
        return out;
    }
    const imu_hump_output_t *o = &r->hump;
    float peak = o->max_peak_mm;
    bool left = o->max_left;
    uint16_t count = o->count;
    uint8_t flags = (uint8_t)(o->flags | IMU_HUMP_FLAG_IMPLEMENTED);

    if (o->running && o->state == (uint8_t)IMU_HUMP_ACTIVE) {
        if (count == 0u || o->event_peak_mm > peak) {
            peak = o->event_peak_mm;
            left = o->event_left;
        }
        if (count != UINT16_MAX) {
            ++count;
        }
        flags |= IMU_HUMP_FLAG_OPEN;
    }
    flags = (uint8_t)((flags & (uint8_t)~IMU_HUMP_FLAG_LEFT) |
                      (count != 0u && left ? IMU_HUMP_FLAG_LEFT : 0u));
    out.gated = o->gated_starts > IMU_HUMP_GATED_MAX
                    ? (uint8_t)IMU_HUMP_GATED_MAX : (uint8_t)o->gated_starts;
    if (o->valid_samples == 0u) {
        out.flags = (uint8_t)((flags | IMU_HUMP_FLAG_FAULT) &
                              IMU_HUMP_FLAGS_MASK);
        return out;
    }
    out.count = count;
    out.flags = (uint8_t)(flags & IMU_HUMP_FLAGS_MASK);
    out.peak_01mm = count == 0u ? 0u : imu_hump_tenths(peak);
    return out;
}

typedef struct {
    bool emit;
    uint16_t emitted;
    uint16_t word;
    uint8_t detail;
} live_t;

/* hump_poll() body before the board. */
static live_t ref_live(const imu_hump_report_t *copy, bool have,
                       uint32_t epoch, uint16_t emitted, uint32_t now)
{
    live_t out = {.emitted = emitted};
    const imu_hump_output_t *h = &copy->hump;
    if (!have || h->run_epoch != epoch || h->count == emitted ||
        now - copy->timestamp_ms > IMU_HUMP_MAX_AGE_MS) {
        return out;
    }
    out.emit = true;
    out.emitted = h->count;
    out.word = imu_hump_tenths(h->last_peak_mm);
    out.detail = imu_hump_detail(h->count, h->last_left);
    return out;
}

/* hump_poll() body after the board (same lines, board accessors). */
static live_t board_live(const imu_hump_board_t *copy, bool have,
                         uint32_t epoch, uint16_t emitted, uint32_t now)
{
    live_t out = {.emitted = emitted};
    if (!have || imu_hump_board_epoch(copy) != epoch ||
        imu_hump_board_count(copy) == emitted ||
        now - imu_hump_board_time(copy) > IMU_HUMP_MAX_AGE_MS) {
        return out;
    }
    out.emit = true;
    out.emitted = imu_hump_board_count(copy);
    out.word = imu_hump_board_last_01mm(copy);
    out.detail = imu_hump_detail(imu_hump_board_count(copy),
                                 imu_hump_board_last_left(copy));
    return out;
}

static unsigned long checks;

static void same(const imu_hump_report_t *r, bool have, uint32_t epoch,
                 uint32_t now, uint16_t emitted)
{
    imu_hump_board_t b;
    memset(&b, 0xa5, sizeof(b));
    imu_hump_pack(&b, &r->hump, r->timestamp_ms);
    const imu_hump_summary_t want = ref_summarize(r, have, epoch, now, 50u);
    const imu_hump_summary_t got =
        imu_hump_board_summarize(&b, have, epoch, now, 50u);
    const imu_hump_summary_t wrap =
        imu_hump_summarize(r, have, epoch, now, 50u);
    if (want.peak_01mm != got.peak_01mm || want.count != got.count ||
        want.flags != got.flags || want.gated != got.gated ||
        want.peak_01mm != wrap.peak_01mm || want.count != wrap.count ||
        want.flags != wrap.flags || want.gated != wrap.gated) {
        fprintf(stderr, "summary differs: want %u %u %02x %u got %u %u "
                "%02x %u\n", want.peak_01mm, want.count, want.flags,
                want.gated, got.peak_01mm, got.count, got.flags, got.gated);
        assert(0);
    }
    const live_t lw = ref_live(r, have, epoch, emitted, now);
    const live_t lg = board_live(&b, have, epoch, emitted, now);
    assert(lw.emit == lg.emit && lw.emitted == lg.emitted);
    if (lw.emit) {
        assert(lw.word == lg.word && lw.detail == lg.detail);
    }
    assert(imu_hump_board_time(&b) == r->timestamp_ms);
    assert(imu_hump_board_epoch(&b) == r->hump.run_epoch);
    ++checks;
}

static uint32_t seed = 0x2545F491u;
static uint32_t rnd(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

static float pick_float(float other)
{
    static const float fixed[] = {
        0.0f, -0.0f, 0.04999f, 0.05f, 0.05001f, 0.15f, 3.0f, 10.849999f,
        10.85f, 10.850001f, 12.7f, 13.4f, 111.0f, 6553.39f, 6553.4f,
        6553.45f, 6553.5f, 7000.0f, 1.0e9f, -1.0f, -10.0f};
    switch (rnd() % 8u) {
    case 0: return fixed[rnd() % (sizeof(fixed) / sizeof(fixed[0]))];
    case 1: return NAN;
    case 2: return (rnd() & 1u) != 0u ? INFINITY : -INFINITY;
    case 3: return nextafterf(other, INFINITY);   /* tie inside 0.1 mm */
    case 4: return nextafterf(other, -INFINITY);
    case 5: return other;
    default: return (float)(rnd() % 1300000u) / 10000.0f;  /* 0..130 mm */
    }
}

static uint16_t pick_u16(void)
{
    static const uint16_t fixed[] = {0u, 1u, 2u, 127u, 128u, 254u, 255u,
                                     256u, 65534u, 65535u};
    return (rnd() & 1u) != 0u ? fixed[rnd() % 10u] : (uint16_t)rnd();
}

static imu_hump_report_t random_report(void)
{
    imu_hump_report_t r;
    memset(&r, 0, sizeof(r));
    imu_hump_output_t *o = &r.hump;
    static const uint32_t epochs[] = {0u, 1u, 2u, 3u, 7u, UINT32_MAX};
    o->run_epoch = (rnd() & 1u) != 0u ? epochs[rnd() % 6u] : rnd();
    static const uint32_t valid[] = {0u, 1u, 2u, UINT32_MAX};
    o->valid_samples = (rnd() & 1u) != 0u ? valid[rnd() % 4u] : rnd();
    o->smoothed_mm = pick_float(1.0f);
    o->max_peak_mm = pick_float(10.0f);
    o->last_peak_mm = pick_float(o->max_peak_mm);
    o->event_peak_mm = pick_float(o->max_peak_mm);
    o->event_travel_mm = pick_float(30.0f);
    o->count = pick_u16();
    o->gated_starts = pick_u16();
    o->long_rejects = pick_u16();
    o->flags = (rnd() & 1u) != 0u ? (uint8_t)(rnd() & IMU_HUMP_FLAGS_MASK)
                                  : (uint8_t)rnd();
    o->state = (uint8_t)(rnd() % 6u);
    o->running = (rnd() & 1u) != 0u;
    o->event_left = (rnd() & 1u) != 0u;
    o->last_left = (rnd() & 1u) != 0u;
    o->max_left = (rnd() & 1u) != 0u;
    o->smooth_ready = (rnd() & 1u) != 0u;
    r.timestamp_ms = (rnd() & 3u) == 0u ? UINT32_MAX - (rnd() % 40u) : rnd();
    return r;
}

static void random_cases(unsigned int n)
{
    for (unsigned int i = 0u; i < n; ++i) {
        const imu_hump_report_t r = random_report();
        const uint32_t ages[] = {0u, 1u, 50u, 51u, 100u, UINT32_MAX};
        const uint32_t now = r.timestamp_ms + ((rnd() & 1u) != 0u
                                 ? ages[rnd() % 6u] : rnd());
        const uint32_t epoch = (rnd() % 4u) != 0u ? r.hump.run_epoch
                                                  : rnd();
        const uint16_t emitted = (rnd() & 1u) != 0u ? r.hump.count
                                                    : pick_u16();
        same(&r, (rnd() % 8u) != 0u, epoch, now, emitted);
        same(&r, true, r.hump.run_epoch, r.timestamp_ms, emitted);
    }
}

static imu_hump_report_t open_run(void)
{
    imu_hump_report_t r = {.timestamp_ms = 1000u};
    r.hump = (imu_hump_output_t){.run_epoch = 5u, .valid_samples = 10u,
        .running = true, .state = (uint8_t)IMU_HUMP_ACTIVE,
        .flags = IMU_HUMP_FLAG_IMPLEMENTED, .count = 1u,
        .max_peak_mm = 10.84f, .max_left = false,
        .event_peak_mm = nextafterf(10.84f, INFINITY), .event_left = true,
        .last_peak_mm = 10.84f, .gated_starts = 300u};
    return r;
}

static void edge_cases(void)
{
    /* Tie inside 0.1 mm: only the float compare picks LEFT. */
    imu_hump_report_t r = open_run();
    imu_hump_board_t b;
    imu_hump_pack(&b, &r.hump, r.timestamp_ms);
    imu_hump_summary_t s = imu_hump_board_summarize(&b, true, 5u, 1000u, 50u);
    assert(s.peak_01mm == 108u && s.count == 2u && s.gated == 255u);
    assert(s.flags == (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_OPEN |
                       IMU_HUMP_FLAG_LEFT));
    same(&r, true, 5u, 1000u, 0u);
    r.hump.event_peak_mm = 10.84f;   /* equal: the closed peak stays */
    same(&r, true, 5u, 1000u, 0u);
    /* OPEN already in the flags after the run ended: count unchanged. */
    r = open_run();
    r.hump.running = false;
    r.hump.flags |= IMU_HUMP_FLAG_OPEN;
    same(&r, true, 5u, 1000u, 0u);
    imu_hump_pack(&b, &r.hump, r.timestamp_ms);
    s = imu_hump_board_summarize(&b, true, 5u, 1000u, 50u);
    assert(s.count == 1u && (s.flags & IMU_HUMP_FLAG_OPEN) != 0u);
    /* Saturated count and an open event. */
    r = open_run();
    r.hump.count = UINT16_MAX;
    same(&r, true, 5u, 1000u, 0u);
    /* Open event with no closed event. */
    r = open_run();
    r.hump.count = 0u;
    r.hump.max_peak_mm = 50.0f;
    same(&r, true, 5u, 1000u, 0u);
    /* No valid sample: FAULT and no value, gated still reported. */
    r = open_run();
    r.hump.valid_samples = 0u;
    same(&r, true, 5u, 1000u, 0u);
    /* Saturation, NaN, negative peaks. */
    const float peaks[] = {6553.4f, 6553.45f, 6553.5f, 1.0e9f, NAN, INFINITY,
                           -INFINITY, -3.0f, 0.0f, 0.05f};
    for (unsigned int i = 0u; i < 10u; ++i) {
        for (unsigned int j = 0u; j < 10u; ++j) {
            r = open_run();
            r.hump.max_peak_mm = peaks[i];
            r.hump.event_peak_mm = peaks[j];
            r.hump.last_peak_mm = peaks[j];
            same(&r, true, 5u, 1000u, 0u);
            r.hump.running = false;
            same(&r, true, 5u, 1000u, 0u);
        }
    }
    /* Freshness and run rules: modular age, even, other, absent. */
    r = open_run();
    r.timestamp_ms = UINT32_MAX - 10u;
    same(&r, true, 5u, 39u, 0u);       /* age 50 across the wrap */
    same(&r, true, 5u, 40u, 0u);       /* age 51 */
    same(&r, true, 4u, UINT32_MAX, 0u);
    same(&r, true, 7u, UINT32_MAX, 0u);
    same(&r, false, 5u, UINT32_MAX, 0u);
    assert(imu_hump_board_summarize(NULL, true, 5u, 0u, 50u).peak_01mm ==
           IMU_HUMP_PEAK_NONE);
    /* Every flag value. */
    for (unsigned int f = 0u; f < 256u; ++f) {
        r = open_run();
        r.hump.flags = (uint8_t)f;
        same(&r, true, 5u, 1000u, 0u);
        r.hump.running = false;
        same(&r, true, 5u, 1000u, 0u);
    }
    /* Every gated value. */
    for (unsigned int g = 0u; g <= UINT16_MAX; g += 7u) {
        r = open_run();
        r.hump.gated_starts = (uint16_t)g;
        same(&r, true, 5u, 1000u, 0u);
    }
    /* Live events: count change, same count, LEFT detail, saturation. */
    r = open_run();
    r.hump.count = 3u;
    r.hump.last_left = true;
    same(&r, true, 5u, 1000u, 2u);
    same(&r, true, 5u, 1000u, 3u);
    r.hump.count = 200u;
    same(&r, true, 5u, 1000u, 2u);
}

/* The 0.1 mm peak grid: every value 0..6553.4 mm packs to the same u16. */
static void tenths_grid(void)
{
    for (uint32_t t = 0u; t <= 65534u; ++t) {
        const float mm = (float)t / 10.0f;
        imu_hump_report_t r = open_run();
        r.hump.running = false;
        r.hump.max_peak_mm = mm;
        r.hump.last_peak_mm = mm;
        imu_hump_board_t b;
        imu_hump_pack(&b, &r.hump, r.timestamp_ms);
        const imu_hump_summary_t s =
            imu_hump_board_summarize(&b, true, 5u, 1000u, 50u);
        assert(s.peak_01mm == imu_hump_tenths(mm));
        assert(imu_hump_board_last_01mm(&b) == imu_hump_tenths(mm));
    }
}

int main(void)
{
    edge_cases();
    tenths_grid();
    random_cases(200000u);
    printf("hump board: %lu summary/live cases identical\n", checks);
    return 0;
}
