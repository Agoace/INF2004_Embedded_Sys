/* Synthetic hump-event fixtures, NOT hardware accuracy evidence. The rig
 * applies the 100 ms roll LPF that imu_core applies before the detector. */
#include "../app_program/subsystem4/imu_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    imu_hump_t h;
    imu_hump_input_t in;
    float roll;
    float x;            /* Travel since rig start, mm. */
    uint32_t seed;
} rig_t;

static void rig(rig_t *r, uint32_t epoch)
{
    *r = (rig_t){.in = {.speed_mm_s = 146.0f, .run_epoch = epoch,
                        .valid = true, .motion_valid = true},
                 .seed = 12345u};
    imu_hump_init(&r->h);
}

/* One 10 ms observation of a true right-wheel height h at speed v. */
static void feed(rig_t *r, float h, float v)
{
    const float alpha = 10.0f / (IMU_FILTER_TAU_MS + 10.0f);

    r->roll += alpha * (asinf(h / IMU_TRACK_MM) - r->roll);
    r->in.height_mm = IMU_TRACK_MM * sinf(r->roll);
    r->in.speed_mm_s = v;
    r->in.travel_mm = v * 0.01f;
    r->x += fabsf(v) * 0.01f;
    imu_hump_step(&r->h, &r->in);
}

static void flat(rig_t *r, float mm, float v)
{
    const float end = r->x + mm;

    while (r->x < end) {
        feed(r, 0.0f, v);
    }
}

/* Measured bring-up hump: 95 mm ramps and a 5 mm crest (195 mm long). */
static float profile(float x, float peak, float crest)
{
    const float up = x < 0.0f ? 0.0f : x < 95.0f ? x / 95.0f :
        x <= 95.0f + crest ? 1.0f :
        fmaxf(0.0f, (190.0f + crest - x) / 95.0f);

    return peak * up; /* A negative peak is a LEFT-wheel hump. */
}

static void hump(rig_t *r, float peak, float crest, float v)
{
    const float start = r->x;

    while (r->x - start < 190.0f + crest) {
        feed(r, profile(r->x - start, peak, crest), v);
    }
}

static imu_hump_summary_t summary(const rig_t *r, uint32_t now)
{
    const imu_hump_report_t report = {.hump = r->h.output,
                                      .timestamp_ms = 1000u};

    return imu_hump_summarize(&report, true, r->in.run_epoch, now, 50u);
}

static void helpers(void)
{
    imu_hump_t h;
    const imu_hump_input_t in = {0};

    imu_hump_init(NULL);
    imu_hump_step(NULL, &in);
    imu_hump_init(&h);
    imu_hump_step(&h, NULL);
    assert(h.output.state == IMU_HUMP_IDLE && !h.output.running);
    assert(imu_hump_tenths(10.84f) == 108u && imu_hump_tenths(10.86f) == 109u);
    assert(imu_hump_tenths(NAN) == 0u && imu_hump_tenths(-1.0f) == 0u);
    assert(imu_hump_tenths(INFINITY) == 0u && imu_hump_tenths(0.0f) == 0u);
    assert(imu_hump_tenths(7000.0f) == IMU_HUMP_PEAK_MAX);
    assert(imu_hump_detail(1u, false) == 1u);
    assert(imu_hump_detail(0u, true) == 0x80u);
    assert(imu_hump_detail(127u, false) == 127u);
    assert(imu_hump_detail(200u, true) == 0xFFu);
}

/* The bring-up hump at line speeds: one RIGHT event, peak = 100 ms mean. */
static void speeds(void)
{
    static const float speed[] = {100.0f, 146.0f, 300.0f, 460.0f};
    static const float floor_mm[] = {9.0f, 8.8f, 7.5f, 6.0f};

    for (unsigned int i = 0u; i < 4u; ++i) {
        rig_t r;

        rig(&r, 1u);
        flat(&r, 300.0f, speed[i]);
        hump(&r, 10.0f, 5.0f, speed[i]);
        flat(&r, 300.0f, speed[i]);
        const imu_hump_output_t *o = &r.h.output;

        printf("fixture hump10 v%.0f: peak %.3f mm\n", (double)speed[i],
               (double)o->max_peak_mm);
        assert(o->count == 1u && o->state == IMU_HUMP_IDLE);
        assert(o->max_peak_mm > floor_mm[i] && o->max_peak_mm <= 10.0f);
        assert(!o->max_left && o->flags == IMU_HUMP_FLAG_IMPLEMENTED);
        assert(o->gated_starts == 0u && o->long_rejects == 0u);
        const imu_hump_summary_t s = summary(&r, 1000u);

        assert(s.count == 1u && s.flags == IMU_HUMP_FLAG_IMPLEMENTED);
        assert(s.peak_01mm == imu_hump_tenths(o->max_peak_mm));
    }
}

/* Highest of several heights; a LEFT event is a flagged bonus. */
static void heights_and_side(void)
{
    rig_t r;

    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 5.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 20.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 3u && r.h.output.max_peak_mm > 18.0f);
    assert(r.h.output.last_peak_mm < 10.0f && !r.h.output.max_left);
    hump(&r, -25.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 4u && r.h.output.max_left);
    assert(r.h.output.last_left && r.h.output.max_peak_mm > 22.0f);
    const imu_hump_summary_t s = summary(&r, 1000u);

    assert(s.flags == (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_LEFT));
    assert(s.count == 4u);
}

/* Step 2d flat floor: sd 0.68 mm; a 2 mm sd stress case; single spikes. */
static void flat_noise(void)
{
    static const float sd[] = {0.68f, 2.0f};

    for (unsigned int k = 0u; k < 2u; ++k) {
        rig_t r;
        float max_mean = 0.0f;

        rig(&r, 1u);
        for (unsigned int i = 0u; i < 3000u; ++i) {
            r.seed = r.seed * 1664525u + 1013904223u;
            const float noise = ((float)(r.seed >> 8) / 16777215.0f - 0.5f) *
                                3.46410162f * sd[k];

            /* Noise is AFTER the LPF, like the step 2d measurement. */
            r.in.height_mm = noise + (i % 500u == 0u ? 12.0f : 0.0f);
            r.in.travel_mm = 1.46f;
            imu_hump_step(&r.h, &r.in);
            max_mean = fmaxf(max_mean, fabsf(r.h.output.smoothed_mm));
        }
        printf("fixture flat sd%.2f: mean max %.3f mm\n", (double)sd[k],
               (double)max_mean);
        assert(r.h.output.count == 0u && r.h.output.state == IMU_HUMP_IDLE);
        assert(max_mean < IMU_HUMP_ON_MM);
        assert(summary(&r, 1000u).peak_01mm == 0u);
    }
}

static void chatter_and_spacing(void)
{
    rig_t r;

    /* Above ON for less than ON_TRAVEL: discarded, nothing counted. */
    rig(&r, 1u);
    flat(&r, 200.0f, 146.0f);
    for (unsigned int i = 0u; i < 15u; ++i) {
        r.in.height_mm = 4.0f;
        r.in.travel_mm = 1.46f;
        imu_hump_step(&r.h, &r.in);
    }
    assert(r.h.output.state == IMU_HUMP_CANDIDATE);
    for (unsigned int i = 0u; i < 3u; ++i) {
        r.in.height_mm = -14.0f;
        imu_hump_step(&r.h, &r.in);
    }
    assert(r.h.output.state == IMU_HUMP_IDLE && r.h.output.count == 0u);
    /* Confirmed, then dips between OFF and ON: still the same event. */
    for (unsigned int i = 0u; i < 300u; ++i) {
        r.in.height_mm = (i / 30u) % 2u == 0u ? 3.6f : 1.8f;
        imu_hump_step(&r.h, &r.in);
    }
    assert(r.h.output.state == IMU_HUMP_ACTIVE && r.h.output.count == 0u);
    for (unsigned int i = 0u; i < 40u; ++i) {
        r.in.height_mm = 0.0f;
        imu_hump_step(&r.h, &r.in);
    }
    assert(r.h.output.count == 1u && r.h.output.state == IMU_HUMP_IDLE);
    assert(r.h.output.flags == IMU_HUMP_FLAG_IMPLEMENTED);

    /* Touching humps merge. A 20 mm flat gap separates them: the ramps
     * below OFF add about 28 mm to the gap. 300 mm apart count twice. */
    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    hump(&r, 8.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 1u);
    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 20.0f, 146.0f);
    hump(&r, 8.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 2u && r.h.output.last_peak_mm > 7.0f);
    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 6.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 2u && r.h.output.max_peak_mm > 8.8f);
    assert(r.h.output.last_peak_mm == r.h.output.max_peak_mm);
}

/* An 800 mm 5 mm slope is not a hump: rejected once, then idle. */
static void long_slope(void)
{
    rig_t r;

    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    for (float x = 0.0f; x < 800.0f; x += 1.46f) {
        feed(&r, 5.0f, 146.0f);
    }
    assert(r.h.output.state == IMU_HUMP_LONG);
    assert(r.h.output.long_rejects == 1u && r.h.output.count == 0u);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.state == IMU_HUMP_IDLE && r.h.output.count == 0u);
    const imu_hump_summary_t s = summary(&r, 1000u);

    assert(s.peak_01mm == 0u && s.count == 0u);
    assert(s.flags == (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_LONG));
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 1u && r.h.output.long_rejects == 1u);
}

static void gates(void)
{
    rig_t r;

    /* 25 mg curve is below the gate; 60 mg and a SEARCH spin block. */
    rig(&r, 1u);
    r.in.lateral_mg = -25.0f;
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 1u && r.h.output.gated_starts == 0u);
    rig(&r, 1u);
    r.in.lateral_mg = 60.0f;
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 0u && r.h.output.gated_starts == 1u);
    rig(&r, 1u);
    r.in.yaw_rate_rad_s = -3.0f;
    for (unsigned int i = 0u; i < 300u; ++i) {
        feed(&r, 5.0f, 0.0f); /* Spin in place with a false roll. */
    }
    assert(r.h.output.count == 0u && r.h.output.gated_starts == 1u);
    /* Slow travel blocks a start; invalid motion too. */
    rig(&r, 1u);
    flat(&r, 50.0f, 20.0f);
    hump(&r, 10.0f, 5.0f, 20.0f);
    flat(&r, 50.0f, 20.0f);
    assert(r.h.output.count == 0u && r.h.output.gated_starts == 1u);
    rig(&r, 1u);
    r.in.motion_valid = false;
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    assert(r.h.output.count == 0u && r.h.output.gated_starts == 1u);
    r.in.motion_valid = true;
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 0u);

    /* A curve during a confirmed event freezes the peak; event counts. */
    rig(&r, 1u);
    flat(&r, 300.0f, 146.0f);
    for (unsigned int i = 0u; i < 60u; ++i) {
        feed(&r, 6.0f, 146.0f);
    }
    assert(r.h.output.state == IMU_HUMP_ACTIVE);
    const float frozen = r.h.output.event_peak_mm;

    r.in.lateral_mg = 80.0f;
    for (unsigned int i = 0u; i < 40u; ++i) {
        feed(&r, 12.0f, 146.0f);
    }
    assert(r.h.output.event_peak_mm == frozen);
    flat(&r, 50.0f, 146.0f); /* The curve ends after the false roll. */
    r.in.lateral_mg = 0.0f;
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.count == 1u && r.h.output.max_peak_mm < 6.01f);
    assert((r.h.output.flags & IMU_HUMP_FLAG_CURVE) != 0u);
}

static void lifecycle(void)
{
    rig_t r;

    /* No run: nothing counted. */
    rig(&r, 0u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(!r.h.output.running && r.h.output.count == 0u);
    assert(r.h.output.flags == 0u);
    /* Run 1 starts mid-flat (smoothing continues); STOP on the crest. */
    r.in.run_epoch = 1u;
    hump(&r, 10.0f, 50.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    assert(r.h.output.running && r.h.output.state == IMU_HUMP_IDLE);
    assert(r.h.output.count == 1u && r.h.output.max_peak_mm > 9.5f);
    for (unsigned int i = 0u; i < 70u; ++i) {
        feed(&r, 8.0f, 146.0f);
    }
    assert(r.h.output.state == IMU_HUMP_ACTIVE);
    /* The summary of an open event counts it, idempotently. */
    const imu_hump_summary_t open = summary(&r, 1000u);

    assert(open.count == 2u && (open.flags & IMU_HUMP_FLAG_OPEN) != 0u);
    assert(summary(&r, 1000u).count == 2u && r.h.output.count == 1u);
    r.in.run_epoch = 2u;
    feed(&r, 8.0f, 0.0f);
    assert(!r.h.output.running && r.h.output.count == 2u);
    assert((r.h.output.flags & IMU_HUMP_FLAG_OPEN) != 0u);
    assert(r.h.output.run_epoch == 1u);
    /* After STOP the value stays; summary for the ended epoch is NONE. */
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    assert(r.h.output.count == 2u);
    assert(summary(&r, 1000u).peak_01mm == IMU_HUMP_PEAK_NONE);
    /* Run 3 clears totals; an invalid sample marks a fault. */
    r.in.run_epoch = 3u;
    feed(&r, 0.0f, 146.0f);
    assert(r.h.output.count == 0u && r.h.output.run_epoch == 3u);
    assert(r.h.output.flags == IMU_HUMP_FLAG_IMPLEMENTED);
    r.in.valid = false;
    imu_hump_step(&r.h, &r.in);
    r.in.valid = true;
    assert((r.h.output.flags & IMU_HUMP_FLAG_FAULT) != 0u);
    assert(!r.h.output.smooth_ready);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    const imu_hump_summary_t fault = summary(&r, 1000u);

    assert(fault.count == 1u && fault.peak_01mm > 80u);
    assert(fault.flags == (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_FAULT));
    /* A direct jump to another odd epoch ends the old run first. */
    r.in.run_epoch = 5u;
    feed(&r, 0.0f, 146.0f);
    assert(r.h.output.count == 0u && r.h.output.run_epoch == 5u);
}

/* A stale, other-run or missing copy never gives a number. */
static void unavailable(void)
{
    rig_t r;
    imu_hump_report_t report;

    rig(&r, 7u);
    flat(&r, 300.0f, 146.0f);
    hump(&r, 10.0f, 5.0f, 146.0f);
    flat(&r, 300.0f, 146.0f);
    report = (imu_hump_report_t){.hump = r.h.output,
                                 .timestamp_ms = 0xFFFFFFF0u};
    const imu_hump_summary_t fresh =
        imu_hump_summarize(&report, true, 7u, 0x00000010u, 50u);

    assert(fresh.count == 1u && fresh.peak_01mm > 80u); /* Wrap is fine. */
    const imu_hump_summary_t none[] = {
        imu_hump_summarize(&report, true, 7u, 0x00000023u, 50u),
        imu_hump_summarize(&report, true, 9u, 0x00000010u, 50u),
        imu_hump_summarize(&report, true, 8u, 0x00000010u, 50u),
        imu_hump_summarize(&report, false, 7u, 0x00000010u, 50u),
        imu_hump_summarize(NULL, true, 7u, 0x00000010u, 50u)};

    for (unsigned int i = 0u; i < 5u; ++i) {
        assert(none[i].peak_01mm == IMU_HUMP_PEAK_NONE);
        assert(none[i].count == 0u && none[i].gated == 0u);
        assert(none[i].flags ==
               (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_FAULT));
    }
    /* Run with no valid sample at all: unavailable, gated kept. */
    rig(&r, 1u);
    r.in.valid = false;
    imu_hump_step(&r.h, &r.in);
    r.h.output.gated_starts = 300u;
    const imu_hump_summary_t dead = summary(&r, 1000u);

    assert(dead.peak_01mm == IMU_HUMP_PEAK_NONE && dead.gated == 255u);
    assert(dead.flags == (IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_FAULT));
}

int main(void)
{
    helpers();
    speeds();
    heights_and_side();
    flat_noise();
    chatter_and_spacing();
    long_slope();
    gates();
    lifecycle();
    unavailable();
    puts("imu hump fixtures passed");
    return 0;
}
