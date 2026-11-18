/* Synthetic encoders/optics, not a physical robot acceptance test. */
#include <assert.h>
#include "../app_program/vehicle/junction_line.h"

static ja_t a;
static lp_t lp;
static vsm_inputs_t in;
static ja_sample_t sample;
static vsm_drive_t drive;
static uint32_t now;

static void tick(int32_t l, int32_t r)
{
    now += 10u;
    sample.line_stamp = now;
    in.counts[0] += l;
    in.counts[1] += r;
    (void)ja_step(&a, &in, &sample, now, &drive);
}

static void init(uint8_t cmd)
{
    ja_init(&a, 131, 1u);
    lp = (lp_t){.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN,
                 .norm = {400u, 400u}, .pair = true};
    in = (vsm_inputs_t){.line = &lp};
    sample = (ja_sample_t){.valid = true};
    now = 0u;
    const ja_command_t c = {.epoch = 1u, .sequence = 1u,
        .command = cmd, .source = 2u, .valid = 1u};
    assert(ja_offer(&a, &c));
    for (unsigned int i = 0; i < 12; ++i) tick(10, 10);
    lp.flags |= LP_FLAG_JUNCTION;
    lp.norm[0] = lp.norm[1] = 1000;
    while (a.phase == JA_PENDING) tick(10, 10);
    assert(a.phase == (cmd == JA_STRAIGHT ? JA_CROSS : JA_BAR_CLEAR));
}

static void centre(void)
{
    lp.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
    lp.norm[0] = lp.norm[1] = 0; /* gap must not fail advance. */
    while (a.phase == JA_BAR_CLEAR) tick(10, 10);
    assert(ja_searching(&a));
    const int32_t d = a.result.search_counts[0] - a.result.edge_counts[0];
    assert(d >= 1627 && d <= 1657); /* >=131 mm, two fresh samples. */
}

static void angle(int32_t cdeg, bool black)
{
    const int32_t diff = (int32_t)(((int64_t)cdeg * 2405 +
                                  (cdeg < 0 ? -9999 : 9999)) / 10000);
    in.counts[0] = a.result.search_counts[0] - diff / 2;
    in.counts[1] = a.result.search_counts[1] + diff - diff / 2;
    lp.norm[0] = black ? 600 : 0;
    lp.norm[1] = 0;
    tick(0, 0);
}

vsm_outcome_t blt_line_step(void *ctx, const vsm_inputs_t *in,
                            uint32_t ms, vsm_drive_t *out);

/* Any delegation during centre advance would run LINE's loss check. */
vsm_outcome_t jl_step(void *ctx, const vsm_inputs_t *input, uint32_t ms,
                       vsm_drive_t *out)
{
    (void)ctx; (void)input; (void)ms; (void)out;
    assert(false);
    return vsm_outcome(VSM_OUT_FAILED, 0u);
}

int main(void)
{
    init(JA_LEFT);
    assert(drive.left_mm_s == 120 && drive.right_mm_s == 120);
    centre();
    /* 20 ms white, then false contacts must not finish. */
    angle(100, false); angle(110, false); angle(120, false);
    angle(158, true);
    assert(a.phase == JA_SEARCH && a.result.first_line_cdeg < 200);
    angle(245, true); angle(3904, true); angle(5990, true);
    assert(a.phase == JA_SEARCH && a.result.claimed);
    /* A fresh departure is required after an early contact. */
    angle(6000, true);
    assert(a.phase == JA_SEARCH);
    angle(6100, false); angle(6200, false); angle(6300, false);
    angle(9000, true);
    assert(a.phase == JA_DONE && a.result.reason == JA_OK);
    assert(a.result.first_line_cdeg < 200);
    assert(a.result.accepted_line_cdeg >= 9000);

    init(JA_RIGHT); centre();
    angle(-100, false); angle(-110, false); angle(-120, false);
    angle(-5990, true);
    assert(a.phase == JA_SEARCH);
    angle(-6000, false); angle(-6000, false); angle(-6000, false);
    angle(-6000, true);
    assert(a.phase == JA_DONE && a.result.accepted_line_cdeg <= -6000);

    init(JA_LEFT);
    tick(0, 0);
    assert(a.phase == JA_BAR_CLEAR); /* no early line-based stop. */
    now += 500;
    tick(0, 0);
    assert(a.phase == JA_FAILED && a.result.reason == JA_STALL);
    init(JA_LEFT);
    tick(-1, 1);
    assert(a.phase == JA_FAILED && a.result.reason == JA_REVERSE);
    init(JA_LEFT);
    in.counts[0] += 2200; in.counts[1] += 2200;
    tick(0, 0);
    assert(a.phase == JA_FAILED && a.result.reason == JA_TRAVEL_LIMIT);
    init(JA_LEFT);
    for (unsigned int i = 0; i < 49; ++i) tick(1, 0);
    assert(a.phase == JA_BAR_CLEAR);
    tick(1, 0);
    assert(a.phase == JA_FAILED && a.result.reason == JA_STALL);
    init(JA_LEFT);
    now += 2000;
    tick(1, 1);
    assert(a.phase == JA_FAILED && a.result.reason == JA_TRAVEL_LIMIT);
    init(JA_LEFT);
    in.echo_new = true; in.echo = 60;
    tick(1, 1); tick(1, 1);
    assert(a.phase == JA_FAILED && a.result.reason == JA_OBSTACLE);
    init(JA_LEFT);
    in.echo_new = true;
    in.echo = (5u << ULTRASONIC_STREAK_SHIFT);
    tick(1, 1);
    assert(a.phase == JA_FAILED && a.result.reason == JA_SONAR_FAULT);

    /* The live wrapper bypasses lt_step through a white sensor gap. */
    init(JA_LEFT);
    jl_t live = {.action = a, .sample = sample};
    lp.norm[0] = lp.norm[1] = 0;
    assert(vsm_outcome_type(blt_line_step(&live, &in, now + 10,
                                         &drive)) == VSM_OUT_RUNNING);
    assert(drive.left_mm_s == 120 && drive.right_mm_s == 120);

    init(JA_LEFT);
    sample.valid = false;
    tick(1, 1);
    assert(a.phase == JA_FAILED && a.result.reason == JA_STALE);

    init(JA_STRAIGHT);
    in.counts[0] += 2000; in.counts[1] += 2000;
    tick(0, 0);
    assert(a.phase == JA_DONE);
    init(JA_UTURN);
    lp.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN;
    lp.norm[0] = lp.norm[1] = 400;
    while (a.phase == JA_BAR_CLEAR) tick(10, 10);
    assert(ja_searching(&a));
    assert(a.result.search_counts[0] - a.result.edge_counts[0] < 450);
    angle(100, false); angle(110, false); angle(120, false);
    angle(4500, true);
    assert(a.phase == JA_SECOND);
    angle(5000, false); angle(5100, false); angle(5200, false);
    angle(18000, true);
    assert(a.phase == JA_DONE);
}
