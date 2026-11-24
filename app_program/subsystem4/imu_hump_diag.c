/* Diagnostic C follows detector decisions; it cannot change them. */
#include "imu_hump_diag.h"
#include "imu_settings.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

void imu_hump_diag_init(imu_hump_diag_t *s)
{
    if (s != NULL) {
        memset(s, 0, sizeof(*s));
    }
}

static uint8_t gates(const imu_hump_diag_input_t *in)
{
    uint8_t gate = 0u;

    if (!in->motion_valid || !isfinite(in->speed_mm_s) ||
        !isfinite(in->yaw_rad_s) || !isfinite(in->lateral_mg)) {
        gate |= IMU_DIAG_GATE_MOTION;
    }
    if (fabsf(in->lateral_mg) > IMU_HUMP_GATE_LATERAL_MG) {
        gate |= IMU_DIAG_GATE_LATERAL;
    }
    if (fabsf(in->yaw_rad_s) > IMU_HUMP_GATE_YAW_RAD_S) {
        gate |= IMU_DIAG_GATE_YAW;
    }
    if (fabsf(in->speed_mm_s) < IMU_HUMP_MIN_SPEED_MM_S) {
        gate |= IMU_DIAG_GATE_SPEED;
    }
    return gate;
}

static bool window_step(imu_hump_diag_t *s,
                          const imu_hump_diag_input_t *in)
{
    if (!in->valid || !isfinite(in->corrected_mm) ||
        !isfinite(in->filtered_mm)) {
        s->samples = 0u;
        s->index = 0u;
        return false;
    }
    s->window[s->index] = (imu_hump_diag_sample_t){
        .corrected_mm = in->corrected_mm,
        .filtered_mm = in->filtered_mm,
        .yaw_rad_s = fabsf(in->yaw_rad_s),
        .yaw_accel_rad_s2 = fabsf(in->yaw_accel_rad_s2),
        .lateral_mg = fabsf(in->lateral_mg), .gate = gates(in)};
    s->index = (s->index + 1u) % IMU_HUMP_MEAN_SAMPLES;
    if (s->samples < IMU_HUMP_MEAN_SAMPLES) {
        ++s->samples;
    }
    return s->samples == IMU_HUMP_MEAN_SAMPLES;
}

static void latch(imu_hump_diag_t *s, const imu_hump_diag_input_t *in)
{
    imu_hump_peak_diag_t *p = &s->event;

    memset(p, 0, sizeof(*p));
    p->ms = in->now_ms - in->run_start_ms;
    p->travel_mm = s->travel_mm;
    p->speed_mm_s = in->speed_mm_s;
    p->yaw_rad_s = in->yaw_rad_s;
    p->yaw_accel_rad_s2 = in->yaw_accel_rad_s2;
    p->lateral_mg = in->lateral_mg;
    p->gate = gates(in);
    p->left = in->hump->event_left;
    p->valid = true;
    for (unsigned int i = 0u; i < IMU_HUMP_MEAN_SAMPLES; ++i) {
        const imu_hump_diag_sample_t *w = &s->window[i];

        p->corrected_mm += w->corrected_mm;
        p->filtered_mm += w->filtered_mm;
        p->window_yaw_rad_s = fmaxf(p->window_yaw_rad_s, w->yaw_rad_s);
        p->window_yaw_accel_rad_s2 =
            fmaxf(p->window_yaw_accel_rad_s2, w->yaw_accel_rad_s2);
        p->window_lateral_mg = fmaxf(p->window_lateral_mg, w->lateral_mg);
        p->window_gate |= w->gate;
    }
    p->corrected_mm /= (float)IMU_HUMP_MEAN_SAMPLES;
    p->filtered_mm /= (float)IMU_HUMP_MEAN_SAMPLES;
}

void imu_hump_diag_step(imu_hump_diag_t *s,
                         const imu_hump_diag_input_t *in,
                         imu_hump_diag_report_t *out)
{
    if (s == NULL || in == NULL || in->hump == NULL || out == NULL) {
        return;
    }
    const imu_hump_output_t *h = in->hump;

    if (h->running && h->run_epoch != s->run_epoch) {
        /* Keep the parallel window aligned with the detector's window. */
        memset(&s->event, 0, sizeof(s->event));
        memset(&s->winner, 0, sizeof(s->winner));
        s->run_epoch = h->run_epoch;
        s->line_samples = 0u;
        s->line_lateral_mg = 0.0f;
        s->line_yaw_accel_rad_s2 = 0.0f;
        s->travel_mm = 0.0f;
        s->event_peak_mm = 0.0f;
        s->winner_peak_mm = 0.0f;
        s->count = 0u;
        s->state = (uint8_t)IMU_HUMP_IDLE;
    }
    const bool ready = window_step(s, in);
    const bool event = h->state == (uint8_t)IMU_HUMP_CANDIDATE ||
                       h->state == (uint8_t)IMU_HUMP_ACTIVE;

    if (h->running) {
        if (in->motion_valid && isfinite(in->travel_mm)) {
            s->travel_mm += fabsf(in->travel_mm);
        }
        if (in->line_active && in->motion_valid &&
            isfinite(in->lateral_mg) && isfinite(in->yaw_accel_rad_s2)) {
            s->line_lateral_mg = fmaxf(s->line_lateral_mg,
                                      fabsf(in->lateral_mg));
            s->line_yaw_accel_rad_s2 = fmaxf(s->line_yaw_accel_rad_s2,
                                           fabsf(in->yaw_accel_rad_s2));
            if (s->line_samples != UINT32_MAX) {
                ++s->line_samples;
            }
        }
    }
    if (event && s->state != (uint8_t)IMU_HUMP_CANDIDATE &&
        s->state != (uint8_t)IMU_HUMP_ACTIVE) {
        s->event_peak_mm = 0.0f;
        s->event.valid = false;
    }
    if (event && ready && h->event_peak_mm > s->event_peak_mm) {
        s->event_peak_mm = h->event_peak_mm;
        latch(s, in);
    }
    const bool closed = h->count != s->count ||
                        h->max_peak_mm > s->winner_peak_mm;

    /* A last eligible increase can coincide with OFF/STOP closure. */
    if (closed && ready && h->last_peak_mm > s->event_peak_mm) {
        s->event_peak_mm = h->last_peak_mm;
        latch(s, in);
        s->event.left = h->last_left;
    }
    /* Count saturation is harmless: a greater max still identifies the
     * winning closed event. Preserve first equal-height event. */
    if (closed &&
        s->event.valid && s->event_peak_mm > s->winner_peak_mm) {
        s->winner = s->event;
        s->winner_peak_mm = s->event_peak_mm;
    }
    s->count = h->count;
    s->state = h->state;
    *out = (imu_hump_diag_report_t){
        .peak = s->winner, .run_epoch = h->run_epoch,
        .timestamp_ms = in->now_ms, .line_samples = s->line_samples,
        .line_lateral_mg = s->line_lateral_mg,
        .line_yaw_accel_rad_s2 = s->line_yaw_accel_rad_s2,
        .valid = h->valid_samples != 0u};
    if (h->state == (uint8_t)IMU_HUMP_ACTIVE && s->event.valid &&
        s->event_peak_mm > s->winner_peak_mm) {
        out->peak = s->event;
    }
}

void imu_hump_diag_finish(imu_hump_diag_report_t *out, uint32_t epoch,
                           uint32_t now_ms, bool have)
{
    if (out == NULL) {
        return;
    }
    if (!have || !out->valid || (epoch & 1u) == 0u ||
        out->run_epoch != epoch ||
        now_ms - out->timestamp_ms > IMU_HUMP_MAX_AGE_MS) {
        memset(out, 0, sizeof(*out));
        out->run_epoch = epoch;
    }
}

/* Print rounding: no printf float support; saturate before conversion. */
long imu_hump_diag_scaled(float value, float scale)
{
    const float x = value * scale;

    if (!isfinite(x)) {
        return 0L;
    }
    if (x >= 2147483520.0f) {
        return (long)INT32_MAX;
    }
    if (x <= -2147483648.0f) {
        return (long)INT32_MIN;
    }
    return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/* Field placement: shift, bits. See imu_hump_diag.h. */
#define F_MS 2u, 0u, 20u
#define F_CORRECTED 2u, 20u, 12u
#define F_TRAVEL 3u, 0u, 16u
#define F_YAW 3u, 16u, 15u
#define F_YAW_ACCEL 4u, 0u, 19u
#define F_LATERAL 4u, 19u, 13u
#define F_WIN_YAW_ACCEL 5u, 0u, 18u
#define F_WIN_YAW 5u, 18u, 14u
#define F_LINE_YAW_ACCEL 6u, 0u, 18u
#define F_WIN_LATERAL 6u, 18u, 12u
#define F_LEFT 6u, 30u, 1u
#define F_PEAK_VALID 6u, 31u, 1u
#define F_LINE_SAMPLES 7u, 0u, 20u
#define F_LINE_LATERAL 7u, 20u, 12u
#define F_FILTERED 8u, 0u, 12u
#define F_SPEED 8u, 12u, 12u
#define F_GATE 8u, 24u, 4u
#define F_WIN_GATE 8u, 28u, 4u
#define F_VALID 3u, IMU_DIAG_W3_VALID_SHIFT, 1u

static uint32_t mask(unsigned int bits)
{
    return bits >= 32u ? UINT32_MAX : (1u << bits) - 1u;
}

static void put(imu_hump_diag_board_t *b, unsigned int word,
                unsigned int shift, unsigned int bits, uint32_t value)
{
    b->word[word] |= (value & mask(bits)) << shift;
}

static uint32_t get(const imu_hump_diag_board_t *b, unsigned int word,
                    unsigned int shift, unsigned int bits)
{
    return (b->word[word] >> shift) & mask(bits);
}

/* Unsigned field: clamp to 0 .. 2^bits - 1. */
static void put_u(imu_hump_diag_board_t *b, unsigned int word,
                  unsigned int shift, unsigned int bits, long value)
{
    const long top = (long)mask(bits);
    put(b, word, shift, bits,
        (uint32_t)(value < 0L ? 0L : value > top ? top : value));
}

/* Signed field: clamp to -2^(bits-1) .. 2^(bits-1) - 1, two's complement. */
static void put_s(imu_hump_diag_board_t *b, unsigned int word,
                  unsigned int shift, unsigned int bits, long value)
{
    const long top = (long)mask(bits - 1u);
    const long low = -top - 1L;
    const long v = value < low ? low : value > top ? top : value;
    put(b, word, shift, bits, (uint32_t)v);
}

static void put_count(imu_hump_diag_board_t *b, unsigned int word,
                      unsigned int shift, unsigned int bits, uint32_t value)
{
    put(b, word, shift, bits, value > mask(bits) ? mask(bits) : value);
}

static long get_s(const imu_hump_diag_board_t *b, unsigned int word,
                  unsigned int shift, unsigned int bits)
{
    const uint32_t sign = 1u << (bits - 1u);
    const uint32_t v = get(b, word, shift, bits);
    return (long)(int32_t)(v ^ sign) - (long)sign;
}

void imu_hump_diag_pack_peak(imu_hump_diag_board_t *out,
                             const imu_hump_peak_diag_t *p)
{
    memset(out, 0, sizeof(*out));
    put_count(out, F_MS, p->ms);
    put_s(out, F_CORRECTED, imu_hump_diag_scaled(p->corrected_mm, 10.0f));
    put_s(out, F_FILTERED, imu_hump_diag_scaled(p->filtered_mm, 10.0f));
    put_u(out, F_TRAVEL, imu_hump_diag_scaled(p->travel_mm, 1.0f));
    put_s(out, F_SPEED, imu_hump_diag_scaled(p->speed_mm_s, 1.0f));
    put_s(out, F_YAW, imu_hump_diag_scaled(p->yaw_rad_s, 1000.0f));
    put_s(out, F_YAW_ACCEL,
          imu_hump_diag_scaled(p->yaw_accel_rad_s2, 1000.0f));
    put_s(out, F_LATERAL, imu_hump_diag_scaled(p->lateral_mg, 1.0f));
    put_u(out, F_WIN_YAW, imu_hump_diag_scaled(p->window_yaw_rad_s, 1000.0f));
    put_u(out, F_WIN_YAW_ACCEL,
          imu_hump_diag_scaled(p->window_yaw_accel_rad_s2, 1000.0f));
    put_u(out, F_WIN_LATERAL,
          imu_hump_diag_scaled(p->window_lateral_mg, 1.0f));
    put(out, F_GATE, p->gate);
    put(out, F_WIN_GATE, p->window_gate);
    put(out, F_LEFT, p->left ? 1u : 0u);
    put(out, F_PEAK_VALID, p->valid ? 1u : 0u);
}

void imu_hump_diag_pack_run(imu_hump_diag_board_t *out,
                            const imu_hump_diag_board_t *peak,
                            const imu_hump_diag_report_t *r)
{
    imu_hump_diag_board_t b = *peak;

    b.word[0] = r->run_epoch;
    b.word[1] = r->timestamp_ms;
    put(&b, F_VALID, r->valid ? 1u : 0u);
    put_count(&b, F_LINE_SAMPLES, r->line_samples);
    put_u(&b, F_LINE_LATERAL, imu_hump_diag_scaled(r->line_lateral_mg, 1.0f));
    put_u(&b, F_LINE_YAW_ACCEL,
          imu_hump_diag_scaled(r->line_yaw_accel_rad_s2, 1000.0f));
    *out = b;
}

void imu_hump_diag_pack(imu_hump_diag_board_t *out,
                        const imu_hump_diag_report_t *r)
{
    imu_hump_diag_board_t peak;

    imu_hump_diag_pack_peak(&peak, &r->peak);
    imu_hump_diag_pack_run(out, &peak, r);
}

void imu_hump_diag_board_finish(imu_hump_diag_board_t *b, uint32_t epoch,
                                uint32_t now_ms, bool have)
{
    if (b == NULL) {
        return;
    }
    if (!have || !imu_hump_diag_board_valid(b) || (epoch & 1u) == 0u ||
        imu_hump_diag_board_epoch(b) != epoch ||
        now_ms - imu_hump_diag_board_time(b) > IMU_HUMP_MAX_AGE_MS) {
        memset(b, 0, sizeof(*b));
        b->word[0] = epoch;
    }
}

void imu_hump_diag_board_unpack(imu_hump_diag_board_values_t *v,
                                const imu_hump_diag_board_t *b)
{
    v->run_epoch = imu_hump_diag_board_epoch(b);
    v->valid = imu_hump_diag_board_valid(b);
    v->peak_valid = get(b, F_PEAK_VALID) != 0u;
    v->ms = get(b, F_MS);
    v->travel_mm = (long)get(b, F_TRAVEL);
    v->corrected_01mm = get_s(b, F_CORRECTED);
    v->filtered_01mm = get_s(b, F_FILTERED);
    v->speed_mm_s = get_s(b, F_SPEED);
    v->yaw_mrad_s = get_s(b, F_YAW);
    v->yaw_accel_mrad_s2 = get_s(b, F_YAW_ACCEL);
    v->lateral_mg = get_s(b, F_LATERAL);
    v->win_yaw_mrad_s = (long)get(b, F_WIN_YAW);
    v->win_yaw_accel_mrad_s2 = (long)get(b, F_WIN_YAW_ACCEL);
    v->win_lateral_mg = (long)get(b, F_WIN_LATERAL);
    v->gate = (uint8_t)get(b, F_GATE);
    v->win_gate = (uint8_t)get(b, F_WIN_GATE);
    v->left = get(b, F_LEFT) != 0u;
    v->line_samples = get(b, F_LINE_SAMPLES);
    v->line_lateral_mg = (long)get(b, F_LINE_LATERAL);
    v->line_yaw_accel_mrad_s2 = (long)get(b, F_LINE_YAW_ACCEL);
}
