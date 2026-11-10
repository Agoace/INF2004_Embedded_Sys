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
