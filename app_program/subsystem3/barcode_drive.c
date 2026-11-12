#include "barcode_drive.h"
#include "barcode.h"
#include <stddef.h>

const char *barcode_drive_reason_name(barcode_drive_reason_t reason)
{
    static const char *const names[] = {
        "IDLE",    "DISTANCE", "BUTTON",    "TIMEOUT",    "FAULT",
        "COMMAND", "CAPACITY", "DIRECTION", "DIVERGENCE", "OVERRUN"};
    const unsigned int index = (unsigned int)reason;
    return index < sizeof(names) / sizeof(names[0]) ? names[index] : "UNKNOWN";
}

static int32_t delta32(int32_t current, int32_t origin)
{
    const uint32_t bits = (uint32_t)current - (uint32_t)origin;
    return bits <= INT32_MAX ? (int32_t)bits
                             : -(int32_t)(UINT32_MAX - bits) - 1;
}

uint32_t barcode_drive_timeout_ms(const barcode_drive_config_t *config)
{
    if (config == NULL || config->speed_mm_s < 25u ||
        config->speed_mm_s > 200u || config->distance_mm < 50u ||
        config->distance_mm > 450u) {
        return 0u;
    }
    return (config->distance_mm * 1000u + config->speed_mm_s - 1u) /
               config->speed_mm_s +
           3000u;
}

bool barcode_drive_config_valid(const barcode_drive_config_t *config)
{
    return config != NULL && config->distance_mm >= 50u &&
           config->distance_mm <= 450u && config->speed_mm_s >= 25u &&
           config->speed_mm_s <= 200u && config->counts_per_m > 0u &&
           config->counts_per_m <= 1000000u &&
           barcode_drive_timeout_ms(config) <= 55000u;
}

uint32_t barcode_drive_adc_ms(const barcode_drive_config_t *config)
{
    const uint32_t window = barcode_drive_timeout_ms(config) + 2000u;
    return (window + BC_MAX_SAMPLES - 3u) / (BC_MAX_SAMPLES - 2u);
}

uint32_t barcode_drive_motor_ms(const barcode_drive_config_t *config)
{
    const uint32_t window = barcode_drive_timeout_ms(config) + 2000u;
    return ((window + (BC_MAX_MOTION - 4u) * 10u - 1u) /
            ((BC_MAX_MOTION - 4u) * 10u)) *
           10u;
}

void barcode_drive_cancel(barcode_drive_t *job, barcode_drive_reason_t reason)
{
    if (job == NULL) {
        return;
    }
    job->active = false;
    job->reason = reason;
    if (reason != BC_DRIVE_BUTTON && reason != BC_DRIVE_DISTANCE) {
        job->inhibited = true;
    }
}

void barcode_drive_observe(barcode_drive_t *job,
                           const barcode_drive_config_t *config, int32_t left,
                           int32_t right)
{
    if (job == NULL || !barcode_drive_config_valid(config)) {
        return;
    }
    const int32_t counts[2] = {left, right};
    const int direction = job->backward ? -1 : 1;
    for (unsigned int i = 0u; i < 2u; ++i) {
        const int64_t movement =
            (int64_t)delta32(counts[i], job->origin[i]) * direction;
        if (movement * 1000 > (int64_t)450 * config->counts_per_m) {
            job->overrun = true;
        }
        const int64_t mm = movement * 1000 / config->counts_per_m;
        job->wheel_mm[i] = mm > INT32_MAX
                               ? INT32_MAX
                               : (mm < INT32_MIN ? INT32_MIN : (int32_t)mm);
    }
    job->travel_mm =
        (int32_t)(((int64_t)job->wheel_mm[0] + job->wheel_mm[1]) / 2);
}

int16_t barcode_drive_step(barcode_drive_t *job,
                           const barcode_drive_config_t *config,
                           uint32_t now_ms, int32_t left, int32_t right,
                           bool press, bool fault)
{
    if (job == NULL) {
        return 0;
    }
    if (!barcode_drive_config_valid(config) || fault) {
        barcode_drive_cancel(job, BC_DRIVE_FAULT);
        return 0;
    }
    if (job->active) {
        barcode_drive_observe(job, config, left, right);
        const int64_t lm = job->wheel_mm[0];
        const int64_t rm = job->wheel_mm[1];
        if (press) {
            barcode_drive_cancel(job, BC_DRIVE_BUTTON);
        } else if (lm < -20 || rm < -20) {
            barcode_drive_cancel(job, BC_DRIVE_DIRECTION);
        } else if (lm - rm > 100 || rm - lm > 100) {
            barcode_drive_cancel(job, BC_DRIVE_DIVERGENCE);
        } else if (now_ms - job->started_ms >=
                   barcode_drive_timeout_ms(config)) {
            barcode_drive_cancel(job, BC_DRIVE_TIMEOUT);
        } else if (job->travel_mm >= (int32_t)config->distance_mm ||
                   lm >= 450 || rm >= 450) {
            barcode_drive_cancel(job, BC_DRIVE_DISTANCE);
        }
    } else if (press && !job->inhibited) {
        job->origin[0] = left;
        job->origin[1] = right;
        job->travel_mm = 0;
        job->wheel_mm[0] = 0;
        job->wheel_mm[1] = 0;
        job->overrun = false;
        job->started_ms = now_ms;
        job->backward = job->next_backward;
        job->next_backward = !job->next_backward;
        job->active = true;
        job->reason = BC_DRIVE_IDLE;
    }
    if (!job->active) {
        return 0;
    }
    const int16_t speed = (int16_t)config->speed_mm_s;
    return job->backward ? (int16_t)-speed : speed;
}
