/* No heap, SDK, kernel, printing or driving decisions. */
#include "imu_capture.h"
#include "imu_fifo.h"
#include <stddef.h>

void imu_capture_begin(imu_capture_t *s, uint32_t now_us)
{
    s->samples = 0u;
    s->batches = 0u;
    s->started_us = now_us;
    s->reason = IMU_CAPTURE_RUNNING;
    s->active = true;
}

void imu_capture_finish(imu_capture_t *s, imu_capture_reason_t reason)
{
    s->reason = reason;
    s->active = false;
}

void imu_capture_append(imu_capture_t *s, const int16_t raw[][3],
                         unsigned int count, uint32_t now_us, bool error)
{
    if (!s->active) {
        return;
    }
    if (error || count > IMU_FIFO_MAX_BATCH || (count > 0u && raw == NULL)) {
        imu_capture_finish(s, IMU_CAPTURE_GAP);
        return;
    }
    if ((uint32_t)(now_us - s->started_us) >= IMU_CAPTURE_US) {
        imu_capture_finish(s, IMU_CAPTURE_TIME);
        return;
    }
    if (s->batches == IMU_CAPTURE_BATCHES) {
        imu_capture_finish(s, IMU_CAPTURE_META_FULL);
        return;
    }
    imu_capture_batch_t *b = &s->batch[s->batches];

    b->timestamp_us = now_us;
    b->first = s->samples;
    b->count = 0u;
    for (unsigned int i = 0u; i < count && s->samples < IMU_CAPTURE_SAMPLES;
         ++i) {
        for (unsigned int axis = 0u; axis < 3u; ++axis) {
            s->raw[s->samples][axis] = raw[i][axis];
        }
        ++s->samples;
        ++b->count;
    }
    ++s->batches;
    if (s->samples == IMU_CAPTURE_SAMPLES) {
        imu_capture_finish(s, IMU_CAPTURE_FULL);
    }
}
