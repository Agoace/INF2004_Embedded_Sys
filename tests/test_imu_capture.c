/* Full-rate capture storage: exact words/order/bounds and gap semantics. */
#include "../app_program/subsystem4/imu_capture.h"
#include "../app_program/subsystem4/imu_fifo.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    static imu_capture_t s;
    int16_t raw[IMU_FIFO_MAX_BATCH][3] = {{-32768, 12345, 32767}};

    imu_capture_begin(&s, 1000u);
    for (uint32_t i = 0u; i < 200u; ++i) {
        imu_capture_append(&s, raw, 4u, 1000u + i * 10000u, false);
    }
    assert(!s.active && s.reason == IMU_CAPTURE_FULL && s.samples == 800u);
    assert(s.batches == 200u && s.batch[199].first == 796u);
    assert(s.raw[796][0] == -32768 && s.raw[796][1] == 12345);
    imu_capture_append(&s, raw, 4u, 2001000u, false);
    assert(s.samples == 800u);
    imu_capture_begin(&s, 0u);
    imu_capture_append(&s, NULL, 0u, 100u, false);
    assert(s.active && s.samples == 0u && s.batches == 1u);
    imu_capture_append(&s, raw, 4u, 10000u, false);
    imu_capture_append(&s, NULL, 0u, 20000u, true);
    assert(!s.active && s.reason == IMU_CAPTURE_GAP && s.samples == 4u);
    imu_capture_begin(&s, UINT32_MAX - 999999u);
    imu_capture_append(&s, raw, 1u, 1000000u, false);
    assert(!s.active && s.reason == IMU_CAPTURE_TIME && s.samples == 0u);
    imu_capture_begin(&s, 0u);
    for (unsigned int i = 0u; i <= IMU_CAPTURE_BATCHES; ++i) {
        imu_capture_append(&s, NULL, 0u, i, false);
    }
    assert(!s.active && s.reason == IMU_CAPTURE_META_FULL);
    assert(s.batches == IMU_CAPTURE_BATCHES);
    imu_capture_begin(&s, 0u);
    imu_capture_append(&s, raw, 9u, 1u, false);
    assert(!s.active && s.reason == IMU_CAPTURE_GAP);
    imu_capture_begin(&s, 0u);
    s.samples = 798u;
    imu_capture_append(&s, raw, 4u, 1u, false);
    assert(s.samples == 800u && s.batch[0].count == 2u);
    imu_capture_begin(&s, 0u);
    imu_capture_finish(&s, IMU_CAPTURE_STOP);
    assert(!s.active && s.reason == IMU_CAPTURE_STOP);
    printf("capture sizeof %zu B; raw4800, metadata3072 B\n", sizeof(s));
    return 0;
}
