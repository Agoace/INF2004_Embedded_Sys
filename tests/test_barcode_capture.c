#include "../app_program/subsystem3/barcode_capture.h"
#include <assert.h>
#include <stddef.h>

int main(void)
{
    barcode_range_t range;
    barcode_range_begin();
    barcode_capture_observe(0u, 178u);
    barcode_capture_observe(1000u, 230u);
    barcode_range_end(&range);
    assert(range.samples == 2u && range.minimum == 178u &&
           range.maximum == 230u);
    barcode_capture_observe(2000u, 4095u);
    barcode_range_end(&range);
    assert(range.samples == 2u && range.maximum == 230u);
    barcode_range_end(NULL);
    barcode_capture_begin();
    assert(barcode_capture_active());
    size_t n;
    size_t m;
    for (uint32_t i = 0u; i < BC_MAX_SAMPLES + 3u; ++i) {
        barcode_capture_observe(UINT32_MAX - 3000u + i * 1000u,
                                (uint16_t)(i % 4096u));
    }
    const barcode_sample_t *rows = barcode_capture_rows(&n);
    assert(n == BC_MAX_SAMPLES && !barcode_capture_active());
    assert(barcode_capture_full() && barcode_capture_gaps() == 0u);
    assert(rows[0].raw == 0u && rows[n - 1u].raw == 4095u);
    for (size_t i = 0u; i < n; ++i) {
        assert(rows[i].flags == 0u);
    }
    const uint32_t last = rows[n - 1u].us;
    barcode_capture_observe(123u, 12u);
    assert(rows[n - 1u].us == last);
    barcode_capture_motor(NULL);
    for (uint32_t i = 0u; i < BC_MAX_MOTION + 5u; ++i) {
        const barcode_motion_t motor = {i * 10000u, (int32_t)i, -(int32_t)i};
        barcode_capture_motor(&motor);
        barcode_capture_motor(&motor); /* duplicate source stamp */
    }
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    assert(m == BC_MAX_MOTION && motors[m - 1u].left == 511);
    assert(barcode_capture_motor_overflow() == 10u);
    assert(barcode_capture_rows(NULL) == rows);
    assert(barcode_capture_motors(NULL) == motors);
    barcode_capture_cost(12u);
    barcode_capture_cost(7u);
    assert(barcode_capture_max_us() == 12u);
    barcode_capture_begin();
    (void)barcode_capture_motors(&m);
    assert(m == 0u && barcode_capture_max_us() == 0u);
    assert(!barcode_capture_full() && barcode_capture_motor_overflow() == 0u);
    barcode_capture_observe(0u, 12u);
    barcode_capture_observe(3000u, 13u);
    rows = barcode_capture_rows(&n);
    assert(n == 2u && rows[1].flags == BC_SAMPLE_GAP);
    assert(barcode_capture_gaps() == 1u);
    barcode_capture_freeze();
    assert(!barcode_capture_active());
    barcode_range_begin();
    barcode_capture_observe(4000u, 99u);
    barcode_range_end(&range);
    assert(range.samples == 1u && range.minimum == 99u && range.maximum == 99u);
    return 0;
}
