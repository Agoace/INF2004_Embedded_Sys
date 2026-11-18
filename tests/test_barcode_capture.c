#include "../app_program/subsystem3/barcode_capture.h"
#include <assert.h>
#include <stddef.h>

int main(void)
{
    barcode_range_t range;
    barcode_range_begin();
    barcode_capture_observe(0u, 151u);
    barcode_capture_observe(1000u, 1660u);
    barcode_range_end(&range);
    assert(range.samples == 2u && range.minimum == 151u &&
           range.maximum == 1660u);
    barcode_capture_observe(2000u, 4095u);
    barcode_range_end(&range);
    assert(range.samples == 2u && range.maximum == 1660u);
    barcode_range_end(NULL);
    barcode_capture_begin(2u);
    assert(barcode_capture_active());
    size_t n;
    size_t m;
    const uint32_t origin = UINT32_MAX - 3000u;
    for (uint32_t i = 0u; i < BC_MAX_SAMPLES * 2u + 6u; ++i) {
        barcode_capture_observe(origin + i * 1000u, (uint16_t)(i % 4096u));
    }
    const barcode_packed_t *rows = barcode_capture_rows(&n);
    assert(n == BC_MAX_SAMPLES && barcode_capture_active());
    assert(barcode_capture_full() && barcode_capture_gaps() == 0u);
    assert(barcode_capture_dropped() == 3u);
    assert(barcode_capture_origin() == origin);
    assert(rows[0].raw_flags == 0u && rows[0].delta_us == 0u);
    for (size_t i = 1u; i < n; ++i) {
        assert(rows[i].delta_us == 2000u);
        assert(rows[i].raw_flags == (uint16_t)((i * 2u) % 4096u));
    }
    barcode_capture_freeze();
    const uint16_t last = rows[n - 1u].raw_flags;
    barcode_capture_observe(123u, 12u);
    assert(rows[n - 1u].raw_flags == last);
    barcode_capture_motor(NULL);
    for (uint32_t i = 0u; i < BC_MAX_MOTION + 5u; ++i) {
        const barcode_motion_t motor = {i * 30000u, (int32_t)i, -(int32_t)i};
        barcode_capture_motor(&motor);
        barcode_capture_motor(&motor);
    }
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    assert(m == BC_MAX_MOTION && motors[m - 1u].left == 511);
    assert(barcode_capture_motor_overflow() == 10u);
    assert(barcode_capture_rows(NULL) == rows);
    assert(barcode_capture_motors(NULL) == motors);
    barcode_capture_cost(12u);
    barcode_capture_cost(7u);
    assert(barcode_capture_max_us() == 12u);
    barcode_capture_begin(2u);
    (void)barcode_capture_motors(&m);
    assert(m == 0u && barcode_capture_max_us() == 0u);
    assert(!barcode_capture_full() && barcode_capture_motor_overflow() == 0u);
    barcode_capture_observe(0u, 12u);
    barcode_capture_observe(3000u, 13u); /* missing raw tick, not retained */
    barcode_capture_observe(4000u, 14u);
    rows = barcode_capture_rows(&n);
    assert(n == 2u && rows[1].raw_flags == (14u | BC_PACKED_GAP));
    assert(rows[1].delta_us == 4000u && barcode_capture_gaps() == 1u);
    barcode_capture_observe(5000u, 4096u); /* invalid on skipped tick */
    barcode_capture_observe(70000u, 14u);  /* u16 delta cannot represent this */
    rows = barcode_capture_rows(&n);
    assert(n == 3u && rows[2].raw_flags == (14u | BC_PACKED_GAP));
    assert(rows[2].delta_us == UINT16_MAX);
    barcode_capture_freeze();
    assert(!barcode_capture_active());
    barcode_range_begin();
    barcode_capture_observe(71000u, 99u);
    barcode_range_end(&range);
    assert(range.samples == 1u && range.minimum == 99u && range.maximum == 99u);
    return 0;
}
