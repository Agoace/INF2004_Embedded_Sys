/* No ADC, GPIO, kernel or driving calls. Not allocated in other images. */
#include "barcode_capture.h"
#if APP_BARCODE_CAPTURE
#include <stddef.h>

static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static uint32_t row_count;
static uint32_t motor_count;
static uint32_t active;
static uint32_t ranging;
static uint32_t maximum_us;
static uint32_t range_count;
static uint32_t range_minmax;
static uint32_t full;
static uint32_t gaps;
static uint32_t motor_overflow;

_Static_assert(sizeof(rows) == 32768u, "ADC capture budget changed");
_Static_assert(sizeof(motors) == 6144u, "motion capture budget changed");

void barcode_range_begin(void)
{
    __atomic_store_n(&ranging, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&range_count, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&range_minmax, 4095u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&ranging, 1u, __ATOMIC_SEQ_CST);
}

void barcode_range_end(barcode_range_t *out)
{
    /* Core 0, reader below writer priority: a preempting writer completes
     * before this task resumes, and no later writer enters this range. */
    __atomic_store_n(&ranging, 0u, __ATOMIC_SEQ_CST);
    if (out != NULL) {
        const uint32_t bits = __atomic_load_n(&range_minmax, __ATOMIC_SEQ_CST);
        *out = (barcode_range_t){
            .samples = __atomic_load_n(&range_count, __ATOMIC_SEQ_CST),
            .minimum = (uint16_t)(bits & UINT32_C(0x0000ffff)),
            .maximum = (uint16_t)(bits >> 16u)};
    }
}

void barcode_capture_begin(void)
{
    barcode_capture_freeze();
    __atomic_store_n(&row_count, 0u, __ATOMIC_SEQ_CST);
    motor_count = 0u;
    motor_overflow = 0u;
    __atomic_store_n(&full, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&gaps, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&maximum_us, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&active, 1u, __ATOMIC_SEQ_CST);
}

void barcode_capture_freeze(void)
{
    __atomic_store_n(&active, 0u, __ATOMIC_SEQ_CST);
}

bool barcode_capture_active(void)
{
    return __atomic_load_n(&active, __ATOMIC_SEQ_CST) != 0u;
}

void barcode_capture_observe(uint32_t us, uint16_t raw)
{
    if (__atomic_load_n(&ranging, __ATOMIC_SEQ_CST) != 0u) {
        const uint32_t bits = __atomic_load_n(&range_minmax, __ATOMIC_SEQ_CST);
        const uint16_t low = (uint16_t)(bits & UINT32_C(0x0000ffff));
        const uint16_t high = (uint16_t)(bits >> 16u);
        const uint32_t next = (uint32_t)(raw < low ? raw : low) |
                              ((uint32_t)(raw > high ? raw : high) << 16u);
        const uint32_t n = __atomic_load_n(&range_count, __ATOMIC_SEQ_CST);
        __atomic_store_n(&range_minmax, next, __ATOMIC_SEQ_CST);
        __atomic_store_n(&range_count, n + 1u, __ATOMIC_SEQ_CST);
    }
    if (!barcode_capture_active()) {
        return;
    }
    const uint32_t index = __atomic_load_n(&row_count, __ATOMIC_SEQ_CST);
    if (index >= BC_MAX_SAMPLES) {
        barcode_capture_freeze();
        return;
    }
    const bool gap = index > 0u && (us - rows[index - 1u].us == 0u ||
                                    us - rows[index - 1u].us > 1500u);
    if (gap) {
        const uint32_t n = __atomic_load_n(&gaps, __ATOMIC_SEQ_CST);
        __atomic_store_n(&gaps, n + 1u, __ATOMIC_SEQ_CST);
    }
    rows[index] = (barcode_sample_t){us, raw, gap ? BC_SAMPLE_GAP : 0u};
    __atomic_store_n(&row_count, index + 1u, __ATOMIC_SEQ_CST);
    if (index + 1u == BC_MAX_SAMPLES) {
        __atomic_store_n(&full, 1u, __ATOMIC_SEQ_CST);
        barcode_capture_freeze();
    }
}

void barcode_capture_cost(uint32_t us)
{
    if (us > __atomic_load_n(&maximum_us, __ATOMIC_SEQ_CST)) {
        __atomic_store_n(&maximum_us, us, __ATOMIC_SEQ_CST);
    }
}

uint32_t barcode_capture_max_us(void)
{
    return __atomic_load_n(&maximum_us, __ATOMIC_SEQ_CST);
}

void barcode_capture_motor(const barcode_motion_t *motor)
{
    if (motor == NULL ||
        (motor_count > 0u && motors[motor_count - 1u].us == motor->us)) {
        return;
    }
    if (motor_count >= BC_MAX_MOTION) {
        ++motor_overflow;
        return;
    }
    motors[motor_count++] = *motor;
}

bool barcode_capture_full(void)
{
    return __atomic_load_n(&full, __ATOMIC_SEQ_CST) != 0u;
}

uint32_t barcode_capture_gaps(void)
{
    return __atomic_load_n(&gaps, __ATOMIC_SEQ_CST);
}

uint32_t barcode_capture_motor_overflow(void)
{
    return motor_overflow;
}

const barcode_sample_t *barcode_capture_rows(size_t *count)
{
    if (count != NULL) {
        *count = __atomic_load_n(&row_count, __ATOMIC_SEQ_CST);
    }
    return rows;
}

const barcode_motion_t *barcode_capture_motors(size_t *count)
{
    if (count != NULL) {
        *count = motor_count;
    }
    return motors;
}
#endif
