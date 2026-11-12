/* One core-0 line-task writer; no ADC, GPIO, kernel or driving calls. */
#include "barcode_capture.h"
#if APP_BARCODE_CAPTURE
#include <stddef.h>

static barcode_packed_t rows[BC_MAX_SAMPLES];
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
static uint32_t dropped;
static uint32_t motor_overflow;
static uint32_t origin;
static uint32_t previous_raw_us;
static uint32_t previous_kept_us;
static uint32_t decimation;
static uint32_t phase;
static bool raw_seen;
static bool pending_gap;

_Static_assert(sizeof(rows) == 24576u, "ADC capture budget changed");
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
    /* The higher-priority core-0 writer completes before this task resumes. */
    __atomic_store_n(&ranging, 0u, __ATOMIC_SEQ_CST);
    if (out != NULL) {
        const uint32_t bits = __atomic_load_n(&range_minmax, __ATOMIC_SEQ_CST);
        *out = (barcode_range_t){
            .samples = __atomic_load_n(&range_count, __ATOMIC_SEQ_CST),
            .minimum = (uint16_t)(bits & UINT32_C(0x0000ffff)),
            .maximum = (uint16_t)(bits >> 16u)};
    }
}

void barcode_capture_begin(uint32_t keep_every_ms)
{
    barcode_capture_freeze();
    __atomic_store_n(&row_count, 0u, __ATOMIC_SEQ_CST);
    motor_count = 0u;
    motor_overflow = 0u;
    decimation =
        keep_every_ms > 0u && keep_every_ms <= 60u ? keep_every_ms : 1u;
    phase = 0u;
    raw_seen = false;
    pending_gap = false;
    origin = 0u;
    __atomic_store_n(&full, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&gaps, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&dropped, 0u, __ATOMIC_SEQ_CST);
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

static void increment(uint32_t *value)
{
    const uint32_t n = __atomic_load_n(value, __ATOMIC_SEQ_CST);
    __atomic_store_n(value, n + 1u, __ATOMIC_SEQ_CST);
}

void barcode_capture_observe(uint32_t us, uint16_t raw)
{
    if (__atomic_load_n(&ranging, __ATOMIC_SEQ_CST) != 0u) {
        const uint32_t bits = __atomic_load_n(&range_minmax, __ATOMIC_SEQ_CST);
        const uint16_t low = (uint16_t)(bits & UINT32_C(0x0000ffff));
        const uint16_t high = (uint16_t)(bits >> 16u);
        const uint32_t next = (uint32_t)(raw < low ? raw : low) |
                              ((uint32_t)(raw > high ? raw : high) << 16u);
        __atomic_store_n(&range_minmax, next, __ATOMIC_SEQ_CST);
        increment(&range_count);
    }
    if (!barcode_capture_active()) {
        return;
    }
    const uint32_t dt = us - previous_raw_us;
    if ((raw_seen && (dt == 0u || dt > 1500u)) || raw > 4095u) {
        pending_gap = true;
        increment(&gaps);
    }
    previous_raw_us = us;
    raw_seen = true;
    if (phase > 0u) {
        --phase;
        return;
    }
    phase = decimation - 1u;
    const uint32_t index = __atomic_load_n(&row_count, __ATOMIC_SEQ_CST);
    if (index >= BC_MAX_SAMPLES) {
        increment(&dropped);
        return; /* Full requests a stop; only its owner may freeze. */
    }
    const uint32_t delta = index == 0u ? 0u : us - previous_kept_us;
    if (delta > UINT16_MAX) {
        pending_gap = true;
        increment(&gaps);
    }
    if (index == 0u) {
        origin = us;
    }
    rows[index] = (barcode_packed_t){
        (uint16_t)((raw & BC_PACKED_RAW) | (pending_gap ? BC_PACKED_GAP : 0u)),
        (uint16_t)(delta <= UINT16_MAX ? delta : UINT16_MAX)};
    pending_gap = false;
    previous_kept_us = us;
    __atomic_store_n(&row_count, index + 1u, __ATOMIC_SEQ_CST);
    if (index + 1u == BC_MAX_SAMPLES) {
        __atomic_store_n(&full, 1u, __ATOMIC_SEQ_CST);
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

uint32_t barcode_capture_dropped(void)
{
    return __atomic_load_n(&dropped, __ATOMIC_SEQ_CST);
}

uint32_t barcode_capture_origin(void)
{
    return origin; /* Read only after freeze, on core 0. */
}

uint32_t barcode_capture_motor_overflow(void)
{
    return motor_overflow;
}

const barcode_packed_t *barcode_capture_rows(size_t *count)
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
