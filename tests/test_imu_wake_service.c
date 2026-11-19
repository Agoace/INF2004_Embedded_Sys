/* Service wake path with IMU_INT1_WAKE=1: GP7 client registration, ISR
 * wake, 10 ms cyclic fallback (>12 ms no edge), one extra pass on a HIGH
 * INT1 level, histogram and counters. The read/compute path is unchanged. */
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_service.h"
#include "../app_program/subsystem4/imu_wake.h"
#include "../app_program/subsystem2/motion.h"
#include "../app_program/platform/drv_gpio_irq.h"

static FP entry_function, cyclic_function;
static drv_gpio_irq_fn int1_fn;
static uint32_t int1_pins;
static jmp_buf stop;
static unsigned int sleeps, sleep_goal, reads, wakes, add_error, init_calls;
static ER wup_result = E_OK;
static uint32_t clock_us;
static bool level_high, stuck, probe_ready = true, empty;

uint32_t drv_time_us(void) { return clock_us; }
uint32_t drv_time_ms(void) { return clock_us / 1000u; }
uint32_t vehicle_calibration_epoch(void) { return 0u; }
bool imu_hw_int1_high(void) { return level_high; }
int drv_gpio_irq_init(void) { ++init_calls; return E_OK; }
int drv_gpio_irq_add(uint32_t pins, drv_gpio_irq_fn fn, void *ctx)
{
    assert(ctx == NULL);
    if (add_error) { return E_OBJ; }
    int1_pins = pins;
    int1_fn = fn;
    return E_OK;
}
void imu_probe_init(imu_probe_t *probe) { memset(probe, 0, sizeof(*probe)); }
bool imu_probe_read(imu_probe_t *probe, uint32_t now, imu_units_t *out)
{
    (void)now;
    ++reads;
    level_high = stuck; /* The burst drains the FIFO below the watermark. */
    probe->ready = probe_ready;
    probe->no_sample = empty;
    probe->fifo_last_samples = empty ? 0u : 4u;
    if (empty) { return false; }
    memset(out, 0, sizeof(*out));
    out->accel_mg[2] = 1000;
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        probe->mean_accel_mg[axis] = (float)out->accel_mg[axis];
    }
    return true;
}
int motion_snapshot_try_read(motion_status_t *out, uint32_t *stamp)
{
    memset(out, 0, sizeof(*out));
    *stamp = clock_us;
    return E_OK;
}
ID tk_cre_cyc(const T_CCYC *config)
{
    assert(config->cyctim == 10u && config->cycphs == 10u);
    cyclic_function = config->cychdr;
    return 6;
}
ER tk_sta_cyc(ID id) { assert(id == 6); return E_OK; }
ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{
    (void)priority; (void)core; (void)name;
    entry_function = entry;
    return 7;
}
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR);
    if (++sleeps >= sleep_goal) { longjmp(stop, 1); }
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id == TSK_SELF); return 0; }
ER tk_wup_tsk(ID id) { assert(id == 7); ++wakes; return wup_result; }

/* Run the task until it sleeps once more; return the probe reads. */
static unsigned int run_once(void)
{
    const unsigned int before = reads;
    sleep_goal = sleeps + 1u;
    if (setjmp(stop) == 0) {
        ((void (*)(INT, void *))entry_function)(0, NULL);
    }
    return reads - before;
}
static void tick(void) { ((void (*)(void *))cyclic_function)(NULL); }

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "add_failure") == 0) {
        /* No INT1 client: IMU still runs on today's 10 ms polling. */
        add_error = 1;
        assert(imu_service_init() == E_OK && int1_fn == NULL);
        assert(imu_service_start() == E_OK);
        for (unsigned int i = 0u; i < 5u; ++i) {
            clock_us += 10000u;
            tick();
        }
        assert(wakes == 5u);
        return 0;
    }
    assert(imu_service_init() == E_OK);
    assert(init_calls == 1u && int1_pins == (1u << IMU_INT1_PIN) && int1_fn);
    /* Edges before the task exists wake nothing. */
    int1_fn(IMU_INT1_PIN, DRV_GPIO_EDGE_RISE, NULL);
    assert(wakes == 0u);
    assert(imu_service_start() == E_OK);
    imu_wake_t w;
    assert(imu_int1_stats_read(&w) == E_OK && w.rises == 1u);
    assert(run_once() == 1u); /* First sample after start, as today. */

    /* Rising edge wakes; falling edge does not. */
    clock_us = 100000u;
    int1_fn(IMU_INT1_PIN, DRV_GPIO_EDGE_RISE, NULL);
    assert(wakes == 1u);
    int1_fn(IMU_INT1_PIN, DRV_GPIO_EDGE_FALL, NULL);
    assert(wakes == 1u);
    /* Cyclic within 12 ms of the edge: no wake. After: fallback wake. */
    clock_us = 100000u + IMU_WAKE_FALLBACK_US;
    tick();
    assert(wakes == 1u);
    clock_us += 1u;
    tick();
    assert(wakes == 2u);
    assert(run_once() == 1u);

    /* A HIGH level after the sample gives exactly ONE extra pass. */
    int1_fn(IMU_INT1_PIN, DRV_GPIO_EDGE_RISE, NULL);
    level_high = true;
    assert(run_once() == 1u); /* The read drains: no extra pass. */
    assert(imu_int1_stats_read(&w) == E_OK && w.extra == 0u);
    /* A probe that keeps INT1 high (stuck) is bounded to one extra. */
    stuck = true;
    assert(run_once() == 2u);
    stuck = false;
    /* Not ready (fault/retry): no extra pass even if INT1 is high. */
    probe_ready = false;
    assert(run_once() == 1u);
    probe_ready = true;
    /* Empty FIFO counts histogram bin 0. */
    empty = true;
    assert(run_once() == 1u);
    empty = false;
    assert(imu_int1_stats_read(&w) == E_OK);
    assert(w.batches[0] == 1u && w.batches[4] == 6u && w.extra == 1u);
    assert(w.rises == 3u && w.falls == 1u && w.fallbacks == 1u);
    assert(w.isr_lost == 0u);
    /* A refused wakeup from the ISR is counted, not retried. */
    wup_result = E_OBJ;
    int1_fn(IMU_INT1_PIN, DRV_GPIO_EDGE_RISE, NULL);
    assert(imu_int1_stats_read(&w) == E_OK && w.isr_lost == 1u);
    wup_result = E_OK;
    assert(imu_int1_stats_read(NULL) == E_PAR);
    return 0;
}
