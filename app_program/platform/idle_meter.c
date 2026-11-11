/*
 * idle_meter.c - CPU load meters, one per core. See idle_meter.h.
 */
#if TM_MQTT
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include <tk/syslib.h>
#include "app_task.h"
#include "cpu_load.h"
#include "drv_time.h"
#include "idle_meter.h"

/* Lowest priority of this kernel build (CNF_MAX_TSKPRI). */
#define IDLE_METER_PRIORITY 32
/*
 * One pass of the idle task runs the loop for this long, then repeats.  Kept well
 * below the 1 Hz Health sample: the task publishes its count at the end of each
 * pass, so this value is also the maximum age of a published count.  At 1 s the
 * reader could sample twice inside one pass and see a zero delta, which the load
 * arithmetic would report as a fully busy core.
 */
#define IDLE_METER_PASS_US 100000u
/* One meter per core: index 0 is core 0, index 1 is core 1 (SMP builds only). */
#define IDLE_METERS 2u

static volatile uint32_t idle_counter[IDLE_METERS];
/* Cross-core view of each counter, published once per pass.  The hot loop's
 * volatile counter is not a snapshot, so a reader on the other core must never
 * touch it; it reads this aligned word with an atomic load instead. */
static uint32_t idle_published[IDLE_METERS];
static cpu_load_t meter[IDLE_METERS];
static bool calibrated[IDLE_METERS];

/*
 * The counting loop. Calibration and the idle task both run exactly this
 * function, so one iteration costs the same cycles in both.
 */
static void __attribute__((noinline, noclone))
idle_meter_loop(volatile uint32_t *counter, uint32_t window_us)
{
    const uint32_t start = drv_time_us();

    while ((uint32_t)(drv_time_us() - start) < window_us) {
        *counter = *counter + 1u;
    }
}

/*
 * Calibrate one meter. The RP2040's two cores run the same clock, so the rate
 * measured here is the peak rate for either core; a per-core clock difference
 * would show up as a constant load offset, which is why these figures are
 * diagnostics rather than control inputs.
 */
static int calibrate_one(unsigned int index)
{
    volatile uint32_t count = 0u;
    UINT irq;

    if (index >= IDLE_METERS) {
        return (int)E_PAR;
    }
    if (!drv_time_ready()) {
        return (int)E_SYS;
    }
    DI(irq);
    const uint32_t start = drv_time_us();

    idle_meter_loop(&count, IDLE_METER_CALIBRATION_US);
    const uint32_t elapsed = drv_time_us() - start;

    EI(irq);
    cpu_load_init(&meter[index], count, elapsed);
    calibrated[index] = (meter[index].peak_rate != 0u);
    return calibrated[index] ? (int)E_OK : (int)E_SYS;
}

int idle_meter_calibrate(void)
{
    return calibrate_one(0u);
}

/*
 * Count forever on one meter, publishing a coherent count at the end of each pass.
 * The publication is deliberately outside the hot loop (RT-0061 finding 1): an
 * atomic store per iteration would cost more than the measurement is worth, and
 * the reader only needs a recent value, not the current one.
 */
static void idle_meter_body(unsigned int index)
{
    for (;;) {
        idle_meter_loop(&idle_counter[index], IDLE_METER_PASS_US);
        __atomic_store_n(&idle_published[index], idle_counter[index],
                         __ATOMIC_SEQ_CST);
    }
}

/* Task body: core-0 meter. */
static void idle_meter_entry(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    idle_meter_body(0u);
}

#if TK_SUPPORT_SMP
/* Task body: core-1 meter. */
static void idle_meter1_entry(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    idle_meter_body(1u);
}
#endif

int idle_meter_start(void)
{
    if (!calibrated[0]) {
        return (int)E_OBJ; /* no calibration: do not spin for nothing */
    }
    return (int)app_task_start((FP)idle_meter_entry, IDLE_METER_PRIORITY,
                               app_prc_core0(), "idle_meter");
}

#if TK_SUPPORT_SMP
/*
 * Phase 5: start the core-1 meter. Only in a two-core build, because in a
 * single-core build the second counting task would land on core 0 as well and
 * corrupt the core-0 load reading that the heartbeat publishes.
 */
int idle_meter_start_core1(void)
{
    if (!calibrated[1]) {
        const int er = calibrate_one(1u);
        if (er != (int)E_OK) {
            return er;
        }
    }
    return (int)app_task_start((FP)idle_meter1_entry, IDLE_METER_PRIORITY,
                               app_prc_core1(), "idle_meter_1");
}
#endif

/*
 * Take one load sample from the published count.
 *
 * RT-0061 finding 1: the reader never touches the hot counter.  If no new
 * publication has arrived since the previous sample, report UNKNOWN rather than
 * let the arithmetic turn a zero delta into a false "fully busy".  That also
 * preserves the documented first-sample rule, because a fresh meter has
 * last_count 0 and reports UNKNOWN whichever way the first call lands.
 */
static uint16_t sample_published(unsigned int index)
{
    if (!calibrated[index]) {
        return CPU_LOAD_UNKNOWN;
    }
    const uint32_t count =
        __atomic_load_n(&idle_published[index], __ATOMIC_SEQ_CST);
    if (count == meter[index].last_count) {
        return CPU_LOAD_UNKNOWN;
    }
    return cpu_load_sample(&meter[index], count, drv_time_us());
}

uint16_t idle_meter_load_permille(void)
{
    return sample_published(0u);
}

#if TK_SUPPORT_SMP
uint16_t idle_meter_load_permille_core1(void)
{
    return sample_published(1u);
}
#endif

uint32_t idle_meter_peak_rate(void)
{
    return meter[0].peak_rate;
}
#endif
