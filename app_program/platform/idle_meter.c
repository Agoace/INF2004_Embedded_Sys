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

/* A sample interval shorter than this cannot be interpreted: the rate would be
 * dominated by scheduling jitter.  Health samples at 1 Hz, so this is half a
 * period and only rejects a duplicate or an early read. */
#define IDLE_METER_MIN_INTERVAL_US 500000u

/* Bounded read-only snapshot of the latest two loads.  Written only by
 * idle_meter_sample_all() and read by any diagnostic.  Each value is an aligned
 * 16-bit word, so a reader can never see a torn figure, though the pair may come
 * from different samples when read mid-update. */
static uint16_t load_pub[IDLE_METERS];

/*
 * Advance BOTH sampling states and publish the result.
 *
 * RT-0069/C2: cpu_load_sample() mutates the meter it is given, so it must have
 * exactly one caller.  Health is that caller, at 1 Hz; diagnostics such as
 * app_task_report() read the published values and never touch the sampling state.
 * The previous version let two cores sample, which raced and let a dump reset the
 * Health interval.
 *
 * RT-0069/C3: after a valid baseline, an unchanged idle count across a full
 * interval means the core did no idle work at all, which is 1000 permille - not
 * UNKNOWN.  cpu_load_sample() reports exactly that for a zero delta.  UNKNOWN is
 * reserved for no calibration, the first sample, and an interval too short to
 * interpret.
 */
void idle_meter_sample_all(void)
{
    const uint32_t now = drv_time_us();
    for (unsigned int i = 0u; i < IDLE_METERS; ++i) {
        if (!calibrated[i]) {
            load_pub[i] = CPU_LOAD_UNKNOWN;
            continue;
        }
        const uint32_t count =
            __atomic_load_n(&idle_published[i], __ATOMIC_SEQ_CST);
        if (!meter[i].have_last) {
            (void)cpu_load_sample(&meter[i], count, now);   /* baseline only */
            load_pub[i] = CPU_LOAD_UNKNOWN;
            continue;
        }
        if ((uint32_t)(now - meter[i].last_us) < IDLE_METER_MIN_INTERVAL_US) {
            load_pub[i] = CPU_LOAD_UNKNOWN;
            continue;
        }
        load_pub[i] = cpu_load_sample(&meter[i], count, now);
    }
}

uint16_t idle_meter_load_permille(void)
{
    return load_pub[0];
}

#if TK_SUPPORT_SMP
uint16_t idle_meter_load_permille_core1(void)
{
    return load_pub[1];
}
#endif

uint32_t idle_meter_peak_rate(void)
{
    return meter[0].peak_rate;
}
#endif
