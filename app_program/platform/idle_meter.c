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
/* One pass of the idle task runs the loop for this long, then repeats. */
#define IDLE_METER_PASS_US 1000000u
/* One meter per core: index 0 is core 0, index 1 is core 1 (SMP builds only). */
#define IDLE_METERS 2u

static volatile uint32_t idle_counter[IDLE_METERS];
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

/* Count forever on one meter. */
static void idle_meter_body(unsigned int index)
{
    for (;;) {
        idle_meter_loop(&idle_counter[index], IDLE_METER_PASS_US);
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

uint16_t idle_meter_load_permille(void)
{
    if (!calibrated[0]) {
        return CPU_LOAD_UNKNOWN;
    }
    return cpu_load_sample(&meter[0], idle_counter[0], drv_time_us());
}

#if TK_SUPPORT_SMP
uint16_t idle_meter_load_permille_core1(void)
{
    if (!calibrated[1]) {
        return CPU_LOAD_UNKNOWN;
    }
    return cpu_load_sample(&meter[1], idle_counter[1], drv_time_us());
}
#endif

uint32_t idle_meter_peak_rate(void)
{
    return meter[0].peak_rate;
}
#endif
