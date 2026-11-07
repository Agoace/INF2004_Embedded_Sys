/*
 * idle_meter.c - CPU load meter. See idle_meter.h.
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

static volatile uint32_t idle_counter;
static cpu_load_t meter;
static bool calibrated;

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

int idle_meter_calibrate(void)
{
    volatile uint32_t count = 0u;
    UINT irq;

    if (!drv_time_ready()) {
        return (int)E_SYS;
    }
    DI(irq);
    const uint32_t start = drv_time_us();

    idle_meter_loop(&count, IDLE_METER_CALIBRATION_US);
    const uint32_t elapsed = drv_time_us() - start;

    EI(irq);
    cpu_load_init(&meter, count, elapsed);
    calibrated = (meter.peak_rate != 0u);
    return calibrated ? (int)E_OK : (int)E_SYS;
}

/* Task body: count forever. */
static void idle_meter_entry(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    for (;;) {
        idle_meter_loop(&idle_counter, IDLE_METER_PASS_US);
    }
}

int idle_meter_start(void)
{
    if (!calibrated) {
        return (int)E_OBJ; /* no calibration: do not spin for nothing */
    }
    return (int)app_task_start((FP)idle_meter_entry, IDLE_METER_PRIORITY,
                               TP_PRC1, "idle_meter");
}

uint16_t idle_meter_load_permille(void)
{
    if (!calibrated) {
        return CPU_LOAD_UNKNOWN;
    }
    return cpu_load_sample(&meter, idle_counter, drv_time_us());
}

uint32_t idle_meter_peak_rate(void)
{
    return meter.peak_rate;
}
#endif
