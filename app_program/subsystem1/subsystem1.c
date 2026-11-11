/* subsystem1.c - subsystem 1 (telemetry, MQTT queues and health) init/start. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem1.h"
#include "telemetry.h"
#if TM_MQTT
#include "robot_network.h"
#include "robot_health.h"
#include "../platform/idle_meter.h"
#endif

/* Create queues, health cyclic and telemetry objects. Network is optional. */
EXPORT ER subsystem1_init(void)
{
#if TM_MQTT
    /* Network failures are diagnostic only; never gate driving startup. */
    CHECK(robot_network_init(), "network queues (noncritical)");
    CHECK(robot_health_init(), "health cyclic (noncritical)");
#endif
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_init();
#endif
}

/* Start the health task and telemetry task. Network is optional. */
EXPORT ER subsystem1_start(void)
{
#if TM_MQTT
    CHECK(robot_health_start(), "health task (noncritical)");
    CHECK(idle_meter_start(), "idle meter (noncritical)");
#if TK_SUPPORT_SMP
    /* Phase 5: the core-1 meter. Only in a two-core build; on one core a second
     * counting task would run beside the first and corrupt the core-0 figure. */
    CHECK(idle_meter_start_core1(), "idle meter core 1 (noncritical)");
#endif
#endif
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_start();
#endif
}
