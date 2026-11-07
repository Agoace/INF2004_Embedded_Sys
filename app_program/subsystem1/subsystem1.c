#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem1.h"
#include "telemetry.h"
#if TM_MQTT
#include "robot_network.h"
#include "robot_health.h"
#endif

EXPORT ER subsystem1_init(void) {
#if TM_MQTT
    /* Network failures are diagnostic only; never gate driving startup. */
    CHECK(robot_network_init(),"network queues (noncritical)");
    CHECK(robot_health_init(),"health cyclic (noncritical)");
#endif
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_init();
#endif
}
EXPORT ER subsystem1_start(void) {
#if TM_MQTT
    CHECK(robot_health_start(),"health task (noncritical)");
#endif
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_start();
#endif
}
