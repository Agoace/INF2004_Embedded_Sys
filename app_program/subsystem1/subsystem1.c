#include "subsystem1.h"
#include "telemetry.h"

EXPORT ER subsystem1_init(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_init();
#endif
}
EXPORT ER subsystem1_start(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)telemetry_start();
#endif
}
