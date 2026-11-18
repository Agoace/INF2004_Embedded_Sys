#include "subsystem5.h"
#include "sonar_service.h"
EXPORT ER subsystem5_init(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)sonar_service_init();
#endif
}
EXPORT ER subsystem5_start(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)sonar_service_start();
#endif
}
