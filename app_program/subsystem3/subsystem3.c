#include "subsystem3.h"
#include "line_service.h"
EXPORT ER subsystem3_init(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)line_service_init();
#endif
}
EXPORT ER subsystem3_start(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)line_service_start();
#endif
}
