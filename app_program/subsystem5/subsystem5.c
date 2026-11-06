#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT ER subsystem5_init(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)drv_ultrasonic_init();
#endif
}
EXPORT ER subsystem5_start(void) { return E_OK; }
