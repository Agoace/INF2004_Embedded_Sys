#include "subsystem3.h"
#include "drv_line.h"

EXPORT ER subsystem3_init(void) {
#if APP_BENCH && !APP_TEST
    return E_OK;
#else
    return (ER)drv_line_init();
#endif
}
EXPORT ER subsystem3_start(void) { return E_OK; }
