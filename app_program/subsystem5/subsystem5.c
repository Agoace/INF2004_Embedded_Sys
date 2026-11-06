#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT ER subsystem5_init(void) { return (ER)drv_ultrasonic_init(); }
EXPORT ER subsystem5_start(void) { return E_OK; }
