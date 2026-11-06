#include "subsystem3.h"
#include "drv_line.h"

EXPORT ER subsystem3_init(void) { return (ER)drv_line_init(); }
EXPORT ER subsystem3_start(void) { return E_OK; }
