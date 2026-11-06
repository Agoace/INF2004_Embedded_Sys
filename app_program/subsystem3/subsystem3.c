#include "subsystem3.h"
#include "line_service.h"

EXPORT ER subsystem3_init(void) { return (ER)line_service_init(); }
EXPORT ER subsystem3_start(void) { return (ER)line_service_start(); }
