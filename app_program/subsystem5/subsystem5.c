#include "subsystem5.h"
#include "sonar_service.h"

EXPORT ER subsystem5_init(void) { return (ER)sonar_service_init(); }
EXPORT ER subsystem5_start(void) { return (ER)sonar_service_start(); }
