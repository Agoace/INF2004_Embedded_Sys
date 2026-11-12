/* One-way boot handoff from usermain to application observers. */
#ifndef APP_STARTUP_H
#define APP_STARTUP_H

#include <stdbool.h>

/* Sole caller: usermain, after all init/start attempts and Vehicle startup.
 * This publishes completed startup writes, not successful hardware health.
 * Never reset the gate or reinitialize published guards at run time. */
void app_startup_complete_publish(void);

/* Acquire before application board/queue access from the early radio task.
 * False means return without application work; never wait for startup. */
bool app_startup_complete(void);

#endif
