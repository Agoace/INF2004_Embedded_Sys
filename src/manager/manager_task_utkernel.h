/**
 * @file manager_task_utkernel.h
 * @brief μT-Kernel wiring for the Manager (Line + Manager tasks).
 *
 * Compiled only when MANAGER_USE_UTKERNEL is defined, which the CMake option
 * USE_UTKERNEL_ADAPTER adds together with the external μT-Kernel port. The
 * bare-metal target and the host tests never include it.
 *
 * Startup order in usermain():
 *   1. motion_task_create()                  (Motion task, priority 10)
 *   2. manager_line_tasks_create()           (Line 11, Manager 12)
 *   3. tk_sta_ker() / the port's scheduler start
 *
 * Create Motion first: the Manager requests the encoder self-test from its
 * first step, so the Motion command queue must already exist.
 */
#ifndef MANAGER_TASK_UTKERNEL_H
#define MANAGER_TASK_UTKERNEL_H

#ifdef MANAGER_USE_UTKERNEL

#include <tk/tkernel.h>

/** Create the 1 ms Line task and the 1 ms Manager task. */
ER   manager_line_tasks_create(void);

/** Wake the Manager task early, e.g. after an event was posted. */
void manager_adapter_wake(void);

#endif /* MANAGER_USE_UTKERNEL */

#endif /* MANAGER_TASK_UTKERNEL_H */
