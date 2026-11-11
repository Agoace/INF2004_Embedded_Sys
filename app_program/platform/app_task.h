/* app_task.h - static-stack application task creation and stack report. */
#ifndef APP_TASK_H
#define APP_TASK_H
#include <tk/tkernel.h>
#include <stdint.h>
/* Registered application tasks.  Stacks are static, 8-byte aligned, prefilled.
   The core passed to app_task_start() must agree with the placement table in
   app_task.c, which owns where each task is allowed to run. */
ID app_task_start(FP entry, PRI priority, UW core, const char *name);
typedef struct { const char *name; uint32_t size, used, free; uint8_t core; } app_stack_t;
unsigned int app_task_stacks(app_stack_t *out, unsigned int capacity);
void app_task_report(void);
#endif
