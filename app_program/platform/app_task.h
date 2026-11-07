#ifndef APP_TASK_H
#define APP_TASK_H
#include <tk/tkernel.h>
#include <stdint.h>
/* Core-0 application tasks only. Stacks are static, 8-byte aligned, prefilled. */
ID app_task_start(FP entry, PRI priority, UW core, const char *name);
typedef struct { const char *name; uint32_t size, used, free; } app_stack_t;
unsigned int app_task_stacks(app_stack_t *out, unsigned int capacity);
void app_task_report(void);
#endif
