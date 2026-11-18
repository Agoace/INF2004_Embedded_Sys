#ifndef HOST_WATCHDOG_H
#define HOST_WATCHDOG_H
#include <stdbool.h>
#include <stdint.h>
/* RP2040 register bits used by the app; real SDK headers are tested on ARM. */
#define WATCHDOG_TICK_CYCLES_BITS 0x1ffu
#define WATCHDOG_TICK_ENABLE_BITS 0x200u
#define WATCHDOG_TICK_RUNNING_BITS 0x400u
#define WATCHDOG_CTRL_ENABLE_BITS 0x40000000u
#define WATCHDOG_REASON_TIMER_BITS 1u
#define WATCHDOG_REASON_FORCE_BITS 2u
#define HOST_WATCHDOG_MAGIC 0x6ab73121u
typedef struct { uint32_t ctrl, load, reason, scratch[8], tick; } host_watchdog_t;
extern host_watchdog_t host_watchdog;
#define watchdog_hw (&host_watchdog)
void watchdog_enable(uint32_t ms, bool pause);
void watchdog_update(void);
bool watchdog_caused_reboot(void);
bool watchdog_enable_caused_reboot(void);
#endif
