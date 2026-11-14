#ifndef PIL_FAKE_WATCHDOG_H
#define PIL_FAKE_WATCHDOG_H
#include <stdint.h>
typedef struct { volatile uint32_t reason, scratch[8]; } watchdog_hw_t;
extern watchdog_hw_t pil_test_watchdog;
#define watchdog_hw (&pil_test_watchdog)
#endif
