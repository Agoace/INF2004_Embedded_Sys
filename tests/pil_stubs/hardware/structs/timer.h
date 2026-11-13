#ifndef PIL_TEST_TIMER_H
#define PIL_TEST_TIMER_H
#include <stdint.h>
typedef struct {
    volatile uint32_t inte, alarm[4], timerawl, intf, intr;
} pil_test_timer_t;
extern pil_test_timer_t pil_test_timer;
#define timer_hw (&pil_test_timer)
#endif
