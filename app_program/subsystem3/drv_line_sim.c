/* Stage 0: real TIMER/alarm0, unavailable ADC. No physical ADC access. */
#include "drv_line_hw.h"
#include "../sim/pil_stage0.h"
#include "hardware/structs/timer.h"

int drv_line_hw_init(void)
{
    timer_hw->inte |= 1u;
    return 0;
}

int drv_line_hw_read(uint16_t raw[3])
{
    (void)raw;
    return -1; /* No operational link: never invent a fresh sample. */
}

void drv_line_hw_alarm(uint32_t at_us)
{
    timer_hw->alarm[0] = at_us;
    if ((int32_t)(at_us - timer_hw->timerawl) <= 0) {
        timer_hw->intf |= 1u;
    }
}

void drv_line_hw_ack(void)
{
    timer_hw->intf &= ~1u;
    timer_hw->intr = 1u;
}
