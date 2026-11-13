/* Stage 0: no physical trigger and no fabricated echo. */
#include "drv_ultrasonic_hw.h"
#include "../sim/pil_stage0.h"

void drv_ultrasonic_hw_init(void)
{
}

bool drv_ultrasonic_hw_echo(void)
{
    return false;
}

void drv_ultrasonic_hw_pulse(void)
{
    /* Native state machine times out: no operational simulated source. */
}
