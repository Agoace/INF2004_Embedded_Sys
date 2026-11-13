/* Only drv_line.o aliases its time call. All values remain real TIMER reads. */
#include "../platform/drv_time.h"
#include "pil_gate1_runtime.h"
static uint32_t last_line_time;
uint32_t pil_gate1_line_time_us(void)
{
    const uint32_t now = drv_time_us();
    last_line_time = now;
    return now;
}
uint32_t pil_gate1_native_entry(void) { return last_line_time; }
