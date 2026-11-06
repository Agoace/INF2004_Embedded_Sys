#include "subsystem3.h"
#include "drv_line.h"
#include "../platform/drv_time.h"
#if APP_TEST
EXPORT void subsystem3_test(void) {
    drv_line_stats_t a, b;
    drv_line_flush(); drv_line_stats(&a);
    const uint32_t start = drv_time_us();
    (void)tk_dly_tsk(1000u);
    const uint32_t elapsed = drv_time_us() - start;
    drv_line_stats(&b); drv_line_flush();
    const uint32_t count = b.samples - a.samples;
    const uint32_t expected = elapsed / 1000u;
    const uint32_t diff = count > expected ? count - expected : expected - count;
    const bool ok = expected > 0u && diff * 100u <= expected && b.missed == a.missed &&
                    b.max[0] <= 4095u && b.max[1] <= 4095u &&
                    b.max[0] > b.min[0] && b.max[1] > b.min[1];
    tm_printf((UB*)"[TEST] S3-U01 %s: %lu samples / %lu us; L %u..%u C %u..%u missed %lu (move card for spread)\n",
              ok ? "PASS" : "FAIL", (unsigned long)count, (unsigned long)elapsed,
              (unsigned int)b.min[0], (unsigned int)b.max[0], (unsigned int)b.min[1],
              (unsigned int)b.max[1], (unsigned long)(b.missed - a.missed));
}
#endif
