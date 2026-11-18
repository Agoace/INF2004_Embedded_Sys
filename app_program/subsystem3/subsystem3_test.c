#include "subsystem3.h"
#include "drv_line.h"
#include "line_service.h"
#include "../platform/drv_time.h"
#if APP_TEST
EXPORT void subsystem3_test(void) {
    drv_line_stats_t a, b;
    line_snapshot_t first,last;
    (void)line_snapshot_read(&first,false);
    drv_line_stats(&a);
    const uint32_t start = drv_time_us();
    (void)tk_dly_tsk(1000u);
    const uint32_t elapsed = drv_time_us() - start;
    drv_line_stats(&b);
    (void)line_snapshot_read(&last,false);
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
    const uint32_t age=drv_time_us()-last.timestamp_us;
    const uint32_t processed=last.processed-first.processed;
    const uint32_t cpu=last.cpu_us-first.cpu_us;
    const uint32_t isr=b.isr_us-a.isr_us;
    const bool task_ok=last.valid && age<=10000u && processed>0u && b.dropped==a.dropped;
    tm_printf((UB*)"[TEST] S3-U02 %s: processed %lu age %lu us; task max/sample %lu us total %lu us (%lu per-mille elapsed upper bound); ISR max %lu us total %lu us (%lu per-mille); barcode raw %u range %u..%u\n",
        task_ok ? "PASS" : "FAIL",(unsigned long)processed,(unsigned long)age,
        (unsigned long)last.sample_max_us,(unsigned long)cpu,
        (unsigned long)(elapsed ? (uint64_t)cpu*1000u/elapsed : 0u),
        (unsigned long)b.isr_max_us,(unsigned long)isr,
        (unsigned long)(elapsed ? (uint64_t)isr*1000u/elapsed : 0u),
        (unsigned int)last.raw_bc,(unsigned int)b.min[2],(unsigned int)b.max[2]);
}
#endif
