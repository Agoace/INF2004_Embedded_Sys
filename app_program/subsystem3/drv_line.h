#ifndef DRV_LINE_H
#define DRV_LINE_H
#include <stdbool.h>
#include <stdint.h>
/* Core 0 only. TIMER alarm 0 / IRQ0, priority 2, 1 ms period. */
/* raw = L/ADC1, C/ADC0, barcode/ADC2; never pass barcode into lp_update. */
typedef struct { uint32_t us; uint16_t raw[3]; } drv_line_sample_t;
typedef struct { uint32_t samples, missed, dropped, isr_us, isr_max_us; uint16_t min[3], max[3]; } drv_line_stats_t;
int drv_line_init(void);
bool drv_line_take(drv_line_sample_t *sample);
void drv_line_flush(void);
void drv_line_listen(int task_id); /* ISR wake after every acquisition, core 0 */
void drv_line_stats(drv_line_stats_t *stats);
#endif
