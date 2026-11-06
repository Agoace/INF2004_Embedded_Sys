#ifndef DRV_LINE_H
#define DRV_LINE_H
#include <stdbool.h>
#include <stdint.h>
/* Core 0 only. TIMER alarm 0 / IRQ0, priority 2, 1 ms period. */
typedef struct { uint32_t us; uint16_t raw[3]; } drv_line_sample_t;
typedef struct { uint32_t samples, missed, dropped; uint16_t min[2], max[2]; } drv_line_stats_t;
int drv_line_init(void);
bool drv_line_take(drv_line_sample_t *sample);
void drv_line_flush(void);
void drv_line_stats(drv_line_stats_t *stats);
#endif
