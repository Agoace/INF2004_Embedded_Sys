#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H
#include "ultrasonic.h"
typedef struct { uint32_t rise_latency_us, pulse_us, distance_mm; } ultrasonic_debug_t;
/* Core-0 task API; echo ISR shares GPIO13 with encoders. */
int drv_ultrasonic_init(void);
bool drv_ultrasonic_trigger(void);
bool drv_ultrasonic_service(void);
bool drv_ultrasonic_busy(void);
ultrasonic_result_t drv_ultrasonic_latest(void);
ultrasonic_debug_t drv_ultrasonic_debug_latest(void);
#endif
