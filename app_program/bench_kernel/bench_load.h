/* Bench idle meter: the robot meter is MQTT-only, the bench is radio-free. */
#ifndef BENCH_KERNEL_LOAD_H
#define BENCH_KERNEL_LOAD_H

/* Calibrate the core-0 meter and start its priority-32 task (task context,
 * once, before the window). A failure leaves [load] at 65535 (unknown). */
void bk_load_start(void);

#endif
