/* Explicit replay profile only. Reuse the exclusive TEST runner branch;
 * never run ordinary tests, actuator commands or watchdog-reset tests. */
#ifndef IMU_SAFETY_BENCH_MAIN_IMPL_H
#define IMU_SAFETY_BENCH_MAIN_IMPL_H
#if !IMU_SAFETY_BENCH || !APP_TEST || APP_BENCH
#error "IMU_SAFETY_BENCH runner hook is TEST-only"
#endif
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "stats.h"
/* Only app_main receives this header. The service capture option stays OFF. */
#undef APP_IMU_STREAM
#define APP_IMU_STREAM 1
#define subsystem4_test imu_safety_bench_run
#endif
