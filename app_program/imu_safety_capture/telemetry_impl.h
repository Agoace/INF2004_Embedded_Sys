/* Forced include ONLY on telemetry.o in an IMU_SAFETY_CAPTURE=1 ROBOT
 * build. The GP20 dump prints the frozen capture before the timeline. */
#ifndef IMU_SAFETY_CAPTURE_TELEMETRY_IMPL_H
#define IMU_SAFETY_CAPTURE_TELEMETRY_IMPL_H
#if !IMU_SAFETY_CAPTURE || APP_TEST || APP_BENCH
#error "IMU_SAFETY_CAPTURE telemetry hook: TEST=0 BENCH=0 ROBOT only"
#endif
#define rt_timeline_report isc_timeline_report
#endif
