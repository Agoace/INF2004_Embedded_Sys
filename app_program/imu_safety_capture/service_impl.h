/* Forced include ONLY on imu_service.o in an IMU_SAFETY_CAPTURE=1 ROBOT
 * build. The detector call is wrapped; detector math, thresholds and the
 * publish order are unchanged. */
#ifndef IMU_SAFETY_CAPTURE_SERVICE_IMPL_H
#define IMU_SAFETY_CAPTURE_SERVICE_IMPL_H
#if !IMU_SAFETY_CAPTURE || APP_TEST || APP_BENCH
#error "IMU_SAFETY_CAPTURE service hook: TEST=0 BENCH=0 ROBOT only"
#endif
#define imu_safety_step isc_safety_step
#endif
