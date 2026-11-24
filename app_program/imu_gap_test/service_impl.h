/* Forced include ONLY on imu_service.o in an IMU_GAP_TEST=1 TEST build
 * (build_make/imu_gap_test.mk). The IMU task's single tk_slp_tsk call goes
 * to imu_gap_test_sleep (gap.c). No kernel header here: a forced include
 * runs before the kernel headers, which define size_t differently. */
#ifndef IMU_GAP_TEST_SERVICE_IMPL_H
#define IMU_GAP_TEST_SERVICE_IMPL_H
#if !IMU_GAP_TEST || !APP_TEST || APP_BENCH
#error "IMU_GAP_TEST service hook: TEST=1 BENCH=0 only"
#endif
/* The kernel prototype "ER tk_slp_tsk(TMO tmout)" also expands, so it
 * declares imu_gap_test_sleep with the kernel types (defined in gap.c). */
#define tk_slp_tsk(timeout) imu_gap_test_sleep(timeout)
#endif
