/* Forced include ONLY on app_main.o of the explicit ROBOT synthetic
 * profile. The full robot app starts as usual; the reporter task starts
 * after the Vehicle task. */
#ifndef IMU_SAFETY_BENCH_ROBOT_IMPL_H
#define IMU_SAFETY_BENCH_ROBOT_IMPL_H
#if !IMU_SAFETY_BENCH || !IMU_SAFETY_BENCH_ROBOT || APP_TEST || APP_BENCH
#error "IMU_SAFETY_BENCH robot hook requires the ROBOT synthetic profile"
#endif
int imu_bench_robot_start(void);
#define vehicle_task_start imu_bench_robot_start
#endif
