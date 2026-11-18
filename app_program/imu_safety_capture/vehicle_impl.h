/* Forced include ONLY on vehicle_task.o in an IMU_SAFETY_CAPTURE=1 ROBOT
 * build. Every Vehicle ESTOP (GP21, MQTT, line, IMU) also requests a
 * capture; a latch capture already running wins. */
#ifndef IMU_SAFETY_CAPTURE_VEHICLE_IMPL_H
#define IMU_SAFETY_CAPTURE_VEHICLE_IMPL_H
#if !IMU_SAFETY_CAPTURE || APP_TEST || APP_BENCH
#error "IMU_SAFETY_CAPTURE vehicle hook: TEST=0 BENCH=0 ROBOT only"
#endif
#define motion_estop isc_motion_estop
#endif
