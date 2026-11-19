# Included only for IMU_GAP_TEST=1: a TEST image with ONE injected 30 ms
# IMU-task delay after a 10 s warm-up, for the option D bench check
# (docs/s4-imu/IMU.md). The hook is a forced include on imu_service.o only.
ifneq ($(TEST):$(BENCH):$(IMU_STREAM):$(SMP),1:0:0:0)
$(error IMU_GAP_TEST=1 requires TEST=1 BENCH=0 IMU_STREAM=0 SMP=0)
endif
mtkernel_3/app_program/imu_gap_test/gap.o mtkernel_3/app_program/subsystem4/imu_service.o: CFLAGS += -DIMU_GAP_TEST=1
mtkernel_3/app_program/subsystem4/imu_service.o: CFLAGS += -include ../app_program/imu_gap_test/service_impl.h
