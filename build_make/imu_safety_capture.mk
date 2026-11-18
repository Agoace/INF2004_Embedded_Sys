# Included only for IMU_SAFETY_CAPTURE=1: the ROBOT app with a RAM capture
# of detector observations around the first IMU safety latch edge. The
# hooks are forced includes on imu_service.o, telemetry.o and
# vehicle_task.o only, so the
# capture objects are absent from every other profile.
ifneq ($(TEST):$(BENCH),0:0)
$(error IMU_SAFETY_CAPTURE=1 is ROBOT only; TEST=0 BENCH=0 required)
endif
ifneq ($(IMU_SAFETY_BENCH):$(IMU_STREAM),0:0)
$(error IMU_SAFETY_CAPTURE=1 excludes IMU_SAFETY_BENCH and IMU_STREAM)
endif
IMU_SAFETY_CAPTURE_OBJS := $(patsubst ../%.c,mtkernel_3/%.o,$(wildcard ../app_program/imu_safety_capture/*.c))
$(IMU_SAFETY_CAPTURE_OBJS) mtkernel_3/app_program/subsystem4/imu_service.o mtkernel_3/app_program/subsystem1/telemetry.o mtkernel_3/app_program/vehicle/vehicle_task.o: CFLAGS += -DIMU_SAFETY_CAPTURE=1
mtkernel_3/app_program/subsystem4/imu_service.o: CFLAGS += -include ../app_program/imu_safety_capture/service_impl.h
mtkernel_3/app_program/subsystem1/telemetry.o: CFLAGS += -include ../app_program/imu_safety_capture/telemetry_impl.h
mtkernel_3/app_program/vehicle/vehicle_task.o: CFLAGS += -include ../app_program/imu_safety_capture/vehicle_impl.h
