# Included only for the explicit IMU_SAFETY_BENCH=1 profile.
ifneq ($(TEST):$(BENCH):$(SMP):$(WIFI):$(NET):$(MQTT):$(KERNEL_RAM),1:0:0:cyw43:lwip:1:1)
$(error IMU_SAFETY_BENCH=1 requires TEST=1 BENCH=0 SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1)
endif
ifneq ($(IMU_STREAM):$(BARCODE_CAPTURE):$(BENCH_KERNEL),0:0:0)
$(error IMU_SAFETY_BENCH=1 excludes IMU_STREAM BARCODE_CAPTURE BENCH_KERNEL)
endif
IMU_SAFETY_BENCH_SUFFIX := _imu_safety_bench
IMU_SAFETY_BENCH_OBJS := $(patsubst ../%.c,mtkernel_3/%.o,$(wildcard ../app_program/imu_safety_bench/*.c))
$(IMU_SAFETY_BENCH_OBJS) mtkernel_3/app_program/subsystem4/imu_service.o mtkernel_3/app_program/app_main.o: CFLAGS += -DIMU_SAFETY_BENCH=1
mtkernel_3/app_program/subsystem4/imu_service.o: CFLAGS += -include ../app_program/imu_safety_bench/service_impl.h
mtkernel_3/app_program/app_main.o: CFLAGS += -include ../app_program/imu_safety_bench/main_impl.h
