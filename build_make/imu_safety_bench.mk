# Included only for an explicit IMU_SAFETY_BENCH profile (1 or
# robot_synthetic). Both force-include the service seam on imu_service.o
# only; the physical probe is never initialized or read.
ifneq ($(SMP):$(WIFI):$(NET):$(MQTT):$(KERNEL_RAM),0:cyw43:lwip:1:1)
$(error IMU_SAFETY_BENCH=$(IMU_SAFETY_BENCH) requires SMP=0 WIFI=cyw43 NET=lwip MQTT=1 KERNEL_RAM=1)
endif
ifneq ($(IMU_STREAM):$(BARCODE_CAPTURE):$(BARCODE_FAIL_CAPTURE):$(BENCH_KERNEL),0:0:0:0)
$(error IMU_SAFETY_BENCH=$(IMU_SAFETY_BENCH) excludes IMU_STREAM BARCODE_CAPTURE BARCODE_FAIL_CAPTURE BENCH_KERNEL)
endif
ifeq ($(IMU_SAFETY_BENCH),1)
ifneq ($(TEST):$(BENCH),1:0)
$(error IMU_SAFETY_BENCH=1 requires TEST=1 BENCH=0)
endif
IMU_SAFETY_BENCH_SUFFIX := _imu_safety_bench
IMU_SAFETY_BENCH_ROBOT_FLAG := 0
IMU_SAFETY_BENCH_MAIN := main_impl.h
else
ifneq ($(TEST):$(BENCH),0:0)
$(error IMU_SAFETY_BENCH=robot_synthetic is the ROBOT app: TEST=0 BENCH=0 required)
endif
IMU_SAFETY_BENCH_SUFFIX := _imu_synth_robot
IMU_SAFETY_BENCH_ROBOT_FLAG := 1
IMU_SAFETY_BENCH_MAIN := robot_impl.h
endif
IMU_SAFETY_BENCH_OBJS := $(patsubst ../%.c,mtkernel_3/%.o,$(wildcard ../app_program/imu_safety_bench/*.c))
$(IMU_SAFETY_BENCH_OBJS) mtkernel_3/app_program/subsystem4/imu_service.o mtkernel_3/app_program/app_main.o: CFLAGS += -DIMU_SAFETY_BENCH=1 -DIMU_SAFETY_BENCH_ROBOT=$(IMU_SAFETY_BENCH_ROBOT_FLAG)
mtkernel_3/app_program/subsystem4/imu_service.o: CFLAGS += -include ../app_program/imu_safety_bench/service_impl.h
mtkernel_3/app_program/app_main.o: CFLAGS += -include ../app_program/imu_safety_bench/$(IMU_SAFETY_BENCH_MAIN)
