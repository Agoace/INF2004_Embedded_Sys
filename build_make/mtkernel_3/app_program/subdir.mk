################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Application-owned SDK hardware headers and selected small sources.
include app_sdk_hw.mk

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
# Replay instrumentation must not add empty objects to ordinary links.
# TEST runner (1) and ROBOT synthetic profile each link only their reporter.
ifeq ($(IMU_SAFETY_BENCH),1)
TEMP_SRCS := $(filter-out ../app_program/imu_safety_bench/robot.c,$(TEMP_SRCS))
else ifeq ($(IMU_SAFETY_BENCH),robot_synthetic)
TEMP_SRCS := $(filter-out ../app_program/imu_safety_bench/bench.c,$(TEMP_SRCS))
else
TEMP_SRCS := $(filter-out ../app_program/imu_safety_bench/%.c,$(TEMP_SRCS))
endif
ifneq ($(IMU_SAFETY_CAPTURE),1)
TEMP_SRCS := $(filter-out ../app_program/imu_safety_capture/%.c,$(TEMP_SRCS))
endif
ifneq ($(BARCODE_LIVE_ENABLE),1)
TEMP_SRCS := $(filter-out ../app_program/barcode_live_vehicle/%.c,$(TEMP_SRCS))
endif
# Empty bench objects can still change veneer ordering and ordinary ELF bytes.
ifneq ($(BENCH_KERNEL),1)
TEMP_SRCS := $(filter-out ../app_program/bench_kernel/%.c,$(TEMP_SRCS))
else
TEMP_SRCS := $(filter-out ../app_program/platform/drv_gpio_irq_hw.c \
    ../app_program/subsystem2/drv_motor_hw.c \
    ../app_program/subsystem3/drv_line_hw.c \
    ../app_program/subsystem4/imu_hw.c \
    ../app_program/subsystem5/drv_ultrasonic_hw.c,$(TEMP_SRCS))
endif
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
