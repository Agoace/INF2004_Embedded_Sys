################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# Application-owned SDK hardware headers and selected small sources.
include app_sdk_hw.mk

# app_program/*.c plus one level of subfolders (one per subsystem)
TEMP_SRCS = $(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
ifeq ($(SIM),1)
# Explicit six-adapter boundary. Time/watchdog and native kernel logic stay real.
SIM_HW_SRCS := ../app_program/subsystem2/drv_motor_hw.c \
    ../app_program/subsystem3/drv_line_hw.c \
    ../app_program/subsystem3/barcode_hw.c \
    ../app_program/subsystem4/imu_hw.c \
    ../app_program/subsystem5/drv_ultrasonic_hw.c \
    ../app_program/platform/drv_gpio_irq_hw.c
TEMP_SRCS := $(filter-out $(SIM_HW_SRCS),$(TEMP_SRCS))
else
# Preserve the original source order, flags and names with SIM unset or zero.
TEMP_SRCS := $(filter-out ../app_program/%_sim.c \
    ../app_program/sim/%.c,$(TEMP_SRCS))
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
