# Application-owned Pico SDK hardware subset (SDK 2.3.0, RP2040).
# Included by mtkernel_3/app_program/subdir.mk, NOT by kernel sources.
# Exhaustive added header paths are APP_SDK_HW_DIRS below, relative to
# PICO_SDK_PATH. They cover PWM/ADC/timer, GPIO/resets/clocks/base, their
# platform support and header-only IRQ/sync declarations. Header availability
# does NOT grant ownership of SDK IRQ, spinlock, timer/alarm or clock APIs.
# Sources: src/rp2_common/hardware_adc/adc.c always; hardware_gpio/gpio.c only
# for WIFI=none (WIFI=cyw43 already builds it as mtkernel_3/cyw43/gpio.o).
# PWM/resets are inline: no .c exists. Timer/clocks .c are NOT compiled.
# GPIO IRQ helper sections are discarded: the kernel owns GPIO IRQ13.
# No pico_runtime, crt0, stdlib, irq.c, sync.c, multicore or time init.
# Fixed clk_sys=125 MHz replaces clock_get_hz (needs SDK clocks state).
# Per-function/data sections and GC are required even for WIFI=none, so
# unused GPIO IRQ helpers do not pull SDK handlers/runtime into the image.
# SDK paths are system headers: SDK 2.3.0 GPIO/timer inline headers have
# conversion/unused-parameter warnings under our strict application flags.
# Application diagnostics remain enabled; no kernel file is changed.

ifeq ($(strip $(PICO_SDK_PATH)),)
$(error PICO_SDK_PATH is required by the application hardware adapters)
endif
APP_SDK_HW_DIRS := \
    src/common/pico_base_headers/include \
    src/common/pico_binary_info/include \
    src/common/pico_util/include \
    src/rp2040/pico_platform/include \
    src/rp2_common/pico_platform_compiler/include \
    src/rp2_common/pico_platform_sections/include \
    src/rp2_common/pico_platform_panic/include \
    src/rp2_common/pico_platform_common/include \
    src/rp2_common/hardware_base/include \
    src/rp2_common/hardware_gpio/include \
    src/rp2_common/hardware_pwm/include \
    src/rp2_common/hardware_adc/include \
    src/rp2_common/hardware_timer/include \
    src/rp2_common/hardware_resets/include \
    src/rp2_common/hardware_clocks/include \
    src/rp2_common/hardware_irq/include \
    src/rp2_common/hardware_sync/include \
    src/rp2_common/hardware_sync_spin_lock/include \
    src/rp2040/hardware_regs/include \
    src/rp2040/hardware_structs/include
CFLAGS += $(addprefix -isystem"$(PICO_SDK_PATH)/,$(addsuffix ",$(APP_SDK_HW_DIRS))) \
          -ffunction-sections -fdata-sections
LFLAGS += -Wl,--gc-sections

APP_SDK_HW_NAMES := adc
ifeq ($(WIFI),none)
APP_SDK_HW_NAMES += gpio
endif
APP_SDK_HW_OBJS := $(addprefix ./mtkernel_3/app_sdk_hw/,$(addsuffix .o,$(APP_SDK_HW_NAMES)))
OBJS += $(APP_SDK_HW_OBJS)
C_DEPS += $(APP_SDK_HW_OBJS:.o=.d)
define APP_SDK_HW_RULE
mtkernel_3/app_sdk_hw/$(1).o: $(PICO_SDK_PATH)/src/rp2_common/hardware_$(1)/$(1).c
	@echo 'Building app SDK hardware: $$<'
	@mkdir -p "$$(@D)"
	$$(GCC) $$(CFLAGS) -D$$(TARGET) $$(INCPATH) -MF"$$(@:%.o=%.d)" -MT"$$(@)" -c -o "$$@" "$$<"
	@echo 'Finished building: $$<'
endef
$(foreach name,$(APP_SDK_HW_NAMES),$(eval $(call APP_SDK_HW_RULE,$(name))))
