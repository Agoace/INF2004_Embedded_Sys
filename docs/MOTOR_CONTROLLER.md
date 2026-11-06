# Motor controller

## μT-Kernel layer

### SDK hardware subset

The app-owned [`build_make/app_sdk_hw.mk`](../build_make/app_sdk_hw.mk)
is included by [`the application build hook`](../build_make/mtkernel_3/app_program/subdir.mk).
No kernel source or template platform makefile changes. `PICO_SDK_PATH` is
required, also for UART/radio-off builds. SDK version: 2.3.0.

Every added include path, relative to `PICO_SDK_PATH`:

```text
src/common/pico_base_headers/include
src/common/pico_binary_info/include
src/common/pico_util/include
src/rp2040/pico_platform/include
src/rp2_common/pico_platform_compiler/include
src/rp2_common/pico_platform_sections/include
src/rp2_common/pico_platform_panic/include
src/rp2_common/pico_platform_common/include
src/rp2_common/hardware_base/include
src/rp2_common/hardware_gpio/include
src/rp2_common/hardware_pwm/include
src/rp2_common/hardware_adc/include
src/rp2_common/hardware_timer/include
src/rp2_common/hardware_resets/include
src/rp2_common/hardware_clocks/include
src/rp2_common/hardware_irq/include
src/rp2_common/hardware_sync/include
src/rp2_common/hardware_sync_spin_lock/include
src/rp2040/hardware_regs/include
src/rp2040/hardware_structs/include
```

Source added: `src/rp2_common/hardware_adc/adc.c`. It is compiled for the
step-2 proof only; no line driver calls it yet. Source added for
`WIFI=none`: `src/rp2_common/hardware_gpio/gpio.c`. For `WIFI=cyw43` the
fragment reuses the template's existing `mtkernel_3/cyw43/gpio.o`, not a
duplicate copy. No PWM/resets .c files exist: the selected APIs are inline.
Timer and clocks .c files are not added.

Per-function/data sections and linker garbage collection are enabled in
all profiles, also radio-off, so unused GPIO SDK interrupt helpers do not
pull SDK IRQ ownership or runtime into the image. PWM setup and duty
writes, GPIO setup/read, reset release and the inline
`timer_time_us_32()` need no SDK runtime. `adc_init()` calls only inline
reset/poll operations. A separate, never-called hardware probe retained at
link time can check these dependencies without running on hardware.

Do not add SDK `pico_runtime`, crt0, stdlib, IRQ, sync, multicore or time
initialization sources. IRQ/sync include paths provide declarations only.
Do not call GPIO SDK IRQ registration, timer alarm helpers,
`irq_set_exclusive_handler()`, or spinlock APIs: the kernel owns these.
`clock_get_hz()` needs SDK clocks runtime state and is not used. Kernel
PLL SYS is fixed at 125 MHz; the servo divider is `125.0f`, 1 us/count.
`reset_unreset_block_num_wait_blocking(RESET_PWM)` is inline and needs no
runtime; it releases PWM even with kernel `USE_PTMR=0`. Reset all PWM
only during the one S2 hardware initialization, before any PWM client
runs. The free-running hardware timer is read, not reinitialized;
64-bit time uses the stable high/low register pair, not SDK alarm/time
initialization. SDK-only adapters must not include `tk/` headers.
