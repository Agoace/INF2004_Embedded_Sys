# Motor controller

## μT-Kernel layer

### Cold-power timer source

Test every kernel image after a cold power-on, not only after a flash.
The RP2040 TIMER uses the watchdog tick generator. Kernel startup selects
12 MHz XOSC for `clk_ref`, but does not set `WATCHDOG_TICK`. At cold power-on
its reset value is `0x200`: enabled, CYCLES 0 (divide by 512). TIMER then
runs about 42.7 times too slowly. A picotool reboot can leave CYCLES 12
from boot ROM setup, hiding the defect. SysTick kernel time is independent.

`usermain()` calls `drv_time_init()` before any subsystem. The SDK-only
adapter calls runtime-free `tick_start(TICK_WATCHDOG, 12)`, where 12 is the
kernel `XOSC_MHz` value, and polls `WATCHDOG_TICK_RUNNING_BITS` with a bounded
CPU-iteration wait. SDK 2.3.0 `tick_is_running()` checks ENABLE only on
RP2040, so it is not sufficient here. Repeated initialization leaves an
already correct generator unchanged. No TIMER count is reset, and the
watchdog reset counter is not enabled. Failure returns `E_SYS` and the
startup `CHECK` prints an error.

`drv_time_check()` compares TIMER microseconds with elapsed `tk_get_otm()`
kernel milliseconds over a nominal 100 ms delay and rejects error greater
than 2%. It first waits for a kernel tick boundary to reduce 10 ms time
quantization. It measures the actual kernel interval; `tk_dly_tsk()` adds
one `TIMER_PERIOD`, so its argument is not the measured duration.
S0-U01 uses a nominal 1 s window and a 1% limit, printing both measurements;
see [the platform test log](../testing/subsystem1/TEST_LOG.md). The updated
image has not yet passed this cold-power hardware test.

The template Wi-Fi `time_us_64()` in
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c` reads the same TIMER
and has the same cold-power defect. The early application initialization
covers later Wi-Fi calls without changes to template kernel or library
files. It cannot correct any time read before `usermain()`.

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
src/rp2_common/hardware_ticks/include
src/rp2_common/hardware_watchdog/include
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
Timer and clocks .c files are not added. Source added:
`src/rp2_common/hardware_ticks/ticks.c`; its RP2040 `tick_start()` only
writes the tick register and needs no SDK runtime or watchdog .c file.

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

SDK directories use `-isystem`, not application `-I` paths. SDK 2.3.0 has
conversion warnings in unused GPIO 64-bit helpers and conversion/unused
parameter warnings in timer inline helpers with our strict flags. System
headers keep vendor diagnostics separate; the application still passes
`-Wall -Wextra -Wconversion -Wshadow -Wformat=2 -Werror` unchanged.

#### Repeat the link-only proof

The source is [`tests/probes/sdk_hw_link_probe.c`](../tests/probes/sdk_hw_link_probe.c).
It is not in the application wildcard and is never called. After a full
build, use [`scripts/check-sdk-hw-link.py`](../scripts/check-sdk-hw-link.py)
before changing the object profile:

```sh
cd build_make
make clean SMP=0 WIFI=none
make SMP=0 WIFI=none -j8 > /tmp/motion-up.log 2>&1
cd ..
python3 scripts/check-sdk-hw-link.py /tmp/motion-up.log --output-dir /tmp/sdk-proof-up
```

Repeat for `SMP=0 WIFI=none TEST=1` and the default SMP/radio profile.
The script rejects a log/profile mismatch, compiles the SDK-only probe
with strict flags, and retains it with `--undefined=app_sdk_hw_link_probe`
in a separate ELF. `adc_init`, GPIO setup, `tick_start` and the probe must be retained,
with no undefined symbols. No probe UF2 is made. Do not call or flash it.

The template radio profile already has a `clock_get_hz()` compatibility
function in `cyw43_utk_compat.c` that returns constant 125000000. This is
not SDK clocks runtime state. Neither the motor adapter nor the link
probe calls it; radio-off builds must not depend on that conditional shim.
The kernel already configures CLK_ADC from PLL USB to 48 MHz in
`startup_clock(CLKATR_USB | CLKATR_ADC | CLKATR_RTC | CLKATR_PREI)`.
This source check is not a measured ADC result; the line driver is not
implemented in this step.
