/* Local RP2040 startup helper. No SDK or kernel objects are required.
 * in_w/out_w, UW and XOSC_MHz come from the platform register interface. */
#ifndef PICO_STARTUP_TIMER_TICK_H
#define PICO_STARTUP_TIMER_TICK_H
#define RP2040_WATCHDOG_TICK_REG (0x4005802cUL)
#define RP2040_TICK_CYCLES       (0x1ffUL)
#define RP2040_TICK_ENABLE       (0x200UL)
#define RP2040_TICK_RUNNING      (0x400UL)
static inline int knl_startup_timer_tick(void)
{
    const UW config = (UW)(RP2040_TICK_ENABLE | XOSC_MHz);
    if (XOSC_MHz == 0 || XOSC_MHz > RP2040_TICK_CYCLES) return 0;
    if ((in_w(RP2040_WATCHDOG_TICK_REG) &
         (RP2040_TICK_ENABLE | RP2040_TICK_CYCLES)) != config)
        out_w(RP2040_WATCHDOG_TICK_REG, config);
    /* TIMER cannot bound a wait until its own source is running. */
    for (unsigned int retry = 0; retry < 100000u; ++retry)
        if ((in_w(RP2040_WATCHDOG_TICK_REG) & RP2040_TICK_RUNNING) != 0u)
            return 1;
    return 0;
}
#endif
