/* Exclusive-profile buttons/USB state. Never configure the IMU GP7 wire. */
#include "barcode_hw.h"
#if APP_BARCODE_CAPTURE
#include "hardware/gpio.h"

/* libtm exports kernel UW (unsigned long). Keep kernel/SDK headers apart.
 * This reads the console task's cached state, not the TinyUSB stack. */
extern unsigned long tm_usb_state(void);

bool barcode_hw_usb_ready(void)
{
    return tm_usb_state() == 3UL; /* Configured host with CDC DTR asserted. */
}

void barcode_hw_button_init(void)
{
    gpio_init(20u);
    gpio_set_dir(20u, GPIO_IN);
    gpio_pull_up(20u);
    gpio_init(21u);
    gpio_set_dir(21u, GPIO_IN);
    gpio_pull_up(21u);
}

bool barcode_hw_button_down(void)
{
    return !gpio_get(20u);
}

bool barcode_hw_drive_down(void)
{
    return !gpio_get(21u);
}
#endif
