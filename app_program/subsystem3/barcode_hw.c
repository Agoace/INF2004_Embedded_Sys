/* SDK-only GP20 input. GP7 belongs to IMU INT1 and is never touched here. */
#include "barcode_hw.h"
#if APP_BARCODE_CAPTURE
#include "hardware/gpio.h"

void barcode_hw_button_init(void)
{
    gpio_init(20u);
    gpio_set_dir(20u, GPIO_IN);
    gpio_pull_up(20u);
}

bool barcode_hw_button_down(void)
{
    return !gpio_get(20u);
}
#endif
