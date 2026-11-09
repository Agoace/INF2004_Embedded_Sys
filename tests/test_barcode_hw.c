#include "../app_program/subsystem3/barcode_hw.h"
#include <assert.h>
#include <stdbool.h>

static unsigned int initialized;
static unsigned int inputs;
static unsigned int pulls;
static bool down[2];

static unsigned int bit(unsigned int pin)
{
    assert(pin == 20u || pin == 21u); /* In particular, never touch GP7. */
    return 1u << (pin - 20u);
}

void gpio_init(unsigned int pin)
{
    initialized |= bit(pin);
}

void gpio_set_dir(unsigned int pin, bool output)
{
    assert(!output);
    inputs |= bit(pin);
}

void gpio_pull_up(unsigned int pin)
{
    pulls |= bit(pin);
}

bool gpio_get(unsigned int pin)
{
    (void)bit(pin);
    return !down[pin - 20u];
}

int main(void)
{
    barcode_hw_button_init();
    assert(initialized == 3u && inputs == 3u && pulls == 3u);
    assert(!barcode_hw_button_down() && !barcode_hw_drive_down());
    down[0] = true;
    assert(barcode_hw_button_down() && !barcode_hw_drive_down());
    down[1] = true;
    assert(barcode_hw_drive_down());
    return 0;
}
