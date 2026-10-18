#include <stdbool.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

// GPIOs exposed on the Pico W headers. Internal wireless and power pins are
// intentionally excluded.
static const uint8_t probe_pins[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
    26, 27, 28,
};

static bool pin_is_likely_connected(uint pin, bool *level) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);

    // A floating pin follows both weak internal pulls. A connected pin usually
    // stays at the level imposed by the external circuit.
    gpio_pull_down(pin);
    sleep_us(100);
    const bool with_pull_down = gpio_get(pin);

    gpio_pull_up(pin);
    sleep_us(100);
    const bool with_pull_up = gpio_get(pin);

    gpio_disable_pulls(pin);

    if (with_pull_down == with_pull_up) {
        *level = with_pull_up;
        return true;
    }

    return false;
}

int main(void) {
    stdio_init_all();

    // Give the USB serial device time to enumerate after boot.
    sleep_ms(2000);

    printf("\nPico W connection probe\n");
    printf("Inputs only; unknown pins are never driven high or low.\n");

    while (true) {
        unsigned int connected_count = 0;

        printf("\n--- scan ---\n");
        for (size_t index = 0; index < count_of(probe_pins); ++index) {
            const uint pin = probe_pins[index];
            bool level = false;

            if (pin_is_likely_connected(pin, &level)) {
                printf("GPIO %-2u: likely connected, held %s\n",
                       pin, level ? "HIGH" : "LOW");
                ++connected_count;
            }
        }

        if (connected_count == 0) {
            printf("No externally held GPIOs detected.\n");
        } else {
            printf("Detected %u likely connected GPIO(s).\n", connected_count);
        }

        printf("Pins that follow both weak pulls are treated as open.\n");
        sleep_ms(2000);
    }
}
