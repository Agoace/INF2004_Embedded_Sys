#include <stdio.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "barcode_interface.h"
#include "ir_module_pins.h"

#define BARCODE_SAMPLE_PERIOD_US 250u
#define BARCODE_DEBUG_INTERVAL_US 500000u

static const barcode_config_t BARCODE_CONFIG = {
    .black_level = true,
    /* 250 us polling gives at least three samples per accepted run. */
    .min_run_us = 750u,
    .quiet_zone_us = 5000u,
};

static const char *status_name(barcode_status_t status) {
    switch (status) {
        case BARCODE_PARTIAL: return "partial";
        case BARCODE_DECODE_FAIL: return "decode_fail";
        case BARCODE_TRUNCATED: return "truncated";
        default: return "no_barcode";
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("RoboCar Code 39 reader: DO GP7, black level=1\n");

    gpio_init(BARCODE_IR_DIGITAL_GPIO);
    gpio_set_dir(BARCODE_IR_DIGITAL_GPIO, GPIO_IN);
    const uint32_t start_us = time_us_32();
    barcode_init(&BARCODE_CONFIG, gpio_get(BARCODE_IR_DIGITAL_GPIO), start_us);

    uint32_t next_sample_us = start_us;
    uint32_t last_debug_us = start_us;
    uint32_t rejected = 0;
    while (true) {
        const uint32_t now_us = time_us_32();
        if ((int32_t)(now_us - next_sample_us) >= 0) {
            next_sample_us += BARCODE_SAMPLE_PERIOD_US;
            if ((int32_t)(now_us - next_sample_us) >= 0)
                next_sample_us = now_us + BARCODE_SAMPLE_PERIOD_US;
            barcode_sample(gpio_get(BARCODE_IR_DIGITAL_GPIO), now_us);

            barcode_result_t result;
            while (barcode_take_result(&result)) {
                if (result.status == BARCODE_OK)
                    printf("barcode text=%s\n", result.text);
                else {
                    ++rejected;
                    printf("barcode reject reason=%s\n", status_name(result.status));
                }
            }
        }
        if ((uint32_t)(now_us - last_debug_us) >= BARCODE_DEBUG_INTERVAL_US) {
            last_debug_us = now_us;
            printf("barcode do=%u rejected=%lu\n",
                   (unsigned)gpio_get(BARCODE_IR_DIGITAL_GPIO),
                   (unsigned long)rejected);
        }
        tight_loop_contents();
    }
}
