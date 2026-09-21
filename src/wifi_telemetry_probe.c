#include <stdio.h>

#include "pico/stdlib.h"

#include "telemetry.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("\nWi-Fi telemetry probe\n");
    printf("Destination: 192.168.18.98:4242\n");

    if (!telemetry_init(TELEMETRY_DEVICE_ID_PROBE)) {
        printf("Telemetry probe stopped before UDP transmission.\n");
        return 1;
    }

    printf("Network failure must not stall the main loop; send errors are "
           "counted and transmission continues.\n");
    while (true) {
        const telemetry_probe_sample_t sample = {
            .state = "running",
        };
        telemetry_send_probe(&sample);
        sleep_ms(1000);
    }
}
