/* Device and time contracts, independent of the selected firmware. */
#include "sim_runtime.h"
#include <assert.h>
#include "drive.h"
#include "hardware/gpio.h"
#include "line_sensors.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"
#include "wifi_log.h"

int firmware_main(void) {
    assert(sim_now() == 0u);
    assert(stdio_init_all() && !stdio_usb_connected());
    const absolute_time_t first = get_absolute_time();
    assert(first.us == 100u);
    assert(to_ms_since_boot(first) == 0u && sim_now() == 100u);
    const absolute_time_t next = delayed_by_us(first, 100u);
    assert(sim_now() == 100u && time_reached(next) && sim_now() == 200u);
    assert(time_us_32() == 300u);
    sleep_us(700u);
    assert(sim_now() == 1000u);
    assert(make_timeout_time_ms(2u).us == 3100u);
    sleep_ms(2u);
    assert(sim_now() == 3100u);
    gpio_init(21u); gpio_set_dir(21u, GPIO_IN); gpio_pull_up(21u);
    assert(!gpio_get(21u));
    line_sensors_init();
    uint16_t raw[3];
    line_sensors_read(raw);
    assert(raw[0] == 202u && raw[1] == 185u && raw[2] == 191u);
    drive_init(); drive_set(150, -150);
    int32_t left, right;
    drive_counts(&left, &right);
    assert(left == 0 && right == 0);
    drive_stop();
    servo_init(15u);
    assert(servo_set_pulse_us(1u) == 1100u);
    assert(servo_set_pulse_us(65535u) == 1900u);
    ultrasonic_init(3u, 2u, NULL);
    assert(ultrasonic_latest() == ULTRASONIC_RESULT_NONE);
    assert(ultrasonic_trigger() && ultrasonic_busy());
    assert(!ultrasonic_trigger() && !ultrasonic_service());
    sleep_us(3214u);
    assert(!ultrasonic_service());
    sleep_us(1u);
    assert(ultrasonic_service() && !ultrasonic_busy());
    assert(!ultrasonic_service());
    assert(ultrasonic_result_status(ultrasonic_latest()) == ULTRASONIC_STATUS_NO_RETURN);
    assert(ultrasonic_result_streak(ultrasonic_latest()) == 0u);
    assert(ultrasonic_debug_latest().rise_latency_us == 215u);
    assert(!ultrasonic_trigger());
    sleep_us(56785u);
    assert(sim_now() == 63100u && ultrasonic_trigger());
    assert(gpio_get(21u));
    wifi_log_init(); wifi_log_poll(63u);
    assert(wifi_log_up() && wifi_log_receiver_known() && wifi_log_dropped() == 0u);
    const uint8_t bytes[2] = {0u, 255u};
    assert(wifi_log_send(1u, bytes, sizeof bytes));
    wifi_log_mirror_text(false);
    assert(sim_printf("escape: \"\\\n") > 0);
    sleep_us(UINT64_C(4294967296));
    assert(time_us_32() == 63200u);
    assert(to_ms_since_boot((absolute_time_t){UINT64_C(4294967296000)}) == 0u);
    sim_event("probe_passed", "\"passed\":true");
    sleep_us(UINT64_MAX); /* End service caps this at the exact scenario deadline. */
    return 1;
}
