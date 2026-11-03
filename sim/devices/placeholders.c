/* World-backed motion/IR/servo and a local Wi-Fi sink. */
#include "sim_runtime.h"
#include <inttypes.h>
#include "sim_world.h"
#include "drive.h"
#include "line_sensors.h"
#include "servo.h"
#include "ultrasonic.h"
#include "wifi_log.h"

static uint16_t pulse = SERVO_CENTRE_US;

void drive_init(void) { world_duty(0u, 0); world_duty(1u, 0); world_reset_counts(); }
void drive_set_wheel(drive_wheel_t wheel, int pct) {
    if (wheel != DRIVE_LEFT && wheel != DRIVE_RIGHT) { sim_error("invalid wheel"); }
    pct = pct > 100 ? 100 : pct < -100 ? -100 : pct;
    world_duty((unsigned int)wheel, pct);
    sim_event("duty", "\"wheel\":%u,\"pct\":%d", (unsigned int)wheel, pct);
}
void drive_set(int left, int right) {
    drive_set_wheel(DRIVE_LEFT, left);
    drive_set_wheel(DRIVE_RIGHT, right);
}
void drive_stop(void) { drive_set(0, 0); }
void drive_counts(int32_t *left, int32_t *right) { world_counts(left, right); }
void line_sensors_init(void) { }
void line_sensors_read(uint16_t raw[LP_SENSORS]) { world_ir(raw); }
void servo_init(unsigned int pin) {
    (void)pin;
    (void)servo_set_pulse_us(SERVO_CENTRE_US);
}
uint16_t servo_set_pulse_us(uint16_t us) {
    pulse = us < SERVO_SAFE_MIN_US ? SERVO_SAFE_MIN_US :
            us > SERVO_SAFE_MAX_US ? SERVO_SAFE_MAX_US : us;
    world_servo(pulse);
    sim_event("servo", "\"pulse_us\":%u", (unsigned int)pulse);
    return pulse;
}
void wifi_log_init(void) { }
void wifi_log_poll(uint32_t now_ms) { (void)now_ms; }
bool wifi_log_up(void) { return true; }
bool wifi_log_receiver_known(void) { return true; }
bool wifi_log_send(unsigned int type, const uint8_t *payload, size_t length) {
    /* Hex preserves original bytes without serialising host structs. */
    printf("{\"t_us\":%" PRIu64 ",\"type\":\"packet\",\"packet_type\":%u,\"hex\":\"",
           sim_now(), type);
    for (size_t i = 0; i < length; ++i) { printf("%02x", (unsigned int)payload[i]); }
    puts("\"}");
    return true;
}
void wifi_log_mirror_text(bool on) { sim_event("mirror_text", "\"on\":%s", on ? "true" : "false"); }
uint32_t wifi_log_dropped(void) { return 0u; }
