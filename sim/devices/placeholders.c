/* Milestone 1 devices: no movement and no line. Not a physics model. */
#include "sim_runtime.h"
#include <inttypes.h>
#include <string.h>
#include "drive.h"
#include "line_sensors.h"
#include "servo.h"
#include "ultrasonic.h"
#include "wifi_log.h"

static int duty[2];
static uint16_t pulse = SERVO_CENTRE_US;
static bool busy, triggered;
static uint64_t trigger_us, ready_us;
static ultrasonic_result_t latest = ULTRASONIC_RESULT_NONE;

void drive_init(void) { duty[0] = duty[1] = 0; }
void drive_set_wheel(drive_wheel_t wheel, int pct) {
    if (wheel != DRIVE_LEFT && wheel != DRIVE_RIGHT) { sim_error("invalid wheel"); }
    pct = pct > 100 ? 100 : pct < -100 ? -100 : pct;
    duty[wheel] = pct;
    sim_event("duty", "\"wheel\":%u,\"pct\":%d", (unsigned int)wheel, pct);
}
void drive_set(int left, int right) {
    drive_set_wheel(DRIVE_LEFT, left);
    drive_set_wheel(DRIVE_RIGHT, right);
}
void drive_stop(void) { drive_set(0, 0); }
void drive_counts(int32_t *left, int32_t *right) { *left = 0; *right = 0; }
void line_sensors_init(void) { }
void line_sensors_read(uint16_t raw[LP_SENSORS]) { memcpy(raw, sim_floor, sizeof sim_floor); }
void servo_init(unsigned int pin) {
    (void)pin;
    (void)servo_set_pulse_us(SERVO_CENTRE_US);
}
uint16_t servo_set_pulse_us(uint16_t us) {
    pulse = us < SERVO_SAFE_MIN_US ? SERVO_SAFE_MIN_US :
            us > SERVO_SAFE_MAX_US ? SERVO_SAFE_MAX_US : us;
    sim_event("servo", "\"pulse_us\":%u", (unsigned int)pulse);
    return pulse;
}
void ultrasonic_init(unsigned int trigger_pin, unsigned int echo_pin, ultrasonic_echo_hook_t hook) {
    (void)trigger_pin; (void)echo_pin; (void)hook;
    busy = triggered = false;
    latest = ULTRASONIC_RESULT_NONE;
}
bool ultrasonic_trigger(void) {
    const uint64_t now = sim_now();
    if (busy || (triggered && now - trigger_us < 60000u)) { return false; }
    trigger_us = now;
    /* Trigger-to-rise latency includes the trigger pulse, as in the driver. */
    ready_us = now + 215u + 3000u;
    triggered = busy = true;
    sim_event("ultrasonic_trigger", "\"ready_us\":%" PRIu64, ready_us);
    return true;
}
bool ultrasonic_service(void) {
    if (!busy || sim_now() < ready_us) { return false; }
    busy = false;
    latest = (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN << ULTRASONIC_STATUS_SHIFT);
    sim_event("ultrasonic_result", "\"word\":%u,\"ready_us\":%" PRIu64,
              (unsigned int)latest, ready_us);
    return true;
}
bool ultrasonic_busy(void) { return busy; }
ultrasonic_result_t ultrasonic_latest(void) { return latest; }
ultrasonic_debug_t ultrasonic_debug_latest(void) {
    return (ultrasonic_debug_t){.rise_latency_us = latest == ULTRASONIC_RESULT_NONE ? 0u : 215u};
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
