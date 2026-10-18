#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define TELEMETRY_MAX_PACKET_BYTES 1400
#define TELEMETRY_DEVICE_ID_PROBE "pico-w-telemetry-probe"

/*
 * Each send returns after local formatting and the raw UDP enqueue; it does
 * not wait for the receiver. Call these functions only from the main loop.
 * Never call a telemetry function from an interrupt handler.
 */
bool telemetry_init(const char *device_id);
bool telemetry_is_connected(void);
uint32_t telemetry_error_count(void);
uint32_t telemetry_session_id(void);

typedef struct {
    const char *state;

    bool has_motor_1_speed_milli_rpm;
    uint32_t motor_1_speed_milli_rpm;
    bool has_motor_2_speed_milli_rpm;
    uint32_t motor_2_speed_milli_rpm;
    bool has_motor_1_duty_percent;
    uint32_t motor_1_duty_percent;
    bool has_motor_2_duty_percent;
    uint32_t motor_2_duty_percent;
    const char *motor_1_direction;
    const char *motor_2_direction;

    bool has_range_mm;
    uint32_t range_mm;
    bool has_ir_mv;
    uint32_t ir_mv;
    bool has_supply_mv;
    uint32_t supply_mv;
    bool has_progress_percent;
    uint32_t progress_percent;
} telemetry_readings_t;

/* Build and send the common flat instrument object. Absent readings are omitted. */
bool telemetry_send_readings(const char *test,
                             const telemetry_readings_t *readings);

/* Send a terminal status-only notice. It contains no measured values. */
bool telemetry_send_terminal_notice(const char *test, const char *outcome,
                                    bool fault);

#endif
