#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_MAX_PACKET_BYTES 1400
#define TELEMETRY_DEVICE_ID_PROBE "pico-w-telemetry-probe"

/*
 * Each send returns after local formatting and the raw UDP enqueue; it does
 * not wait for the receiver. Call these functions only from the main loop.
 * NEVER call telemetry_send_probe() or telemetry_send_motor_line() from an
 * interrupt handler.
 */
bool telemetry_init(const char *device_id);
bool telemetry_is_connected(void);
uint32_t telemetry_error_count(void);
uint32_t telemetry_session_id(void);

typedef struct {
    const char *state;
} telemetry_probe_sample_t;

bool telemetry_send_probe(const telemetry_probe_sample_t *sample);

typedef struct {
    /* NULL strings and false has_* members omit optional fields. */
    const char *test_type;
    const char *state;
    const char *direction;
    const char *startup_state;
    const char *stall_state;
    const char *emergency_state;
    const char *error_state;

    bool has_motor_number;
    uint32_t motor_number;
    bool has_duty_percent;
    uint32_t duty_percent;
    bool has_revolutions_requested;
    uint32_t revolutions_requested;
    bool has_revolution_index;
    uint32_t revolution_index;
    bool has_absolute_count;
    uint32_t absolute_count;
    bool has_elapsed_ms;
    uint32_t elapsed_ms;

    bool has_ideal_counts;
    uint32_t ideal_counts;
    bool has_target_counts;
    uint32_t target_counts;
    bool has_count_at_stop;
    int32_t count_at_stop;
    bool has_absolute_count_at_stop;
    uint32_t absolute_count_at_stop;
    bool has_count_at_trigger;
    int32_t count_at_trigger;
    bool has_absolute_count_at_trigger;
    uint32_t absolute_count_at_trigger;
    bool has_count_after_settle;
    int32_t count_after_settle;
    bool has_absolute_count_after_settle;
    uint32_t absolute_count_after_settle;
    bool has_coast_overshoot_counts;
    int32_t coast_overshoot_counts;
    bool has_boundary_count_reached;
    uint32_t boundary_count_reached;
    bool has_undershoot_counts;
    uint32_t undershoot_counts;
    bool has_drift_counts;
    int32_t drift_counts;
    bool has_drift_millidegrees;
    int32_t drift_millidegrees;
    bool has_drift_per_revolution_millidegrees;
    int32_t drift_per_revolution_millidegrees;
    bool has_elapsed_us;
    uint32_t elapsed_us;
    bool has_mean_us_per_revolution;
    uint32_t mean_us_per_revolution;
    bool has_mean_ms_per_revolution;
    uint32_t mean_ms_per_revolution;

    bool has_counts_per_rev_milli;
    uint32_t counts_per_rev_milli;
    bool has_sensitivity_millicounts;
    uint32_t sensitivity_millicounts;
    bool has_jog_counts;
    int32_t jog_counts;
    bool has_jog_final_absolute_count;
    uint32_t jog_final_absolute_count;
    bool has_jog_combined_count;
    uint32_t jog_combined_count;
    bool has_smallest_jog_movement;
    uint32_t smallest_jog_movement;
    bool has_alignment_resolution_cpr_milli;
    uint32_t alignment_resolution_cpr_milli;
    bool has_jog_primary_cpr_milli;
    uint32_t jog_primary_cpr_milli;
    bool has_jog_secondary_cpr_milli;
    uint32_t jog_secondary_cpr_milli;

    bool has_split_index;
    uint32_t split_index;
    bool has_split_total;
    uint32_t split_total;
    bool has_split_revolution;
    uint32_t split_revolution;
    bool has_split_absolute_count;
    uint32_t split_absolute_count;
    bool has_split_elapsed_us;
    uint32_t split_elapsed_us;
    bool has_split_delta_counts;
    uint32_t split_delta_counts;
    bool has_split_delta_us;
    uint32_t split_delta_us;

    bool has_completed;
    bool completed;
} telemetry_motor_sample_t;

/*
 * Build and send one complete motor packet. This is the single definition of
 * the motor field names and format; absent sample fields are omitted.
 */
bool telemetry_send_motor_line(const telemetry_motor_sample_t *sample);

#endif
