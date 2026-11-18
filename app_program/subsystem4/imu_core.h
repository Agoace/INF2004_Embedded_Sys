/* Observe-only gravity/bias/filter/integration ideas: Jaison, b2a6bb6.
 * Pure logic. No SDK, kernel, packed protocol or driving decision. */
#ifndef IMU_CORE_H
#define IMU_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_settings.h"
#include "imu_roll.h"
#include "imu_hump.h"

#define IMU_STILL_MS 1000u
#define IMU_STILL_MIN_SAMPLES 100u
#define IMU_STILL_STD_MG 20.0f
#define IMU_FILTER_TAU_MS 100.0f
#define IMU_MAX_GAP_MS 100u
#define IMU_ENCODER_WINDOW_MS 100u

typedef struct {
    float accel_mg[3];       /* Already mapped: X right, Y forward, Z up. */
    float travel_mm;         /* Signed mean wheel travel, not total path. */
    float yaw_rad;           /* Encoder heading, positive LEFT/CCW. */
    uint32_t now_ms;
    uint32_t travel_ms;      /* Timestamp of the encoder observation. */
    uint32_t calibration_epoch; /* Odd during the whole line spin routine. */
    uint32_t run_epoch;      /* Odd during a vehicle run or GP21 TEST drive. */
    bool accel_valid;
    bool motion_valid;
    bool wheels_still;       /* BOTH wheel counts unchanged, zero duty. */
    bool calibration_allowed;
} imu_core_input_t;

typedef struct {
    float accel_mg[3];       /* Bias removed. */
    float bias_mg[3];
    float still_std_mg[3];
    float travel_mm;
    float speed_mm_s;
    float forward_accel_mm_s2; /* Filtered encoder derivative. */
    float yaw_rate_rad_s;
    float yaw_accel_rad_s2;
    float lateral_fake_mm_s2;
    float forward_fake_mm_s2;
    imu_roll_output_t roll_only; /* PRIMARY, pitch quality never gates. */
    imu_hump_output_t hump;  /* Step 3 run events; MARK never resets. */
    float left_mm;           /* Step2b caster-flat diagnostic. */
    float right_mm;          /* Step2b full-model diagnostic. */
    float caster_mm;         /* Wheels-flat alternative, not simultaneous. */
    float left_peak_mm;
    float right_peak_mm;     /* Step2b gated diagnostic, not primary. */
    float caster_peak_mm;
    float right_smoothed_mm;
    float left_residual_mm;
    bool support_peak_eligible;
    uint32_t support_peak_rejects;
    uint32_t refresh_rejects;
    int32_t refresh_tilt_ddeg;
    float height_mm;
    float peak_mm;
    float distance_mm;       /* Signed distance since MARK/calibration. */
    int32_t raw_pitch_ddeg;  /* Before bias removal. */
    int32_t raw_roll_ddeg;
    int32_t pitch_ddeg;
    int32_t roll_ddeg;
    int32_t filtered_pitch_ddeg;
    int32_t filtered_roll_ddeg;
    int32_t corrected_pitch_ddeg;
    int32_t corrected_roll_ddeg;
    uint32_t samples;
    uint32_t valid_samples;
    uint32_t invalid_samples;
    uint32_t motion_misses;
    uint32_t integrated_samples;
    uint32_t gap_resets;
    uint32_t calibrations;
    uint32_t calibration_rejects;
    bool calibrated;
    bool valid;
    bool motion_valid;
} imu_core_output_t;

typedef struct {
    imu_core_output_t output;
    imu_roll_t roll_estimator;
    imu_hump_t hump_detector;
    /* Diagnostic inputs retained without recomputing either roll sine. */
    float hump_corrected_mm;
    float hump_filtered_mm;
    float hump_travel_mm;
    bool hump_valid;
    float mean[3];
    float m2[3];
    float minimum[3];
    float maximum[3];
    float previous_travel;
    float previous_yaw;
    float previous_yaw_rate;
    float window_yaw;
    float mark_travel;
    float previous_speed;
    float window_distance;
    float pitch_rad;
    float roll_rad;
    float corrected_rad;
    float corrected_roll_rad;
    float support_window[IMU_SUPPORT_WINDOW_SAMPLES];
    bool support_bad[IMU_SUPPORT_WINDOW_SAMPLES];
    float support_sum;
    unsigned int support_count;
    unsigned int support_index;
    unsigned int support_bad_count;
    uint32_t still_start;
    uint32_t still_count;
    uint32_t previous_ms;
    uint32_t previous_travel_ms;
    uint32_t velocity_ms;
    uint32_t window_ms;
    uint32_t epoch;
    bool have_sample;
    bool have_reference;
    bool have_motion;
    bool have_speed;
    bool have_filter;
} imu_core_t;

void imu_core_init(imu_core_t *core);
/* Always consumes one observation, including invalid/fault observations.
 * Never bridges a bad/old encoder interval with a later tilt sample. */
void imu_core_step(imu_core_t *core, const imu_core_input_t *input);
/* MARK resets integral/support peaks. Calibration resets integral only;
 * support peaks persist until MARK (or fresh boot), not a driving START.
 * Hump totals follow run_epoch only; MARK and calibration keep them. */
void imu_core_mark(imu_core_t *core);

#endif
