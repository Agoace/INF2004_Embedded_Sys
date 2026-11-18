/* Pure RIGHT-wheel roll height. No SDK, kernel or driving decisions. */
#ifndef IMU_ROLL_H
#define IMU_ROLL_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_ROLL_TRACK_MM 111.0f
#define IMU_ROLL_WHEELBASE_MM 80.0f
#define IMU_ROLL_MEAN_SAMPLES 10u
#define IMU_ROLL_PLATEAU_CAPACITY 31u
#ifndef IMU_ROLL_PLATEAU_SAMPLES
#define IMU_ROLL_PLATEAU_SAMPLES 20u
#endif

_Static_assert(IMU_ROLL_PLATEAU_SAMPLES > 0u &&
               IMU_ROLL_PLATEAU_SAMPLES <= IMU_ROLL_PLATEAU_CAPACITY,
               "roll median window must be within 1..31");

typedef struct {
    float height_mm;
    float smoothed_mm;
    float peak_mm;
    float plateau_mm;
    float expected_pitch_rad;
    float quality_ratio;
    uint32_t quality_samples;
    uint32_t quality_matches;
    bool smooth_ready;
    bool plateau_ready;
    bool quality_good;
} imu_roll_output_t;

typedef struct {
    imu_roll_output_t output;
    float mean_window[IMU_ROLL_MEAN_SAMPLES];
    float plateau_window[IMU_ROLL_PLATEAU_CAPACITY];
    float mean_sum;
    uint32_t mean_count;
    uint32_t mean_index;
    uint32_t plateau_count;
    uint32_t plateau_index;
} imu_roll_t;

void imu_roll_init(imu_roll_t *state);
/* MARK resets positive peaks, windows and quality totals, not last height
 * or expected pitch. smoothed_mm may be stale while smooth_ready is false.
 * plateau_ready means the CURRENT window is full, not that a peak exists. */
void imu_roll_mark(imu_roll_t *state);
/* height = track_mm * roll_sine, positive for right-side-up.
 * expected_pitch = -asin(clamp(height / (2 * wb_mm), -1, 1)).
 * Quality is abs(measured_pitch - expected_pitch) <= tolerance_rad.
 * It NEVER gates height, smoothing, either peak or either window.
 *
 * Only a full 10-sample mean enters the median. Even median windows use
 * the mean of the two middle values. plateau_mm is the largest median
 * since MARK, not the current median. Both peaks start at zero, so signed
 * negative heights do not create a positive peak.
 *
 * valid=false, nonfinite inputs, |roll_sine|>1, nonpositive geometry or
 * negative tolerance clear windows/readiness and quality_good only.
 * Last height, numeric smoothed value, peaks and quality totals survive.
 * Quality totals count valid raw samples since MARK; both counters and
 * ratio freeze together at UINT32_MAX samples, rather than wrap.
 * Nominal geometry defaults above are not physical measurements. */
void imu_roll_step(imu_roll_t *state, float roll_sine,
                   float measured_pitch_rad, float track_mm, float wb_mm,
                   float tolerance_rad, bool valid);

#endif
