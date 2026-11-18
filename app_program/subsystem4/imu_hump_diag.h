/* Bounded diagnostic C. No detector or driving decisions. */
#ifndef IMU_HUMP_DIAG_H
#define IMU_HUMP_DIAG_H

#include "imu_hump.h"

#define IMU_DIAG_GATE_MOTION 0x01u
#define IMU_DIAG_GATE_LATERAL 0x02u
#define IMU_DIAG_GATE_YAW 0x04u
#define IMU_DIAG_GATE_SPEED 0x08u

/* Signed heights, even for a LEFT event. Window values are magnitudes. */
typedef struct {
    uint32_t ms;
    float travel_mm;
    float corrected_mm;
    float filtered_mm;
    float speed_mm_s;
    float yaw_rad_s;
    float yaw_accel_rad_s2;
    float lateral_mg;
    float window_yaw_rad_s;
    float window_yaw_accel_rad_s2;
    float window_lateral_mg;
    uint8_t gate;
    uint8_t window_gate;
    bool left;
    bool valid;
} imu_hump_peak_diag_t;

typedef struct {
    imu_hump_peak_diag_t peak;
    uint32_t run_epoch;
    uint32_t timestamp_ms;
    uint32_t line_samples;
    float line_lateral_mg;
    float line_yaw_accel_rad_s2;
    bool valid; /* Fresh, same run; peak.valid also needs a counted hump. */
} imu_hump_diag_report_t;

typedef struct {
    float corrected_mm;
    float filtered_mm;
    float yaw_rad_s;
    float yaw_accel_rad_s2;
    float lateral_mg;
    uint8_t gate;
} imu_hump_diag_sample_t;

typedef struct {
    imu_hump_diag_sample_t window[IMU_HUMP_MEAN_SAMPLES];
    imu_hump_peak_diag_t event;
    imu_hump_peak_diag_t winner;
    uint32_t run_epoch;
    uint32_t line_samples;
    float line_lateral_mg;
    float line_yaw_accel_rad_s2;
    float travel_mm;
    float event_peak_mm;
    float winner_peak_mm;
    uint16_t count;
    unsigned int samples;
    unsigned int index;
    uint8_t state;
} imu_hump_diag_t;

typedef struct {
    const imu_hump_output_t *hump;
    float corrected_mm;
    float filtered_mm;
    float travel_mm; /* Per-observation travel; accumulate magnitude. */
    float speed_mm_s;
    float yaw_rad_s;
    float yaw_accel_rad_s2;
    float lateral_mg;
    uint32_t now_ms;
    uint32_t run_start_ms;
    bool valid; /* Same validity as the height fed to imu_hump_step. */
    bool motion_valid;
    bool line_active;
} imu_hump_diag_input_t;

void imu_hump_diag_init(imu_hump_diag_t *s);
/* Call once AFTER the final core step/replay, not during replay. Window
 * follows the detector across runs. Invalid samples clear the window.
 * Ties keep the first peak, rejected candidates/LONG never replace the
 * winner. ACTIVE is included virtually, like imu_hump_summarize(). */
void imu_hump_diag_step(imu_hump_diag_t *s,
                         const imu_hump_diag_input_t *in,
                         imu_hump_diag_report_t *out);
/* Leave stale/other-run data unavailable, including run-wide maxima. */
void imu_hump_diag_finish(imu_hump_diag_report_t *out, uint32_t epoch,
                           uint32_t now_ms, bool have);
/* One USB line; integer units in the labels. Not called while driving. */
void imu_hump_diag_print(const imu_hump_diag_report_t *r);

#endif
