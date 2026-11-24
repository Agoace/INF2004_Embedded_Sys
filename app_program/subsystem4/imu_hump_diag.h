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

/*
 * CR1 shared diagnostic board: 9 words (36 B), masks and shifts only. Each
 * value is the integer the USB line prints (imu_hump_diag_scaled(), round
 * half away from zero), two's complement when signed, saturated to its
 * field. Inside the field ranges the printed line is unchanged.
 *
 *   W0 31..0 run_epoch                 W1 31..0 timestamp_ms
 *   W2 19..0 peak ms (u20)             W2 31..20 corrected 0.1 mm (s12)
 *   W3 15..0 travel mm (u16)           W3 30..16 yaw mrad/s (s15)  W3 31 valid
 *   W4 18..0 yaw accel mrad/s2 (s19)   W4 31..19 lateral mg (s13)
 *   W5 17..0 window yaw accel (u18)    W5 31..18 window yaw mrad/s (u14)
 *   W6 17..0 line yaw accel (u18)      W6 29..18 window lateral mg (u12)
 *   W6 30 left                         W6 31 peak valid
 *   W7 19..0 line samples (u20)        W7 31..20 line lateral mg (u12)
 *   W8 11..0 filtered 0.1 mm (s12)     W8 23..12 speed mm/s (s12)
 *   W8 27..24 gate                     W8 31..28 window gate
 *
 * Peak bits and run bits never share a field, so the writer can keep the
 * peak bits and repack them only when the peak changes.
 */
#define IMU_HUMP_DIAG_BOARD_WORDS 9u
typedef struct {
    uint32_t word[IMU_HUMP_DIAG_BOARD_WORDS];
} imu_hump_diag_board_t;

#define IMU_DIAG_W3_VALID_SHIFT 31u

static inline uint32_t imu_hump_diag_board_epoch(const imu_hump_diag_board_t *b)
{
    return b->word[0];
}

static inline uint32_t imu_hump_diag_board_time(const imu_hump_diag_board_t *b)
{
    return b->word[1];
}

static inline bool imu_hump_diag_board_valid(const imu_hump_diag_board_t *b)
{
    return ((b->word[3] >> IMU_DIAG_W3_VALID_SHIFT) & 1u) != 0u;
}

/* Board values in the print units. */
typedef struct {
    uint32_t run_epoch;
    uint32_t ms;
    uint32_t line_samples;
    long travel_mm;
    long corrected_01mm;
    long filtered_01mm;
    long speed_mm_s;
    long yaw_mrad_s;
    long yaw_accel_mrad_s2;
    long lateral_mg;
    long win_yaw_mrad_s;
    long win_yaw_accel_mrad_s2;
    long win_lateral_mg;
    long line_lateral_mg;
    long line_yaw_accel_mrad_s2;
    uint8_t gate;
    uint8_t win_gate;
    bool left;
    bool valid;
    bool peak_valid;
} imu_hump_diag_board_values_t;

/* The print rounding: value * scale, half away from zero, int32 clamp. */
long imu_hump_diag_scaled(float value, float scale);
/* Peak bits only (all run bits zero). */
void imu_hump_diag_pack_peak(imu_hump_diag_board_t *out,
                             const imu_hump_peak_diag_t *peak);
/* out = peak bits OR the run bits of r. peak may alias out. */
void imu_hump_diag_pack_run(imu_hump_diag_board_t *out,
                            const imu_hump_diag_board_t *peak,
                            const imu_hump_diag_report_t *r);
/* Both steps. */
void imu_hump_diag_pack(imu_hump_diag_board_t *out,
                        const imu_hump_diag_report_t *r);
/* imu_hump_diag_finish() for a board copy. */
void imu_hump_diag_board_finish(imu_hump_diag_board_t *b, uint32_t epoch,
                                uint32_t now_ms, bool have);
void imu_hump_diag_board_unpack(imu_hump_diag_board_values_t *v,
                                const imu_hump_diag_board_t *b);
/* One USB line from a board copy. Not called while driving. */
void imu_hump_diag_board_print(const imu_hump_diag_board_t *b);

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
/* One USB line; integer units in the labels. Not called while driving.
 * Same line as imu_hump_diag_board_print() of the packed report. */
void imu_hump_diag_print(const imu_hump_diag_report_t *r);

#endif
