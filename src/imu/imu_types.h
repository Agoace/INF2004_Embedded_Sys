/**
 * @file imu_types.h
 * @brief Platform-independent types for the IMU producer.
 *
 * No Pico SDK or RTOS header is included here, so the production state
 * machine compiles unchanged for the host tests.
 *
 * Body axes: +X right, +Y forward, +Z up. Module +Y is robot forward by
 * default; the axis remap below is the only place that assumption lives.
 */
#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IMU_AXIS_COUNT 3

/* ---- Terrain state ----------------------------------------------------- */

typedef enum
{
    IMU_TERRAIN_INIT         = 0,
    IMU_TERRAIN_LEVEL        = 1,
    IMU_TERRAIN_ON_HUMP      = 2,
    IMU_TERRAIN_SENSOR_FAULT = 3
} imu_terrain_t;

/* ---- Discrete events --------------------------------------------------- */

typedef enum
{
    IMU_EVT_NONE          = 0,
    IMU_EVT_SELFTEST      = 1,
    IMU_EVT_CALIBRATED    = 2,
    IMU_EVT_HUMP_START    = 3,
    IMU_EVT_HUMP_END      = 4,
    IMU_EVT_COLLISION     = 5,
    IMU_EVT_SENSOR_FAULT  = 6
} imu_event_id_t;

/** detail values carried by IMU_EVT_HUMP_END. */
#define IMU_EVT_DETAIL_REJECTED 0u   /**< Peak below the 15 mm floor.          */
#define IMU_EVT_DETAIL_RETAINED 1u   /**< Peak retained as the run maximum.    */

/**
 * @brief One pull-queue event.
 *
 * `seq` is the snapshot sequence at the time the event was posted, so the
 * future Manager can order events against samples. `detail` is a status byte.
 * `value` is the measured peak in mm, a milli-g magnitude, or a diagnostic.
 */
typedef struct
{
    uint8_t id;
    uint8_t seq;
    uint8_t detail;
    int16_t value;
} imu_event_t;

/* ---- Snapshot ---------------------------------------------------------- */

/** Two aligned words. See imu_snapshot.h for the bit layout. */
typedef struct
{
    uint32_t word[2];
} imu_snapshot_t;

/** Decoded snapshot, used by the pack and unpack helpers only. */
typedef struct
{
    uint16_t      heading_ddeg;      /**< 0..3599, unsigned deci-degrees.     */
    int16_t       pitch_ddeg;        /**< -512..511, positive nose-up.        */
    int16_t       turn_rate_dps;     /**< -512..511, degrees per second.      */
    uint16_t      highest_hump_mm;   /**< Largest retained hump this run.     */
    imu_terrain_t terrain;
    bool          accel_valid;
    bool          mag_valid;
    bool          calibrated;
    bool          selftest_passed;
    bool          fault_latched;
    uint8_t       sequence;
} imu_snapshot_fields_t;

/* ---- Encoder input ----------------------------------------------------- */

/**
 * @brief The most recent encoder-derived information, supplied by the owner.
 *
 * The IMU never reads encoder GPIO and never converts counts to millimetres.
 */
typedef struct
{
    int16_t distance_delta_mm;    /**< Signed forward travel since last call. */
    int16_t encoder_left_delta;   /**< Left encoder counts, same interval.    */
    int16_t encoder_right_delta;  /**< Right encoder counts, same interval.   */

    /**
     * Freeze the terrain classifier for this cycle.
     *
     * The Manager's recovery gate sets this while the chassis reverses. The
     * core then keeps sampling, health checks, collision and diagnostics
     * running, but freezes the terrain state, the dwell counters, the current
     * and peak hump height, and stops emitting hump events, so a reverse can
     * neither start nor modify a hump episode. Zero, i.e. false, means normal
     * forward operation, so a caller that does not set the field keeps the
     * previous behaviour.
     */
    bool terrain_pause;
} imu_encoder_input_t;

/* ---- Turn cross-check -------------------------------------------------- */

typedef enum
{
    IMU_CROSSCHECK_UNKNOWN  = 0,
    IMU_CROSSCHECK_AGREE    = 1,
    IMU_CROSSCHECK_DISAGREE = 2
} imu_crosscheck_t;

/* ---- One device reading ------------------------------------------------ */

/** Raw device counts, as the driver reads them. */
typedef struct
{
    int16_t accel_raw[IMU_AXIS_COUNT];  /**< 1 mg per LSB at +/-2 g.        */
    int16_t mag_raw[IMU_AXIS_COUNT];    /**< +/-1.3 gauss gain.             */
} imu_sample_t;

/* ---- Axis remap -------------------------------------------------------- */

/**
 * @brief Signed axis remap from sensor axes to body axes.
 *
 * `src[a]` names the sensor axis that feeds body axis `a`, and `sign[a]` its
 * sign. Remount the module and only this structure changes.
 */
typedef struct
{
    int8_t src[IMU_AXIS_COUNT];
    int8_t sign[IMU_AXIS_COUNT];
} imu_axis_map_t;

/* ---- Diagnostics ------------------------------------------------------- */

/** Everything that is deliberately not in the compact Manager snapshot. */
typedef struct
{
    float         accel_counts_mg[IMU_AXIS_COUNT];  /**< Uncorrected, from counts. */
    float         accel_raw_g[IMU_AXIS_COUNT];      /**< Body frame, uncorrected. */
    float         accel_filt_g[IMU_AXIS_COUNT];     /**< Body frame, filtered.*/
    float         mag_raw_gauss[IMU_AXIS_COUNT];    /**< Body frame.             */
    float         mag_filt_gauss[IMU_AXIS_COUNT];   /**< Body frame, filtered.   */
    float         accel_bias_g[IMU_AXIS_COUNT];     /**< Estimated zero offset.  */
    int16_t       accel_counts[IMU_AXIS_COUNT];     /**< Raw device counts.      */
    float         gravity_gain;                     /**< 1.0 until calibrated.   */
    uint8_t       accel_ctrl4;                      /**< Live CTRL_REG4_A value. */
    bool          accel_ctrl4_valid;
    float         pitch_deg;
    float         roll_deg;
    float         tilt_deg;
    float         heading_deg;
    float         turn_rate_dps;
    float         hump_mm;                          /**< Current hump height.    */
    float         collision_residual_g;
    imu_terrain_t terrain;
    imu_crosscheck_t turn_crosscheck;
    int16_t       distance_delta_mm;                /**< Last delta consumed.    */
    int16_t       encoder_left_delta;
    int16_t       encoder_right_delta;
    int16_t       hump_peak_mm;
    uint16_t      highest_hump_mm;
    bool          drdy_seen;                        /**< DRDY asserted this step. */
    /* Counters */
    uint32_t      cycles;
    uint32_t      missed_drdy;
    uint32_t      i2c_errors;
    uint32_t      invalid_samples;
    uint32_t      self_test_failures;
    uint32_t      calibration_rejects;
    uint32_t      recoveries;
    uint32_t      events_posted;
    uint32_t      events_dropped;
    uint32_t      events_popped;
    uint32_t      queue_high_water;
    uint32_t      missed_distance;
    uint32_t      run_distance_mm;                  /**< Reader-owned odometer.  */
    uint32_t      snapshot_failures;                /**< Seqlock retries used up. */
    uint8_t       sequence;
} imu_diag_t;

#endif /* IMU_TYPES_H */
