/**
 * @file imu_config.h
 * @brief Every tunable numeric value for the IMU producer.
 *
 * Nothing else in this slice hard-codes a threshold, a scale factor, a bus
 * address, a pin, or a period. Change a value here and rebuild.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* ---- Bus, pins, and device addresses ---------------------------------- */

#define IMU_CFG_I2C_INSTANCE        0       /**< I2C0.                        */
#define IMU_CFG_SDA_GPIO            0       /**< GP0 = I2C0 SDA.              */
#define IMU_CFG_SCL_GPIO            1       /**< GP1 = I2C0 SCL.              */
#define IMU_CFG_DRDY_GPIO           6       /**< Data-ready input.            */
#define IMU_CFG_I2C_BAUD_HZ         400000
#define IMU_CFG_ACCEL_ADDR          0x19    /**< SA0 tied high.               */
#define IMU_CFG_MAG_ADDR            0x1E

/* ---- Cadence ----------------------------------------------------------- */

#define IMU_CFG_PERIOD_MS           13      /**< One IMU cycle.               */
#define IMU_CFG_DIAG_PERIOD_MS      100     /**< 10 Hz USB diagnostic line.   */

/* ---- Self-test and stationary calibration ------------------------------ */

/** Calibration window. 2000 ms at 13 ms is 154 samples. */
#define IMU_CFG_CAL_WINDOW_MS       2000u

/** Every sample magnitude must stay inside this band to accept a window. */
#define IMU_CFG_CAL_MIN_G           0.90f
#define IMU_CFG_CAL_MAX_G           1.10f

/** Sample standard deviation of the magnitude must not exceed this. */
#define IMU_CFG_CAL_MAX_STD_G       0.03f

/**
 * Expected gravity vector while calibrating, in body axes and milli-g.
 *
 * The robot must be stationary and level during the window. The per-axis
 * estimate absorbs the accelerometer zero offset plus any small static
 * mounting error. A robot calibrated on a slope learns that slope as level,
 * so calibrate on a flat surface.
 */
#define IMU_CFG_CAL_GRAVITY_BODY_MG { 0.0f, 0.0f, 1000.0f }

/* ---- Filtering --------------------------------------------------------- */

/** Exponential low-pass alpha for the acceleration and magnetic vectors. */
#define IMU_CFG_LPF_ALPHA           0.20f

/* ---- Plausibility limits, used to reject non-finite or dead readings ---- */

#define IMU_CFG_ACCEL_MIN_G         0.10f   /**< Below this: invalid sample.  */
#define IMU_CFG_MAG_MIN_GAUSS       0.05f
#define IMU_CFG_MAG_MAX_GAUSS       12.00f

/* ---- Terrain thresholds (degrees, three consecutive samples) ----------- */

#define IMU_CFG_LEVEL_DEG           4.0f    /**< |pitch| at most: level.      */
#define IMU_CFG_CLIMB_DEG           7.0f    /**< pitch at least: climbing.    */
#define IMU_CFG_DESCEND_DEG         (-7.0f) /**< pitch at most: descending.   */
#define IMU_CFG_TERRAIN_SAMPLES     3

/** A hump peak below this is a false start and is not retained. */
#define IMU_CFG_HUMP_MIN_MM         15

/** Upper clamp for the integrated hump height and the retained peak. */
#define IMU_CFG_HUMP_MAX_MM         60000

/* ---- Collision --------------------------------------------------------- */

/**
 * Filtered magnitude residual above the 1 g baseline that counts as an impact.
 *
 * Reachable at the configured full scale: 1.5 g of residual needs 2.5 g of
 * measured acceleration, and IMU_CFG_ACCEL_FS_G is 8 g (saturating near
 * 8.19 g). At the previous +/-2 g setting this value could never be reached,
 * because the reading saturates at 2.047 g.
 *
 * NOTE: the 0.20 low-pass also attenuates a single-sample spike heavily, so an
 * impact must persist for several 13 ms samples to reach this threshold.
 *
 * Guarded so a build can retune the sensitivity without editing this file.
 */
#ifndef IMU_CFG_COLLISION_RESIDUAL_G
#define IMU_CFG_COLLISION_RESIDUAL_G    1.5f
#endif

#define IMU_CFG_COLLISION_SAMPLES       2

/** Residual below which the detector starts to re-arm. */
#ifndef IMU_CFG_COLLISION_RELEASE_G
#define IMU_CFG_COLLISION_RELEASE_G     1.2f
#endif

#define IMU_CFG_COLLISION_RELEASE_SAMPLES 10

/* ---- Fault handling ---------------------------------------------------- */

/** Retry the bus identity and configuration at most this often. */
#define IMU_CFG_FAULT_RETRY_MS      1000u

/* ---- Self-test staging ------------------------------------------------- */

/**
 * Cycles to skip after a successful configuration before the first live
 * sample is checked. 2 cycles is 26 ms, longer than one 100 Hz accelerometer
 * and one 75 Hz magnetometer conversion.
 */
#define IMU_CFG_SELFTEST_WAIT_CYCLES 2u

/** Give up and fault if no plausible sample arrives within this many cycles. */
#define IMU_CFG_SELFTEST_MAX_CYCLES  40u

/* ---- Turn cross-check -------------------------------------------------- */

/**
 * Minimum turn rate before the encoder cross-check makes a judgement. Below
 * this the encoder difference is noise, so the cross-check reports UNKNOWN.
 */
#define IMU_CFG_CROSSCHECK_MIN_DPS  5.0f

/* ---- Axis remap -------------------------------------------------------- */

/**
 * Signed axis remap from sensor axes to body axes (X right, Y forward, Z up).
 *
 * The default is the module mounted with its +Y pointing forward and its +X
 * pointing right. Remounting the module changes only these two structures.
 */
#define IMU_CFG_ACCEL_AXIS_MAP  { { 0, 1, 2 }, { 1, 1, 1 } }
#define IMU_CFG_MAG_AXIS_MAP    { { 0, 1, 2 }, { 1, 1, 1 } }

/* ---- Event queue ------------------------------------------------------- */

#define IMU_CFG_EVENT_QUEUE_LEN     16

/* ---- Accelerometer full scale ------------------------------------------ */

/**
 * Accelerometer full scale in g.
 *
 * This one value re-derives the scale factor and the register bits, so the
 * driver, the conversion, and the tests cannot disagree.
 *
 *   full scale | mg per LSB | CTRL_REG4_A FS bits
 *   2 g        | 1          | 00
 *   4 g        | 2          | 01
 *   8 g        | 4          | 10
 *   16 g       | 12         | 11
 *
 * 8 g is the chosen operating point. The collision detector needs about 2.5 g
 * of measurable acceleration, and a +/-2 g device saturates at 2.047 g, so the
 * shipped 1.5 g residual is unreachable at 2 g. The cost of 8 g is resolution:
 * 4 mg per LSB instead of 1 mg.
 */
#define IMU_CFG_ACCEL_FS_G          8

#if IMU_CFG_ACCEL_FS_G == 2
#define IMU_CFG_ACCEL_MG_PER_LSB    1.0f
#define IMU_CFG_ACCEL_FS_BITS       0x00u
#elif IMU_CFG_ACCEL_FS_G == 4
#define IMU_CFG_ACCEL_MG_PER_LSB    2.0f
#define IMU_CFG_ACCEL_FS_BITS       0x10u
#elif IMU_CFG_ACCEL_FS_G == 8
#define IMU_CFG_ACCEL_MG_PER_LSB    4.0f
#define IMU_CFG_ACCEL_FS_BITS       0x20u
#elif IMU_CFG_ACCEL_FS_G == 16
#define IMU_CFG_ACCEL_MG_PER_LSB    12.0f
#define IMU_CFG_ACCEL_FS_BITS       0x30u
#else
#error "IMU_CFG_ACCEL_FS_G must be 2, 4, 8, or 16"
#endif

/** Largest magnitude the configured full scale can report, with headroom. */
#define IMU_CFG_ACCEL_MAX_G         (1.1f * (float) IMU_CFG_ACCEL_FS_G)

/* ---- Magnetometer scale factors ---------------------------------------- */

/** Gain at +/-1.3 gauss. */
#define IMU_CFG_MAG_LSB_PER_GAUSS_XY    1100.0f
#define IMU_CFG_MAG_LSB_PER_GAUSS_Z     980.0f

/** Gravity used to convert milli-g to g. */
#define IMU_CFG_MG_PER_G                1000.0f

#endif /* IMU_CONFIG_H */
