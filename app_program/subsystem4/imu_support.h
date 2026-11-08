/* Pure support-height hypotheses. No driving outputs or terrain decisions. */
#ifndef IMU_SUPPORT_H
#define IMU_SUPPORT_H

/* Nominal examples/tests, not a new physical measurement.
 * Firmware settings are in imu_settings.h; geometry arguments are explicit. */
#define IMU_SUPPORT_WHEELBASE_MM 80.0f
#define IMU_SUPPORT_TRACK_MM 111.0f

typedef struct {
    float left_mm;
    float right_mm;
    float caster_mm;
} imu_support_t;

/* Positive pitch is nose-up; positive roll is right-side-up.
 * wheelbase_mm is the caster-to-drive-axle distance; track_mm is wheel spacing.
 * Inputs must be finite, with positive geometry lengths in mm.
 *
 * left_mm/right_mm assume the caster is flat. caster_mm instead assumes
 * both drive wheels are flat. These are ALTERNATIVE hypotheses, not three
 * simultaneous contact heights. Values are signed; never clamp to zero.
 * These are step2b diagnostics; step2c primary is roll-only in imu_roll.h.
 * This module supplies no threshold or primary-peak quality decision.
 */
imu_support_t imu_support(float pitch_rad, float roll_rad,
                          float wheelbase_mm, float track_mm);
/* Same hypotheses from sin(pitch), sin(roll), each in [-1, 1]. Allows a
 * caller to reuse the pitch sine already used by its height integral. */
imu_support_t imu_support_from_sines(float pitch_sine, float roll_sine,
                                     float wheelbase_mm, float track_mm);

#endif
