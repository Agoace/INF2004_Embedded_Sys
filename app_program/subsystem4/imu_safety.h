/* Pure IMU safety detector: overturn and lift triggers from the mapped,
 * bias-corrected acceleration.
 *
 * No Pico SDK, no kernel, no PWM, no driving decision. The IMU task owns the
 * sensor; Vehicle owns the stop. This module decides only whether an
 * observation is a candidate trigger, and latches the first one.
 *
 * Ownership (RT-0089/RT-0091):
 *  - Input is `imu_core_output_t.accel_mg`: bias-corrected and mapped
 *    X right, Y forward, Z up. Never the yaw/encoder-compensated vector.
 *  - Each 10 ms input is the mean of about four 400 Hz FIFO samples, so a
 *    short pulse can be suppressed. The bounds below are candidates until
 *    recorded lift traces confirm the separation.
 *  - The hump mean, hump travel gate and estimated hump height are NOT
 *    inputs here. Hump reporting stays independent of the stop.
 *  - Lift wins over overturn when both confirm in the same observation;
 *    either reason causes the same stop.
 *
 * The upright reference is the fixed mapped +Z. A slow lift, or a robot
 * already held still and level in the air, produces no detectable inertial
 * change; IMU-only detection is best effort and does not cover every
 * airborne pose.
 */
#ifndef IMU_SAFETY_H
#define IMU_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

/* --- Candidate bounds (require trace evidence before a device gate). --- */

/* Upright pose bounds for accepting a level bias. Provisional: they
 * separate the recorded top-up and upside-down six-face poses. */
#define IMU_SAFETY_UPRIGHT_Z_MG   600.0f
#define IMU_SAFETY_UPRIGHT_XY_MG  400.0f

/* Overturn: tilt from +Z beyond the limit, for consecutive observations. */
#define IMU_SAFETY_OVERTURN_DEG     75.0f
#define IMU_SAFETY_OVERTURN_SAMPLES 5u

/* Lift, free fall: gravity magnitude below the bound. */
#define IMU_SAFETY_FREEFALL_MG      400.0f
#define IMU_SAFETY_FREEFALL_SAMPLES 2u

/* Lift, vertical change: |(a - prior gravity) . prior gravity unit|. */
#define IMU_SAFETY_VERTICAL_MG      500.0f
#define IMU_SAFETY_VERTICAL_SAMPLES 3u

/* A reading this close to 1 g is quiet enough to update the gravity
 * history and to count as part of an all-clear window. */
#define IMU_SAFETY_NEAR_ONE_G_MG 200.0f

/* Gravity history smoothing. Frozen during a transient. */
#define IMU_SAFETY_GRAVITY_ALPHA 0.10f
#define IMU_SAFETY_GRAVITY_MG    1000.0f

/* A reader blocks start and stops motion when the published observation is
 * older than this. Five missed 10 ms samples. */
#define IMU_SAFETY_MAX_AGE_MS 50u

/* Continuous fresh, valid, still, upright time before a GP21 clear. */
#define IMU_SAFETY_CLEAR_MS 1000u

/* Elapsed-time clamp, so a preempted gap cannot satisfy a window at once. */
#define IMU_SAFETY_MAX_DELTA_MS 100u

/* Latitude for the |a| / 1 g comparison against normalised gravity. */
#define IMU_SAFETY_G_UNIT_MG 1.0f

typedef enum {
    IMU_SAFETY_NONE = 0,
    IMU_SAFETY_OVERTURN = 1,
    IMU_SAFETY_LIFT = 2,
    IMU_SAFETY_IMU_UNAVAILABLE = 3
} imu_safety_trigger_t;

/* Output flags. */
#define IMU_SAFETY_FLAG_VALID        0x01u /* Accel valid and calibrated. */
#define IMU_SAFETY_FLAG_UPRIGHT      0x02u /* Raw pose inside the bounds. */
#define IMU_SAFETY_FLAG_GRAVITY      0x04u /* Gravity history is usable. */
#define IMU_SAFETY_FLAG_FREEFALL     0x08u /* This observation is low-g. */
#define IMU_SAFETY_FLAG_NEAR_ONE_G   0x10u /* This observation is quiet. */

typedef struct {
    float accel_mg[3]; /* Bias-corrected, mapped X right/Y forward/Z up. */
    uint32_t now_ms;
    bool accel_valid;  /* Fresh, non-empty sensor read. */
    bool calibrated;   /* Level bias is a usable reference. */
    bool wheels_still; /* Both encoder counts unchanged, zero duty. */
} imu_safety_input_t;

/* Board copy. Small on purpose: Vehicle reads it on the 10 ms path and the
 * large diagnostic snapshot must stay out of that path. */
typedef struct {
    uint32_t observation_ms; /* IMU observation time of this step. */
    uint32_t clear_ms;       /* Continuous all-clear time, clamped. */
    int32_t tilt_ddeg;       /* Tilt from +Z, deci-degrees. */
    float g_mg;              /* |accel|. */
    float vertical_mg;       /* Signed vertical change vs prior gravity. */
    uint16_t overturn_samples; /* Consecutive over-limit observations. */
    uint16_t lift_samples;     /* Consecutive vertical-change observations. */
    uint8_t current_trigger;   /* imu_safety_trigger_t, this observation. */
    uint8_t latched_trigger;   /* imu_safety_trigger_t, first latched. */
    uint8_t stops;             /* Latched triggers, saturating. */
    uint8_t flags;             /* IMU_SAFETY_FLAG_*. */
} imu_safety_output_t;

typedef struct {
    imu_safety_output_t output;
    float gravity_mg[3]; /* Quiet gravity history; frozen in a transient. */
    uint16_t freefall_samples;
    uint32_t previous_ms; /* Observation time of the previous step. */
    bool have_previous;
    bool gravity_ready;   /* Internal: survives a step that clears flags. */
} imu_safety_t;

/* Raw mapped acceleration is inside the upright bounds. Use before
 * accepting a level bias, and to refuse a refresh from a tilted pose. */
bool imu_safety_upright_ok(const float accel_mg[3]);

void imu_safety_init(imu_safety_t *s);

/* One observation. Never blocks, allocates or prints. After a latch the
 * detection run counters stop advancing, but the all-clear window keeps
 * measuring so a safe restart is possible. */
void imu_safety_step(imu_safety_t *s, const imu_safety_input_t *in);

/* Explicit safe restart. Clears the latch and every run counter. Refuses
 * and returns false unless the all-clear window has completed. */
bool imu_safety_clear(imu_safety_t *s);

/* Effective trigger for a reader at now_ms. Returns IMU_UNAVAILABLE when no
 * copy exists, the copy is stale, or the observation was unusable;
 * otherwise the latched trigger. Pure: it reads the copy only. */
uint8_t imu_safety_evaluate(const imu_safety_output_t *out, bool have,
                            uint32_t now_ms, uint32_t max_age_ms);

/* Tilt from the fixed +Z in deci-degrees. Non-finite or zero magnitude
 * gives 0. Shared with the host tests. */
int32_t imu_safety_tilt_ddeg(const float accel_mg[3]);

#endif /* IMU_SAFETY_H */
