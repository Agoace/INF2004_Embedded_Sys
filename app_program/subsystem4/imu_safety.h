/* Port of smojjofy's detector, 8ac6e38..038ff23; finite/age guards adapted.
 * Pure IMU safety detector: overturn and lift triggers from the mapped,
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
 *  - Lift and overturn are mutually exclusive in one observation because of
 *    the separation gate (see imu_safety_step()). LIFT is reported on the
 *    stated precedence anyway, so a later change to either gate cannot
 *    silently reverse the order.
 *
 * The upright reference is the fixed mapped +Z. A slow lift, or a robot
 * already held still and level in the air, produces no detectable inertial
 * change; IMU-only detection is best effort and does not cover every
 * airborne pose.
 *
 * Deliberate limitation (RT-0092/F4), not an oversight: the vertical-change
 * rule needs |a| away from 1 g. A tilt rotates gravity while |a| stays near
 * 1 g, so without that separation a plain tilt would read as a large
 * vertical change and latch LIFT. The cost is reduced lift coverage for a
 * combined tilt-and-lift that keeps |a| within IMU_SAFETY_NEAR_ONE_G_MG of
 * 1 g and stays below the overturn limit. Free fall, a magnitude change and
 * a tilt past the overturn limit are all still covered. Do not remove the
 * separation without a pure-tilt false-stop check.
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

/* Counters mean consecutive usable observations, not continuous wall time.
 * Missing observations do not add counts or reset them. A stale board stops
 * Vehicle separately. The clear window below DOES require continuous time.
 * Do not describe the trigger counts as a guaranteed time-to-stop bound. */
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

/* The window above claims CONTINUOUS observation. A longer gap between two
 * observations restarts it instead of crediting time that was never
 * observed, so sparse samples cannot add up to a safe-restart window
 * (RT-0094/F1). On the 10 ms cadence a 30 ms gap is two missed observations
 * and still counts; 40 ms, which is three missed, restarts the window. */
#define IMU_SAFETY_CLEAR_GAP_MS 30u

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

/* Why a reader sees IMU_SAFETY_IMU_UNAVAILABLE. `imu_safety_evaluate()`
 * reports one value for every cause, so a field report cannot say which one
 * happened. This separates them for the live event detail, the console line
 * and the host tests (RT-0099). */
typedef enum {
    IMU_SAFETY_CAUSE_NONE = 0,
    IMU_SAFETY_CAUSE_STALE = 1,   /* copy exists but is older than the limit */
    IMU_SAFETY_CAUSE_INVALID = 2, /* newest sample is not valid or not
                                   * calibrated, so no fresh observation */
    IMU_SAFETY_CAUSE_ABSENT = 3,  /* no copy was ever published: the writer
                                   * has not run yet, or every read was busy */
    IMU_SAFETY_CAUSE_POSE = 4,    /* REFUSAL ONLY: the sample is fresh and
                                   * valid but outside the upright bounds.
                                   * imu_safety_unavailable_cause() never
                                   * returns this value (RT-0101/F2). */
    IMU_SAFETY_CAUSE_READ_FAILED = 5,  /* No usable sensor read: the probe
                                        * returned no valid observation. */
    IMU_SAFETY_CAUSE_NOT_CALIBRATED = 6, /* A sample exists, but the level
                                          * bias is not trusted yet. */
    IMU_SAFETY_CAUSE_ZERO_VECTOR = 7     /* The mapped vector is non-finite or
                                          * exactly zero: a degenerate read. */
} imu_safety_cause_t;

/* Output flags. */
#define IMU_SAFETY_FLAG_VALID        0x01u /* Accel valid and calibrated. */
#define IMU_SAFETY_FLAG_UPRIGHT      0x02u /* Raw pose inside the bounds. */
#define IMU_SAFETY_FLAG_GRAVITY      0x04u /* Gravity history is usable. */
#define IMU_SAFETY_FLAG_FREEFALL     0x08u /* This observation is low-g. */
#define IMU_SAFETY_FLAG_NEAR_ONE_G   0x10u /* This observation is quiet. */
/* Exactly one of these is set when VALID is clear, so a reader can split the
 * `invalid` cause without another board field (plan Phase 1.3). */
#define IMU_SAFETY_FLAG_READ_FAILED     0x20u /* accel_valid was false. */
#define IMU_SAFETY_FLAG_NOT_CALIBRATED  0x40u /* sample ok, bias not trusted. */
#define IMU_SAFETY_FLAG_ZERO_VECTOR     0x80u /* non-finite or all-zero. */

/* Why the one-second clear window is not advancing. Diagnostic only: it never
 * changes the decision. A clear needs every bit clear. */
#define IMU_SAFETY_BLOCK_NOT_UPRIGHT   0x01u
#define IMU_SAFETY_BLOCK_NOT_NEAR_ONE_G 0x02u
#define IMU_SAFETY_BLOCK_WHEELS_MOVING 0x04u
#define IMU_SAFETY_BLOCK_GAP           0x08u

typedef struct {
    float accel_mg[3]; /* Bias-corrected, mapped X right/Y forward/Z up. */
    uint32_t now_ms;
    bool accel_valid;  /* Fresh, non-empty sensor read. */
    bool calibrated;   /* Level bias is a usable reference. */
    bool wheels_still; /* Both encoder counts unchanged, zero duty. */
    bool read_failed;  /* Probe returned no sample; vector may be retained. */
} imu_safety_input_t;

/* Board copy. Small on purpose: Vehicle reads it on the 10 ms path and the
 * large diagnostic snapshot must stay out of that path. */
typedef struct {
    uint32_t observation_ms; /* IMU observation time of this step. */
    uint32_t clear_ms;       /* Continuous all-clear time, clamped. */
    int32_t tilt_ddeg;       /* Tilt from +Z, deci-degrees. 0 while the
                              * sample is inside the upright bounds, where the
                              * tilt cannot reach the overturn limit: the angle
                              * is then not computed, to keep atan2f out of the
                              * upright path. */
    float g_mg;              /* |accel|. */
    float vertical_mg;       /* Signed vertical change vs prior gravity. */
    uint16_t overturn_samples; /* Consecutive over-limit observations. */
    uint16_t lift_samples;     /* Consecutive vertical-change observations. */
    uint8_t current_trigger;   /* imu_safety_trigger_t, this observation. */
    uint8_t latched_trigger;   /* imu_safety_trigger_t, first latched. */
    uint8_t stops;             /* Latched triggers, saturating. */
    uint8_t flags;             /* IMU_SAFETY_FLAG_*. */
    uint8_t clear_blockers;    /* IMU_SAFETY_BLOCK_*: why the clear window is
                                * not advancing. Diagnostic only. */
} imu_safety_output_t;

/* One coherent writer report. The token changes only after a successful
 * clear attempt. Readers must not infer acknowledgement from NONE alone. */
typedef struct {
    imu_safety_output_t value;
    uint32_t applied_request;
} imu_safety_report_t;

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

/* Cause of an IMU_UNAVAILABLE result. Returns CAUSE_NONE when the reader is
 * not unavailable. An absent copy wins over an invalid sample, and an invalid
 * sample wins over staleness, because the more specific cause is the more
 * useful one to report. Pure: reads the copy only. */
uint8_t imu_safety_unavailable_cause(const imu_safety_output_t *out, bool have,
                                     uint32_t now_ms, uint32_t max_age_ms);

/* Tilt from the fixed +Z in deci-degrees. Non-finite or zero magnitude
 * gives 0. Always computes the angle, so it is for tests and callers that
 * need the value outside the upright bounds; imu_safety_step() computes it
 * only outside those bounds. */
int32_t imu_safety_tilt_ddeg(const float accel_mg[3]);

#endif /* IMU_SAFETY_H */
