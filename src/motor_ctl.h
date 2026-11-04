/*
 * Motor controller core: the only owner of the actuators
 * (docs/ADR_MOTOR_CONTROLLER.md).
 *
 * Values in, measurements out. No intent: no steer mixing, no line mode,
 * no "move complete" decision.
 *
 *   In:  left and right wheel speed (mm/s, signed, forward positive),
 *        servo angle (degrees, positive = left), stop, emergency stop.
 *   Out: duty for each motor (permille, signed) and the servo pulse (us)
 *        for the hardware layer; and measurements: wheel travel (mm),
 *        wheel speed (mm/s), heading change (0.01 degree, positive = left),
 *        servo settled, fault bits.
 *
 * Speed loop for each wheel (adapted from Motion, Buddy 2,
 * origin/motor-line-manager src/motion/motion_control.c wheel_control()):
 *   duty = feed-forward + P * error + I
 *   feed-forward = offset + slope * set point, with a separate offset and
 *   slope for each wheel and each direction (the hardware quirks: the
 *   wheels differ, a loaded wheel does not turn below about 12 %, the left
 *   reverse is weaker). The I term removes what is left, for example the
 *   battery voltage drop.
 *   - The set point rises at most accel_cpt_q8 per tick; it falls at once.
 *   - Start kick: from rest or on a direction change, the duty is at least
 *     kick_permille until the wheel has turned kick_counts. No movement in
 *     kick_timeout_ms: stall.
 *   - Stall: duty at or above stall_min_permille and no count for
 *     stall_timeout_ms.
 *
 * Protection (the same in every situation, so not intent):
 *   - Stall (each wheel) and emergency stop: latched, all duty 0 until
 *     mc_clear_faults(); that stops the wheels, so a speed sent while
 *     latched has no effect.
 *   - Stale command: no mc_set_speed() for stale_ms. Both set points go to
 *     0 and MC_FAULT_STALE is set; the next mc_set_speed() clears it.
 *   - mc_stop(): duty 0 at once, not latched.
 *
 * Call mc_tick() every cfg.tick_ms with the encoder counts (forward
 * positive, cumulative). Pure logic: no hardware calls. Host-tested by
 * tests/test_motor_ctl.c.
 */
#ifndef MOTOR_CTL_H
#define MOTOR_CTL_H

#include <stdbool.h>
#include <stdint.h>

#define MC_WHEELS 2u
#define MC_LEFT 0u
#define MC_RIGHT 1u
#define MC_FORWARD 0u
#define MC_REVERSE 1u

#define MC_PERMILLE_MAX 1000

/* Fault bits (mc_faults()). */
#define MC_FAULT_STALL_LEFT 0x01u
#define MC_FAULT_STALL_RIGHT 0x02u
#define MC_FAULT_ESTOP 0x04u
#define MC_FAULT_STALE 0x08u /* not latched */
#define MC_FAULT_LATCHED (MC_FAULT_STALL_LEFT | MC_FAULT_STALL_RIGHT | MC_FAULT_ESTOP)

typedef struct {
    uint16_t tick_ms;             /* mc_tick() period */
    uint16_t counts_per_m;        /* encoder counts per metre of wheel travel */
    uint16_t counts_per_deg_x100; /* (right - left) counts per degree of turn, x100 */
    /* Feed-forward, indexed [wheel][MC_FORWARD or MC_REVERSE]. */
    uint16_t ff_offset_permille[MC_WHEELS][2];
    uint16_t ff_slope_q8[MC_WHEELS][2]; /* permille per count/tick, Q8 */
    uint16_t kp_q8;                     /* permille per count/tick of error, Q8 */
    uint16_t ki_q8;                     /* permille per count/tick of error per tick, Q8 */
    uint16_t integral_max_permille;
    uint16_t accel_cpt_q8; /* set point rise per tick, counts/tick Q8; 0 = step */
    uint16_t max_permille; /* duty limit */
    uint16_t kick_permille;
    uint16_t kick_counts;
    uint16_t kick_timeout_ms;
    uint16_t stall_min_permille;
    uint16_t stall_timeout_ms;
    uint16_t stale_ms;
    /* Servo. */
    uint16_t servo_centre_us;
    uint16_t servo_min_us;
    uint16_t servo_max_us;
    uint16_t servo_us_per_deg_x10;  /* pulse change per degree, x10 */
    uint16_t servo_settle_base_ms;  /* settle estimate = base + per_deg * |change| */
    uint16_t servo_settle_per_deg_x10_ms;
} mc_config_t;

typedef struct {
    int32_t target_q8;   /* asked speed, counts/tick Q8 */
    int32_t setpoint_q8; /* after the rise limit */
    int32_t integral_q8; /* permille Q8 */
    int32_t last_count;
    int32_t origin;      /* count at mc_init() */
    int32_t kick_travel;
    int16_t speed_cpt;   /* measured, counts in the last tick */
    int16_t duty;        /* permille, signed */
    uint16_t kick_ms;
    uint16_t still_ms;
    bool kicking;
} mc_wheel_t;

typedef struct {
    mc_config_t cfg;
    mc_wheel_t w[MC_WHEELS];
    uint32_t command_ms;
    uint32_t last_tick_ms;
    uint32_t servo_settled_ms;
    uint16_t servo_us;
    int8_t servo_deg;
    uint8_t faults;
    bool has_command;
} mc_t;

typedef struct {
    int16_t duty_permille[MC_WHEELS]; /* signed, forward positive */
    uint16_t servo_us;
} mc_output_t;

void mc_init(mc_t *mc, const mc_config_t *cfg, const int32_t counts[MC_WHEELS], uint32_t now_ms);

/* Wheel speeds in mm/s, signed. No effect while a fault is latched. */
void mc_set_speed(mc_t *mc, int16_t left_mm_s, int16_t right_mm_s, uint32_t now_ms);

/* Both set points and duties to 0 at once. Not latched. */
void mc_stop(mc_t *mc);

/* Latched: duty 0 until mc_clear_faults(). Safe to call at any time. */
void mc_estop(mc_t *mc);

/* Clear the latched faults; the wheels stay stopped until a new speed. */
void mc_clear_faults(mc_t *mc);

/* Servo angle in degrees, positive = left. The pulse is clamped to the
 * safe range. Returns the angle that was set (after the clamp). */
int mc_set_servo(mc_t *mc, int deg, uint32_t now_ms);

/* Run the loop. counts: cumulative encoder counts, forward positive. */
void mc_tick(mc_t *mc, const int32_t counts[MC_WHEELS], uint32_t now_ms, mc_output_t *out);

/* Measurements. */
int32_t mc_travel_mm(const mc_t *mc, unsigned int wheel);    /* since mc_init() */
int16_t mc_speed_mm_s(const mc_t *mc, unsigned int wheel);   /* last tick */
int32_t mc_heading_cdeg(const mc_t *mc);                     /* since mc_init(), left + */
bool mc_servo_settled(const mc_t *mc, uint32_t now_ms);
int mc_servo_deg(const mc_t *mc);

static inline uint8_t mc_faults(const mc_t *mc) {
    return mc->faults;
}

#endif
