#include "obstacle_scan.h"

#include "pico/time.h"
#include "servo.h"

typedef enum {
    PHASE_IDLE,
    PHASE_SETTLE,  /* Servo moves to the point. */
    PHASE_MEASURE, /* Readings at the point. */
    PHASE_RETURN,  /* Servo moves back to the centre. */
} phase_t;

static phase_t phase = PHASE_IDLE;
static unsigned int index_now;
static unsigned int readings_taken;
static uint32_t phase_start_us;
static uint32_t settle_us;
static obstacle_profile_t profile;

static void move_to(uint16_t pulse_us, uint32_t settle_ms, phase_t next) {
    (void)servo_set_pulse_us(pulse_us);
    phase_start_us = time_us_32();
    settle_us = settle_ms * 1000u;
    phase = next;
}

void obstacle_scan_init(unsigned int servo_pin) {
    servo_init(servo_pin);
    phase = PHASE_IDLE;
    index_now = 0u;
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        profile.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
    }
}

bool obstacle_scan_start(void) {
    if (phase != PHASE_IDLE) {
        return false;
    }
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        profile.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
    }
    index_now = 0u;
    readings_taken = 0u;
    move_to(obstacle_scan_pulse_us(0u), OBSTACLE_SCAN_SETTLE_FIRST_MS,
            PHASE_SETTLE);
    return true;
}

void obstacle_scan_abort(void) {
    if (phase != PHASE_IDLE) {
        (void)servo_set_pulse_us(SERVO_CENTRE_US);
        phase = PHASE_IDLE;
    }
}

bool obstacle_scan_busy(void) {
    return phase != PHASE_IDLE;
}

unsigned int obstacle_scan_progress(void) {
    return index_now;
}

const obstacle_profile_t *obstacle_scan_profile(void) {
    return &profile;
}

/* Add one published result. Returns FAULT, POINT, or NONE. */
static obstacle_scan_event_t take_reading(ultrasonic_result_t result) {
    if (ultrasonic_result_fault(result)) {
        (void)servo_set_pulse_us(SERVO_CENTRE_US);
        phase = PHASE_IDLE;
        return OBSTACLE_SCAN_FAULT;
    }
    profile.point[index_now] =
        obstacle_scan_combine(profile.point[index_now], result);
    ++readings_taken;
    if (readings_taken < OBSTACLE_SCAN_READINGS) {
        return OBSTACLE_SCAN_NONE;
    }

    ++index_now;
    readings_taken = 0u;
    if (index_now < OBSTACLE_SCAN_POINTS) {
        move_to(obstacle_scan_pulse_us(index_now),
                OBSTACLE_SCAN_SETTLE_STEP_MS, PHASE_SETTLE);
    } else {
        move_to(SERVO_CENTRE_US, OBSTACLE_SCAN_SETTLE_RETURN_MS,
                PHASE_RETURN);
    }
    return OBSTACLE_SCAN_POINT;
}

obstacle_scan_event_t obstacle_scan_service(void) {
    const uint32_t now = time_us_32();

    switch (phase) {
    case PHASE_IDLE:
        return OBSTACLE_SCAN_NONE;

    case PHASE_SETTLE:
        if ((now - phase_start_us) >= settle_us) {
            phase = PHASE_MEASURE;
        }
        return OBSTACLE_SCAN_NONE;

    case PHASE_MEASURE:
        if (ultrasonic_service()) {
            return take_reading(ultrasonic_latest());
        }
        /* ultrasonic_trigger() returns false until the 60 ms minimum
         * interval has passed; try again on the next call. A STUCK_HIGH
         * result is published at once and the driver is not busy. */
        if (!ultrasonic_busy() && ultrasonic_trigger() && !ultrasonic_busy()) {
            return take_reading(ultrasonic_latest());
        }
        return OBSTACLE_SCAN_NONE;

    case PHASE_RETURN:
        if ((now - phase_start_us) >= settle_us) {
            phase = PHASE_IDLE;
            return OBSTACLE_SCAN_DONE;
        }
        return OBSTACLE_SCAN_NONE;
    }
    return OBSTACLE_SCAN_NONE;
}
