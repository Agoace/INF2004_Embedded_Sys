/* Kernel-side port of src/ultrasonic.c; result and timeout rules unchanged. */
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "drv_ultrasonic.h"
#include "drv_ultrasonic_hw.h"
#include "../platform/drv_gpio_irq.h"
#include "../platform/drv_time.h"

/* Trigger pulse width required by the HC-SR04 family. */
#define TRIGGER_PULSE_US 10u
/* Echo must go HIGH within this time after the trigger starts.
 * Measured trigger-to-rise latency on the HC-SR04+: 212-217 us. */
#define RISE_TIMEOUT_US 2000u
/* Echo HIGH time limit: 3000 us x 343 m/s / 2 = about 515 mm.
 * This only stops the wait. The 511 mm field limit is applied to the
 * converted distance, so the boundary does not depend on timer precision.
 * An echo beyond 511 mm, or no echo, is reported as NO_RETURN (unknown). */
#define ECHO_TIMEOUT_US 3000u
/* Minimum time between triggers, so a late echo cannot match a new ping. */
#define MIN_TRIGGER_INTERVAL_US 60000u

/* Speed of sound at about 20 degrees C, in m/s. */
#define SPEED_OF_SOUND_M_S 343u

#define GPIO_IRQ_EDGE_RISE 8u
#define GPIO_IRQ_EDGE_FALL 4u

typedef enum {
    PHASE_IDLE = 0,
    PHASE_WAIT_RISE,
    PHASE_WAIT_FALL,
    PHASE_DONE,
} phase_t;

/* Shared with the echo interrupt. Read them together in a critical section. */
static volatile uint8_t phase = PHASE_IDLE;
static volatile uint32_t trigger_us;
static volatile uint32_t rise_us;
static volatile uint32_t fall_us;

/* Written only by drv_ultrasonic_service() and drv_ultrasonic_trigger(). */
static bool has_triggered;
static ID listener;
void drv_ultrasonic_listen(int id) { UINT irq; DI(irq); listener=(ID)id; EI(irq); }
static uint8_t failure_streak;
static volatile ultrasonic_result_t latest_result = ULTRASONIC_RESULT_NONE;
static ultrasonic_debug_t latest_debug;

/*
 * The shared dispatcher acknowledges edges before this callback.
 * One TIMER timestamp per callback; phase changes only, no conversion.
 *
 * If interrupts are disabled for longer than the echo pulse, both edges are
 * pending on entry and receive the same timestamp. The pulse then reads as
 * 0 us. The shortest real pulse is about 100 us, so keep critical sections
 * on this core shorter than that.
 */
static void echo_irq_handler(unsigned int pin, uint32_t events, void *ctx) {
    (void)pin; (void)ctx;

    const uint32_t now = drv_time_us();
    /* Task service is 10 ms in the kernel robot, not the reference's fast
     * main loop. Enforce the 2 ms rise deadline here too, so a late echo
     * cannot become VALID merely because service has not run yet. */
    if ((events & GPIO_IRQ_EDGE_RISE) != 0u && phase == PHASE_WAIT_RISE &&
        now - trigger_us < RISE_TIMEOUT_US) {
        rise_us = now;
        phase = PHASE_WAIT_FALL;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u && phase == PHASE_WAIT_FALL) {
        fall_us = now;
        phase = PHASE_DONE;
    }
    if (listener>0) (void)tk_wup_tsk(listener);
}

int drv_ultrasonic_init(void) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    static bool ready;
    if (ready) return (int)E_OK;
    if (drv_time_init() < E_OK) return (int)E_SYS;
    drv_ultrasonic_hw_init();
    int er = drv_gpio_irq_init();
    if (er >= E_OK) er = drv_gpio_irq_add(1u << 2u, echo_irq_handler, 0);
    if (er >= E_OK) ready = true;
    return er;
}

static void publish(ultrasonic_status_t status, uint32_t latency_us,
                    uint32_t pulse_us) {
    uint32_t distance_mm = 0u;
    uint16_t distance_code = ULTRASONIC_DISTANCE_NONE;

    if (pulse_us != 0u) {
        /* Wide arithmetic, rounded to the nearest mm. Round trip, so / 2. */
        distance_mm = (pulse_us * SPEED_OF_SOUND_M_S + 1000u) / 2000u;
        if (distance_mm > ULTRASONIC_DISTANCE_MAX_MM) {
            /* Outside the decision field: same meaning as no echo. */
            status = ULTRASONIC_STATUS_NO_RETURN;
        }
    }

    if (status == ULTRASONIC_STATUS_VALID) {
        distance_code = distance_mm < ULTRASONIC_DISTANCE_MIN_MM
                            ? ULTRASONIC_DISTANCE_TOO_NEAR
                            : (uint16_t)distance_mm;
        failure_streak = 0u;
    } else if (status == ULTRASONIC_STATUS_NO_RETURN) {
        /* The sensor answered the trigger, so it works. */
        failure_streak = 0u;
    } else if (failure_streak < ULTRASONIC_FAULT_STREAK) {
        ++failure_streak;
    }

    latest_debug.rise_latency_us = latency_us;
    latest_debug.pulse_us = pulse_us;
    latest_debug.distance_mm = distance_mm;

    /* One aligned 16-bit store: readers on either core see a whole word. */
    latest_result = (ultrasonic_result_t)(
        ((uint32_t)distance_code << ULTRASONIC_DISTANCE_SHIFT) |
        ((uint32_t)status << ULTRASONIC_STATUS_SHIFT) |
        ((uint32_t)failure_streak << ULTRASONIC_STREAK_SHIFT));
}

bool drv_ultrasonic_trigger(void) {
    if (phase != PHASE_IDLE) {
        return false;
    }

    const uint32_t now = drv_time_us();
    if (has_triggered && (now - trigger_us) < MIN_TRIGGER_INTERVAL_US) {
        return false;
    }
    has_triggered = true;

    if (drv_ultrasonic_hw_echo()) {
        trigger_us = now;
        publish(ULTRASONIC_STATUS_STUCK_HIGH, 0u, 0u);
        return true;
    }

    /* Arm before the pulse so an early rising edge cannot be missed. */
    UINT saved; DI(saved);
    trigger_us = now;
    rise_us = 0u;
    fall_us = 0u;
    phase = PHASE_WAIT_RISE;
    EI(saved);

    drv_ultrasonic_hw_pulse();
    return true;
}

bool drv_ultrasonic_service(void) {
    ultrasonic_status_t status;
    uint32_t latency_us = 0u;
    uint32_t pulse_us = 0u;

    /* Decide and leave PHASE_* in one critical section, so the interrupt
     * cannot change the phase between the check and the reset. */
    UINT saved; DI(saved);
    const uint32_t now = drv_time_us();
    switch ((phase_t)phase) {
        case PHASE_DONE:
            latency_us = rise_us - trigger_us;
            pulse_us = fall_us - rise_us;
            /* An echo that ended after the timeout is still out of range. */
            status = pulse_us <= ECHO_TIMEOUT_US ? ULTRASONIC_STATUS_VALID
                                                 : ULTRASONIC_STATUS_NO_RETURN;
            break;
        case PHASE_WAIT_RISE:
            if ((now - trigger_us) < RISE_TIMEOUT_US) {
                EI(saved);
                return false;
            }
            status = ULTRASONIC_STATUS_NO_RISE;
            break;
        case PHASE_WAIT_FALL:
            if ((now - rise_us) < ECHO_TIMEOUT_US) {
                EI(saved);
                return false;
            }
            /* The HC-SR04+ holds Echo HIGH when no echo returns. */
            latency_us = rise_us - trigger_us;
            status = ULTRASONIC_STATUS_NO_RETURN;
            break;
        case PHASE_IDLE:
        default:
            EI(saved);
            return false;
    }
    phase = PHASE_IDLE;
    EI(saved);

    /* Pulse width is valid only for a complete echo. */
    publish(status, latency_us,
            status == ULTRASONIC_STATUS_VALID ? pulse_us : 0u);
    return true;
}

bool drv_ultrasonic_busy(void) {
    return phase != PHASE_IDLE;
}

ultrasonic_result_t drv_ultrasonic_latest(void) {
    return latest_result;
}

ultrasonic_debug_t drv_ultrasonic_debug_latest(void) {
    return latest_debug;
}
