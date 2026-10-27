#include "ultrasonic.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

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

#define ECHO_EDGES (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)

typedef enum {
    PHASE_IDLE = 0,
    PHASE_WAIT_RISE,
    PHASE_WAIT_FALL,
    PHASE_DONE,
} phase_t;

static uint trigger_gpio;
static uint echo_gpio;
static ultrasonic_echo_hook_t echo_hook;

/* Shared with the echo interrupt. Read them together in a critical section. */
static volatile uint8_t phase = PHASE_IDLE;
static volatile uint32_t trigger_us;
static volatile uint32_t rise_us;
static volatile uint32_t fall_us;

/* Written only by ultrasonic_service() and ultrasonic_trigger(). */
static bool has_triggered;
static uint8_t failure_streak;
static volatile ultrasonic_result_t latest_result = ULTRASONIC_RESULT_NONE;
static ultrasonic_debug_t latest_debug;

/*
 * Keep this handler in RAM so a flash cache miss does not delay the
 * timestamp. It takes one timer sample and sets the phase; nothing more.
 *
 * If interrupts are disabled for longer than the echo pulse, both edges are
 * pending on entry and receive the same timestamp. The pulse then reads as
 * 0 us. The shortest real pulse is about 100 us, so keep critical sections
 * on this core shorter than that.
 */
static void __not_in_flash_func(echo_irq_handler)(void) {
    const uint32_t events = gpio_get_irq_event_mask(echo_gpio) & ECHO_EDGES;
    if (events == 0u) {
        return;
    }
    gpio_acknowledge_irq(echo_gpio, events);

    const uint32_t now = time_us_32();
    if ((events & GPIO_IRQ_EDGE_RISE) != 0u && phase == PHASE_WAIT_RISE) {
        rise_us = now;
        phase = PHASE_WAIT_FALL;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u && phase == PHASE_WAIT_FALL) {
        fall_us = now;
        phase = PHASE_DONE;
        if (echo_hook != NULL) {
            echo_hook();
        }
    }
}

void ultrasonic_init(uint trigger_pin, uint echo_pin, ultrasonic_echo_hook_t hook) {
    trigger_gpio = trigger_pin;
    echo_gpio = echo_pin;
    echo_hook = hook;

    gpio_init(trigger_gpio);
    gpio_set_dir(trigger_gpio, GPIO_OUT);
    gpio_put(trigger_gpio, false);

    gpio_init(echo_gpio);
    gpio_set_dir(echo_gpio, GPIO_IN);
    gpio_disable_pulls(echo_gpio);

    /* A raw handler shares IO_IRQ_BANK0 with other GPIO users, such as the
     * encoder callback, without replacing their callback. */
    gpio_add_raw_irq_handler(echo_gpio, echo_irq_handler);
    gpio_set_irq_enabled(echo_gpio, ECHO_EDGES, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
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
        distance_code = (uint16_t)distance_mm;
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

bool ultrasonic_trigger(void) {
    if (phase != PHASE_IDLE) {
        return false;
    }

    const uint32_t now = time_us_32();
    if (has_triggered && (now - trigger_us) < MIN_TRIGGER_INTERVAL_US) {
        return false;
    }
    has_triggered = true;

    if (gpio_get(echo_gpio)) {
        trigger_us = now;
        publish(ULTRASONIC_STATUS_STUCK_HIGH, 0u, 0u);
        return true;
    }

    /* Arm before the pulse so an early rising edge cannot be missed. */
    const uint32_t saved = save_and_disable_interrupts();
    trigger_us = now;
    rise_us = 0u;
    fall_us = 0u;
    phase = PHASE_WAIT_RISE;
    restore_interrupts(saved);

    gpio_put(trigger_gpio, true);
    busy_wait_us_32(TRIGGER_PULSE_US);
    gpio_put(trigger_gpio, false);
    return true;
}

bool ultrasonic_service(void) {
    ultrasonic_status_t status;
    uint32_t latency_us = 0u;
    uint32_t pulse_us = 0u;

    /* Decide and leave PHASE_* in one critical section, so the interrupt
     * cannot change the phase between the check and the reset. */
    const uint32_t saved = save_and_disable_interrupts();
    const uint32_t now = time_us_32();
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
                restore_interrupts(saved);
                return false;
            }
            status = ULTRASONIC_STATUS_NO_RISE;
            break;
        case PHASE_WAIT_FALL:
            if ((now - rise_us) < ECHO_TIMEOUT_US) {
                restore_interrupts(saved);
                return false;
            }
            /* The HC-SR04+ holds Echo HIGH when no echo returns. */
            latency_us = rise_us - trigger_us;
            status = ULTRASONIC_STATUS_NO_RETURN;
            break;
        case PHASE_IDLE:
        default:
            restore_interrupts(saved);
            return false;
    }
    phase = PHASE_IDLE;
    restore_interrupts(saved);

    /* Pulse width is valid only for a complete echo. */
    publish(status, latency_us,
            status == ULTRASONIC_STATUS_VALID ? pulse_us : 0u);
    return true;
}

bool ultrasonic_busy(void) {
    return phase != PHASE_IDLE;
}

ultrasonic_result_t ultrasonic_latest(void) {
    return latest_result;
}

ultrasonic_debug_t ultrasonic_debug_latest(void) {
    return latest_debug;
}
