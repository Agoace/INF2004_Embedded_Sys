#include "drive.h"

#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#define PWM_WRAP 6249u /* 125 MHz / 6250 = 20 kHz */

typedef struct {
    unsigned int forward_pin;
    unsigned int reverse_pin;
    unsigned int a_pin;
    unsigned int b_pin;
    int count_sign; /* +1 if forward counts up */
} wheel_pins_t;

static const wheel_pins_t PINS[2] = {
    [DRIVE_LEFT] = {.forward_pin = 8u, .reverse_pin = 9u, .a_pin = 16u, .b_pin = 17u,
                    .count_sign = +1},
    [DRIVE_RIGHT] = {.forward_pin = 10u, .reverse_pin = 11u, .a_pin = 4u, .b_pin = 5u,
                     .count_sign = -1},
};

typedef struct {
    volatile int32_t count;
    volatile uint8_t previous;
} encoder_t;

static encoder_t encoders[2];

static const int8_t quadrature_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};

static uint8_t encoder_state(const wheel_pins_t *w) {
    return (uint8_t)((gpio_get(w->a_pin) ? 2u : 0u) | (gpio_get(w->b_pin) ? 1u : 0u));
}

static void encoder_irq(unsigned int gpio, uint32_t events) {
    (void)events;
    for (unsigned int i = 0u; i < 2u; ++i) {
        if (gpio == PINS[i].a_pin || gpio == PINS[i].b_pin) {
            encoder_t *e = &encoders[i];
            const uint8_t now = encoder_state(&PINS[i]);
            e->count += quadrature_table[(e->previous << 2u) | now];
            e->previous = now;
            return;
        }
    }
}

/* Forward and reverse pins share one PWM slice (A and B channels). */
static void pwm_pair_init(unsigned int forward_pin, unsigned int reverse_pin) {
    gpio_set_function(forward_pin, GPIO_FUNC_PWM);
    gpio_set_function(reverse_pin, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(pwm_gpio_to_slice_num(forward_pin), &config, true);
    pwm_set_gpio_level(forward_pin, 0u);
    pwm_set_gpio_level(reverse_pin, 0u);
}

void drive_init(void) {
    pwm_pair_init(PINS[DRIVE_LEFT].forward_pin, PINS[DRIVE_LEFT].reverse_pin);
    pwm_pair_init(PINS[DRIVE_RIGHT].forward_pin, PINS[DRIVE_RIGHT].reverse_pin);
    for (unsigned int i = 0u; i < 2u; ++i) {
        const wheel_pins_t *w = &PINS[i];

        const unsigned int pins[2] = {w->a_pin, w->b_pin};
        for (unsigned int p = 0u; p < 2u; ++p) {
            gpio_init(pins[p]);
            gpio_set_dir(pins[p], GPIO_IN);
            gpio_disable_pulls(pins[p]);
        }
        encoders[i].count = 0;
        encoders[i].previous = encoder_state(w);
    }
    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(PINS[0].a_pin, edges, true, encoder_irq);
    gpio_set_irq_enabled(PINS[0].b_pin, edges, true);
    gpio_set_irq_enabled(PINS[1].a_pin, edges, true);
    gpio_set_irq_enabled(PINS[1].b_pin, edges, true);
}

void drive_set_wheel(drive_wheel_t wheel, int duty_pct) {
    if (duty_pct > 100) duty_pct = 100;
    if (duty_pct < -100) duty_pct = -100;
    const wheel_pins_t *w = &PINS[wheel];
    const unsigned int magnitude = (unsigned int)(duty_pct < 0 ? -duty_pct : duty_pct);
    const uint16_t level = (uint16_t)(((PWM_WRAP + 1u) * magnitude) / 100u);
    pwm_set_gpio_level(w->forward_pin, duty_pct > 0 ? level : 0u);
    pwm_set_gpio_level(w->reverse_pin, duty_pct < 0 ? level : 0u);
}

void drive_set(int left_pct, int right_pct) {
    drive_set_wheel(DRIVE_LEFT, left_pct);
    drive_set_wheel(DRIVE_RIGHT, right_pct);
}

void drive_stop(void) {
    drive_set(0, 0);
}

void drive_counts(int32_t *left, int32_t *right) {
    const uint32_t saved = save_and_disable_interrupts();
    const int32_t l = encoders[DRIVE_LEFT].count;
    const int32_t r = encoders[DRIVE_RIGHT].count;
    restore_interrupts(saved);
    *left = l * PINS[DRIVE_LEFT].count_sign;
    *right = r * PINS[DRIVE_RIGHT].count_sign;
}
