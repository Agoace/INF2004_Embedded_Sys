#include "motor_ctl_pico.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/critical_section.h"
#include "pico/stdlib.h"

#define MOTOR_PWM_WRAP 6249u /* 125 MHz / 6250 = 20 kHz */
#define SERVO_PIN 15u
#define SERVO_PERIOD_US 20000u

typedef struct {
    unsigned int forward_pin;
    unsigned int reverse_pin;
    unsigned int a_pin;
    unsigned int b_pin;
    int32_t count_sign; /* +1 if forward counts up */
} wheel_pins_t;

/*
 * Sides and encoder signs: measured 2026-10-27
 * (docs/s2-motion/HARDWARE.md).
 */
static const wheel_pins_t PINS[MC_WHEELS] = {
    [MC_LEFT] = {8u, 9u, 16u, 17u, +1},
    [MC_RIGHT] = {10u, 11u, 4u, 5u, -1},
};

static const int8_t QUADRATURE[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};

static volatile int32_t raw_count[MC_WHEELS];
static volatile uint8_t previous_ab[MC_WHEELS];

static mc_t mc;
static critical_section_t lock;
static uint32_t ticks;
static uint32_t tick_max_us;
static uint16_t servo_level;

uint32_t mcp_now_ms(void) {
    return (uint32_t)(time_us_64() / 1000u);
}

static uint8_t read_ab(const wheel_pins_t *w) {
    return (uint8_t)((gpio_get(w->a_pin) ? 2u : 0u) | (gpio_get(w->b_pin) ? 1u : 0u));
}

static void encoder_irq(unsigned int gpio, uint32_t events) {
    (void)events;
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        if (gpio == PINS[i].a_pin || gpio == PINS[i].b_pin) {
            const uint8_t now = read_ab(&PINS[i]);
            raw_count[i] += QUADRATURE[(previous_ab[i] << 2u) | now];
            previous_ab[i] = now;
            return;
        }
    }
}

/* Call inside the lock. */
static void read_counts(int32_t counts[MC_WHEELS]) {
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        counts[i] = raw_count[i] * PINS[i].count_sign;
    }
}

static void write_motor(unsigned int wheel, int16_t permille) {
    const wheel_pins_t *w = &PINS[wheel];
    const uint32_t magnitude = (uint32_t)(permille < 0 ? -permille : permille);
    const uint16_t level = (uint16_t)(((MOTOR_PWM_WRAP + 1u) * magnitude) / (uint32_t)MC_PERMILLE_MAX);
    pwm_set_gpio_level(w->forward_pin, permille > 0 ? level : 0u);
    pwm_set_gpio_level(w->reverse_pin, permille < 0 ? level : 0u);
}

static void write_servo(uint16_t pulse_us) {
    if (pulse_us != servo_level) {
        pwm_set_gpio_level(SERVO_PIN, pulse_us); /* 1 count = 1 us */
        servo_level = pulse_us;
    }
}

/* Heartbeat hook: interrupt context. */
static void on_tick(uint32_t now_us, void *ctx) {
    (void)ctx;
    int32_t counts[MC_WHEELS];
    mc_output_t out;
    critical_section_enter_blocking(&lock);
    read_counts(counts);
    mc_tick(&mc, counts, mcp_now_ms(), &out);
    /* PWM writes inside the lock: an mcp_estop() cannot be overwritten by
     * a duty computed before it. */
    write_motor(MC_LEFT, out.duty_permille[MC_LEFT]);
    write_motor(MC_RIGHT, out.duty_permille[MC_RIGHT]);
    write_servo(out.servo_us);
    critical_section_exit(&lock);
    ticks++;
    const uint32_t took = time_us_32() - now_us;
    if (took > tick_max_us) {
        tick_max_us = took;
    }
}

static void motor_pwm_init(const wheel_pins_t *w) {
    gpio_set_function(w->forward_pin, GPIO_FUNC_PWM);
    gpio_set_function(w->reverse_pin, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
    pwm_set_gpio_level(w->forward_pin, 0u);
    pwm_set_gpio_level(w->reverse_pin, 0u);
    pwm_init(pwm_gpio_to_slice_num(w->forward_pin), &config, true);
}

static void servo_pwm_init(uint16_t pulse_us) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    const unsigned int slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / 1000000.0f); /* 1 MHz */
    pwm_config_set_wrap(&config, SERVO_PERIOD_US - 1u);
    pwm_init(slice, &config, false);
    pwm_set_gpio_level(SERVO_PIN, pulse_us);
    servo_level = pulse_us;
    pwm_set_enabled(slice, true);
}

bool mcp_init(hb_t *hb, const mc_config_t *cfg) {
    const uint32_t hb_ms = HB_TICK_US / 1000u;
    if (cfg->tick_ms == 0u || cfg->tick_ms % hb_ms != 0u) {
        return false;
    }
    critical_section_init(&lock);
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        const wheel_pins_t *w = &PINS[i];
        motor_pwm_init(w);
        const unsigned int pins[2] = {w->a_pin, w->b_pin};
        for (unsigned int p = 0u; p < 2u; ++p) {
            gpio_init(pins[p]);
            gpio_set_dir(pins[p], GPIO_IN);
            gpio_disable_pulls(pins[p]);
        }
        raw_count[i] = 0;
        previous_ab[i] = read_ab(w);
    }
    const int32_t zero[MC_WHEELS] = {0, 0};
    mc_init(&mc, cfg, zero, mcp_now_ms());
    servo_pwm_init(mc.servo_us);

    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(PINS[MC_LEFT].a_pin, edges, true, encoder_irq);
    gpio_set_irq_enabled(PINS[MC_LEFT].b_pin, edges, true);
    gpio_set_irq_enabled(PINS[MC_RIGHT].a_pin, edges, true);
    gpio_set_irq_enabled(PINS[MC_RIGHT].b_pin, edges, true);

    return hb_add_hook(hb, (uint16_t)(cfg->tick_ms / hb_ms), 0u, on_tick, 0) >= 0;
}

void mcp_set_speed(int16_t left_mm_s, int16_t right_mm_s) {
    critical_section_enter_blocking(&lock);
    mc_set_speed(&mc, left_mm_s, right_mm_s, mcp_now_ms());
    critical_section_exit(&lock);
}

void mcp_stop(void) {
    critical_section_enter_blocking(&lock);
    mc_stop(&mc);
    write_motor(MC_LEFT, 0); /* at once, not at the next tick */
    write_motor(MC_RIGHT, 0);
    critical_section_exit(&lock);
}

void mcp_estop(void) {
    critical_section_enter_blocking(&lock);
    mc_estop(&mc);
    write_motor(MC_LEFT, 0); /* at once, not at the next tick */
    write_motor(MC_RIGHT, 0);
    critical_section_exit(&lock);
}

void mcp_clear_faults(void) {
    critical_section_enter_blocking(&lock);
    mc_clear_faults(&mc);
    critical_section_exit(&lock);
}

int mcp_set_servo(int deg) {
    critical_section_enter_blocking(&lock);
    const int set = mc_set_servo(&mc, deg, mcp_now_ms());
    critical_section_exit(&lock);
    return set;
}

void mcp_status(mcp_status_t *s) {
    const uint32_t now = mcp_now_ms();
    critical_section_enter_blocking(&lock);
    read_counts(s->counts);
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        s->travel_mm[i] = mc_travel_mm(&mc, i);
        s->speed_mm_s[i] = mc_speed_mm_s(&mc, i);
        s->duty_permille[i] = mc.w[i].duty;
    }
    s->heading_cdeg = mc_heading_cdeg(&mc);
    s->servo_us = mc.servo_us;
    s->servo_deg = (int8_t)mc_servo_deg(&mc);
    s->servo_settled = mc_servo_settled(&mc, now);
    s->faults = mc_faults(&mc);
    s->ticks = ticks;
    s->tick_max_us = tick_max_us;
    critical_section_exit(&lock);
}
