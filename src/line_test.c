/*
 * Bench test for the normalised line position (src/line_position.c).
 *
 * The test prints the line position every 100 ms. LINE_LOST and
 * LINE_FOUND events are printed at once. Calibrate first:
 *
 *   GP20  spin calibration: the robot turns on the spot over the line,
 *         half a wheel turn left, back to the centre, half right,
 *         back to the centre
 *   GP21  hand calibration: for 4 s, slide the robot across the line
 *
 * Spin limits for USB power: duty at most 20 %, the second motor starts
 * 50 ms after the first, and a stall (no encoder count for 500 ms) or a
 * time limit stops both motors at once.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "ir_module_pins.h"
#include "line_position.h"
#include "pico/stdlib.h"

#define SPIN_BUTTON_PIN 20
#define HAND_BUTTON_PIN 21

#define MOTOR_1_PWM_PIN 8
#define MOTOR_1_REVERSE_PIN 9
#define MOTOR_2_PWM_PIN 10
#define MOTOR_2_REVERSE_PIN 11
#define ENCODER_1_A_PIN 16
#define ENCODER_1_B_PIN 17
#define ENCODER_2_A_PIN 4
#define ENCODER_2_B_PIN 5

#define PWM_WRAP 6249u /* 125 MHz / 6250 = 20 kHz */
#define COUNTS_PER_WHEEL_TURN 2535
#define SPIN_DUTY_PERCENT 20u /* 15 % stalled on the floor (2026-10-27) */
#define SPIN_DUTY_LIMIT_PERCENT 20u
_Static_assert(SPIN_DUTY_PERCENT <= SPIN_DUTY_LIMIT_PERCENT,
               "USB power: keep the spin duty at or below 20 %");
#define SPIN_STAGGER_MS 50u
#define STALL_GRACE_MS 1000u
#define STALL_TIMEOUT_MS 500u
#define SPIN_STEP_COUNTS (COUNTS_PER_WHEEL_TURN / 4) /* a quarter wheel turn */
/* Last step: stop when the line is this close to the centre (-100..+100). */
#define SPIN_CENTRE_POSITION 20
#define SPIN_STEP_TIMEOUT_MS 6000u
#define SPIN_PAUSE_MS 200u

#define SAMPLE_PERIOD_US 1000u
#define SAMPLES_PER_SENSOR 8u
#define CALIBRATE_MS 4000u
#define PRINT_PERIOD_MS 100u
#define BUTTON_POLL_MS 20u

static const unsigned int sensor_gpio[LP_SENSORS] = {
    LINE_IR_LEFT_ANALOG_GPIO,
    LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

/* ---------------- encoders and motors (spin calibration only) ---------- */

typedef struct {
    unsigned int a_pin;
    unsigned int b_pin;
    volatile int32_t count;
    volatile uint8_t previous;
} encoder_t;

static encoder_t encoder_1 = {.a_pin = ENCODER_1_A_PIN, .b_pin = ENCODER_1_B_PIN};
static encoder_t encoder_2 = {.a_pin = ENCODER_2_A_PIN, .b_pin = ENCODER_2_B_PIN};

static const int8_t quadrature_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};

static uint8_t encoder_state(const encoder_t *e) {
    return (uint8_t)((gpio_get(e->a_pin) ? 2u : 0u) | (gpio_get(e->b_pin) ? 1u : 0u));
}

static void encoder_step(encoder_t *e) {
    const uint8_t now = encoder_state(e);
    e->count += quadrature_table[(e->previous << 2u) | now];
    e->previous = now;
}

static void encoder_irq(unsigned int gpio, uint32_t events) {
    (void)events;
    if (gpio == encoder_1.a_pin || gpio == encoder_1.b_pin) {
        encoder_step(&encoder_1);
    } else if (gpio == encoder_2.a_pin || gpio == encoder_2.b_pin) {
        encoder_step(&encoder_2);
    }
}

static void encoder_init(encoder_t *e, bool set_callback) {
    const unsigned int pins[2] = {e->a_pin, e->b_pin};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_disable_pulls(pins[i]);
    }
    e->previous = encoder_state(e);
    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    if (set_callback) {
        gpio_set_irq_enabled_with_callback(e->a_pin, edges, true, encoder_irq);
    } else {
        gpio_set_irq_enabled(e->a_pin, edges, true);
    }
    gpio_set_irq_enabled(e->b_pin, edges, true);
}

static int32_t encoder_read(const encoder_t *e) {
    const uint32_t saved = save_and_disable_interrupts();
    const int32_t count = e->count;
    restore_interrupts(saved);
    return count;
}

static void motor_pins_init(unsigned int forward_pin, unsigned int reverse_pin) {
    gpio_set_function(forward_pin, GPIO_FUNC_PWM);
    gpio_set_function(reverse_pin, GPIO_FUNC_PWM);
    const unsigned int slice = pwm_gpio_to_slice_num(forward_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(forward_pin, 0u);
    pwm_set_gpio_level(reverse_pin, 0u);
}

/* direction: +1 forward, -1 reverse, 0 stop (coast). */
static void motor_set(unsigned int forward_pin, unsigned int reverse_pin, int direction) {
    const uint16_t level =
        (uint16_t)(((PWM_WRAP + 1u) * SPIN_DUTY_PERCENT) / 100u);
    pwm_set_gpio_level(forward_pin, direction > 0 ? level : 0u);
    pwm_set_gpio_level(reverse_pin, direction < 0 ? level : 0u);
}

static void motors_stop(void) {
    motor_set(MOTOR_1_PWM_PIN, MOTOR_1_REVERSE_PIN, 0);
    motor_set(MOTOR_2_PWM_PIN, MOTOR_2_REVERSE_PIN, 0);
}

static uint32_t abs_count(int32_t value) {
    return value < 0 ? (uint32_t)(-value) : (uint32_t)value;
}

static void read_sensors(uint16_t raw[LP_SENSORS]);

/* What ends a spin step for each wheel, measured from the spin start
 * ("home") so that coasting and errors do not add up step by step. */
typedef enum {
    GOAL_OUT,  /* wheel is SPIN_STEP_COUNTS away from home */
    GOAL_HOME, /* wheel has come back to (or just past) home */
} spin_goal_t;

static bool wheel_done(spin_goal_t goal, int32_t from_home, int32_t from_home_at_start) {
    if (goal == GOAL_OUT) {
        /* Starting on one side: first cross home, then go out on the other. */
        const bool started_out =
            abs_count(from_home_at_start) >= (uint32_t)SPIN_STEP_COUNTS / 2u;
        const bool other_side = (from_home < 0) != (from_home_at_start < 0);
        return (!started_out || other_side) &&
               abs_count(from_home) >= (uint32_t)SPIN_STEP_COUNTS;
    }
    return from_home == 0 || (from_home < 0) != (from_home_at_start < 0);
}

/* Turns on the spot, sampling the line sensors every 1 ms.
 * direction +1: motor 1 forward, motor 2 reverse (robot turns right).
 * Each wheel stops by itself when it reaches its goal. With stop_on_line
 * (the calibration is already valid), both wheels stop as soon as the line
 * is under the centre sensor; the encoder goal is then only a limit.
 * Returns false (motors stopped) on a stall or the time limit. */
static bool spin_step(lp_t *lp, int direction, spin_goal_t goal, int32_t home_1,
                      int32_t home_2, bool stop_on_line) {
    const int32_t start_1 = encoder_read(&encoder_1) - home_1;
    const int32_t start_2 = encoder_read(&encoder_2) - home_2;
    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    bool running_1 = true;
    bool running_2 = false; /* starts SPIN_STAGGER_MS later */
    bool started_2 = false;
    int32_t last_1 = start_1;
    int32_t last_2 = start_2;
    uint32_t moved_1_ms = t0;
    uint32_t moved_2_ms = t0;
    bool on_line = false;

    motor_set(MOTOR_1_PWM_PIN, MOTOR_1_REVERSE_PIN, direction);
    absolute_time_t next_sample = get_absolute_time();

    while (running_1 || running_2 || !started_2) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!started_2 && now - t0 >= SPIN_STAGGER_MS) {
            started_2 = true;
            running_2 = true;
            moved_2_ms = now;
            motor_set(MOTOR_2_PWM_PIN, MOTOR_2_REVERSE_PIN, -direction);
        }

        const int32_t c1 = encoder_read(&encoder_1) - home_1;
        const int32_t c2 = encoder_read(&encoder_2) - home_2;
        if (c1 != last_1) { last_1 = c1; moved_1_ms = now; }
        if (c2 != last_2) { last_2 = c2; moved_2_ms = now; }
        if (running_1 && (on_line || wheel_done(goal, c1, start_1))) {
            running_1 = false;
            motor_set(MOTOR_1_PWM_PIN, MOTOR_1_REVERSE_PIN, 0);
        }
        if (running_2 && (on_line || wheel_done(goal, c2, start_2))) {
            running_2 = false;
            motor_set(MOTOR_2_PWM_PIN, MOTOR_2_REVERSE_PIN, 0);
        }
        if (on_line && !started_2) {
            started_2 = true; /* line found before motor 2 started */
        }

        const bool past_grace = now - t0 >= STALL_GRACE_MS;
        const bool stall_1 = running_1 && past_grace && now - moved_1_ms >= STALL_TIMEOUT_MS;
        const bool stall_2 = running_2 && past_grace && now - moved_2_ms >= STALL_TIMEOUT_MS;
        if (stall_1 || stall_2 || now - t0 >= SPIN_STEP_TIMEOUT_MS) {
            motors_stop();
            printf("[spin] STOPPED: %s  from start M1 %ld  M2 %ld\n",
                   stall_1 ? "motor 1 stalled" : stall_2 ? "motor 2 stalled" : "time limit",
                   (long)c1, (long)c2);
            return false;
        }

        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            uint16_t raw[LP_SENSORS];
            read_sensors(raw);
            if (stop_on_line) {
                (void)lp_update(lp, raw, now);
                on_line = lp->norm[LP_CENTRE] >= LP_FOUND_PCT &&
                          lp->position >= -SPIN_CENTRE_POSITION &&
                          lp->position <= SPIN_CENTRE_POSITION;
            } else {
                lp_calib_add(lp, raw);
            }
        }
    }
    printf("[spin] %+d done in %lu ms%s; from start M1 %ld  M2 %ld\n", direction,
           (unsigned long)(to_ms_since_boot(get_absolute_time()) - t0),
           on_line ? ", stopped on the line" : "",
           (long)(encoder_read(&encoder_1) - home_1),
           (long)(encoder_read(&encoder_2) - home_2));
    return true;
}

/* Sampling during the short pause between steps (robot still). */
static void spin_pause(lp_t *lp) {
    const absolute_time_t end = make_timeout_time_ms(SPIN_PAUSE_MS);
    while (!time_reached(end)) {
        uint16_t raw[LP_SENSORS];
        read_sensors(raw);
        lp_calib_add(lp, raw);
        sleep_us(SAMPLE_PERIOD_US);
    }
}

/* ---------------- line sensors ------------------------------------------ */

static void read_sensors(uint16_t raw[LP_SENSORS]) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_select_input(sensor_gpio[s] - PICO_ADC_GPIO_MIN);
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < SAMPLES_PER_SENSOR; ++i) {
            sum += adc_read();
        }
        raw[s] = (uint16_t)(sum / SAMPLES_PER_SENSOR);
    }
}

/* 21-character bar: '|' at the centre, '^' at the line position. */
static void print_bar(int position) {
    char bar[22];
    for (int i = 0; i < 21; ++i) bar[i] = '-';
    bar[10] = '|';
    bar[(position + 100) / 10] = '^';
    bar[21] = '\0';
    printf("L %s R", bar);
}

static bool calib_report(lp_t *lp);

static void spin_calibrate(lp_t *lp) {
    /* Left, centre, right: a quarter wheel turn each way, measured from the
     * start. Then the calibration is made, and the last step turns back
     * until the line is under the centre sensor.
     * Direction +1 (motor 1 forward, motor 2 reverse) turns the robot right:
     * motor 1 is the left wheel (bench, 2026-10-27). */
    printf("\n[spin] Spin calibration at %u %% duty, %u counts (a quarter wheel turn).\n",
           SPIN_DUTY_PERCENT, SPIN_STEP_COUNTS);
    const int32_t home_1 = encoder_read(&encoder_1);
    const int32_t home_2 = encoder_read(&encoder_2);
    lp_calib_begin(lp);
    bool ok = true;
    printf("[spin] step 1: left\n");
    ok = spin_step(lp, -1, GOAL_OUT, home_1, home_2, false);
    if (ok) {
        spin_pause(lp);
        printf("[spin] step 2: back through the centre to the right\n");
        ok = spin_step(lp, +1, GOAL_OUT, home_1, home_2, false);
    }
    if (ok) {
        spin_pause(lp);
    }
    motors_stop();
    if (!ok) {
        printf("[spin] Spin not complete. The calibration is not changed.\n\n");
        return;
    }
    const bool calibrated = calib_report(lp);
    printf("[spin] step 3: back to the centre%s\n",
           calibrated ? ", stop on the line" : " by encoder count");
    (void)spin_step(lp, -1, GOAL_HOME, home_1, home_2, calibrated);
    motors_stop();
    printf("\n");
}

static void hand_calibrate(lp_t *lp) {
    printf("\n[calib] Slide the robot slowly across the line, left and right,\n");
    printf("[calib] so every sensor passes over it. %u s...\n", CALIBRATE_MS / 1000u);
    lp_calib_begin(lp);

    absolute_time_t next_sample = get_absolute_time();
    const absolute_time_t end = make_timeout_time_ms(CALIBRATE_MS);
    absolute_time_t next_tick = make_timeout_time_ms(1000u);
    unsigned int seconds_left = CALIBRATE_MS / 1000u;
    while (!time_reached(end)) {
        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            uint16_t raw[LP_SENSORS];
            read_sensors(raw);
            lp_calib_add(lp, raw);
        }
        if (time_reached(next_tick)) {
            next_tick = delayed_by_ms(next_tick, 1000u);
            --seconds_left;
            printf("[calib] %u s  range L %u-%u  C %u-%u  R %u-%u\n", seconds_left,
                   lp->cal_min[0], lp->cal_max[0], lp->cal_min[1], lp->cal_max[1],
                   lp->cal_min[2], lp->cal_max[2]);
        }
    }
    (void)calib_report(lp);
    printf("\n");
}

static bool calib_report(lp_t *lp) {
    printf("[calib] seen range L %u-%u  C %u-%u  R %u-%u\n", lp->cal_min[0],
           lp->cal_max[0], lp->cal_min[1], lp->cal_max[1], lp->cal_min[2],
           lp->cal_max[2]);
    unsigned int bad = 0u;
    const lp_calib_result_t result = lp_calib_end(lp, &bad);
    if (result == LP_CALIB_OK) {
        printf("[calib] OK. Line reads %s than the floor.\n",
               lp_line_reads_high(lp) ? "HIGHER" : "LOWER");
        for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
            printf("[calib]   %c: floor %4u  line %4u\n", "LCR"[s], lp_floor(lp, s),
                   lp_line(lp, s));
        }
    } else if (result == LP_CALIB_LOW_CONTRAST) {
        printf("[calib] FAILED: range below %u ADC on:%s%s%s.\n",
               LP_MIN_CONTRAST_ADC, (bad & 1u) ? " L" : "", (bad & 2u) ? " C" : "",
               (bad & 4u) ? " R" : "");
        if (lp_calibrated(lp)) {
            printf("[calib] The previous calibration stays in use.\n");
        }
    } else {
        printf("[calib] FAILED: no data.\n");
    }
    return result == LP_CALIB_OK;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nNormalised line position test\n");
    printf("L/C/R = GP%u/GP%u/GP%u. GP20: spin calibration. GP21: hand calibration.\n",
           LINE_IR_LEFT_ANALOG_GPIO, LINE_IR_CENTRE_ANALOG_GPIO,
           LINE_IR_RIGHT_ANALOG_GPIO);

    adc_init();
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_gpio_init(sensor_gpio[s]);
    }
    gpio_init(SPIN_BUTTON_PIN);
    gpio_set_dir(SPIN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SPIN_BUTTON_PIN);
    gpio_init(HAND_BUTTON_PIN);
    gpio_set_dir(HAND_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(HAND_BUTTON_PIN);
    motor_pins_init(MOTOR_1_PWM_PIN, MOTOR_1_REVERSE_PIN);
    motor_pins_init(MOTOR_2_PWM_PIN, MOTOR_2_REVERSE_PIN);
    motors_stop();
    encoder_init(&encoder_1, true);
    encoder_init(&encoder_2, false);

    lp_t lp;
    lp_init(&lp);
    printf("Not calibrated. Put the robot on the line and press GP20 (spin)\n");
    printf("or GP21 (slide by hand).\n\n");

    bool previous_spin = false;
    bool previous_hand = false;
    uint16_t raw[LP_SENSORS] = {0u, 0u, 0u};
    absolute_time_t next_sample = get_absolute_time();
    absolute_time_t next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
    absolute_time_t next_button_poll = get_absolute_time();

    while (true) {
        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            read_sensors(raw);
            const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            const vc_event_t event = lp_update(&lp, raw, now_ms);
            if (vc_event_type(event) == VC_EVENT_LINE_LOST) {
                const vc_line_side_t side = (vc_line_side_t)vc_event_payload(event);
                printf(">>> LINE_LOST  last seen %s\n",
                       side == VC_LINE_SIDE_LEFT    ? "LEFT"
                       : side == VC_LINE_SIDE_RIGHT ? "RIGHT"
                                                    : "CENTRE (unknown side)");
            } else if (vc_event_type(event) == VC_EVENT_LINE_FOUND) {
                printf(">>> LINE_FOUND\n");
            }
        }

        if (time_reached(next_print)) {
            next_print = delayed_by_ms(next_print, PRINT_PERIOD_MS);
            printf("raw %4u %4u %4u  n %3u %3u %3u %%  ", raw[0], raw[1], raw[2],
                   lp.norm[0], lp.norm[1], lp.norm[2]);
            if (lp_line_seen(&lp)) {
                printf("pos %+4d  ", lp.position);
                print_bar(lp.position);
                printf("%s\n", lp_junction(&lp) ? "  JUNCTION" : "");
            } else {
                printf("no line\n");
            }
        }

        if (time_reached(next_button_poll)) {
            next_button_poll = make_timeout_time_ms(BUTTON_POLL_MS);
            const bool spin_button = !gpio_get(SPIN_BUTTON_PIN);
            const bool hand_button = !gpio_get(HAND_BUTTON_PIN);
            if ((spin_button && !previous_spin) || (hand_button && !previous_hand)) {
                if (spin_button) {
                    spin_calibrate(&lp);
                } else {
                    hand_calibrate(&lp);
                }
                next_sample = get_absolute_time();
                next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
            }
            previous_spin = spin_button;
            previous_hand = hand_button;
        }

        tight_loop_contents();
    }
}
