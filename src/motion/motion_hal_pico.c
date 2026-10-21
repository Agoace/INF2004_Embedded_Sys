/**
 * @file motion_hal_pico.c
 * @brief Pico SDK implementation of motion_hal.h: motor PWM on GP8-GP11
 *        (sole owner), time, critical sections and the debug pin.
 *
 * PWM setup matches src/motor_speed_calibration.c: both pins of each motor
 * on PWM, wrap 6249 at clkdiv 1 (about 20 kHz). Forward pin carries the duty
 * and the reverse pin is 0, or the other way round.
 */
#include "motion_hal.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/time.h"

#include "encoder.h"
#include "motion_config.h"

#define PWM_WRAP    6249u

typedef struct
{
    uint fwd_pin;
    uint rev_pin;
} motor_pins_t;

static const motor_pins_t g_motor_1 = { MOTOR_1_FWD_GPIO, MOTOR_1_REV_GPIO };
static const motor_pins_t g_motor_2 = { MOTOR_2_FWD_GPIO, MOTOR_2_REV_GPIO };

static inline const motor_pins_t *
side_to_pins (motor_id_t motor)
{
#if MOTION_MOTOR_1_IS_LEFT
    return (MOTOR_LEFT == motor) ? &g_motor_1 : &g_motor_2;
#else
    return (MOTOR_LEFT == motor) ? &g_motor_2 : &g_motor_1;
#endif
}

static void
init_motor (const motor_pins_t * p_pins)
{
    gpio_set_function(p_pins->fwd_pin, GPIO_FUNC_PWM);
    gpio_set_function(p_pins->rev_pin, GPIO_FUNC_PWM);

    uint const slice = pwm_gpio_to_slice_num(p_pins->fwd_pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(p_pins->fwd_pin, 0u);
    pwm_set_gpio_level(p_pins->rev_pin, 0u);
}

/** Initialise PWM (outputs 0), encoders and the debug pin. */
void
motion_hal_pico_init (void)
{
    init_motor(&g_motor_1);
    init_motor(&g_motor_2);
    encoder_init();

#if (MOTION_DEBUG_GPIO >= 0)
    gpio_init(MOTION_DEBUG_GPIO);
    gpio_set_dir(MOTION_DEBUG_GPIO, GPIO_OUT);
    gpio_put(MOTION_DEBUG_GPIO, false);
#endif
}

uint16_t
motion_hal_encoder_count (motor_id_t motor)
{
    return encoder_count(motor);
}

void
motion_hal_encoder_edges (motor_id_t motor, uint32_t * p_a, uint32_t * p_b)
{
    encoder_diag_t diag;

    encoder_get_diag(motor, &diag);
    *p_a = diag.a_edges;
    *p_b = diag.b_edges;
}

void
motion_hal_pwm_write (motor_id_t motor, int16_t permille)
{
    const motor_pins_t * p_pins = side_to_pins(motor);
    int32_t magnitude = (permille < 0) ? -(int32_t) permille : permille;

    if (magnitude > 1000)
    {
        magnitude = 1000;
    }

    uint16_t const level =
        (uint16_t) (((uint32_t) magnitude * (PWM_WRAP + 1u)) / 1000u);

    /* Zero the opposite pin first so both pins are never high together. */
    if (permille >= 0)
    {
        pwm_set_gpio_level(p_pins->rev_pin, 0u);
        pwm_set_gpio_level(p_pins->fwd_pin, level);
    }
    else
    {
        pwm_set_gpio_level(p_pins->fwd_pin, 0u);
        pwm_set_gpio_level(p_pins->rev_pin, level);
    }
}

void
motion_hal_pwm_zero_all (void)
{
    pwm_set_gpio_level(MOTOR_1_FWD_GPIO, 0u);
    pwm_set_gpio_level(MOTOR_1_REV_GPIO, 0u);
    pwm_set_gpio_level(MOTOR_2_FWD_GPIO, 0u);
    pwm_set_gpio_level(MOTOR_2_REV_GPIO, 0u);
}

uint32_t
motion_hal_irq_save (void)
{
    return save_and_disable_interrupts();
}

void
motion_hal_irq_restore (uint32_t state)
{
    restore_interrupts(state);
}

uint32_t
motion_hal_time_us (void)
{
    return time_us_32();
}

void
motion_hal_debug_pin (bool level)
{
#if (MOTION_DEBUG_GPIO >= 0)
    gpio_put(MOTION_DEBUG_GPIO, level);
#else
    (void) level;
#endif
}
