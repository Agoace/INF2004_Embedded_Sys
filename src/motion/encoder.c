/**
 * @file encoder.c
 * @brief Quadrature encoder ISR (Pico SDK). Single writer of the wrapping
 *        16-bit counters. Follows docs/INTERRUPT_CONVENTIONS.md: short
 *        handler, no printf, no floating point, volatile shared state.
 */
#include "encoder.h"

#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "motion_config.h"

typedef struct
{
    uint                a_pin;
    uint                b_pin;
    int8_t              sign;
    volatile uint16_t   count;
    volatile uint8_t    prev_state;
    volatile uint32_t   a_edges;
    volatile uint32_t   b_edges;
    volatile uint32_t   invalid;
} encoder_state_t;

/* Index 0 = Motor 1, 1 = Motor 2 (physical order). */
static encoder_state_t g_enc[2] =
{
    { .a_pin = ENC_1_A_GPIO, .b_pin = ENC_1_B_GPIO, .sign = ENC_1_SIGN },
    { .a_pin = ENC_2_A_GPIO, .b_pin = ENC_2_B_GPIO, .sign = ENC_2_SIGN },
};

/* Same table as src/motor_rotation_drift.c: index = (prev << 2) | current. */
static const int8_t g_quad_table[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static inline uint8_t
read_state (const encoder_state_t * p_enc)
{
    return (uint8_t) ((gpio_get(p_enc->a_pin) << 1u) | gpio_get(p_enc->b_pin));
}

static inline encoder_state_t *
side_to_enc (motor_id_t motor)
{
#if MOTION_MOTOR_1_IS_LEFT
    return &g_enc[(MOTOR_LEFT == motor) ? 0 : 1];
#else
    return &g_enc[(MOTOR_LEFT == motor) ? 1 : 0];
#endif
}

static void
service_encoder (encoder_state_t * p_enc)
{
    uint32_t const a_events = gpio_get_irq_event_mask(p_enc->a_pin);
    uint32_t const b_events = gpio_get_irq_event_mask(p_enc->b_pin);

    if ((0u == a_events) && (0u == b_events))
    {
        return;
    }

    if (0u != a_events)
    {
        gpio_acknowledge_irq(p_enc->a_pin, a_events);
        p_enc->a_edges++;
    }

    if (0u != b_events)
    {
        gpio_acknowledge_irq(p_enc->b_pin, b_events);
        p_enc->b_edges++;
    }

    uint8_t const current = read_state(p_enc);
    uint8_t const prev    = p_enc->prev_state;

    if (3u == (uint8_t) (current ^ prev))
    {
        /* Both phases moved between two services: direction unknown. */
        p_enc->invalid++;
    }
    else
    {
        int8_t const step = g_quad_table[(prev << 2u) | current];

        /* uint16_t arithmetic wraps by design (Tribunal 1). */
        p_enc->count = (uint16_t) (p_enc->count + (int16_t) (step * p_enc->sign));
    }

    p_enc->prev_state = current;
}

static void
encoder_irq_handler (void)
{
    service_encoder(&g_enc[0]);
    service_encoder(&g_enc[1]);
}

void
encoder_init (void)
{
    uint32_t mask = 0u;

    for (uint32_t i = 0u; i < 2u; i++)
    {
        encoder_state_t * p_enc = &g_enc[i];

        gpio_init(p_enc->a_pin);
        gpio_set_dir(p_enc->a_pin, GPIO_IN);
        gpio_disable_pulls(p_enc->a_pin);
        gpio_init(p_enc->b_pin);
        gpio_set_dir(p_enc->b_pin, GPIO_IN);
        gpio_disable_pulls(p_enc->b_pin);

        p_enc->count      = 0u;
        p_enc->prev_state = read_state(p_enc);

        mask |= (1u << p_enc->a_pin) | (1u << p_enc->b_pin);
    }

    /* Raw handler: does not take the single shared SDK GPIO callback. */
    gpio_add_raw_irq_handler_masked(mask, encoder_irq_handler);

    for (uint32_t i = 0u; i < 2u; i++)
    {
        uint32_t const events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

        gpio_set_irq_enabled(g_enc[i].a_pin, events, true);
        gpio_set_irq_enabled(g_enc[i].b_pin, events, true);
    }

    irq_set_enabled(IO_IRQ_BANK0, true);
}

uint16_t
encoder_count (motor_id_t motor)
{
    return side_to_enc(motor)->count;
}

void
encoder_count_pair (uint16_t * p_left, uint16_t * p_right)
{
    uint32_t const irq = save_and_disable_interrupts();

    *p_left  = side_to_enc(MOTOR_LEFT)->count;
    *p_right = side_to_enc(MOTOR_RIGHT)->count;
    restore_interrupts(irq);
}

void
encoder_get_diag (motor_id_t motor, encoder_diag_t * p_diag)
{
    encoder_state_t const * p_enc = side_to_enc(motor);
    uint32_t const irq = save_and_disable_interrupts();

    p_diag->a_edges = p_enc->a_edges;
    p_diag->b_edges = p_enc->b_edges;
    p_diag->invalid = p_enc->invalid;
    restore_interrupts(irq);
}
