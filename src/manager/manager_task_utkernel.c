/**
 * @file manager_task_utkernel.c
 * @brief μT-Kernel adapter for the Manager: one Line task and one Manager
 *        task, both on a 1 ms cyclic handler.
 *
 * TEMPLATE: the repository does not contain the μT-Kernel port yet, so this
 * file is compiled only when MANAGER_USE_UTKERNEL is defined. The bare-metal
 * target and the host tests never depend on it.
 *
 * Mapping to the approved plan:
 *  - Line task (priority 11): samples the three ADC sensors, publishes the
 *    newest observation to the Manager's one-element mailbox. It never queues
 *    observations.
 *  - Manager task (priority 12): debounces the two buttons, receives the
 *    discrete event queue (button events from here, motion_evt_t forwarded by
 *    the Motion task), and runs one manager_step() per millisecond.
 *  - Motion task (priority 10, motion_task_utkernel.c): keeps its 20 ms
 *    cyclic handler and remains the only PWM owner.
 */
#ifdef MANAGER_USE_UTKERNEL

#include <tk/tkernel.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "ir_module_pins.h"
#include "line_detector.h"

#include "manager.h"
#include "manager_button.h"
#include "manager_task_utkernel.h"
#include "manager_types.h"
#include "motion_config.h"

#define LINE_TASK_PRI        11
#define MANAGER_TASK_PRI     12
#define LINE_TASK_STACK      1024
#define MANAGER_TASK_STACK   1024

#define LINE_SENSOR_COUNT    3u
#define LINE_SAMPLES         8u
#define BUTTON_A_GPIO        20u
#define BUTTON_B_GPIO        21u

/* ---- Buttons: the ISR captures, the Manager task debounces -------------- */

typedef struct
{
    volatile bool     down;
    volatile uint32_t edge_ms;
    volatile uint32_t edge_seq;
} button_capture_t;

static button_capture_t       g_capture[MANAGER_BUTTON_COUNT];
static manager_button_state_t g_filter[MANAGER_BUTTON_COUNT];
static uint32_t               g_filter_seen[MANAGER_BUTTON_COUNT];

static const uint8_t g_button_gpio[MANAGER_BUTTON_COUNT] =
{
    BUTTON_A_GPIO, BUTTON_B_GPIO,
};

static ID  g_line_task_id;
static ID  g_manager_task_id;
static ID  g_line_cyc_id;
static ID  g_manager_cyc_id;

static void
capture_edge (uint32_t button, uint32_t gpio)
{
    uint32_t const events = gpio_get_irq_event_mask(gpio);

    if (0u == events)
    {
        return;
    }

    gpio_acknowledge_irq(gpio, events);

    if (0u == (events & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)))
    {
        return;
    }

    g_capture[button].down     = !gpio_get(gpio);
    g_capture[button].edge_ms  = time_us_32() / 1000u;
    g_capture[button].edge_seq = g_capture[button].edge_seq + 1u;
}

static void
button_a_irq (void)
{
    capture_edge(MANAGER_BUTTON_GP20, BUTTON_A_GPIO);
}

static void
button_b_irq (void)
{
    capture_edge(MANAGER_BUTTON_GP21, BUTTON_B_GPIO);
}

static uint32_t
now_ms (void)
{
    return time_us_32() / 1000u;
}

static void
poll_buttons (uint32_t now)
{
    for (uint32_t b = 0u; b < MANAGER_BUTTON_COUNT; b++)
    {
        uint32_t const irq     = save_and_disable_interrupts();
        bool const     down    = g_capture[b].down;
        uint32_t const edge_ms = g_capture[b].edge_ms;
        uint32_t const seq     = g_capture[b].edge_seq;
        restore_interrupts(irq);

        if (seq == g_filter_seen[b])
        {
            continue;
        }

        g_filter_seen[b] = seq;

        if (manager_button_edge(&g_filter[b], down, edge_ms))
        {
            (void) manager_post_button((manager_button_t) b, now);
        }
    }
}

/* ---- Line task --------------------------------------------------------- */

static const uint8_t g_line_gpio[LINE_SENSOR_COUNT] =
{
    LINE_IR_LEFT_ANALOG_GPIO, LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

static const line_detector_config_t g_line_cfg =
{
    .black_threshold  = 1200,
    .white_threshold  = 2600,
    .debounce_buffers = 3,
    .fault_low        = 10,
    .fault_high       = 4085,
    .fault_buffers    = 100,
};

static line_detector_t g_lines[LINE_SENSOR_COUNT];

static void
sample_and_publish (void)
{
    uint32_t const now_us = time_us_32();
    uint16_t samples[LINE_SAMPLES];
    line_observation_t obs;

    obs.black_pattern = 0u;
    obs.unknown_mask  = 0u;
    obs.fault_mask    = 0u;
    obs.reserved      = 0u;

    for (uint32_t s = 0u; s < LINE_SENSOR_COUNT; s++)
    {
        adc_select_input(g_line_gpio[s] - PICO_ADC_GPIO_MIN);

        for (uint32_t i = 0u; i < LINE_SAMPLES; i++)
        {
            samples[i] = adc_read();
        }

        line_event_t ev;

        (void) line_detector_process(&g_lines[s], samples, LINE_SAMPLES,
                                     now_us, &ev);

        uint8_t const bit = (uint8_t) (1u << s);

        switch (g_lines[s].current)
        {
            case LINE_BLACK:        obs.black_pattern |= bit; break;
            case LINE_SENSOR_FAULT: obs.fault_mask    |= bit; break;
            case LINE_UNKNOWN:      obs.unknown_mask  |= bit; break;
            default: break;
        }
    }

    manager_publish_line(&obs);
}

static void
line_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    adc_init();

    for (uint32_t s = 0u; s < LINE_SENSOR_COUNT; s++)
    {
        adc_gpio_init(g_line_gpio[s]);
        line_detector_init(&g_lines[s], &g_line_cfg);
    }

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);
        sample_and_publish();
    }
}

/* ---- Manager task ------------------------------------------------------ */

static void
manager_task (INT stacd, void * exinf)
{
    (void) stacd;
    (void) exinf;

    manager_init(now_ms());

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);

        uint32_t const now = now_ms();

        poll_buttons(now);
        manager_step(now);
    }
}

/* ---- Cyclic handlers --------------------------------------------------- */

static void
line_cyclic_handler (void * exinf)
{
    (void) exinf;
    (void) tk_wup_tsk(g_line_task_id);      /* 1 ms: publish one sample. */
}

static void
manager_cyclic_handler (void * exinf)
{
    (void) exinf;
    (void) tk_wup_tsk(g_manager_task_id);   /* 1 ms: one Manager step.   */
}

void
manager_adapter_wake (void)
{
    if (g_manager_task_id > 0)
    {
        (void) tk_wup_tsk(g_manager_task_id);
    }
}

/* ---- Creation ---------------------------------------------------------- */

static ER
create_task (ID * p_id, const char * name, INT pri, INT stack,
             void (*entry)(INT, void *))
{
    T_CTSK ctsk;

    ctsk.tskatr  = TA_HLNG | TA_RNG0;
    ctsk.exinf   = (void *) name;
    ctsk.task    = entry;
    ctsk.itskpri = pri;
    ctsk.stksz   = stack;

    *p_id = tk_cre_tsk(&ctsk);

    return (*p_id < E_OK) ? (ER) *p_id : E_OK;
}

static ER
create_cyclic (ID * p_id, void (*handler)(void *))
{
    T_CCYC ccyc;

    ccyc.cycatr = TA_HLNG;
    ccyc.exinf  = 0;
    ccyc.cychdr = handler;
    ccyc.cyctim = 1;        /* ms, with a 1 ms system tick */
    ccyc.cycphs = 0;

    *p_id = tk_cre_cyc(&ccyc);

    return (*p_id < E_OK) ? (ER) *p_id : E_OK;
}

ER
manager_line_tasks_create (void)
{
    uint32_t const boot_ms = now_ms();
    ER er;

    for (uint32_t b = 0u; b < MANAGER_BUTTON_COUNT; b++)
    {
        gpio_init(g_button_gpio[b]);
        gpio_set_dir(g_button_gpio[b], GPIO_IN);
        gpio_pull_up(g_button_gpio[b]);

        manager_button_init(&g_filter[b], !gpio_get(g_button_gpio[b]), boot_ms);
        g_filter_seen[b] = 0u;
    }

    /* One raw handler per button: the encoder ISR keeps its own handler. */
    gpio_add_raw_irq_handler(BUTTON_A_GPIO, button_a_irq);
    gpio_add_raw_irq_handler(BUTTON_B_GPIO, button_b_irq);
    gpio_set_irq_enabled(BUTTON_A_GPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         true);
    gpio_set_irq_enabled(BUTTON_B_GPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    er = create_task(&g_line_task_id, "line", LINE_TASK_PRI, LINE_TASK_STACK,
                     line_task);
    if (E_OK != er)
    {
        return er;
    }

    er = create_task(&g_manager_task_id, "manager", MANAGER_TASK_PRI,
                     MANAGER_TASK_STACK, manager_task);
    if (E_OK != er)
    {
        return er;
    }

    er = create_cyclic(&g_line_cyc_id, line_cyclic_handler);
    if (E_OK != er)
    {
        return er;
    }

    er = create_cyclic(&g_manager_cyc_id, manager_cyclic_handler);
    if (E_OK != er)
    {
        return er;
    }

    /* Motion (priority 10) is created first, so its command queue exists
     * before the Manager asks for the encoder self-test. */
    er = tk_sta_tsk(g_line_task_id, 0);
    if (E_OK != er)
    {
        return er;
    }

    er = tk_sta_tsk(g_manager_task_id, 0);
    if (E_OK != er)
    {
        return er;
    }

    (void) tk_sta_cyc(g_line_cyc_id);
    (void) tk_sta_cyc(g_manager_cyc_id);

    return E_OK;
}

#endif /* MANAGER_USE_UTKERNEL */
