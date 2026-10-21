/**
 * @file robocar_line_follow.c
 * @brief Buddy 2 + Buddy 3 integration target (bare Pico SDK, pre-RTOS).
 *
 * Built only when Buddy 3's line_detector.c is present (after merging the
 * ziven-line_tracing_algo branch). It reuses line_detector.h and
 * ir_module_pins.h unchanged and drives the robot through the motion
 * library:
 *
 *   1 ms main loop   = Line & Barcode Task stand-in (3 x ADC -> pattern ->
 *                      line_bridge_update() -> STEER)
 *   20 ms timer IRQ  = Motion Control Task stand-in (motion_tick())
 *   main loop events = minimal Vehicle Controller stand-in
 *
 * Behaviour: encoder self-test at power-up (FR17; wheels twitch), then IDLE.
 * Start with button GP21 or the 5 s auto-start (FR18). Line lost for more
 * than 1 s stops the robot (the real LINE_SEARCH belongs to the VC).
 * Button GP20 is ESTOP. Barcode capture is left out here because the
 * standalone capture loop blocks for about 200 ms.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#include "ir_module_pins.h"
#include "line_detector.h"

#include "line_motion_bridge.h"
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hal.h"

#define LINE_SENSOR_COUNT       3u
#define LINE_SAMPLES            8u
#define ESTOP_GPIO              20u
#define START_GPIO              21u
#define AUTOSTART_MS            5000u
#define LINE_LOST_STOP_MS       1000u
#define REPORT_MS               500u

static const uint8_t g_line_gpio[LINE_SENSOR_COUNT] =
{
    LINE_IR_LEFT_ANALOG_GPIO, LINE_IR_CENTRE_ANALOG_GPIO, LINE_IR_RIGHT_ANALOG_GPIO,
};

/* Same thresholds as robocar_ir_sensors.c (Buddy 3 calibrates these). */
static const line_detector_config_t g_line_cfg =
{
    .black_threshold  = 1200,
    .white_threshold  = 2600,
    .debounce_buffers = 3,
    .fault_low        = 10,
    .fault_high       = 4085,
    .fault_buffers    = 100,
};

static line_detector_t   g_lines[LINE_SENSOR_COUNT];
static repeating_timer_t g_tick_timer;

typedef enum { VC_SELFTEST, VC_IDLE, VC_RUN, VC_FAULT } vc_state_t;

static bool
tick_cb (repeating_timer_t * p_rt)
{
    (void) p_rt;
    motion_tick();
    return true;
}

static void
estop_irq (void)
{
    uint32_t const ev = gpio_get_irq_event_mask(ESTOP_GPIO);

    gpio_acknowledge_irq(ESTOP_GPIO, ev);

    if (0u != (ev & GPIO_IRQ_EDGE_FALL))
    {
        motion_request_stop(true);
    }
}

static void
sample_line (uint8_t * p_pattern, uint8_t * p_unknown, uint8_t * p_fault)
{
    uint32_t const now_us = time_us_32();
    uint16_t samples[LINE_SAMPLES];

    *p_pattern = 0u;
    *p_unknown = 0u;
    *p_fault   = 0u;

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

        uint8_t const bit = (uint8_t) (1u << s);   /* L = 1, C = 2, R = 4 */

        switch (g_lines[s].current)
        {
            case LINE_BLACK:        *p_pattern |= bit; break;
            case LINE_SENSOR_FAULT: *p_fault   |= bit; break;
            case LINE_UNKNOWN:      *p_unknown |= bit; break;
            default: break;
        }
    }
}

static void
send (motion_cmd_id_t id, int16_t a, int16_t spd)
{
    static uint8_t seq;
    motion_cmd_t cmd = { (uint8_t) id, ++seq, a, spd, 0 };

    (void) motion_command(&cmd);
}

int
main (void)
{
    stdio_init_all();
    motion_hal_pico_init();     /* PWM = 0 first. */
    motion_init();

    adc_init();
    for (uint32_t s = 0u; s < LINE_SENSOR_COUNT; s++)
    {
        adc_gpio_init(g_line_gpio[s]);
        line_detector_init(&g_lines[s], &g_line_cfg);
    }

    gpio_init(START_GPIO);
    gpio_set_dir(START_GPIO, GPIO_IN);
    gpio_pull_up(START_GPIO);
    gpio_init(ESTOP_GPIO);
    gpio_set_dir(ESTOP_GPIO, GPIO_IN);
    gpio_pull_up(ESTOP_GPIO);
    gpio_add_raw_irq_handler(ESTOP_GPIO, estop_irq);
    gpio_set_irq_enabled(ESTOP_GPIO, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    sleep_ms(1500);
    printf("\nrobocar_line_follow: Buddy 2 motion + Buddy 3 line sensing\n");

    (void) add_repeating_timer_ms(-(int32_t) MOTION_TICK_MS, tick_cb, NULL,
                                  &g_tick_timer);

    line_bridge_mark();

    vc_state_t state = VC_SELFTEST;
    send(MOTION_CMD_SELFTEST, 0, 0);

    uint32_t idle_since   = 0u;
    uint32_t lost_since   = 0u;
    bool     lost         = false;
    uint32_t last_report  = 0u;
    line_status_t status  = LINE_STATUS_UNKNOWN;
    uint8_t  pattern      = 0u;

    for (;;)
    {
        uint32_t const now = to_ms_since_boot(get_absolute_time());
        uint8_t unknown;
        uint8_t fault;

        sample_line(&pattern, &unknown, &fault);
        status = line_bridge_update(pattern, unknown, fault);

        motion_evt_t ev;

        while (motion_get_event(&ev))
        {
            printf("< motion evt %u arg %u detail 0x%02x achieved %d\n",
                   ev.id, ev.arg, ev.detail, ev.achieved);

            if (MOTION_EVT_SELFTEST == ev.id)
            {
                state = (1u == ev.arg) ? VC_IDLE : VC_FAULT;
                idle_since = now;
                printf("self-test %s\n", (VC_IDLE == state) ? "PASS" : "FAIL");
            }
            else if ((MOTION_EVT_STALL == ev.id) ||
                     ((MOTION_EVT_STOPPED == ev.id) && (1u == ev.arg)))
            {
                state = VC_FAULT;
                printf("FAULT 0x%03lx\n", (unsigned long) motion_get_fault_bits());
            }
            else
            {
                /* DONE / STEER_STALE / REJECTED: logged only. */
            }
        }

        switch (state)
        {
            case VC_IDLE:
            {
                bool const button = !gpio_get(START_GPIO);

                if (button || ((now - idle_since) >= AUTOSTART_MS))
                {
                    motion_odometer_reset();
                    line_bridge_mark();
                    send(MOTION_CMD_FOLLOW, 0, 0);
                    state = VC_RUN;
                    lost  = false;
                    printf("RUN (FOLLOW)\n");
                }
                break;
            }

            case VC_RUN:
            {
                if (LINE_STATUS_LOST == status)
                {
                    if (!lost)
                    {
                        lost       = true;
                        lost_since = now;
                    }
                    else if ((now - lost_since) >= LINE_LOST_STOP_MS)
                    {
                        send(MOTION_CMD_STOP, 0, 0);
                        state      = VC_IDLE;
                        idle_since = now;
                        printf("line lost > %u ms: STOP\n", LINE_LOST_STOP_MS);
                    }
                }
                else
                {
                    lost = false;
                }
                break;
            }

            default:
            {
                break;
            }
        }

        if ((now - last_report) >= REPORT_MS)
        {
            last_report = now;
            printf("vc=%d line=%d pattern=%u motion=%s travel=%ld mm speed=%ld mm/s\n",
                   (int) state, (int) status, (unsigned) pattern,
                   motion_state_name(motion_get_state()),
                   (long) line_bridge_travel_mm(),
                   (long) line_bridge_speed_mm_s());
        }

        sleep_ms(1);
    }
}
