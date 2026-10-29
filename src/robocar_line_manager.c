/**
 * @file robocar_line_manager.c
 * @brief Integrated bare-metal line-following checkpoint: Manager + Line +
 *        Motion on one core.
 *
 *  20 ms timer IRQ = Motion Control Tick   (motion_tick(), PID deadline)
 *  1 ms main loop  = Line sampling, Motion event forwarding, Manager step
 *  GPIO raw IRQs   = GP20 / GP21 short-press edges
 *
 * SAFETY LIMITATION: this checkpoint has NO physical emergency stop. GP20 and
 * GP21 are both line-follow toggles. To stop the robot, remove motor power.
 * The Manager stops the wheels when the line is lost, when a recovery runs out
 * of distance, and when a line sensor or the encoder self-test fails.
 *
 * Why raw GPIO handlers: the Pico SDK has one shared GPIO callback per core,
 * and the encoder ISR already uses a raw handler. Raw handlers let the two
 * button edges coexist with the encoder edges (docs/INTERRUPT_CONVENTIONS.md).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "ir_module_pins.h"
#include "line_detector.h"

#include "manager.h"
#include "manager_button.h"
#include "manager_types.h"
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hal.h"

#include "imu_config.h"
#include "imu_core.h"
#include "imu_feed.h"
#include "imu_hal.h"
#include "imu_snapshot.h"

#define LINE_SENSOR_COUNT       3u
#define LINE_SAMPLES            8u
#define BUTTON_A_GPIO           20u
#define BUTTON_B_GPIO           21u
#define MANAGER_PERIOD_MS       1u
#define REPORT_MS               500u
#define IMU_REPORT_MS           100u    /* 10 Hz IMU diagnostic line. */

static const uint8_t g_line_gpio[LINE_SENSOR_COUNT] =
{
    LINE_IR_LEFT_ANALOG_GPIO, LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
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

static line_detector_t        g_lines[LINE_SENSOR_COUNT];
static repeating_timer_t      g_tick_timer;
static imu_feed_t             g_imu_feed;

/* ---- Button edge capture (ISR -> 1 ms loop) ---------------------------- */

typedef struct
{
    volatile bool     down;         /* Level at the last captured edge.        */
    volatile uint32_t edge_ms;      /* Time of the last captured edge.         */
    volatile uint32_t edge_seq;     /* Increments per accepted edge.           */
} button_capture_t;

static button_capture_t       g_capture[MANAGER_BUTTON_COUNT];
static manager_button_state_t g_filter[MANAGER_BUTTON_COUNT];
static uint32_t               g_filter_seen[MANAGER_BUTTON_COUNT];

static const uint8_t g_button_gpio[MANAGER_BUTTON_COUNT] =
{
    BUTTON_A_GPIO, BUTTON_B_GPIO,
};

/** Capture one edge. Keep it short: no print, no Manager call, no sleep. */
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

    /* Active low with the internal pull-up. */
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

/* ---- 20 ms Motion tick ------------------------------------------------- */

static bool
tick_cb (repeating_timer_t * p_rt)
{
    (void) p_rt;
    motion_tick();      /* The tick context is Motion's only writer. */
    return true;
}

/* ---- 1 ms work --------------------------------------------------------- */

static void
sample_line (line_observation_t * p_obs)
{
    uint32_t const now_us = time_us_32();
    uint16_t samples[LINE_SAMPLES];

    p_obs->black_pattern = 0u;
    p_obs->unknown_mask  = 0u;
    p_obs->fault_mask    = 0u;
    p_obs->reserved      = 0u;

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
            case LINE_BLACK:        p_obs->black_pattern |= bit; break;
            case LINE_SENSOR_FAULT: p_obs->fault_mask    |= bit; break;
            case LINE_UNKNOWN:      p_obs->unknown_mask  |= bit; break;
            default: break;
        }
    }
}

/** Forward every Motion event to the Manager's bounded queue. */
static void
forward_motion_events (void)
{
    motion_evt_t ev;

    while (motion_get_event(&ev))
    {
        manager_evt_t mev;

        mev.kind           = (uint8_t) MANAGER_EVT_MOTION;
        mev.payload.motion = ev;
        (void) manager_post_event(&mev);
    }
}

/** Turn captured edges into debounced short presses. */
static void
poll_buttons (uint32_t now_ms)
{
    for (uint32_t b = 0u; b < MANAGER_BUTTON_COUNT; b++)
    {
        uint32_t const irq    = save_and_disable_interrupts();
        bool const     down   = g_capture[b].down;
        uint32_t const edge_ms = g_capture[b].edge_ms;
        uint32_t const seq    = g_capture[b].edge_seq;
        restore_interrupts(irq);

        if (seq == g_filter_seen[b])
        {
            continue;
        }

        g_filter_seen[b] = seq;

        if (manager_button_edge(&g_filter[b], down, edge_ms))
        {
            (void) manager_post_button((manager_button_t) b, now_ms);
        }
    }
}

static void
report (const line_observation_t * p_obs)
{
    manager_stats_t stats;

    manager_get_stats(&stats);

    printf("mgr=%s dir=%s fault=0x%03lx line=0x%02x recov=%lu reacq=%lu "
           "tog=%lu ovr=%lu\n",
           manager_state_name(manager_get_state()),
           manager_dir_name(manager_get_line_direction()),
           (unsigned long) manager_get_fault_bits(),
           (unsigned) p_obs->black_pattern,
           (unsigned long) stats.recoveries,
           (unsigned long) stats.reacquisitions,
           (unsigned long) stats.button_toggles,
           (unsigned long) stats.line_overwrites);
}

/**
 * Hand the IMU this cycle's real encoder travel.
 *
 * Motion stays the encoder owner: motion_consume_travel() hands out signed
 * per-wheel deltas and imu_feed converts them. While the Manager is in
 * RECOVERY the chassis is reversing, so the feed publishes zero distance and
 * a recovery can never start or modify a hump episode.
 */
static void
feed_imu_encoder_input (void)
{
    int16_t d_left  = 0;
    int16_t d_right = 0;

    motion_consume_travel(&d_left, &d_right);

    bool const reverse = (MANAGER_RECOVERY == manager_get_state());
    imu_encoder_input_t input;

    (void) imu_feed_step(&g_imu_feed, d_left, d_right, reverse, &input);
    imu_publish_encoder_input(&input);
}

/** 10 Hz IMU line: terrain, height, the encoder feed and sensor health. */
static void
report_imu (uint32_t now_ms)
{
    imu_diag_t diag;
    imu_snapshot_t snapshot;

    imu_diag_read(&diag);

    if (!imu_snapshot_read(&snapshot))
    {
        return;
    }

    imu_snapshot_fields_t fields;

    imu_snapshot_unpack(&snapshot, &fields);

    long const hump_tenths = (long) lroundf(diag.hump_mm * 10.0f);
    long const hump_abs    = (hump_tenths < 0) ? -hump_tenths : hump_tenths;

    printf("imu t=%lums acc=%ld,%ld,%ldmg pitch=%ldddg roll=%ldddg "
           "tilt=%ldddg terr=%s h=%s%ld.%ldmm hi=%umm "
           "dd=%dmm enc=%d/%d gain=%ld ctrl4=0x%02x i2c=%lu inv=%lu "
           "cal=%lu miss=%lu\n",
           (unsigned long) now_ms,
           (long) lroundf(diag.accel_filt_g[0] * 1000.0f),
           (long) lroundf(diag.accel_filt_g[1] * 1000.0f),
           (long) lroundf(diag.accel_filt_g[2] * 1000.0f),
           (long) lroundf(diag.pitch_deg * 10.0f),
           (long) lroundf(diag.roll_deg * 10.0f),
           (long) lroundf(diag.tilt_deg * 10.0f),
           imu_terrain_name(diag.terrain),
           (hump_tenths < 0) ? "-" : "", hump_abs / 10, hump_abs % 10,
           (unsigned) fields.highest_hump_mm,
           (int) diag.distance_delta_mm,
           (int) diag.encoder_left_delta, (int) diag.encoder_right_delta,
           (long) lroundf(diag.gravity_gain * 1000.0f),
           (unsigned) diag.accel_ctrl4,
           (unsigned long) diag.i2c_errors,
           (unsigned long) diag.invalid_samples,
           (unsigned long) diag.calibration_rejects,
           (unsigned long) diag.missed_distance);
}

int
main (void)
{
    stdio_init_all();
    motion_hal_pico_init();     /* PWM = 0 first. */
    motion_init();

    (void) imu_hal_bus_init();  /* I2C0 GP0/GP1, DRDY GP6. */

    adc_init();
    for (uint32_t s = 0u; s < LINE_SENSOR_COUNT; s++)
    {
        adc_gpio_init(g_line_gpio[s]);
        line_detector_init(&g_lines[s], &g_line_cfg);
    }

    uint32_t const boot_ms = to_ms_since_boot(get_absolute_time());

    for (uint32_t b = 0u; b < MANAGER_BUTTON_COUNT; b++)
    {
        gpio_init(g_button_gpio[b]);
        gpio_set_dir(g_button_gpio[b], GPIO_IN);
        gpio_pull_up(g_button_gpio[b]);

        manager_button_init(&g_filter[b], !gpio_get(g_button_gpio[b]), boot_ms);
        g_filter_seen[b] = 0u;
    }

    /* One raw handler per button, so the encoder ISR is untouched. */
    gpio_add_raw_irq_handler(BUTTON_A_GPIO, button_a_irq);
    gpio_add_raw_irq_handler(BUTTON_B_GPIO, button_b_irq);
    gpio_set_irq_enabled(BUTTON_A_GPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         true);
    gpio_set_irq_enabled(BUTTON_B_GPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
                         true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    sleep_ms(1500);
    printf("\nrobocar_line_manager: Manager + Line + Motion + IMU checkpoint\n");
    printf("WARNING: no physical emergency stop in this build.\n");
    printf("GP20 and GP21 are both line-follow toggles. "
           "Remove motor power to stop.\n");
    printf("IMU: LSM303DLHC on I2C0, %.0f Hz step, moving hump profile. "
           "Keep the robot parked and flat for the first 2 s.\n",
           1000.0f / (float) IMU_CFG_PERIOD_MS);

    (void) add_repeating_timer_ms(-(int32_t) MOTION_TICK_MS, tick_cb, NULL,
                                  &g_tick_timer);

    manager_init(boot_ms);

    imu_feed_init(&g_imu_feed);
    imu_init(boot_ms);          /* Self-test, then flat calibration (~2 s). */

    uint32_t last_report     = boot_ms;
    uint32_t last_imu_report = boot_ms;
    uint32_t next_imu        = boot_ms;

    for (;;)
    {
        uint32_t const now_ms = to_ms_since_boot(get_absolute_time());
        line_observation_t obs;

        sample_line(&obs);
        manager_publish_line(&obs);

        forward_motion_events();
        poll_buttons(now_ms);
        manager_step(now_ms);

        /* 13 ms IMU cycle. The Manager step has already run, so the reverse
         * gate in the feed sees this cycle's state. */
        if ((int32_t) (now_ms - next_imu) >= 0)
        {
            feed_imu_encoder_input();
            imu_step(now_ms);

            next_imu += IMU_CFG_PERIOD_MS;

            if ((int32_t) (now_ms - next_imu) >= 0)
            {
                next_imu = now_ms + IMU_CFG_PERIOD_MS;
            }
        }

        if ((now_ms - last_report) >= REPORT_MS)
        {
            last_report = now_ms;
            report(&obs);
        }

        if ((now_ms - last_imu_report) >= IMU_REPORT_MS)
        {
            last_imu_report = now_ms;
            report_imu(now_ms);
        }

        sleep_ms(MANAGER_PERIOD_MS);
    }
}
