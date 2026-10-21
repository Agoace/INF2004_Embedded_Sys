/**
 * @file motion_bench.c
 * @brief Buddy 2 bench target for the motion-control library.
 *
 * Bare Pico SDK stand-in for the μT-Kernel Motion Control Task: a 20 ms
 * repeating timer calls motion_tick() (the cyclic handler in the RTOS
 * build). The tick is integer-only and short, so it is acceptable in the
 * timer IRQ (docs/INTERRUPT_CONVENTIONS.md). The main loop is the "Vehicle
 * Controller": it reads USB serial commands, prints events and CSV logs.
 *
 * Evidence this target produces (tribunal / design-review tests):
 *  - A4 PID tuning: "a 0" (step), "v 150 150", "log" -> CSV step responses.
 *  - NFR13: "f 1000" and "r 90" x10 trials, read the DONE "achieved" value.
 *  - NFR16: GP20 button -> motion_request_stop(true). Probe GP20 and the
 *    PWM pins (or GP22 debug pin) on a logic analyser.
 *  - Stall: hold a wheel during "f 1000" -> STALL event.
 *  - WCET: "p" prints the longest motion_tick() time.
 *
 * Power the motors from VIN, not the Raspberry Pi USB port
 * (docs/USB_POWER_SAFETY.md).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#include "motion_config.h"
#include "motion_control.h"
#include "motion_hal.h"
#include "motion_units.h"
#include "robot_shared.h"

#define ESTOP_BUTTON_GPIO   20u
#define LOG_RING_LEN        64u     /* power of two; 1.28 s of ticks */
#define LINE_MAX            64u

static repeating_timer_t g_tick_timer;
static motion_log_t      g_log_ring[LOG_RING_LEN];
static volatile uint32_t g_log_head;
static uint32_t          g_log_tail;
static volatile bool     g_log_on;
static uint32_t          g_log_lost;

static bool              g_steer_refresh;
static int8_t            g_steer;
static uint8_t           g_seq;

/* ---- Tick (timer IRQ) --------------------------------------------------- */
static bool
tick_cb (repeating_timer_t * p_rt)
{
    (void) p_rt;
    motion_tick();
    return true;
}

static void
log_hook (const motion_log_t * p_log)
{
    if (!g_log_on)
    {
        return;
    }

    uint32_t const head = g_log_head;

    if ((head - g_log_tail) >= LOG_RING_LEN)
    {
        g_log_lost++;
        return;
    }

    g_log_ring[head & (LOG_RING_LEN - 1u)] = *p_log;
    g_log_head = head + 1u;
}

/* ---- ESTOP button (raw handler: leaves the SDK callback free) ---------- */
static void
estop_irq (void)
{
    uint32_t const ev = gpio_get_irq_event_mask(ESTOP_BUTTON_GPIO);

    if (0u != (ev & GPIO_IRQ_EDGE_FALL))
    {
        gpio_acknowledge_irq(ESTOP_BUTTON_GPIO, ev);
        motion_request_stop(true);
    }
    else if (0u != ev)
    {
        gpio_acknowledge_irq(ESTOP_BUTTON_GPIO, ev);
    }
}

static void
estop_init (void)
{
    gpio_init(ESTOP_BUTTON_GPIO);
    gpio_set_dir(ESTOP_BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(ESTOP_BUTTON_GPIO);
    gpio_add_raw_irq_handler(ESTOP_BUTTON_GPIO, estop_irq);
    gpio_set_irq_enabled(ESTOP_BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

/* ---- Console ------------------------------------------------------------ */
static void
send (motion_cmd_id_t id, int a, int spd)
{
    motion_cmd_t cmd;

    cmd.id         = (uint8_t) id;
    cmd.seq        = ++g_seq;
    cmd.arg_a      = (int16_t) a;
    cmd.speed_mm_s = (int16_t) spd;
    cmd.reserved   = 0;

    if (!motion_command(&cmd))
    {
        printf("! queue full\n");
    }
    else
    {
        printf("> cmd %u seq %u a=%d spd=%d\n", (unsigned) id, (unsigned) g_seq, a, spd);
    }
}

static void
print_help (void)
{
    printf(
        "motion_bench commands (mm, mm/s, degrees; + angle = left):\n"
        "  f <mm> [spd]     forward          b <mm> [spd]   reverse\n"
        "  r <deg> [spd]    rotate in place  t <deg> [spd]  pivot turn\n"
        "  F [spd]          follow (uses steer)\n"
        "  s <-127..127>    set + refresh steer   q  stop refreshing steer\n"
        "  v <l> <r>        wheel speed set-points (PID step tests)\n"
        "  a 0|1            accel ramp off (step) / on\n"
        "  g kp ki ff ffoff ksync accel   gains, Q8 (256 = 1.0)\n"
        "  w                toggle SLOW      k  encoder self-test\n"
        "  x stop  e estop  z reset  l log CSV on/off  p status  h help\n");
}

static void
print_status (void)
{
    motion_stats_t st;
    motion_gains_t g;
    uint32_t w1;
    uint32_t w2;

    motion_get_stats(&st);
    motion_get_gains(&g);
    motion_get_snapshot(&w1, &w2);

    printf("state=%s faults=0x%03lx L=%d R=%d cpt odo=%u cm\n",
           motion_state_name(motion_get_state()),
           (unsigned long) motion_get_fault_bits(),
           msnap_unpack_speed(w1, MSNAP_W1_L_SHIFT),
           msnap_unpack_speed(w1, MSNAP_W1_R_SHIFT),
           (unsigned) (w2 & 0xFFFFu));
    printf("ticks=%lu wcet=%lu us last=%lu us overruns=%lu stops=%lu "
           "cmd_drop=%lu evt_drop=%lu log_lost=%lu\n",
           (unsigned long) st.ticks, (unsigned long) st.tick_wcet_us,
           (unsigned long) st.tick_last_us, (unsigned long) st.tick_overruns,
           (unsigned long) st.stop_requests, (unsigned long) st.cmd_dropped,
           (unsigned long) st.evt_dropped, (unsigned long) g_log_lost);
    printf("gains kp=%ld ki=%ld ff=%ld ffoff=%ld ksync=%ld accel=%ld | "
           "D=%d mm W=%d mm, 200 mm/s = %ld cpt\n",
           (long) g.kp_q8, (long) g.ki_q8, (long) g.ff_q8,
           (long) g.ff_offset_permille, (long) g.ksync_q8,
           (long) g.accel_q8_per_tick, MOTION_WHEEL_DIAMETER_MM,
           MOTION_WHEELBASE_MM, (long) (motion_mm_s_to_cpt_q8(200) / 256));
}

static void
handle_line (char * p_line)
{
    char  c = p_line[0];
    char * p_args = p_line + 1;
    long  a = 0;
    long  b = 0;
    int   n = sscanf(p_args, "%ld %ld", &a, &b);

    switch (c)
    {
        case 'f': send(MOTION_CMD_FWD, (int) a, (n > 1) ? (int) b : 0); break;
        case 'b': send(MOTION_CMD_REV, (int) a, (n > 1) ? (int) b : 0); break;
        case 'r': send(MOTION_CMD_ROTATE, (int) a, (n > 1) ? (int) b : 0); break;
        case 't': send(MOTION_CMD_TURN, (int) a, (n > 1) ? (int) b : 0); break;
        case 'F': send(MOTION_CMD_FOLLOW, 0, (n > 0) ? (int) a : 0); break;
        case 'v': send(MOTION_CMD_SPEED, (int) a, (int) b); break;
        case 'k': send(MOTION_CMD_SELFTEST, 0, 0); break;
        case 'x': send(MOTION_CMD_STOP, 0, 0); break;
        case 'e': send(MOTION_CMD_ESTOP, 0, 0); break;
        case 'z': send(MOTION_CMD_RESET, 0, 0); break;
        case 's':
        {
            g_steer = (int8_t) ((a > 127) ? 127 : ((a < -127) ? -127 : a));
            g_steer_refresh = true;
            printf("> steer %d (refreshing)\n", g_steer);
            break;
        }
        case 'q': g_steer_refresh = false; printf("> steer refresh off\n"); break;
        case 'w':
        {
            static bool slow = false;

            slow = !slow;
            motion_set_slow(slow);
            printf("> slow %d\n", slow);
            break;
        }
        case 'a':
        {
            motion_gains_t g;

            motion_get_gains(&g);
            g.accel_q8_per_tick = (0 == a) ? 0 : MOTION_ACCEL_Q8_PER_TICK;
            motion_set_gains(&g);
            printf("> accel %ld\n", (long) g.accel_q8_per_tick);
            break;
        }
        case 'g':
        {
            motion_gains_t g;
            long v[6];

            motion_get_gains(&g);

            if (6 == sscanf(p_args, "%ld %ld %ld %ld %ld %ld",
                            &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]))
            {
                g.kp_q8 = v[0]; g.ki_q8 = v[1]; g.ff_q8 = v[2];
                g.ff_offset_permille = v[3]; g.ksync_q8 = v[4];
                g.accel_q8_per_tick = v[5];
                motion_set_gains(&g);
            }

            print_status();
            break;
        }
        case 'l':
        {
            g_log_on = !g_log_on;
            if (g_log_on)
            {
                printf("tick,state,sp_l_q8,sp_r_q8,spd_l,spd_r,duty_l,duty_r,trav_l,trav_r\n");
            }
            break;
        }
        case 'p': print_status(); break;
        case 'h':
        case '?': print_help(); break;
        default: break;
    }
}

static void
print_event (const motion_evt_t * p_e)
{
    static const char * const names[MOTION_EVT_COUNT] =
    {
        "NONE", "DONE", "STOPPED", "STALL", "STEER_STALE", "SELFTEST", "REJECTED",
    };

    printf("< %s seq=%u arg=%u detail=0x%02x achieved=%d t=%u\n",
           (p_e->id < MOTION_EVT_COUNT) ? names[p_e->id] : "?",
           p_e->seq, p_e->arg, p_e->detail, p_e->achieved, p_e->time_ms);
}

int
main (void)
{
    stdio_init_all();

    /* Motors to 0 before anything else. */
    motion_hal_pico_init();
    motion_init();
    motion_set_log_hook(log_hook);
    estop_init();

    sleep_ms(1500);
    printf("\nmotion_bench: Buddy 2 motion control. Power motors from VIN.\n");
    print_help();

    (void) add_repeating_timer_ms(-(int32_t) MOTION_TICK_MS, tick_cb, NULL,
                                  &g_tick_timer);

    char     line[LINE_MAX];
    uint32_t len = 0u;
    uint32_t last_steer_ms = 0u;

    for (;;)
    {
        int ch = getchar_timeout_us(0);

        while (PICO_ERROR_TIMEOUT != ch)
        {
            if (('\r' == ch) || ('\n' == ch))
            {
                if (len > 0u)
                {
                    line[len] = '\0';
                    handle_line(line);
                    len = 0u;
                }
            }
            else if (len < (LINE_MAX - 1u))
            {
                line[len++] = (char) ch;
            }
            else
            {
                len = 0u;
            }

            ch = getchar_timeout_us(0);
        }

        /* Stand-in for the Line & Barcode Task: refresh STEER every 10 ms. */
        uint32_t const now = to_ms_since_boot(get_absolute_time());

        if (g_steer_refresh && ((now - last_steer_ms) >= 10u))
        {
            last_steer_ms = now;
            motion_set_steer(g_steer);
        }

        motion_evt_t evt;

        while (motion_get_event(&evt))
        {
            print_event(&evt);
        }

        while (g_log_tail != g_log_head)
        {
            const motion_log_t * p = &g_log_ring[g_log_tail & (LOG_RING_LEN - 1u)];

            printf("%lu,%u,%d,%d,%d,%d,%d,%d,%ld,%ld\n",
                   (unsigned long) p->tick, p->state,
                   p->setpoint_cpt_q8[0], p->setpoint_cpt_q8[1],
                   p->speed_cpt[0], p->speed_cpt[1],
                   p->duty_permille[0], p->duty_permille[1],
                   (long) p->travel[0], (long) p->travel[1]);
            g_log_tail++;
        }

        sleep_ms(1);
    }
}
