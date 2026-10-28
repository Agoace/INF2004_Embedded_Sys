/**
 * @file motion_sequence.c
 * @brief Runs a fixed list of movement steps with closed-loop correction.
 *
 * Two control layers:
 *  1. During a move, the motion library (src/motion/) runs a PI speed loop
 *     on each wheel every 20 ms and keeps the wheels in sync. This replaces
 *     the open-loop duty trim of fixed_speed.c.
 *  2. After a move, this sequencer compares the result with the plan:
 *      - Distance: target mm minus achieved mm. Drive the remainder.
 *      - Heading: planned heading minus heading measured from the encoders
 *        since the start of the run. Rotate in place by the error.
 *     It sends at most MAX_CORRECTIONS corrections per step, then goes on.
 *
 * Soft real time: the 20 ms motion tick runs in a timer IRQ. Each step has
 * a deadline (2 x expected time + STEP_MARGIN_MS). A late step stops the
 * car and ends the run. Tick overruns are printed at the end of the run.
 *
 * Buttons (same as motion_bench and the team pin table):
 *   GP21  start the sequence; press again during a run to stop it.
 *   GP20  emergency stop (interrupt). The next GP21 press clears the fault.
 *
 * Power the motors from VIN, not the Raspberry Pi USB port
 * (docs/USB_POWER_SAFETY.md).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#include "encoder.h"
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hal.h"
#include "motion_units.h"
#include "robot_shared.h"

/* ---- Settings: edit these ---------------------------------------------- */
#define CRUISE_SPEED_MM_S       150     /* Speed for every step.           */
#define CORRECTION_SPEED_MM_S   80      /* Slower speed for corrections.   */
#define DISTANCE_TOLERANCE_MM   10      /* Correct if further off than this. */
#define HEADING_TOLERANCE_DEG   3       /* Correct if further off than this. */
#define MAX_CORRECTIONS         3       /* Per step.                       */
#define STEP_MARGIN_MS          1000u   /* Added to each step deadline.    */

typedef enum
{
    STEP_FORWARD = 0,   /* amount = mm                                    */
    STEP_REVERSE,       /* amount = mm                                    */
    STEP_LEFT,          /* amount = degrees, rotate in place              */
    STEP_RIGHT,         /* amount = degrees, rotate in place              */
    STEP_PIVOT_LEFT,    /* amount = degrees, left wheel held              */
    STEP_PIVOT_RIGHT,   /* amount = degrees, right wheel held             */
    STEP_PAUSE,         /* amount = ms                                    */
} step_kind_t;

typedef struct
{
    step_kind_t kind;
    int16_t     amount;
} step_t;

/* The run. Default: go around an obstacle on its right side and end up
 * facing the start direction. Add, remove or reorder lines freely. */
static const step_t g_sequence[] =
{
    { STEP_FORWARD,     300 },
    { STEP_RIGHT,        90 },
    { STEP_FORWARD,     200 },
    { STEP_LEFT,         90 },
    { STEP_FORWARD,     300 },
    { STEP_LEFT,         90 },
    { STEP_FORWARD,     200 },
    { STEP_RIGHT,        90 },
    { STEP_PAUSE,       500 },
    { STEP_REVERSE,     300 },
};

/* ------------------------------------------------------------------------ */
#define STEP_COUNT          (sizeof(g_sequence) / sizeof(g_sequence[0]))
#define START_BUTTON_GPIO   21u
#define ESTOP_BUTTON_GPIO   20u
#define POLL_MS             1u
#define DEBOUNCE_MS         200u

typedef enum
{
    SEQ_IDLE = 0,
    SEQ_MOVING,         /* Waiting for the DONE event of g_wait_seq.      */
    SEQ_PAUSING,
} seq_state_t;

static repeating_timer_t g_tick_timer;
static seq_state_t       g_state;
static uint32_t          g_step;
static uint32_t          g_step_start_ms;
static uint32_t          g_deadline_ms;
static uint32_t          g_corrections;
static int32_t           g_along_mm;        /* Distance done in this step. */
static int32_t           g_planned_cdeg;    /* Heading the plan expects.   */
static uint8_t           g_seq;
static uint8_t           g_wait_seq;
static uint8_t           g_last_cmd;
static odometer_t        g_odo[MOTOR_COUNT];

static const char * const g_step_names[] =
{
    "FORWARD", "REVERSE", "LEFT", "RIGHT", "PIVOT_LEFT", "PIVOT_RIGHT", "PAUSE",
};

/* ---- Tick (timer IRQ) --------------------------------------------------- */
static bool
tick_cb (repeating_timer_t * p_rt)
{
    (void) p_rt;
    motion_tick();
    return true;
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
buttons_init (void)
{
    gpio_init(START_BUTTON_GPIO);
    gpio_set_dir(START_BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(START_BUTTON_GPIO);

    gpio_init(ESTOP_BUTTON_GPIO);
    gpio_set_dir(ESTOP_BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(ESTOP_BUTTON_GPIO);
    gpio_add_raw_irq_handler(ESTOP_BUTTON_GPIO, estop_irq);
    gpio_set_irq_enabled(ESTOP_BUTTON_GPIO, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

/* ---- Heading from the encoders ------------------------------------------ */
static void
odometry_mark (void)
{
    uint16_t l;
    uint16_t r;

    encoder_count_pair(&l, &r);
    odometer_mark(&g_odo[MOTOR_LEFT], l);
    odometer_mark(&g_odo[MOTOR_RIGHT], r);
}

/** Call often (each loop): odometer_t needs a sample every < 32767 counts. */
static void
odometry_update (void)
{
    uint16_t l;
    uint16_t r;

    encoder_count_pair(&l, &r);
    (void) odometer_update(&g_odo[MOTOR_LEFT], l);
    (void) odometer_update(&g_odo[MOTOR_RIGHT], r);
}

/** Heading change since the run start, in 1/100 degree (+ = left/CCW).
 *  theta = (sR - sL) / W ; in counts: (dR - dL) * 180 * D / (CPR * W). */
static int32_t
measured_heading_cdeg (void)
{
    int64_t const diff = (int64_t) g_odo[MOTOR_RIGHT].total -
                         g_odo[MOTOR_LEFT].total;

    return (int32_t) ((diff * 18000 * MOTION_WHEEL_DIAMETER_MM) /
                      ((int64_t) MOTION_COUNTS_PER_REV * MOTION_WHEELBASE_MM));
}

/* ---- Commands ----------------------------------------------------------- */
static uint32_t
now_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

static bool
send (motion_cmd_id_t id, int32_t a, int32_t speed)
{
    motion_cmd_t cmd;

    cmd.id         = (uint8_t) id;
    cmd.seq        = ++g_seq;
    cmd.arg_a      = (int16_t) a;
    cmd.speed_mm_s = (int16_t) speed;
    cmd.reserved   = 0;

    if (!motion_command(&cmd))
    {
        printf("! command queue full\n");
        return false;
    }

    g_wait_seq = cmd.seq;
    g_last_cmd = cmd.id;
    return true;
}

/** Expected time of a move at speed, for the step deadline. */
static uint32_t
expected_ms (motion_cmd_id_t id, int32_t a, int32_t speed)
{
    int32_t mm = abs(a);

    if ((MOTION_CMD_ROTATE == id) || (MOTION_CMD_TURN == id))
    {
        /* Wheel arc = deg / 360 * pi * W (doubled for a pivot). */
        mm = (mm * MOTION_WHEELBASE_MM * 355) / (360 * 113);
        mm = (MOTION_CMD_TURN == id) ? (2 * mm) : mm;
    }

    return (uint32_t) ((mm * 1000) / speed);
}

static bool
send_move (motion_cmd_id_t id, int32_t a, int32_t speed)
{
    if (!send(id, a, speed))
    {
        return false;
    }

    g_deadline_ms = now_ms() + (2u * expected_ms(id, a, speed)) + STEP_MARGIN_MS;
    g_state = SEQ_MOVING;
    return true;
}

/* ---- Sequencer ---------------------------------------------------------- */
static void
end_run (const char * p_reason)
{
    motion_stats_t st;
    long const     err = (long) (g_planned_cdeg - measured_heading_cdeg());

    motion_get_stats(&st);
    g_state = SEQ_IDLE;
    printf("Run %s. Heading error %c%ld.%02ld deg. Tick WCET %lu us, "
           "overruns %lu.\n",
           p_reason, (err < 0) ? '-' : '+', labs(err) / 100, labs(err) % 100,
           (unsigned long) st.tick_wcet_us,
           (unsigned long) st.tick_overruns);
    printf("Press GP21 to run again.\n");
}

static void
abort_run (const char * p_reason)
{
    motion_request_stop(false);
    end_run(p_reason);
}

static void
start_step (void)
{
    if (g_step >= STEP_COUNT)
    {
        end_run("complete");
        return;
    }

    const step_t * p = &g_sequence[g_step];

    g_corrections   = 0u;
    g_along_mm      = 0;
    g_step_start_ms = now_ms();
    printf("Step %lu/%u %s %d\n", (unsigned long) (g_step + 1u),
           (unsigned) STEP_COUNT, g_step_names[p->kind], p->amount);

    bool ok = true;

    switch (p->kind)
    {
        case STEP_FORWARD:
            ok = send_move(MOTION_CMD_FWD, p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_REVERSE:
            ok = send_move(MOTION_CMD_REV, p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_LEFT:
            g_planned_cdeg += p->amount * 100;
            ok = send_move(MOTION_CMD_ROTATE, p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_RIGHT:
            g_planned_cdeg -= p->amount * 100;
            ok = send_move(MOTION_CMD_ROTATE, -p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_PIVOT_LEFT:
            g_planned_cdeg += p->amount * 100;
            ok = send_move(MOTION_CMD_TURN, p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_PIVOT_RIGHT:
            g_planned_cdeg -= p->amount * 100;
            ok = send_move(MOTION_CMD_TURN, -p->amount, CRUISE_SPEED_MM_S);
            break;
        case STEP_PAUSE:
        default:
            g_deadline_ms = g_step_start_ms + (uint32_t) p->amount;
            g_state = SEQ_PAUSING;
            break;
    }

    if (!ok)
    {
        abort_run("aborted (command not accepted)");
    }
}

static void
next_step (void)
{
    printf("  done in %lu ms\n", (unsigned long) (now_ms() - g_step_start_ms));
    g_step++;
    start_step();
}

/** A move of this step finished: correct distance first, then heading. */
static void
on_move_done (int16_t achieved)
{
    const step_t * p = &g_sequence[g_step];
    bool const linear = (STEP_FORWARD == p->kind) || (STEP_REVERSE == p->kind);

    if (linear)
    {
        /* A FWD in a REVERSE step (or the other way) moves back. */
        bool const with_step = ((MOTION_CMD_FWD == g_last_cmd) ==
                                (STEP_FORWARD == p->kind));

        g_along_mm += with_step ? achieved : -achieved;
    }

    printf("  achieved %d %s\n", achieved, linear ? "mm" : "deg");

    if (g_corrections >= MAX_CORRECTIONS)
    {
        printf("  correction limit reached\n");
        next_step();
        return;
    }

    int32_t const remaining = linear ? (p->amount - g_along_mm) : 0;

    if (abs(remaining) > DISTANCE_TOLERANCE_MM)
    {
        bool const fwd = ((remaining > 0) == (STEP_FORWARD == p->kind));

        g_corrections++;
        printf("  correct distance %+ld mm\n", (long) remaining);
        if (!send_move(fwd ? MOTION_CMD_FWD : MOTION_CMD_REV, abs(remaining),
                       CORRECTION_SPEED_MM_S))
        {
            abort_run("aborted (command not accepted)");
        }
        return;
    }

    int32_t const err_cdeg = g_planned_cdeg - measured_heading_cdeg();
    int32_t const err_deg  = (err_cdeg + ((err_cdeg >= 0) ? 50 : -50)) / 100;

    if (abs(err_deg) > HEADING_TOLERANCE_DEG)
    {
        if (abs(err_deg) > 45)
        {
            abort_run("aborted (heading error over 45 deg: check "
                      "MOTION_WHEELBASE_MM and the wheels)");
            return;
        }

        g_corrections++;
        printf("  correct heading %+ld deg\n", (long) err_deg);
        if (!send_move(MOTION_CMD_ROTATE, err_deg, CORRECTION_SPEED_MM_S))
        {
            abort_run("aborted (command not accepted)");
        }
        return;
    }

    next_step();
}

static void
handle_event (const motion_evt_t * p_e)
{
    switch (p_e->id)
    {
        case MOTION_EVT_DONE:
            if ((SEQ_MOVING == g_state) && (p_e->seq == g_wait_seq))
            {
                on_move_done(p_e->achieved);
            }
            break;
        case MOTION_EVT_STOPPED:
            if (SEQ_IDLE != g_state)
            {
                end_run((1u == p_e->arg) ? "stopped by ESTOP" : "stopped");
            }
            break;
        case MOTION_EVT_STALL:
            printf("! stall on %s wheel\n",
                   (MOTOR_LEFT == p_e->arg) ? "left" : "right");
            if (SEQ_IDLE != g_state)
            {
                end_run("aborted (stall)");
            }
            break;
        case MOTION_EVT_REJECTED:
            if (SEQ_IDLE != g_state)
            {
                abort_run("aborted (command rejected)");
            }
            break;
        default:
            break;
    }
}

static void
start_run (void)
{
    /* Clears a latched ESTOP or stall. Ignored when there is no fault. */
    (void) send(MOTION_CMD_RESET, 0, 0);
    motion_odometer_reset();
    odometry_mark();
    g_planned_cdeg = 0;
    g_step = 0u;
    printf("\nRun start: %u steps at %d mm/s.\n", (unsigned) STEP_COUNT,
           CRUISE_SPEED_MM_S);
    start_step();
}

int
main (void)
{
    stdio_init_all();

    /* Motors to 0 before anything else. */
    motion_hal_pico_init();
    motion_init();
    buttons_init();

    sleep_ms(1500);
    printf("\nmotion_sequence: closed-loop movement steps.\n");
    printf("WARNING: power motors from VIN, not USB.\n");
    printf("D=%d mm, W=%d mm. GP21 start/stop, GP20 emergency stop.\n",
           MOTION_WHEEL_DIAMETER_MM, MOTION_WHEELBASE_MM);

    (void) add_repeating_timer_ms(-(int32_t) MOTION_TICK_MS, tick_cb, NULL,
                                  &g_tick_timer);

    bool     previous_start = false;
    uint32_t last_press_ms  = 0u;

    for (;;)
    {
        odometry_update();

        bool const start = !gpio_get(START_BUTTON_GPIO);

        if (start && !previous_start && ((now_ms() - last_press_ms) >= DEBOUNCE_MS))
        {
            last_press_ms = now_ms();

            if (SEQ_IDLE == g_state)
            {
                start_run();
            }
            else
            {
                abort_run("stopped by GP21");
            }
        }
        previous_start = start;

        motion_evt_t evt;

        while (motion_get_event(&evt))
        {
            handle_event(&evt);
        }

        if ((SEQ_IDLE != g_state) && ((int32_t) (now_ms() - g_deadline_ms) >= 0))
        {
            if (SEQ_PAUSING == g_state)
            {
                next_step();
            }
            else
            {
                abort_run("aborted (step deadline missed)");
            }
        }

        sleep_ms(POLL_MS);
    }
}
