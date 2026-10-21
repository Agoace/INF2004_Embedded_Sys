/**
 * Host unit tests for the motion core (no Pico needed).
 *
 *   cc -std=c11 -Wall -Wextra -Isrc/common -Isrc/motion \
 *      tests/motion/test_motion.c src/motion/motion_control.c \
 *      src/motion/line_motion_bridge.c -lm -o /tmp/tm && /tmp/tm
 *
 * A simple simulated robot implements motion_hal.h: each wheel has a dead
 * band, a first-order speed lag and a wrapping 16-bit encoder counter.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "encoder.h"
#include "line_motion_bridge.h"
#include "motion_config.h"
#include "motion_control.h"
#include "motion_hal.h"
#include "motion_units.h"
#include "robot_shared.h"

/* ---- Simulated hardware ------------------------------------------------ */
typedef struct
{
    int16_t  duty;
    double   speed;         /* counts per tick */
    double   pos;           /* exact counts    */
    uint16_t count;         /* wrapping        */
    uint32_t edges;
    bool     blocked;
    int      sign;          /* +1 normal, -1 wired backwards */
} sim_wheel_t;

static sim_wheel_t g_sim[2];
static uint32_t    g_time_us;
static int         g_stop_on_time_call = -1;    /* trigger stop inside tick */
static int         g_time_calls;

uint16_t motion_hal_encoder_count (motor_id_t m) { return g_sim[m].count; }
uint16_t encoder_count (motor_id_t m) { return g_sim[m].count; }

void motion_hal_encoder_edges (motor_id_t m, uint32_t * a, uint32_t * b)
{
    *a = g_sim[m].edges;
    *b = g_sim[m].edges;
}

void motion_hal_pwm_write (motor_id_t m, int16_t permille)
{
    g_sim[m].duty = permille;
}

void motion_hal_pwm_zero_all (void)
{
    g_sim[0].duty = 0;
    g_sim[1].duty = 0;
}

uint32_t motion_hal_irq_save (void) { return 0u; }
void     motion_hal_irq_restore (uint32_t s) { (void) s; }
void     motion_hal_debug_pin (bool l) { (void) l; }

uint32_t motion_hal_time_us (void)
{
    g_time_calls++;

    if (g_time_calls == g_stop_on_time_call)
    {
        motion_request_stop(false);
    }

    return g_time_us;
}

/* Advance physics by one 20 ms tick, then run the controller. */
static void sim_step (void)
{
    for (int m = 0; m < 2; m++)
    {
        sim_wheel_t * w = &g_sim[m];
        double target = 0.0;
        int    d      = w->duty;

        if (!w->blocked && (abs(d) > 70))
        {
            target = (d > 0 ? 1 : -1) * (abs(d) - 50) * 0.235; /* ~220 at 1000 */
        }

        w->speed += (target - w->speed) * 0.5;      /* lag */

        if (w->blocked)
        {
            w->speed = 0.0;
        }

        double const old = w->pos;

        w->pos  += w->speed;

        long const delta = (long) w->pos - (long) old;

        w->count  = (uint16_t) (w->count + w->sign * delta);
        w->edges += (uint32_t) labs(delta);
    }

    g_time_us += 20000u;
    g_time_calls = 0;
    motion_tick();
}

static void sim_reset (void)
{
    memset(g_sim, 0, sizeof(g_sim));
    g_sim[0].sign = 1;
    g_sim[1].sign = 1;
    g_sim[0].count = 65000;      /* start near wrap on purpose */
    g_sim[1].count = 65500;
    g_time_us = 1000000u;
    g_stop_on_time_call = -1;
    motion_init();
}

/* ---- Tiny test framework ------------------------------------------------ */
static int g_fail;
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); g_fail++; } } while (0)

static bool wait_event (motion_evt_id_t id, int max_ticks, motion_evt_t * out)
{
    for (int i = 0; i < max_ticks; i++)
    {
        sim_step();
        motion_evt_t e;

        while (motion_get_event(&e))
        {
            if (e.id == id) { if (out) *out = e; return true; }
        }
    }

    return false;
}

static motion_cmd_t cmd (motion_cmd_id_t id, int16_t a, int16_t spd, uint8_t seq)
{
    motion_cmd_t c = { (uint8_t) id, seq, a, spd, 0 };
    return c;
}

/* ---- Tests ------------------------------------------------------------- */
static void test_delta16 (void)
{
    puts("delta16 / odometer wrap");
    CHECK(enc_delta16(5u, 65530u) == 11);
    CHECK(enc_delta16(65530u, 5u) == -11);
    CHECK(enc_delta16(100u, 100u) == 0);

    odometer_t o;
    odometer_mark(&o, 65000u);
    (void) odometer_update(&o, 200u);       /* +736 across wrap */
    CHECK(odometer_update(&o, 100u) == 636);
}

static void test_snapshot_pack (void)
{
    puts("snapshot 9-bit speed packing");
    uint32_t w = msnap_pack_speed(-200) | (msnap_pack_speed(221) << 9);
    CHECK(msnap_unpack_speed(w, 0) == -200);
    CHECK(msnap_unpack_speed(w, 9) == 221);
    CHECK(msnap_unpack_speed(msnap_pack_speed(999), 0) == 255);
}

static void test_fwd (int16_t mm)
{
    printf("FWD %d mm\n", mm);
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_FWD, mm, 200, 7);
    CHECK(motion_command(&c));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_DONE, 1000, &e));
    CHECK(e.seq == 7);
    CHECK(e.arg == MOTION_DONE_OK);
    double const got_mm = motion_counts_to_mm((int32_t) ((g_sim[0].pos + g_sim[1].pos) / 2));
    printf("  achieved %d mm (event), %.1f mm (sim), L-R %.0f counts\n",
           e.achieved, got_mm, g_sim[0].pos - g_sim[1].pos);
    CHECK(abs((int) got_mm - mm) <= 30);       /* NFR13 +/-3 cm on 1 m */
    CHECK(fabs(g_sim[0].pos - g_sim[1].pos) < 30.0);
    CHECK(motion_get_state() == MOTION_STATE_IDLE);
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);
}

static void test_rev (void)
{
    puts("REV 100 mm");
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_REV, 100, 0, 1);
    CHECK(motion_command(&c));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_DONE, 500, &e));
    CHECK(g_sim[0].pos < 0 && g_sim[1].pos < 0);
    CHECK(abs(e.achieved - 100) <= 5);
}

static void test_rotate (int16_t deg, bool pivot)
{
    printf("%s %d deg\n", pivot ? "TURN(pivot)" : "ROTATE", deg);
    sim_reset();
    motion_cmd_t c = cmd(pivot ? MOTION_CMD_TURN : MOTION_CMD_ROTATE, deg, 150, 3);
    CHECK(motion_command(&c));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_DONE, 1000, &e));
    printf("  achieved %d deg, L %.0f R %.0f counts\n", e.achieved, g_sim[0].pos, g_sim[1].pos);
    CHECK(abs(e.achieved - deg) <= 5);         /* NFR13 +/-5 deg */
    if (pivot)
    {
        CHECK(deg > 0 ? (fabs(g_sim[0].pos) < 5 && g_sim[1].pos > 0)
                      : (fabs(g_sim[1].pos) < 5 && g_sim[0].pos > 0));
    }
    else
    {
        CHECK(deg > 0 ? (g_sim[0].pos < 0 && g_sim[1].pos > 0)
                      : (g_sim[0].pos > 0 && g_sim[1].pos < 0));
    }
}

static void test_stop (void)
{
    puts("STOP fast path, queue flush, command after STOP survives");
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_FWD, 1000, 300, 1);
    CHECK(motion_command(&c));
    for (int i = 0; i < 20; i++) sim_step();
    CHECK(g_sim[0].duty != 0);
    motion_cmd_t queued = cmd(MOTION_CMD_ROTATE, 90, 0, 2);
    CHECK(motion_command(&queued));         /* waits behind the move */
    motion_cmd_t s = cmd(MOTION_CMD_STOP, 0, 0, 0);
    CHECK(motion_command(&s));
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);   /* zero before any tick */
    motion_cmd_t after = cmd(MOTION_CMD_FWD, 50, 0, 9);
    CHECK(motion_command(&after));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_STOPPED, 2, &e));
    CHECK(e.arg == 0);
    /* The ROTATE queued before STOP was flushed; FWD after STOP runs. */
    CHECK(wait_event(MOTION_EVT_DONE, 500, &e));
    CHECK(e.seq == 9);
}

static void test_stop_race (void)
{
    puts("STOP arriving mid-tick never gets overwritten by a PWM write");
    sim_reset();
    motion_cmd_t f = cmd(MOTION_CMD_FOLLOW, 0, 200, 1);
    CHECK(motion_command(&f));
    for (int i = 0; i < 20; i++) { motion_set_steer(0); sim_step(); }
    CHECK(g_sim[0].duty != 0);
    motion_set_steer(0);
    g_stop_on_time_call = 2;    /* inside run_follow, after stop handling */
    sim_step();
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);
    g_stop_on_time_call = -1;
    sim_step();
    CHECK(motion_get_state() == MOTION_STATE_IDLE);
}

static void test_estop_reset (void)
{
    puts("ESTOP latches; commands rejected until RESET");
    sim_reset();
    motion_request_stop(true);
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_STOPPED, 2, &e));
    CHECK(e.arg == 1);
    CHECK(motion_get_state() == MOTION_STATE_MOTOR_FAULT);
    CHECK(motion_get_fault_bits() & FAULT_ESTOP);
    motion_cmd_t c = cmd(MOTION_CMD_FWD, 100, 0, 4);
    CHECK(motion_command(&c));
    CHECK(wait_event(MOTION_EVT_REJECTED, 3, &e));
    CHECK(g_sim[0].duty == 0);
    motion_cmd_t r = cmd(MOTION_CMD_RESET, 0, 0, 5);
    CHECK(motion_command(&r));
    sim_step();
    CHECK(motion_get_state() == MOTION_STATE_IDLE);
    CHECK(motion_get_fault_bits() == 0u);
}

static void test_stall (void)
{
    puts("Stall: blocked wheel -> STALL event, fault, PWM 0");
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_FWD, 1000, 200, 1);
    CHECK(motion_command(&c));
    for (int i = 0; i < 30; i++) sim_step();
    g_sim[1].blocked = true;
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_STALL, 40, &e));
    CHECK(e.arg == MOTOR_RIGHT);
    CHECK(motion_get_fault_bits() & FAULT_MOTOR_STALL_R);
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);

    puts("Stall at start: kick times out");
    sim_reset();
    g_sim[0].blocked = true;
    CHECK(motion_command(&c));
    CHECK(wait_event(MOTION_EVT_STALL, 20, &e));
    CHECK(e.arg == MOTOR_LEFT);
}

static void test_follow (void)
{
    puts("FOLLOW: holds until STEER is fresh, steers, stale timeout");
    sim_reset();
    motion_cmd_t f = cmd(MOTION_CMD_FOLLOW, 0, 200, 1);
    CHECK(motion_command(&f));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_STEER_STALE, 3, &e));
    CHECK(g_sim[0].duty == 0);

    for (int i = 0; i < 40; i++) { motion_set_steer(0); sim_step(); }
    CHECK(g_sim[0].speed > 40 && g_sim[1].speed > 40);

    for (int i = 0; i < 40; i++) { motion_set_steer(127); sim_step(); }
    printf("  steer +127: L %.1f R %.1f cpt\n", g_sim[0].speed, g_sim[1].speed);
    CHECK(g_sim[0].speed > 40 && g_sim[1].speed < 3);

    for (int i = 0; i < 40; i++) { motion_set_steer(-64); sim_step(); }
    CHECK(g_sim[1].speed > g_sim[0].speed);

    motion_set_slow(true);
    for (int i = 0; i < 40; i++) { motion_set_steer(0); sim_step(); }
    printf("  slow: L %.1f cpt\n", g_sim[0].speed);
    CHECK(g_sim[0].speed < 35);
    motion_set_slow(false);

    /* Stop writing STEER: after ~100 ms the wheels must be held. */
    CHECK(wait_event(MOTION_EVT_STEER_STALE, 10, &e));
    for (int i = 0; i < 10; i++) sim_step();
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);
    CHECK(motion_get_state() == MOTION_STATE_FOLLOW);

    puts("FOLLOW -> ROTATE preempts at a junction");
    for (int i = 0; i < 5; i++) { motion_set_steer(0); sim_step(); }
    motion_cmd_t r = cmd(MOTION_CMD_ROTATE, -90, 0, 2);
    CHECK(motion_command(&r));
    CHECK(wait_event(MOTION_EVT_DONE, 500, &e));
    CHECK(e.seq == 2);
}

static void test_selftest (void)
{
    puts("SELFTEST pass, then fail with a reversed encoder");
    sim_reset();
    motion_cmd_t t = cmd(MOTION_CMD_SELFTEST, 0, 0, 1);
    CHECK(motion_command(&t));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_SELFTEST, 200, &e));
    printf("  pass=%u mask=0x%02X\n", e.arg, e.detail);
    CHECK(e.arg == 1 && e.detail == 0x3F);
    CHECK(motion_get_state() == MOTION_STATE_IDLE);

    sim_reset();
    g_sim[1].sign = -1;
    CHECK(motion_command(&t));
    CHECK(wait_event(MOTION_EVT_SELFTEST, 200, &e));
    printf("  pass=%u mask=0x%02X\n", e.arg, e.detail);
    CHECK(e.arg == 0);
    CHECK(motion_get_fault_bits() & FAULT_ENCODER);
}

static void test_queue_and_reject (void)
{
    puts("Queue full returns false; invalid command rejected");
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_FWD, 500, 0, 1);
    int ok = 0;
    for (int i = 0; i < 10; i++) ok += motion_command(&c) ? 1 : 0;
    CHECK(ok == (int) MOTION_CMD_QUEUE_LEN - 1);
    motion_request_stop(false);
    sim_step();
    motion_cmd_t bad = cmd(MOTION_CMD_FWD, -5, 0, 8);
    CHECK(motion_command(&bad));
    motion_evt_t e;
    CHECK(wait_event(MOTION_EVT_REJECTED, 3, &e));
    CHECK(e.seq == 8);
}

static void test_tick_cost (void)
{
    puts("snapshot words");
    sim_reset();
    motion_cmd_t c = cmd(MOTION_CMD_SPEED, 200, -200, 1);
    CHECK(motion_command(&c));
    for (int i = 0; i < 50; i++) sim_step();
    uint32_t w1, w2;
    motion_get_snapshot(&w1, &w2);
    int16_t l = msnap_unpack_speed(w1, MSNAP_W1_L_SHIFT);
    int16_t r = msnap_unpack_speed(w1, MSNAP_W1_R_SHIFT);
    printf("  w1 L=%d R=%d state=%u\n", l, r,
           (unsigned) ((w1 >> MSNAP_W1_STATE_SHIFT) & MSNAP_W1_STATE_MASK));
    CHECK(l > 40 && r < -40);
    CHECK(((w1 >> MSNAP_W1_STATE_SHIFT) & MSNAP_W1_STATE_MASK) == MOTION_STATE_SPEED);
    printf("  target %d cpt for 200 mm/s\n", motion_mm_s_to_cpt_q8(200) / 256);
}

static void test_bridge (void)
{
    puts("line bridge: Ziven's pattern table -> STEER, travel across wrap");
    int8_t err;
    CHECK(line_bridge_classify(1u, 0u, 0u, &err) == LINE_STATUS_TRACKING && err == -2);
    CHECK(line_bridge_classify(6u, 0u, 0u, &err) == LINE_STATUS_TRACKING && err == 1);
    CHECK(line_bridge_classify(0u, 0u, 0u, &err) == LINE_STATUS_LOST);
    CHECK(line_bridge_classify(7u, 0u, 0u, &err) == LINE_STATUS_JUNCTION);
    CHECK(line_bridge_classify(5u, 0u, 0u, &err) == LINE_STATUS_SPLIT);
    CHECK(line_bridge_classify(2u, 1u, 0u, &err) == LINE_STATUS_UNKNOWN);
    CHECK(line_bridge_classify(2u, 0u, 4u, &err) == LINE_STATUS_FAULT);
    CHECK(line_bridge_steer_from_error(2) == 110);

    sim_reset();
    line_bridge_mark();
    motion_cmd_t f = cmd(MOTION_CMD_FOLLOW, 0, 200, 1);
    CHECK(motion_command(&f));
    for (int i = 0; i < 100; i++) { (void) line_bridge_update(4u, 0u, 0u); sim_step(); }
    printf("  line right only: L %.1f R %.1f cpt\n", g_sim[0].speed, g_sim[1].speed);
    CHECK(g_sim[0].speed > g_sim[1].speed + 20);    /* turns right */
    for (int i = 0; i < 100; i++) { (void) line_bridge_update(0u, 0u, 0u); sim_step(); }
    CHECK(g_sim[0].speed > g_sim[1].speed + 20);    /* lost: keeps last bias */
    for (int i = 0; i < 100; i++) { (void) line_bridge_update(2u, 0u, 0u); sim_step(); }
    int32_t mm = line_bridge_travel_mm();
    printf("  travel %ld mm (counters wrapped), speed %ld mm/s\n", (long) mm,
           (long) line_bridge_speed_mm_s());
    CHECK(mm > 400 && mm < 1500);
    CHECK(line_bridge_speed_mm_s() > 150);
    for (int i = 0; i < 20; i++) { (void) line_bridge_update(2u, 0u, 4u); sim_step(); }
    CHECK(g_sim[0].duty == 0 && g_sim[1].duty == 0);   /* sensor fault -> stale -> hold */
}

int main (void)
{
    test_delta16();
    test_snapshot_pack();
    test_fwd(1000);
    test_fwd(150);
    test_rev();
    test_rotate(90, false);
    test_rotate(-90, false);
    test_rotate(90, true);
    test_rotate(-45, true);
    test_stop();
    test_stop_race();
    test_estop_reset();
    test_stall();
    test_follow();
    test_selftest();
    test_queue_and_reject();
    test_tick_cost();
    test_bridge();
    printf("\n%s (%d failure%s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
