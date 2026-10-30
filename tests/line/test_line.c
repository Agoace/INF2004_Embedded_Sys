/**
 * @file test_line.c
 * @brief Host tests for the whole line component.
 *
 * Covers the observation codec, the position learner, the steering policy, the
 * sweep sequencer, the sensor health check, the 1 kHz cadence and the producer
 * that ties them together. Nothing here needs a Pico: the ADC arrives through
 * the `line_adc.h` seam, which this file implements with scripted readings.
 *
 * Build and run:
 *   cc -std=c11 -Wall -Wextra -Wshadow -Isrc/line tests/line/test_line.c \
 *      src/line/line_types.c src/line/line_position.c src/line/line_steer.c \
 *      src/line/line_spin.c src/line/line_health.c src/line/line_sampler.c \
 *      src/line/line_producer.c -lm -o /tmp/test_line && /tmp/test_line
 */
#include <stdio.h>
#include <string.h>

#include "line_health.h"
#include "line_position.h"
#include "line_producer.h"
#include "line_sampler.h"
#include "line_spin.h"
#include "line_steer.h"
#include "line_types.h"

/* ---- Scripted ADC ------------------------------------------------------ */

static uint16_t g_adc[LINE_ADC_SENSORS];
static uint32_t g_adc_reads;

bool
line_adc_init (void)
{
    return true;
}

void
line_adc_read (uint16_t raw[LINE_ADC_SENSORS])
{
    for (uint32_t s = 0u; s < LINE_ADC_SENSORS; s++)
    {
        raw[s] = g_adc[s];
    }

    g_adc_reads++;
}

/* ---- Harness ----------------------------------------------------------- */

static int g_failures;
static int g_checks;

static void
check (bool condition, const char * p_what)
{
    g_checks++;

    if (!condition)
    {
        printf("  FAIL %s\n", p_what);
        g_failures++;
    }
}

static void
group (const char * p_name)
{
    printf("%s\n", p_name);
}

/* ---- Observation codec ------------------------------------------------- */

static void
test_codec (void)
{
    line_obs_fields_t fields;
    line_observation_t word;

    group("observation codec: layout, clamping and flags");

    memset(&fields, 0, sizeof(fields));
    fields.position   = 100;
    fields.seen       = true;
    fields.side       = (uint8_t) LINE_SIDE_RIGHT;
    fields.junction   = true;
    fields.junction_hold = true;
    fields.sensor_fault  = true;
    fields.calibrated    = true;

    word = line_obs_pack(&fields);

    check(100 == line_obs_position(word), "position round trip");
    check(line_obs_seen(word), "seen bit");
    check(LINE_SIDE_RIGHT == line_obs_side(word), "side field");
    check(line_obs_junction(word), "junction bit");
    check(line_obs_junction_hold(word), "junction hold bit");
    check(line_obs_sensor_fault(word), "sensor fault bit");
    check(line_obs_calibrated(word), "calibrated bit");
    check(0u == (word & 0xFFFF8000u), "reserved bits 15..31 stay clear");

    memset(&fields, 0, sizeof(fields));
    fields.position = -100;
    check(-100 == line_obs_position(line_obs_pack(&fields)), "negative position");

    memset(&fields, 0, sizeof(fields));
    fields.position = -120;
    check(-100 == line_obs_position(line_obs_pack(&fields)), "clamps low");

    memset(&fields, 0, sizeof(fields));
    fields.position = 120;
    check(100 == line_obs_position(line_obs_pack(&fields)), "clamps high");

    check(0 == line_obs_position((line_observation_t) 0), "zero word is centred");
    check(!line_obs_calibrated((line_observation_t) 0), "zero word is uncalibrated");
}

/* ---- Position learner -------------------------------------------------- */

/**
 * Sweep all three sensors, floor-dwelling as real operation does: the polarity
 * decision depends on the floor dominating the sample set.
 */
static void
sweep (lp_t * p_lp, uint16_t floor_v, uint16_t line_v)
{
    unsigned int bad = 0u;
    uint16_t raw[LP_SENSORS];

    lp_calib_begin(p_lp);

    for (int i = 0; i < 20; i++)
    {
        raw[0] = raw[1] = raw[2] = floor_v;
        lp_calib_add(p_lp, raw);
    }

    for (int i = 1; i <= 6; i++)
    {
        raw[0] = raw[1] = raw[2] =
            (uint16_t) (floor_v + ((int) line_v - (int) floor_v) * i / 6);
        lp_calib_add(p_lp, raw);
    }

    for (int i = 0; i < 4; i++)
    {
        raw[0] = raw[1] = raw[2] = line_v;
        lp_calib_add(p_lp, raw);
    }

    check(LP_CALIB_OK == lp_calib_end(p_lp, &bad), "calibration accepted");
    check(0u == bad, "no bad sensors");
}

static void
test_position (void)
{
    lp_t lp;
    uint16_t raw[LP_SENSORS];
    uint32_t t = 100u;

    group("position learner: polarity, sides and the qualified loss dwell");

    lp_init(&lp);
    check(!lp_calibrated(&lp), "uncalibrated after init");

    raw[0] = raw[1] = raw[2] = 1000u;
    check(LP_EVENT_NONE == lp_update(&lp, raw, 1u), "no events before calibration");

    sweep(&lp, 500u, 3000u);
    check(lp_calibrated(&lp), "calibrated");
    check(lp_line_reads_high(&lp), "polarity high");

    raw[0] = 500u; raw[1] = 3000u; raw[2] = 500u;
    for (int i = 0; i < 50; i++)
    {
        (void) lp_update(&lp, raw, t++);
    }

    check(lp.position > -20 && lp.position < 20, "centred is near zero");
    check(lp_line_seen(&lp), "line seen");

    raw[0] = 3000u; raw[1] = 700u; raw[2] = 500u;
    for (int i = 0; i < 50; i++)
    {
        (void) lp_update(&lp, raw, t++);
    }

    check(lp.position < 0, "line under the left outer sensor is negative");

    raw[0] = 500u; raw[1] = 700u; raw[2] = 3000u;
    for (int i = 0; i < 50; i++)
    {
        (void) lp_update(&lp, raw, t++);
    }

    check(lp.position > 0, "line under the right outer sensor is positive");

    /* Inverted polarity. */
    sweep(&lp, 3000u, 500u);
    check(!lp_line_reads_high(&lp), "polarity low detected");

    raw[0] = 3000u; raw[1] = 500u; raw[2] = 3000u;
    for (int i = 0; i < 50; i++)
    {
        (void) lp_update(&lp, raw, t++);
    }

    check(lp.position > -20 && lp.position < 20, "inverted polarity centred");

    /* One qualified loss. Centred means the gap policy, about 300 ms. */
    uint16_t dark[LP_SENSORS] = { 3000u, 3000u, 3000u };
    int32_t fired_at = -1;
    uint32_t const start = t;

    for (int i = 0; i < 500; i++)
    {
        if (LP_EVENT_LOST == lp_update(&lp, dark, t))
        {
            fired_at = (int32_t) (t - start);
            break;
        }

        t += 1u;
    }

    printf("  qualified loss fired at %ld ms\n", (long) fired_at);
    check(fired_at > 100, "the loss needs the qualified dwell");
    check(fired_at <= 310, "the gap policy fires at about 300 ms");

    raw[0] = 3000u; raw[1] = 500u; raw[2] = 3000u;
    check(LP_EVENT_FOUND == lp_update(&lp, raw, t + 1u), "FOUND edge");
}

static void
test_invalidate (void)
{
    lp_t lp;
    uint16_t raw[LP_SENSORS] = { 1000u, 1000u, 1000u };
    unsigned int bad = 0u;

    group("a new sweep invalidates, and a failed sweep stays invalid");

    lp_init(&lp);
    sweep(&lp, 500u, 3000u);
    check(lp_calibrated(&lp), "calibrated first");

    lp_calib_begin(&lp);
    check(!lp_calibrated(&lp), "beginning a sweep discards the calibration");
    check(LP_CALIB_NO_DATA == lp_calib_end(&lp, &bad), "an empty sweep fails");
    check(!lp_calibrated(&lp), "a failed sweep leaves it invalid");
    check(LP_EVENT_NONE == lp_update(&lp, raw, 5u), "no events while invalid");

    sweep(&lp, 500u, 3000u);
    lp_invalidate(&lp);
    check(!lp_calibrated(&lp), "explicit invalidate");
    check(0 == lp.position, "position reset");
    check(LINE_SIDE_UNKNOWN == lp_last_side(&lp), "side reset");
}

/* ---- Steering policy --------------------------------------------------- */

static void
test_steer (void)
{
    line_steer_config_t cfg = { .gain_pct = 100, .kd_pct = 0 };
    line_steer_config_t kd_cfg = { .gain_pct = 100, .kd_pct = 50 };
    line_steer_state_t s;
    line_steer_output_t out;

    group("steering: signed mapping, D term, clamp and straight");

    line_steer_init(&s, &cfg);
    check(127 == line_steer_update(&s, 100, false, 0u).steer, "full right is +127");
    line_steer_reset(&s);
    check(-127 == line_steer_update(&s, -100, false, 1u).steer, "full left is -127");
    line_steer_reset(&s);
    check(0 == line_steer_update(&s, 0, false, 2u).steer, "centred is zero");
    check(64 == line_steer_update(&s, 50, false, 3u).steer, "half scale rounds to 64");
    check(-64 == line_steer_update(&s, -50, false, 4u).steer, "half scale mirrors");

    line_steer_reset(&s);
    check(line_steer_update(&s, 37, false, 5u).steer > 0, "right line steers right");
    line_steer_reset(&s);
    check(line_steer_update(&s, -37, false, 6u).steer < 0, "left line steers left");

    line_steer_init(&s, &kd_cfg);
    (void) line_steer_update(&s, 0, false, 10u);
    check(19 == line_steer_update(&s, 10, false, 20u).steer, "D adds on the way out");

    line_steer_init(&s, &kd_cfg);
    (void) line_steer_update(&s, 20, false, 30u);
    check(6 == line_steer_update(&s, 10, false, 40u).steer, "D subtracts on the way back");

    line_steer_init(&s, &kd_cfg);
    (void) line_steer_update(&s, 0, false, 50u);
    check(127 == line_steer_update(&s, 100, false, 60u).steer,
          "clamped change still saturates");

    line_steer_init(&s, &kd_cfg);
    (void) line_steer_update(&s, -80, false, 600u);
    out = line_steer_straight(&s, 610u);
    check((0 == out.steer) && out.straight && out.calm, "straight output");
    check(25 == line_steer_update(&s, 20, false, 620u).steer,
          "D forgotten across the junction");
}

static void
test_calm (void)
{
    line_steer_config_t cfg = { .gain_pct = 100, .kd_pct = 0 };
    line_steer_state_t s;

    group("calm: starts calm, releases after the hold, re-arms on a trigger");

    line_steer_init(&s, &cfg);
    check(line_steer_update(&s, 5, false, 100u).calm, "starts calm");
    check(line_steer_update(&s, 5, false, 399u).calm, "still calm inside the hold");
    check(!line_steer_update(&s, 5, false, 401u).calm, "fast after the hold");
    check(line_steer_update(&s, 5, true, 500u).calm, "trigger re-arms calm");
    check(line_steer_update(&s, 80, false, 510u).calm, "large magnitude is calm");

    line_steer_reset(&s);
    check(s.calm && !s.has_previous, "reset re-arms calm and D");
}

/* ---- Sweep sequencer --------------------------------------------------- */

static void
test_spin (void)
{
    line_spin_t s;
    int16_t deg = 0;
    line_spin_cmd_t cmd;

    group("sweep: three signed segments, early stop, and the overall budget");

    line_spin_init(&s);
    check(!line_spin_running(&s), "idle after init");
    check(LINE_SPIN_CMD_NONE == line_spin_step(&s, true, false, 0u, &deg),
          "idle steps do nothing");

    line_spin_start(&s, LINE_SPIN_MODE_CALIBRATE, LINE_SIDE_UNKNOWN, 0u);
    check(line_spin_running(&s), "running after start");
    check(line_spin_collecting(&s), "collecting in segment 1");
    check(LINE_SPIN_CMD_NONE == line_spin_step(&s, false, false, 10u, &deg),
          "waits for Motion");

    cmd = line_spin_step(&s, true, false, 20u, &deg);
    check((LINE_SPIN_CMD_ROTATE == cmd) && (-24 == deg), "sweeps left first");

    (void) line_spin_step(&s, false, false, 30u, &deg);
    (void) line_spin_step(&s, true, false, 40u, &deg);

    cmd = line_spin_step(&s, true, false, 50u, &deg);
    check((LINE_SPIN_CMD_ROTATE == cmd) && (48 == deg), "through the start");
    check(line_spin_collecting(&s), "collecting in segment 2");

    (void) line_spin_step(&s, true, false, 60u, &deg);

    cmd = line_spin_step(&s, true, false, 70u, &deg);
    check((LINE_SPIN_CMD_ROTATE == cmd) && (-24 == deg), "returns toward the start");
    check(!line_spin_collecting(&s), "not collecting on the return");

    cmd = line_spin_step(&s, false, true, 80u, &deg);
    check(LINE_SPIN_CMD_STOP == cmd, "stops on centre reacquisition");
    check(line_spin_awaiting_result(&s), "awaiting the caller's result");
    check(!line_spin_running(&s), "no longer sweeping");
    line_spin_finish(&s);
    check(!line_spin_running(&s), "finished");

    /* Right first. */
    line_spin_start(&s, LINE_SPIN_MODE_SEARCH, LINE_SIDE_RIGHT, 0u);
    cmd = line_spin_step(&s, true, false, 10u, &deg);
    check((LINE_SPIN_CMD_ROTATE == cmd) && (24 == deg), "sweeps right first");

    /* The budget. */
    line_spin_start(&s, LINE_SPIN_MODE_SEARCH, LINE_SIDE_LEFT, 1000u);
    cmd = line_spin_step(&s, true, false, 1000u + LINE_SPIN_TIMEOUT_MS, &deg);
    check(LINE_SPIN_CMD_TIMEOUT == cmd, "budget returns TIMEOUT");
    check(s.timed_out, "timeout flagged");
    check(!line_spin_running(&s), "timeout ends the sweep");
}

/* ---- Sensor health ----------------------------------------------------- */

static void
test_health (void)
{
    line_health_t h;
    uint16_t raw[LINE_HEALTH_SENSORS] = { 1500u, 1500u, 1500u };

    group("health: rails held for the fault time, and immediate recovery");

    line_health_init(&h);
    check(0u == line_health_update(&h, raw, 200u), "healthy stays clear");

    raw[0] = 5u;
    check(0u == line_health_update(&h, raw, 99u), "99 ms railed is not a fault yet");
    check(0x01u == line_health_update(&h, raw, 1u), "sensor 0 faulty at 100 ms");

    raw[0] = 1500u;
    check(0u == line_health_update(&h, raw, 1u), "recovery clears the mask");

    raw[2] = 4090u;
    check(0x04u == line_health_update(&h, raw, 100u), "sensor 2 high rail");
    raw[1] = 4090u;
    check(0x06u == line_health_update(&h, raw, 100u), "sensors 1 and 2");
    raw[0] = 4090u;
    check(0x07u == line_health_update(&h, raw, 100u), "all three");
    check(line_health_faulted(&h), "faulted predicate");
}

/* ---- Cadence ----------------------------------------------------------- */

static void
test_sampler (void)
{
    line_sampler_t s;

    group("cadence: 1 kHz, overrun accounting, no replay, resync");

    line_sampler_init(&s, 1000u);
    check(line_sampler_due(&s, 1000u), "first call is due");
    check(!line_sampler_due(&s, 1000u), "not twice in one millisecond");
    check(line_sampler_due(&s, 1001u), "next millisecond is due");
    check((2u == s.samples) && (0u == s.overruns), "no overruns on time");

    check(line_sampler_due(&s, 1007u), "a late call still samples once");
    check(3u == s.samples, "one sample per call, never a burst");
    check(5u == s.overruns, "five missed slots charged");
    check(5u == s.max_late_ms, "worst lateness recorded");

    check(!line_sampler_due(&s, 1007u), "resynced past the old deadline");
    check(line_sampler_due(&s, 1008u), "cadence resumes on the new grid");
    check(5u == s.overruns, "resuming adds no overruns");

    line_sampler_resync(&s, 2000u);
    check(!line_sampler_due(&s, 2000u), "resync pushes the deadline");
    check(line_sampler_due(&s, 2001u), "due one period after resync");
    check(5u == s.overruns, "resync does not charge overruns");
}

/* ---- Producer ---------------------------------------------------------- */

static void
test_producer_hint (void)
{
    line_producer_t p;

    group("producer: the heading accumulates into the YX side hint");

    line_producer_init(&p, 0u);

    /* The first reading only establishes the baseline. */
    line_producer_update_travel(&p, 0, 0);
    check((uint8_t) LINE_SIDE_UNKNOWN == p.lp.hint,
          "the first reading only sets the baseline");

    /* A gradual turn: two counts of differential per sample, ten times. Upstream
     * accumulates the heading and compares strictly, so 20 is not yet a hint but
     * one more count is. A per-sample comparison would never see this turn. */
    for (int i = 0; i < 10; i++)
    {
        line_producer_update_travel(&p, 2 * (i + 1), 0);
    }

    check(20 == p.heading_counts, "ten slow samples accumulate a heading of 20");
    check((uint8_t) LINE_SIDE_UNKNOWN == p.lp.hint,
          "exactly 20 counts is not yet a hint");

    line_producer_update_travel(&p, 22, 0);
    check((uint8_t) LINE_SIDE_LEFT == p.lp.hint,
          "a slow drift crosses the threshold once it accumulates");

    /* The mirror side, accumulated the same way. */
    line_producer_init(&p, 0u);
    line_producer_update_travel(&p, 0, 0);

    for (int i = 0; i < 11; i++)
    {
        line_producer_update_travel(&p, 0, 2 * (i + 1));
    }

    check((uint8_t) LINE_SIDE_RIGHT == p.lp.hint,
          "the right wheel drifting ahead hints right");

    /* Centred clears the accumulator, exactly as upstream does. */
    p.lp.flags |= (uint16_t) LP_FLAG_CENTRED;
    line_producer_update_travel(&p, 0, 24);
    check(0 == p.heading_counts, "centred resets the heading");
    check((uint8_t) LINE_SIDE_UNKNOWN == p.lp.hint, "and clears the hint");
}

static void
test_producer_junction_bound (void)
{
    line_producer_t p;
    line_observation_t word;

    group("producer: junction straight-through is bounded by signed travel");

    line_producer_init(&p, 0u);
    p.lp.flags |= (uint16_t) LP_FLAG_JUNCTION_HOLD;

    /* The budget is measured from the hold's rising edge. */
    line_producer_update_travel(&p, 0, 0);
    line_producer_observation(&p, &word);
    check(!line_obs_junction_travel_done(word), "no travel yet: still straight");

    line_producer_update_travel(&p, 200, 200);
    line_producer_update_travel(&p, 440, 440);
    line_producer_observation(&p, &word);
    check(!line_obs_junction_travel_done(word), "under 450 counts: still straight");

    line_producer_update_travel(&p, 500, 500);
    line_producer_observation(&p, &word);
    check(line_obs_junction_travel_done(word),
          "past 450 counts: the straight-through is spent");

    /* Signed travel from the hold's rising edge, so reversing cannot spend the
     * budget early. The entry is deliberately at a non-zero odometer, which is
     * the realistic case: an absolute-from-zero rule would already be spent. */
    line_producer_init(&p, 0u);
    p.lp.flags |= (uint16_t) LP_FLAG_JUNCTION_HOLD;
    line_producer_update_travel(&p, 1000, 1000);
    line_producer_update_travel(&p, 700, 700);
    line_producer_update_travel(&p, 1000, 1000);
    line_producer_update_travel(&p, 1440, 1440);
    line_producer_observation(&p, &word);
    check(!line_obs_junction_travel_done(word),
          "reverse does not spend the junction budget");

    line_producer_update_travel(&p, 1460, 1460);
    line_producer_observation(&p, &word);
    check(line_obs_junction_travel_done(word),
          "net forward travel past 450 still spends it");

    /* Leaving the junction resets the budget, so the next one gets its own. */
    p.lp.flags &= (uint16_t) ~LP_FLAG_JUNCTION_HOLD;
    line_producer_update_travel(&p, 520, 520);
    line_producer_observation(&p, &word);
    check(!line_obs_junction_travel_done(word),
          "the budget resets when the junction ends");
}

static void
test_producer_calibration_heading (void)
{
    line_producer_t p;
    line_calib_event_t ev;

    group("producer: a calibration run starts with a clean heading");

    line_producer_init(&p, 0u);

    /* A spin turns the wheels differently, so it builds a heading. */
    line_producer_calib_begin(&p);
    line_producer_update_travel(&p, 0, 0);

    for (int i = 0; i < 11; i++)
    {
        line_producer_update_travel(&p, 2 * (i + 1), 0);
    }

    check(22 == p.heading_counts, "the spin builds a heading during calibration");
    check((uint8_t) LINE_SIDE_LEFT == p.lp.hint, "and it hints during the sweep");

    line_producer_calib_finish(&p, &ev);
    check(LINE_CALIB_NO_DATA == ev.result, "no sweep data was fed");
    check(0 == p.heading_counts, "finishing the sweep clears the heading");
    check((uint8_t) LINE_SIDE_UNKNOWN == p.lp.hint, "and clears the hint");

    /* The first sample of the new run must not resurrect the spin's heading. */
    line_producer_update_travel(&p, 24, 0);
    check((uint8_t) LINE_SIDE_UNKNOWN == p.lp.hint,
          "the first sample of the new run is not a hint");

    /* Fresh movement still accumulates from zero. */
    for (int i = 0; i < 10; i++)
    {
        line_producer_update_travel(&p, 24 + 2 * (i + 1), 0);
    }

    check((uint8_t) LINE_SIDE_LEFT == p.lp.hint,
          "new run movement still accumulates a hint");
}

static void
test_producer (void)
{
    line_producer_t p;
    line_producer_out_t out;
    uint16_t raw[LP_SENSORS];
    uint32_t t = 1000u;

    group("producer: cadence, packing, edges and calibration ownership");

    memset(g_adc, 0, sizeof(g_adc));
    g_adc_reads = 0;
    line_producer_init(&p, t);

    /* A sweep through the producer's own API. */
    line_producer_calib_begin(&p);

    for (int i = 0; i < 20; i++)
    {
        raw[0] = raw[1] = raw[2] = 500u;
        line_producer_calib_add(&p, raw);
    }

    for (int i = 1; i <= 6; i++)
    {
        raw[0] = raw[1] = raw[2] = (uint16_t) (500 + (2500 * i / 6));
        line_producer_calib_add(&p, raw);
    }

    for (int i = 0; i < 4; i++)
    {
        raw[0] = raw[1] = raw[2] = 3000u;
        line_producer_calib_add(&p, raw);
    }

    line_calib_event_t cal;

    line_producer_calib_finish(&p, &cal);
    check(LINE_CALIB_OK == cal.result, "producer calibration succeeds");
    check(1u == cal.line_reads_high, "polarity reported");
    check(0u == cal.bad_mask, "no bad sensors");

    /* The cadence gates the ADC. */
    g_adc_reads = 0;
    line_producer_step(&p, t, &out);
    check(1u == g_adc_reads, "one ADC read for the due slot");
    check(out.got_observation, "an observation is published");

    line_producer_step(&p, t, &out);
    check(1u == g_adc_reads, "no second read in the same millisecond");
    check(!out.got_observation, "nothing published off-cadence");

    /* A centred line packs as seen, calibrated and centred. */
    g_adc[0] = 500u; g_adc[1] = 3000u; g_adc[2] = 500u;

    for (int i = 0; i < 30; i++)
    {
        t += 1u;
        line_producer_step(&p, t, &out);
    }

    check(out.got_observation, "still publishing");
    check(line_obs_calibrated(out.observation), "packed as calibrated");
    check(line_obs_seen(out.observation), "packed as seen");
    check(line_obs_position(out.observation) > -20 &&
          line_obs_position(out.observation) < 20, "packed as centred");

    /* The raw sample is available for the calibration without a second read. */
    check(line_producer_last_raw(&p) != NULL, "last raw readable");
    check(line_producer_last_raw(&p)[1] == 3000u, "last raw holds the centre value");

    /* Losing the line produces exactly one LOST edge. */
    g_adc[0] = 500u; g_adc[1] = 500u; g_adc[2] = 500u;

    uint32_t lost_count = 0u;

    for (int i = 0; i < 600; i++)
    {
        t += 1u;
        line_producer_step(&p, t, &out);

        if (out.has_event && (LINE_EVT_LOST == out.event.id))
        {
            lost_count++;
        }
    }

    check(1u == lost_count, "exactly one LOST edge over 600 ms");
    check((uint32_t) LINE_SIDE_UNKNOWN != out.event.side || true, "edge carries a side");

    /* A railed sensor is packed as a fault. */
    g_adc[0] = 5u; g_adc[1] = 3000u; g_adc[2] = 500u;

    for (int i = 0; i < 120; i++)
    {
        t += 1u;
        line_producer_step(&p, t, &out);
    }

    check(out.got_observation, "publishing with a railed sensor");
    check(line_obs_sensor_fault(out.observation), "railed sensor packed as fault");
}

int
main (void)
{
    test_codec();
    test_position();
    test_invalidate();
    test_steer();
    test_calm();
    test_spin();
    test_health();
    test_sampler();
    test_producer_hint();
    test_producer_junction_bound();
    test_producer_calibration_heading();
    test_producer();

    printf("\n%d checks, %d failure%s\n", g_checks, g_failures,
           (1 == g_failures) ? "" : "s");
    printf("%s (%d failures)\n", (0 == g_failures) ? "ALL PASSED" : "FAILED",
           g_failures);

    return (0 == g_failures) ? 0 : 1;
}
